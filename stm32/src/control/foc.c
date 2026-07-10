#include "control/foc.h"
#include "control/foc_math.h"
#include "common/settings.h"
#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE
#include "common/bbox.h"
#endif
#include "control/pid.h"
#include "hall/hall.h"
#include "fault/fault.h"
#include "main.h"
#include <stdio.h>
#include <math.h>

extern ADC_HandleTypeDef hadc1;   /* injected rank 1 = phase A (master) */
extern ADC_HandleTypeDef hadc2;   /* injected rank 1 = phase B (slave)  */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

typedef struct {
    PidState pid_d;
    PidState pid_q;
    PidState pid_speed;           /* outer speed loop (Iq command)            */
    float    iq_ref;
    volatile float iq_target;     /* slew target for iq_ref (torque mode)     */
    float    offset_a, offset_b, offset_c;
    volatile float ia, ib, ic;   /* last phase currents (A)     */
    volatile float id, iq;        /* last rotating-frame currents */
    volatile bool  enabled;
    volatile bool  oc_trip;       /* software overcurrent backstop latched     */
    volatile bool  hall_trip;     /* invalid Hall code (0/7) latched           */
    uint8_t        hall_bad;      /* consecutive ticks with an invalid code    */
    volatile bool  speed_mode;    /* outer speed loop drives iq_ref           */
    volatile float omega_ref;     /* active (ramped) speed ref (elec rad/s)   */
    volatile float omega_target;  /* slew target for omega_ref                */
    uint16_t       speed_div;     /* 40 kHz → speed-loop prescaler counter    */
    volatile bool  force_mode;    /* open-loop forced-angle drive (Hall cal) */
    volatile float force_theta;   /* commanded electrical angle (rad)         */
    volatile float force_vd;      /* forced d-axis voltage (V)                */
    volatile float force_vq;      /* forced q-axis voltage (V)                */
#if FOC_DEBUG_ENABLE
    volatile bool  rotor_v_mode;  /* fixed vd/vq in live Hall θe frame        */
    volatile float rotor_vd;      /* rotor-frame d-axis voltage (V)           */
    volatile float rotor_vq;      /* rotor-frame q-axis voltage (V)           */
    volatile float vd_cmd;        /* last commanded d-axis voltage (V)        */
    volatile float vq_cmd;        /* last commanded q-axis voltage (V)        */
#endif

    /* 6-step block commutation */
    volatile bool  block_mode;
    volatile float block_duty;    /* commanded duty [0, BLOCK_DUTY_MAX]       */
    volatile float block_ramp;    /* ramped duty for soft start/stop          */
    int8_t         block_dir;     /* +1 CCW, −1 CW (swaps hi/lo in table)    */

    /* Ls capture (current-loop autotune): record d-axis current at the PWM
     * rate while a step voltage is applied in force_mode. */
#if FOC_DEBUG_ENABLE
    volatile bool  ls_capture;
    volatile uint8_t ls_idx;
    volatile float ls_vstep;
    float          ls_buf[TUNE_LS_CAP_N];

    /* Relay autotune (speed-loop): bang-bang iq_ref around relay_oref. */
    volatile bool  relay_mode;
    volatile float relay_h;       /* relay amplitude (A)                      */
    volatile float relay_oref;    /* speed setpoint (electrical rad/s)        */
    volatile float relay_hyst;    /* error hysteresis band (rad/s)            */
    volatile int8_t relay_sign;   /* current relay output sign                */
#endif
} FocState;

static FocState s;

/* 6-step commutation table: Hall state → {high_phase, low_phase}.
 * Phase indices: 0=A, 1=B, 2=C. Initialised from settings.h; can be runtime-
 * swapped for direction reversal. States 0 and 7 are invalid. */
static uint8_t s_block_hi[8] = BLOCK_TABLE_HI;
static uint8_t s_block_lo[8] = BLOCK_TABLE_LO;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                    */
/* ------------------------------------------------------------------ */

static void start_pwm_channels(void)
{
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    /* CH4 generates TRGO for ADC; no PWM output needed */
    HAL_TIM_PWM_Start(&htim1,  TIM_CHANNEL_4);
}

/* Move `cur` toward `tgt` by at most `step` (slew-rate limit) */
static float slew_toward(float cur, float tgt, float step)
{
    if (cur < tgt - step) return cur + step;
    if (cur > tgt + step) return cur - step;
    return tgt;
}

static void set_neutral_duty(void)
{
    TIM1->CCR1 = PWM_ARR / 2;
    TIM1->CCR2 = PWM_ARR / 2;
    TIM1->CCR3 = PWM_ARR / 2;
}

static float clamp_voltage(float v)
{
    if (v >  VDQ_MAX_V) return  VDQ_MAX_V;
    if (v < -VDQ_MAX_V) return -VDQ_MAX_V;
    return v;
}

/* Clamp the dq voltage vector inside the linear SVM circle (|v| ≤ VDQ_MAX_V),
 * d-axis first: vd keeps the full budget, vq gets the remainder. Used by the
 * open-loop drive paths; the closed current loop enforces the same circle
 * through the PI output limits so anti-windup stays truthful. */
static void limit_vdq_circle(float *vd, float *vq)
{
    float d = clamp_voltage(*vd);
    float q_max = sqrtf(VDQ_MAX_V * VDQ_MAX_V - d * d);
    if (*vq >  q_max) *vq =  q_max;
    if (*vq < -q_max) *vq = -q_max;
    *vd = d;
}

static bool calibrate_offsets(void)
{
    /* Poll for 128 dual injected-simultaneous conversions. ADC1 inj rank 1
     * samples phase A, ADC2 inj rank 1 phase B; */
    int32_t sum_a = 0, sum_b = 0;
    const int N = 128;

    HAL_ADCEx_InjectedStart(&hadc2);   /* slave first (no IT) */
    HAL_ADCEx_InjectedStart(&hadc1);   /* master, external TRGO trigger */

    for (int i = 0; i < N; i++) {
        /* Master JEOC flags both simultaneous conversions complete. Bound the
         * wait: if TRGO is not triggering conversions (e.g. TIM1 stopped), this
         * would otherwise hang the CLI forever. */
        uint32_t t0 = HAL_GetTick();
        while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC)) {
            if ((HAL_GetTick() - t0) >= 50u) {                           /* timeout */
                HAL_ADCEx_InjectedStop(&hadc1);
                HAL_ADCEx_InjectedStop(&hadc2);
                return false;                        /* calibration timed out */
            }
        }
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
        sum_a += (int32_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        sum_b += (int32_t)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    }

    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);

    s.offset_a = (float)(sum_a / N);
    s.offset_b = (float)(sum_b / N);
    /* Ic = -(Ia+Ib); the reconstructed phase-C zero-current offset is the
     * negated sum of the A/B offsets so foc_get_offset_c() stays meaningful. */
    s.offset_c = -(s.offset_a + s.offset_b);
    return true;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void foc_init(void)
{
    s.iq_ref     = 0.0f;
    s.iq_target  = 0.0f;
    s.enabled    = false;
    s.speed_mode = false;
    s.omega_ref  = 0.0f;
    s.omega_target = 0.0f;
    s.speed_div  = 0;
    s.force_mode = false;
#if FOC_DEBUG_ENABLE
    s.rotor_v_mode = false;
    s.vd_cmd = 0.0f;
    s.vq_cmd = 0.0f;
#endif
    s.block_mode = false;
    s.block_duty = 0.0f;
    s.block_ramp = 0.0f;
    s.block_dir  = +1;

    pid_init(&s.pid_d, PID_D_KP, PID_D_KI, -VDQ_MAX_V, VDQ_MAX_V);
    pid_init(&s.pid_q, PID_Q_KP, PID_Q_KI, -VDQ_MAX_V, VDQ_MAX_V);
    pid_init(&s.pid_speed, SPEED_KP, SPEED_KI,
             -MOTOR_CURRENT_LIMIT_A, MOTOR_CURRENT_LIMIT_A);

    start_pwm_channels();
    set_neutral_duty();

    /* Calibrate with MOE disabled — TIM1 TRGO runs but outputs are idle.
     * (A timeout here leaves offsets at 0; the 40 kHz loop is started by main.) */
    (void)calibrate_offsets();
}

void foc_enable(void)
{
    if (s.force_mode || s.block_mode
#if FOC_DEBUG_ENABLE
        || s.rotor_v_mode
#endif
    ) return;
    pid_reset(&s.pid_d);
    pid_reset(&s.pid_q);
    set_neutral_duty();
    s.enabled = true;
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void foc_disable(void)
{
    /* Clear flag first so ISR stops writing CCRs */
    s.enabled    = false;
    s.speed_mode = false;
#if FOC_DEBUG_ENABLE
    s.relay_mode = false;
#endif
    s.block_mode = false;
    s.force_mode = false;
#if FOC_DEBUG_ENABLE
    s.rotor_v_mode = false;
#endif
    s.iq_ref     = 0.0f;   /* re-enable must not resume the old torque command */
    s.iq_target  = 0.0f;
    set_neutral_duty();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    pid_reset(&s.pid_d);
    pid_reset(&s.pid_q);
    pid_reset(&s.pid_speed);
}

void foc_emergency_stop(void)
{
    /* ISR-safe minimal kill: cut all 6 outputs in one register write and stop
     * the current loop from driving them. No neutral-duty / PID work here —
     * MOE=0 disconnects the bridge regardless of CCR. fault_poll() runs the
     * full foc_disable() afterward in thread context. */
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    s.enabled    = false;
    s.speed_mode = false;
#if FOC_DEBUG_ENABLE
    s.relay_mode = false;
#endif
    s.force_mode = false;
#if FOC_DEBUG_ENABLE
    s.rotor_v_mode = false;
#endif
    s.block_mode = false;
#if FOC_DEBUG_ENABLE
    s.ls_capture = false;
#endif
#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE
    bbox_trigger();   /* freeze the black box around the fault, if armed */
#endif
}

void foc_set_iq_ref(float iq_a)
{
    s.speed_mode = false;            /* manual torque command exits speed mode */
    if (iq_a >  MOTOR_CURRENT_LIMIT_A) iq_a =  MOTOR_CURRENT_LIMIT_A;
    if (iq_a < -MOTOR_CURRENT_LIMIT_A) iq_a = -MOTOR_CURRENT_LIMIT_A;
    s.iq_target = iq_a;              /* iq_ref slews toward it in the ISR */
}

/* Speed-loop outer cascade */

void foc_set_speed_ref(float omega_e)
{
    /* Reset the integrator only on entry to avoid carrying stale windup;
     * the reference ramp starts from the measured speed so entry is bumpless.
     * Re-issuing spd while already in speed mode just retargets the ramp. */
    if (!s.speed_mode) {
        pid_reset(&s.pid_speed);
        s.omega_ref = hall_get_omega_e();
    }
    s.omega_target = omega_e;
    s.speed_div    = 0;
    s.speed_mode   = true;
}

void foc_speed_disable(void)
{
    s.speed_mode = false;
    pid_reset(&s.pid_speed);
}

bool  foc_is_speed_mode(void) { return s.speed_mode; }
float foc_get_speed_ref(void) { return s.omega_target; }

#if FOC_DEBUG_ENABLE
void foc_set_speed_kp(float kp) { pid_set_gains(&s.pid_speed, kp, s.pid_speed.ki); }
void foc_set_speed_ki(float ki) { pid_set_gains(&s.pid_speed, s.pid_speed.kp, ki); }
float foc_get_speed_kp(void)  { return s.pid_speed.kp; }
float foc_get_speed_ki(void)  { return s.pid_speed.ki; }

void foc_set_kp(float kp)
{
    pid_set_gains(&s.pid_d, kp, s.pid_d.ki);
    pid_set_gains(&s.pid_q, kp, s.pid_q.ki);
}

void foc_set_ki(float ki)
{
    pid_set_gains(&s.pid_d, s.pid_d.kp, ki);
    pid_set_gains(&s.pid_q, s.pid_q.kp, ki);
}
#endif

bool foc_recalibrate(void)
{
    /* Requires FOC disabled; caller must verify */
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);
    bool ok = calibrate_offsets();
    HAL_ADCEx_InjectedStart(&hadc2);      /* slave first (no IT) */
    HAL_ADCEx_InjectedStart_IT(&hadc1);   /* master drives the 40 kHz ISR */
    return ok;
}

bool foc_force_begin(float vmag)
{
    if (s.enabled || s.block_mode
#if FOC_DEBUG_ENABLE
        || s.rotor_v_mode
#endif
    ) return false;
    s.force_vd    = clamp_voltage(vmag);
    s.force_vq    = 0.0f;
    s.force_theta = 0.0f;
    s.force_mode  = true;
    TIM1->BDTR |= TIM_BDTR_MOE;
    return true;
}

void foc_force_set_angle(float theta) { s.force_theta = theta; }
void foc_force_set_vq(float vq)       { s.force_vq = clamp_voltage(vq); }

void foc_force_end(void)
{
    s.force_mode = false;
    set_neutral_duty();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

#if FOC_DEBUG_ENABLE
bool foc_rotor_voltage_begin(float vd, float vq)
{
    if (s.enabled || s.force_mode || s.block_mode) return false;
    foc_rotor_voltage_set(vd, vq);
    s.rotor_v_mode = true;
    TIM1->BDTR |= TIM_BDTR_MOE;
    return true;
}

void foc_rotor_voltage_set(float vd, float vq)
{
    s.rotor_vd = clamp_voltage(vd);
    s.rotor_vq = clamp_voltage(vq);
}

void foc_rotor_voltage_end(void)
{
    bool was_on = s.rotor_v_mode;
    s.rotor_v_mode = false;
    if (was_on) {
        set_neutral_duty();
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
}
#endif


void foc_block_enable(float duty)
{
    if (s.enabled || s.force_mode || s.block_mode
#if FOC_DEBUG_ENABLE
        || s.rotor_v_mode
#endif
    ) return;
    s.block_duty = fabsf(duty);
    if (s.block_duty > BLOCK_DUTY_MAX) s.block_duty = BLOCK_DUTY_MAX;
    s.block_dir  = (duty >= 0.0f) ? (int8_t)+1 : (int8_t)-1;
    s.block_ramp = 0.0f;
    s.block_mode = true;
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void foc_block_disable(void)
{
    s.block_mode = false;
    s.block_ramp = 0.0f;
    set_neutral_duty();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

void  foc_block_set_duty(float duty)
{
    float  d       = fabsf(duty);
    if (d > BLOCK_DUTY_MAX) d = BLOCK_DUTY_MAX;
    int8_t new_dir = (duty >= 0.0f) ? (int8_t)+1 : (int8_t)-1;
    if (new_dir != s.block_dir) s.block_ramp = 0.0f;   /* reset ramp so reversal doesn't trip OCP */
    s.block_duty = d;
    s.block_dir  = new_dir;
}
bool  foc_is_block_mode(void)         { return s.block_mode; }
#if FOC_DEBUG_ENABLE
bool  foc_is_rotor_voltage_mode(void) { return s.rotor_v_mode; }
#endif

bool  foc_is_enabled(void)   { return s.enabled; }
bool  foc_oc_tripped(void)   { return s.oc_trip; }
void  foc_clear_oc_trip(void){ s.oc_trip = false; }
bool  foc_hall_tripped(void)    { return s.hall_trip; }
void  foc_clear_hall_trip(void) { s.hall_trip = false; s.hall_bad = 0; }
float foc_get_iq_ref(void)   { return s.iq_ref; }
float foc_get_iq_target(void){ return s.iq_target; }
#if FOC_DEBUG_ENABLE
float foc_get_kp(void)       { return s.pid_q.kp; }
float foc_get_ki(void)       { return s.pid_q.ki; }
float foc_get_vd_cmd(void)   { return s.vd_cmd; }
float foc_get_vq_cmd(void)   { return s.vq_cmd; }
#endif
float foc_get_ia(void)       { return s.ia; }
float foc_get_ib(void)       { return s.ib; }
float foc_get_ic(void)       { return s.ic; }
float foc_get_id(void)       { return s.id; }
float foc_get_iq(void)       { return s.iq; }
float foc_get_offset_a(void) { return s.offset_a; }
float foc_get_offset_b(void) { return s.offset_b; }
float foc_get_offset_c(void) { return s.offset_c; }

/* Autotune: blocking, thread-context */

#if FOC_DEBUG_ENABLE
/* Average the d-axis current (phase currents projected at the forced angle)
 * over `ms` milliseconds, read from thread context. */
static float tune_measure_id(uint32_t ms)
{
    float sum = 0.0f; uint32_t cnt = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < ms) {
        float al, be, idc, iqc;
        foc_clarke(s.ia, s.ib, s.ic, &al, &be);
        foc_park(al, be, s.force_theta, &idc, &iqc);
        sum += idc; cnt++;
        HAL_Delay(1);
    }
    return (cnt > 0) ? (sum / (float)cnt) : 0.0f;
}

bool foc_tune_current(char *report, size_t n)
{
    if (s.enabled || fault_is_active()) {
        snprintf(report, n, "  ctune: requires FOC disabled and no fault\r\n");
        return false;
    }

    /* Align rotor to the d-axis (angle 0) and hold there for the whole test */
    if (!foc_force_begin(TUNE_ALIGN_V)) {
        snprintf(report, n, "  ctune: could not enter force mode\r\n");
        return false;
    }
    s.force_theta = 0.0f;
    HAL_Delay(TUNE_ALIGN_MS);

    /* --- Rs: two voltage points cancel the constant dead-time voltage error --- */
    s.force_vd = TUNE_RS_V_LO;
    HAL_Delay(TUNE_RS_SETTLE_MS);
    float i_lo = tune_measure_id(TUNE_RS_SETTLE_MS);
    s.force_vd = TUNE_RS_V_HI;
    HAL_Delay(TUNE_RS_SETTLE_MS);
    float i_hi = tune_measure_id(TUNE_RS_SETTLE_MS);

    float di = i_hi - i_lo;
    if (di < 1.0e-3f) {                 /* no measurable current → open / disconnected */
        foc_force_end();
        snprintf(report, n, "  ctune: Rs measurement failed (di=%.4f A)\r\n", (double)di);
        return false;
    }
    float Rs = (TUNE_RS_V_HI - TUNE_RS_V_LO) / di;

    /* --- Ls: step from 0 V, capture the L/R current rise at the PWM rate --- */
    s.force_vd = 0.0f;
    HAL_Delay(TUNE_LS_DECAY_MS);
    for (uint8_t i = 0; i < TUNE_LS_CAP_N; i++) s.ls_buf[i] = 0.0f;
    s.ls_idx     = 0;
    s.ls_vstep   = TUNE_LS_VSTEP_V;
    s.ls_capture = true;
    uint32_t t0 = HAL_GetTick();
    while (s.ls_idx < TUNE_LS_CAP_N && (HAL_GetTick() - t0) < 50u) { }
    s.ls_capture = false;
    s.force_vd = 0.0f;
    foc_force_end();

    /* Fit τ = Ls/Rs using the measured settled current as the asymptote, so the
     * result is independent of voltage / dead-time scaling: i(t)=Iss(1-e^-t/τ).
     * Sample k has only seen (k − TUNE_LS_DELAY_TICKS)·Ts of drive (CCR preload
     * + trigger position), so use that as the time base or τ comes out ~30%
     * high with τ ≈ 4 ticks. */
    float i_ss = 0.25f * (s.ls_buf[TUNE_LS_CAP_N-1] + s.ls_buf[TUNE_LS_CAP_N-2]
                        + s.ls_buf[TUNE_LS_CAP_N-3] + s.ls_buf[TUNE_LS_CAP_N-4]);
    float tau_sum = 0.0f; uint32_t tau_n = 0;
    for (uint8_t k = 1; k < TUNE_LS_CAP_N; k++) {
        float t = ((float)k - TUNE_LS_DELAY_TICKS) * PWM_DT_S;
        float r = (i_ss > 1.0e-3f) ? (s.ls_buf[k] / i_ss) : 0.0f;
        if (t > 0.0f && r > 0.1f && r < 0.85f) {   /* well-conditioned part of the rise */
            tau_sum += -t / logf(1.0f - r);
            tau_n++;
        }
    }
    bool  ls_ok = (tau_n > 0);
    float Ls    = ls_ok ? Rs * (tau_sum / (float)tau_n) : MOTOR_LS_H;
    if (ls_ok && (Ls < MOTOR_LS_H / 3.0f || Ls > MOTOR_LS_H * 3.0f)) {
        ls_ok = false;                      /* implausible — fall back to datasheet */
        Ls    = MOTOR_LS_H;
    }

    /* Apply: Kp = Ls*wc, Ki = Rs*wc */
    float kp = Ls * TUNE_WC_RAD_S;
    float ki = Rs * TUNE_WC_RAD_S;
    foc_set_kp(kp);
    foc_set_ki(ki);

    snprintf(report, n,
        "  ctune results:\r\n"
        "    Rs = %.3f ohm  (datasheet %.3f)\r\n"
        "    Ls = %.1f uH   (datasheet %.1f)%s\r\n"
        "    -> kp = %.4f   ki = %.1f   (applied)\r\n",
        (double)Rs, (double)MOTOR_RS_DATASHEET_OHM,
        (double)(Ls * 1.0e6f), (double)(MOTOR_LS_H * 1.0e6f),
        ls_ok ? "" : "  [implausible, used datasheet]",
        (double)kp, (double)ki);
    return true;
}

bool foc_tune_speed(float omega_ref, char *report, size_t n)
{
    if (s.enabled || fault_is_active()) {
        snprintf(report, n, "  stune: requires FOC disabled and no fault\r\n");
        return false;
    }
    if (omega_ref <= 0.0f) {
        snprintf(report, n, "  stune: setpoint must be > 0\r\n");
        return false;
    }

    /* Engage the current loop and start the relay about the setpoint */
    foc_enable();
    s.relay_sign = +1;
    s.relay_h    = TUNE_SPEED_H;
    s.relay_oref = omega_ref;
    s.relay_hyst = TUNE_SPEED_HYST_FRAC * omega_ref;
    s.speed_div  = 0;
    s.relay_mode = true;

    /* Spin up toward the setpoint */
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < TUNE_SPEED_SPINUP_MS) {
        if (fault_is_active()) {
            foc_disable();
            snprintf(report, n, "  stune: fault during spin-up\r\n");
            return false;
        }
        if (hall_get_omega_e() > 0.5f * omega_ref) break;
        HAL_Delay(2);
    }

    /* Measure the limit cycle: timestamp rising error zero-crossings (speed
     * falling through the setpoint) and the per-cycle peak-to-peak amplitude. */
    const uint32_t need = TUNE_SPEED_CYCLES + 3u;   /* crossings to collect */
    uint32_t cross_ms[TUNE_SPEED_CYCLES + 4u];
    float    cyc_amp [TUNE_SPEED_CYCLES + 4u];
    uint32_t ncross = 0;
    float    wmax = -1.0e9f, wmin = 1.0e9f;
    float    prev_e = omega_ref - hall_get_omega_e();
    uint32_t tmo = HAL_GetTick();

    while (ncross < need) {
        if (fault_is_active()) {
            s.relay_mode = false; foc_disable();
            snprintf(report, n, "  stune: fault during measurement\r\n");
            return false;
        }
        if ((HAL_GetTick() - tmo) > TUNE_SPEED_TIMEOUT_MS) {
            s.relay_mode = false; foc_disable();
            snprintf(report, n,
                "  stune: timeout — no stable limit cycle (try another rpm)\r\n");
            return false;
        }
        float w = hall_get_omega_e();
        if (w > wmax) wmax = w;
        if (w < wmin) wmin = w;
        float e = omega_ref - w;
        if (prev_e <= 0.0f && e > 0.0f) {           /* rising crossing */
            if (ncross > 0) cyc_amp[ncross - 1] = wmax - wmin;
            cross_ms[ncross++] = HAL_GetTick();
            wmax = -1.0e9f; wmin = 1.0e9f;
        }
        prev_e = e;
        HAL_Delay(1);
    }

    s.relay_mode = false;
    foc_disable();

    /* Average period and half-amplitude over the post-transient cycles (skip 2) */
    float Tu_sum = 0.0f, a_sum = 0.0f; uint32_t m = 0;
    for (uint32_t j = 2; j + 1 < ncross; j++) {
        Tu_sum += (float)(cross_ms[j + 1] - cross_ms[j]) * 1.0e-3f;
        a_sum  += 0.5f * cyc_amp[j];
        m++;
    }
    if (m == 0 || a_sum <= 0.0f) {
        snprintf(report, n, "  stune: could not resolve limit cycle\r\n");
        return false;
    }
    float Tu = Tu_sum / (float)m;
    float a  = a_sum  / (float)m;               /* half peak-to-peak (rad/s) */
    float Ku = 4.0f * s.relay_h / (M_PI_F * a);

    /* Tyreus–Luyben PI (gentle): Kp = Ku/3.2, Ti = 2.2*Tu */
    float kp = Ku / 3.2f;
    float ki = kp / (2.2f * Tu);
    foc_set_speed_kp(kp);
    foc_set_speed_ki(ki);

    snprintf(report, n,
        "  stune results (relay h=%.2f A @ %.0f RPM):\r\n"
        "    Ku = %.5f   Tu = %.3f s   amp = %.1f RPM\r\n"
        "    -> skp = %.5f   ski = %.5f   (applied, Tyreus-Luyben)\r\n",
        (double)s.relay_h, (double)OMEGA_E_TO_RPM(omega_ref),
        (double)Ku, (double)Tu, (double)OMEGA_E_TO_RPM(a),
        (double)kp, (double)ki);
    return true;
}
#endif

/* ------------------------------------------------------------------ */
/* Current loop — 40 kHz, runs in ADC injected-conversion ISR          */
/* ------------------------------------------------------------------ */

static void foc_speed_loop(void)
{
    /* Slew the speed reference toward the setpoint so a step command ramps */
    s.omega_ref = slew_toward(s.omega_ref, s.omega_target,
                              SPEED_REF_SLEW_RAD_S2 * SPEED_DT_S);

    /* hall_get_omega_e() already reports the physical-frame signed speed (it folds
     * in HALL_PHYS_DIR_SIGN). Run the PI in that frame, then map the current
     * command back to the control frame so positive error always yields negative
     * feedback regardless of the board's θe-vs-rotation polarity. */
    float omega_meas = hall_get_omega_e();
    float i_cmd = pid_update(&s.pid_speed, s.omega_ref - omega_meas, SPEED_DT_S);
    s.iq_ref = (float)HALL_PHYS_DIR_SIGN * i_cmd;
}

#if FOC_DEBUG_ENABLE
/* Relay (bang-bang) law for speed-loop autotune. Drives iq_ref to ±relay_h
 * about the setpoint with a hysteresis band, forcing a steady limit cycle. */
static void foc_relay_update(void)
{
    float e = s.relay_oref - hall_get_omega_e();
    if      (e >  s.relay_hyst) s.relay_sign = +1;
    else if (e < -s.relay_hyst) s.relay_sign = -1;
    /* within the band: hold the previous sign */
    s.iq_ref = (float)HALL_PHYS_DIR_SIGN * (float)s.relay_sign * s.relay_h;
}
#endif

static void foc_current_loop(void)
{
    /* On STM32H7 the master ADC1 injected JEOS fires one callback per TIM1 TRGO
     * = 40 kHz, with the dual injected-simultaneous pair already converted:
     * ADC1 inj rank 1 = phase A, ADC2 inj rank 1 = phase B. Phase C is
     * reconstructed as -(Ia+Ib) since there is no third current channel.      */

    /* 1. Read phase currents (used by FOC, force-mode, software overcurrent, and CLI) */
    float ia = ((float)(ADC1->JDR1) - s.offset_a) * ADC_CURRENT_SCALE;
    float ib = ((float)(ADC2->JDR1) - s.offset_b) * ADC_CURRENT_SCALE;
    float ic = -(ia + ib);

    s.ia = ia; s.ib = ib; s.ic = ic;

    /* Advance the Hall observer every tick so θ̂ tracks continuously, even while
     * disabled or in force mode (its output is only used when enabled). Black-box
     * sample right after, so θ̂/ω̂/innovation are fresh; runs for every mode and
     * before the OC check below, so the trip tick itself is still recorded
     * (foc_emergency_stop() freezes the capture). id/iq/iq_ref hold the last
     * values written by whichever control branch ran. */
    hall_observer_update(PWM_DT_S);
#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE
    bbox_sample(s.iq, s.id, s.iq_ref, ia);
#endif

    /* Software overcurrent backstop. The DRV8316 hardware OCP is the last line of
     * defence; trip here first (lower threshold) so a bad electrical angle, sign
     * error, or runaway fails soft in firmware instead of slamming the gate
     * driver. Runs before the force/enabled branches, so it also guards the
     * open-loop hcal/ctune drive. foc_emergency_stop() cuts MOE in one register
     * write; fault_poll() surfaces the latch and full-disables in thread ctx. */
    if (fabsf(ia) > MOTOR_OC_TRIP_A || fabsf(ib) > MOTOR_OC_TRIP_A ||
        fabsf(ic) > MOTOR_OC_TRIP_A) {
        foc_emergency_stop();
        s.oc_trip = true;
        return;
    }

    /* Invalid Hall code supervision. A disconnected cable reads 0b111 (pull-
     * ups) and a short reads 0b000; both would otherwise feed a fabricated
     * angle (FOC/rotor modes) or the {0,0} filler table entry (block mode)
     * into the drive. Sensor skew during a normal edge can pass through 0/7
     * for a few µs, so require the code to persist before tripping. Force
     * mode is exempt — it does not consume the Hall angle. */
    {
        uint8_t hst = hall_state_now();
        if (hst == 0u || hst == 7u) {
            if (s.hall_bad < 0xFFu) s.hall_bad++;
        } else {
            s.hall_bad = 0;
        }
        if (s.hall_bad >= HALL_INVALID_TRIP_TICKS &&
            (s.enabled || s.block_mode
#if FOC_DEBUG_ENABLE
             || s.rotor_v_mode
#endif
            )) {
            foc_emergency_stop();
            s.hall_trip = true;
            return;
        }
    }

    /* 6-step block commutation with soft ramp */
    if (s.block_mode) {
        float ramp = s.block_ramp;
        float tgt  = s.block_duty;
        if (ramp < tgt) {
            ramp += BLOCK_RAMP_STEP;
            if (ramp > tgt) ramp = tgt;
        } else if (ramp > tgt) {
            ramp -= BLOCK_RAMP_STEP;
            if (ramp < tgt) ramp = tgt;
        }
        s.block_ramp = ramp;

        uint8_t st = hall_get_sector();       /* 0..7, valid 1..6 */
        uint8_t hi = s_block_hi[st];
        uint8_t lo = s_block_lo[st];

        /* Invalid Hall code (filler {0,0} entry): hold all phases at 50%
         * (zero line-line volts) until the supervision above latches the
         * trip, instead of half-driving phase A. */
        if (hi == lo) {
            set_neutral_duty();
            return;
        }

        if (s.block_dir < 0) { uint8_t tmp = hi; hi = lo; lo = tmp; }

        float half = (float)PWM_ARR * 0.5f;
        float d    = half * ramp;

        float ccr[3];
        ccr[0] = half;                       /* default: 50% = float */
        ccr[1] = half;
        ccr[2] = half;
        ccr[hi] = half + d;                  /* high-side PWM */
        ccr[lo] = half - d;                  /* low-side PWM  */

        TIM1->CCR1 = (uint16_t)ccr[0];
        TIM1->CCR2 = (uint16_t)ccr[1];
        TIM1->CCR3 = (uint16_t)ccr[2];
        return;
    }

    /* Open-loop forced-angle drive (Hall calibration / Rs-Ls autotune): apply a
     * stationary voltage vector at the commanded angle, bypassing the PI loop. */
    if (s.force_mode) {
        float theta = s.force_theta;
        float st = sinf(theta), ct = cosf(theta);
        /* Compute id/iq so CLI diagnostics (idq/status) read live values */
        {
            float al, be;
            float id, iq;
            foc_clarke(ia, ib, ic, &al, &be);
            foc_park_sc(al, be, st, ct, &id, &iq);
            s.id = id;
            s.iq = iq;
        }
        float vd = s.force_vd;
        float vq = s.force_vq;
        /* Ls capture: drive the step voltage on the d-axis and log d-axis current
         * per cycle.  (vq stays 0; the Ls step is pure d-axis.) */
#if FOC_DEBUG_ENABLE
        if (s.ls_capture && s.ls_idx < TUNE_LS_CAP_N) {
            s.ls_buf[s.ls_idx++] = s.id;
            vd = s.ls_vstep;
        }
#endif
        limit_vdq_circle(&vd, &vq);
#if FOC_DEBUG_ENABLE
        s.vd_cmd = vd;
        s.vq_cmd = vq;
#endif
        float va, vb;
        uint16_t c1, c2, c3;
        foc_inv_park_sc(vd, vq, st, ct, &va, &vb);
        foc_svm(va, vb, VBUS_V, ia, ib, ic, &c1, &c2, &c3);
        TIM1->CCR1 = c1; TIM1->CCR2 = c2; TIM1->CCR3 = c3;
        return;
    }

#if FOC_DEBUG_ENABLE
    if (s.rotor_v_mode) {
        float ialpha, ibeta;
        foc_clarke(ia, ib, ic, &ialpha, &ibeta);

        float theta_e = hall_get_theta_e();
        float st = sinf(theta_e), ct = cosf(theta_e);
        float id, iq;
        foc_park_sc(ialpha, ibeta, st, ct, &id, &iq);
        s.id = id;
        s.iq = iq;

        float vd = s.rotor_vd;
        float vq = s.rotor_vq;
        limit_vdq_circle(&vd, &vq);
        float valpha, vbeta;
        uint16_t c1, c2, c3;
        s.vd_cmd = vd;
        s.vq_cmd = vq;
        foc_inv_park_sc(vd, vq, st, ct, &valpha, &vbeta);
        foc_svm(valpha, vbeta, VBUS_V, ia, ib, ic, &c1, &c2, &c3);
        TIM1->CCR1 = c1; TIM1->CCR2 = c2; TIM1->CCR3 = c3;
        return;
    }
#endif

    if (!s.enabled) return;

    /* Outer-loop slot (downsampled to 1 kHz): relay autotune or speed PI
     * generates iq_ref in place for the current loop below. */
    if ((
#if FOC_DEBUG_ENABLE
         s.relay_mode ||
#endif
         s.speed_mode) && (++s.speed_div >= SPEED_LOOP_DIV)) {
        s.speed_div = 0;
#if FOC_DEBUG_ENABLE
        if (s.relay_mode) foc_relay_update();
        else
#endif
                         foc_speed_loop();
    }

    /* Torque mode: slew iq_ref toward the commanded target. (Speed/relay
     * modes write iq_ref directly above and must not be rate-limited here.) */
    if (!s.speed_mode
#if FOC_DEBUG_ENABLE
        && !s.relay_mode
#endif
    )
        s.iq_ref = slew_toward(s.iq_ref, s.iq_target,
                               IQ_REF_SLEW_A_PER_S * PWM_DT_S);

    /* 2. Clarke → αβ, Park → dq (sin/cos computed once, shared with the
     * inverse Park below) */
    float ialpha, ibeta;
    foc_clarke(ia, ib, ic, &ialpha, &ibeta);

    float theta_e = hall_get_theta_e();
    float sin_t = sinf(theta_e), cos_t = cosf(theta_e);
    float id_meas, iq_meas;
    foc_park_sc(ialpha, ibeta, sin_t, cos_t, &id_meas, &iq_meas);

    s.id = id_meas;
    s.iq = iq_meas;

    /* 3. PI outputs, d-axis first. The q-axis output clamp is retargeted every
     * tick to the remainder of the voltage circle (√(V²−vd²)), so the combined
     * vector never leaves the linear SVM range and the back-calculation
     * anti-windup inside pid_update() sees the true applied limit — no hidden
     * windup while saturated.
     * (Back-EMF feedforward / cross-coupling decoupling intentionally off;
     * recover θe-frame speed via HALL_PHYS_DIR_SIGN·hall_get_omega_e() if
     * re-enabling: vd −= ωe·Ls·iq, vq += ωe·Ke.) */
    float vd = pid_update(&s.pid_d, -id_meas, PWM_DT_S);
    float vq_max = sqrtf(VDQ_MAX_V * VDQ_MAX_V - vd * vd);
    pid_set_output_limits(&s.pid_q, -vq_max, vq_max);
    float vq = pid_update(&s.pid_q, s.iq_ref - iq_meas, PWM_DT_S);
#if FOC_DEBUG_ENABLE
    s.vd_cmd = vd;
    s.vq_cmd = vq;
#endif

    /* 4. Inverse Park → αβ */
    float valpha, vbeta;
    foc_inv_park_sc(vd, vq, sin_t, cos_t, &valpha, &vbeta);

    /* 5. SVM (with dead-time compensation) → CCR values */
    uint16_t ccr1, ccr2, ccr3;
    foc_svm(valpha, vbeta, VBUS_V, ia, ib, ic, &ccr1, &ccr2, &ccr3);

    /* Write to timer registers */
    if (s.enabled) {
        TIM1->CCR1 = ccr1;
        TIM1->CCR2 = ccr2;
        TIM1->CCR3 = ccr3;
    }
}

/* ------------------------------------------------------------------ */
/* HAL weak callback overrides                                          */
/* ------------------------------------------------------------------ */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        foc_current_loop();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        hall_update();
    }
}
