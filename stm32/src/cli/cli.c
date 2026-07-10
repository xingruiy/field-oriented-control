#include "cli/cli.h"
#include "control/foc.h"
#include "hall/hall.h"
#include "common/settings.h"
#if FOC_DEBUG_ENABLE
#include "drv8316/drv8316.h"
#endif
#include "fault/fault.h"
#include "control/foc_math.h"
#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE
#include "common/bbox.h"
#endif
#include "encoder/encoder.h"
#include "arm/arm_pos.h"
#include "can/can.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

extern UART_HandleTypeDef huart3;

#define CMD_BUF_SIZE  64
#define OUT_BUF_SIZE  256

static uint8_t  s_rx_byte;                 /* single-byte interrupt RX target */
static char     s_accum_buf[CMD_BUF_SIZE]; /* accumulates chars across bytes  */
static uint16_t s_accum_len;
static char     s_cmd_buf[CMD_BUF_SIZE];   /* completed line for dispatch     */
static volatile int s_cmd_ready;

/* ------------------------------------------------------------------ */
/* Transmit helpers                                                    */
/* ------------------------------------------------------------------ */

void cli_print(const char *str)
{
    /* Block until the whole string is sent. A fixed timeout truncates long
     * output (e.g. 'help' ~1.3 KB ≈ 114 ms at 115200, over a 100 ms budget),
     * which drops the trailing CRLF and runs the prompt onto the same line. */
    HAL_UART_Transmit(&huart3, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

static void cli_printf(const char *fmt, ...)
{
    char buf[OUT_BUF_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
    }
}

/* ------------------------------------------------------------------ */
/* Command dispatch                                                    */
/* ------------------------------------------------------------------ */

static void cmd_help(void)
{
    cli_print(
        "\r\n"
        "  help / ?    print this list\r\n"
        "  status      FOC state summary\r\n"
        "  ia/ib/ic    phase current (A)\r\n"
        "  iabc        all three phase currents\r\n"
        "  idq         Id and Iq (rotating frame)\r\n"
        "  adc         raw ADC counts and offsets\r\n"
#if FOC_DEBUG_ENABLE
        "  hall        Hall state, theta_e, speed + innovation/glitch stats\r\n"
        "  hall rst    reset the innovation/glitch stats\r\n"
#else
        "  hall        Hall state, theta_e, speed\r\n"
#endif
        "  enc         TMAG5273 external angle encoder status\r\n"
        "  enc rst     zero the encoder turn counter\r\n"
        "  apos <deg>  TMAG arm position PID target (absolute total degrees)\r\n"
        "  apos off    disable arm position PID\r\n"
        "  apk/aki/akd [v] get/set arm position PID gains\r\n"
        "  en          enable FOC (MOE on; refused while fault latched)\r\n"
        "  dis         disable FOC (MOE off)\r\n"
        "  iq <A>      set Iq reference in amperes (negative = reverse)\r\n"
#if FOC_DEBUG_ENABLE
        "  kp [<v>]    get/set current-loop Kp (both d and q axes)\r\n"
        "  ki [<v>]    get/set current-loop Ki (both d and q axes)\r\n"
        "  vout        last commanded Vd/Vq voltage\r\n"
#endif
        "  spd <rpm>   set speed target (RPM, neg=reverse); 'spd off' = manual\r\n"
#if FOC_DEBUG_ENABLE
        "  rv <vd> <vq> fixed rotor-frame voltage using live theta_e; 'rv off'\r\n"
        "  skp [<v>]   get/set speed-loop Kp\r\n"
        "  ski [<v>]   get/set speed-loop Ki\r\n"
        "  ctune       autotune current loop (measure Rs/Ls; requires dis)\r\n"
        "  stune [rpm] autotune speed loop (relay; requires dis)\r\n"
        "  drv         dump DRV8316 registers\r\n"
#endif
        "  fault       show latched fault decode + SPI error count\r\n"
        "  clrf        clear DRV8316 faults + software latch\r\n"
        "  cal         re-calibrate ADC offsets (requires dis first)\r\n"
#if FOC_DEBUG_ENABLE
        "  hoff [<r>]  get/set Hall angle offset in radians (tune to stop vibration)\r\n"
        "  hpll [kp ki] get/set Hall PLL gains (live; smooths theta/speed)\r\n"
#endif
        "  hcal        calibrate Hall angle table (open-loop, requires dis)\r\n"
#if FOC_DEBUG_ENABLE
        "  hchk        check Hall angles vs forced ground truth (requires dis)\r\n"
        "  bb          black-box recorder status; 'bb arm' / 'bb trig'\r\n"
#endif
        "  blk <d>     enable 6-step block commutation (duty 0..1, neg=rev)\r\n"
        "  blk off     disable 6-step\r\n"
    );
}

static void cmd_status(void)
{
    float  omega_e = hall_get_omega_e();
    float  rpm     = fabsf(omega_e) * (60.0f / M_TWOPI_F);  /* 1 pole pair → mech = elec */
    int8_t dir     = hall_get_dir();
    const char *dir_str = (dir > 0) ? "CCW" : (dir < 0) ? "CW" : "---";
    cli_printf(
        "  enabled : %s\r\n"
        "  mode    : %s\r\n"
        "  spd_ref : %.1f RPM\r\n"
        "  iq_ref  : %.4f A\r\n"
        "  Id      : %.4f A\r\n"
        "  Iq      : %.4f A\r\n"
        "  speed   : %.1f RPM %s\r\n",
        foc_is_enabled() ? "yes" : "no",
#if FOC_DEBUG_ENABLE
        foc_is_rotor_voltage_mode() ? "rotor-v" :
#endif
        (foc_is_speed_mode() ? "speed" : "manual"),
        OMEGA_E_TO_RPM(foc_get_speed_ref()),
        foc_get_iq_ref(),
        foc_get_id(),
        foc_get_iq(),
        rpm, dir_str
    );
}

static void cmd_iabc(void)
{
    cli_printf("  Ia=%.4f  Ib=%.4f  Ic=%.4f  A\r\n",
               foc_get_ia(), foc_get_ib(), foc_get_ic());
}

static void cmd_idq(void)
{
    cli_printf("  Id=%.4f  Iq=%.4f  A\r\n",
               foc_get_id(), foc_get_iq());
}

#if FOC_DEBUG_ENABLE
static void cmd_vout(void)
{
    cli_printf("  Vd=%.4f  Vq=%.4f  V\r\n",
               foc_get_vd_cmd(), foc_get_vq_cmd());
}
#endif

static void cmd_adc(void)
{
    cli_printf(
        "  A1.JDR1=%u  A2.JDR1=%u  (raw: phase A, phase B)\r\n"
        "  off_a=%.1f  off_b=%.1f  off_c=%.1f  (counts)\r\n",
        (unsigned)ADC1->JDR1,
        (unsigned)ADC2->JDR1,
        foc_get_offset_a(), foc_get_offset_b(), foc_get_offset_c()
    );
}

static void cmd_hall(const char *arg)
{
#if FOC_DEBUG_ENABLE
    if (arg && !strcmp(arg, "rst")) {
        hall_dbg_reset();
        cli_print("  hall debug stats reset (innovation + glitch counters)\r\n");
        return;
    }
#else
    (void)arg;
#endif
    float   theta = hall_get_theta_e();
    float   omega = hall_get_omega_e();
    float   rpm   = fabsf(omega) * (60.0f / M_TWOPI_F);
    int8_t  dir   = hall_get_dir();
    const char *dir_str = (dir > 0) ? "CCW" : (dir < 0) ? "CW" : "---";
#if FOC_DEBUG_ENABLE
    const float r2d = 180.0f / M_PI_F;
#endif
    cli_printf(
#if FOC_DEBUG_ENABLE
        "  state=0x%X  sector=%u  theta_e=%.4f rad  speed=%.1f RPM  dir=%s\r\n"
        "  innov: last=%+.1f  max=%+.1f deg  (edges=%lu)  min_period=%u  glitches=%lu\r\n",
#else
        "  state=0x%X  sector=%u  theta_e=%.4f rad  speed=%.1f RPM  dir=%s\r\n",
#endif
        (unsigned)hall_get_state(),
        (unsigned)hall_get_sector(),
        theta, rpm, dir_str
#if FOC_DEBUG_ENABLE
        ,
        hall_get_innov_last() * r2d,
        hall_get_innov_max()  * r2d,
        (unsigned long)hall_get_innov_count(),
        (unsigned)hall_get_min_period(),
        (unsigned long)hall_get_glitch_edges()
#endif
    );
#if FOC_DEBUG_ENABLE
    cli_print("  innov by sector:");
    for (uint8_t st = 1u; st <= 6u; st++)
        cli_printf("  %u:%+.1f", st, hall_get_innov_sector(st) * r2d);
    cli_print(" deg\r\n");
#endif
}

static void cmd_enc(const char *arg)
{
    if (arg && !strcmp(arg, "rst")) {
        if (arm_pos_is_active()) {
            cli_print("  error: arm position PID active (apos off first)\r\n");
            return;
        }
        encoder_reset_turns();
        cli_print("  encoder turn counter zeroed\r\n");
        return;
    }
    static const char *var[] = { "?", "A1", "A2", "?" };
    cli_printf("  enc=%s (TMAG5273%s)  angle=%.2f deg  turns=%ld  total=%.1f deg\r\n"
               "  speed=%.1f deg/s  mag=%u  errs=%lu\r\n",
               encoder_is_ok() ? "OK" : "FAIL",
               var[encoder_get_variant() & 0x3],
               encoder_get_angle_deg(), (long)encoder_get_turns(),
               encoder_get_total_deg(), encoder_get_speed_dps(),
               (unsigned)encoder_get_magnitude(),
               (unsigned long)encoder_get_err_count());
}

static void cmd_apos(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  arm=%s  target=%.2f deg  pos=%.2f deg  err=%.2f deg  out=%.1f RPM\r\n"
                   "  gains: kp=%.4f ki=%.4f kd=%.4f  limits=[%.1f, %.1f] tol=%.2f deg\r\n"
                   "  enc=%s\r\n",
                   arm_pos_status_str(arm_pos_get_status()),
                   arm_pos_get_target_deg(), encoder_get_total_deg(),
                   arm_pos_get_error_deg(), arm_pos_get_output_rpm(),
                   arm_pos_get_kp(), arm_pos_get_ki(), arm_pos_get_kd(),
                   (double)ARM_POS_MIN_DEG, (double)ARM_POS_MAX_DEG,
                   (double)ARM_POS_TOL_DEG,
                   encoder_is_ok() ? "OK" : "FAIL");
        return;
    }
    if (!strcmp(arg, "off")) {
        arm_pos_stop();
        cli_print("  arm position PID off\r\n");
        return;
    }
    float target = strtof(arg, NULL);
    if (!arm_pos_set_target_deg(target)) {
        cli_printf("  error: arm target rejected (%s)\r\n",
                   arm_pos_status_str(arm_pos_get_status()));
        return;
    }
    cli_printf("  arm target set to %.2f deg  (pos %.2f deg)\r\n",
               arm_pos_get_target_deg(), encoder_get_total_deg());
}

static void cmd_apk(const char *arg)
{
    if (arg && *arg) arm_pos_set_kp(strtof(arg, NULL));
    cli_printf("  apk = %.4f\r\n", arm_pos_get_kp());
}

static void cmd_aki(const char *arg)
{
    if (arg && *arg) arm_pos_set_ki(strtof(arg, NULL));
    cli_printf("  aki = %.4f\r\n", arm_pos_get_ki());
}

static void cmd_akd(const char *arg)
{
    if (arg && *arg) arm_pos_set_kd(strtof(arg, NULL));
    cli_printf("  akd = %.4f\r\n", arm_pos_get_kd());
}

static void cmd_iq(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  iq_ref = %.4f A  (target %.4f)\r\n",
                   foc_get_iq_ref(), foc_get_iq_target());
        return;
    }
    arm_pos_stop();
    can_motion_release();
    float val = strtof(arg, NULL);
    foc_set_iq_ref(val);
    cli_printf("  iq_ref target set to %.4f A (slew-limited)\r\n",
               foc_get_iq_target());
}

static void cmd_spd(const char *arg)
{
    if (!arg || *arg == '\0') {
        float meas = fabsf(hall_get_omega_e()) * (60.0f / M_TWOPI_F);
        cli_printf("  spd_ref = %.1f RPM  meas = %.1f RPM  mode = %s\r\n",
                   OMEGA_E_TO_RPM(foc_get_speed_ref()), meas,
                   foc_is_speed_mode() ? "speed" : "manual");
        return;
    }
    if (!strcmp(arg, "off")) {
        arm_pos_stop();
        can_motion_release();
        foc_speed_disable();
        foc_set_iq_ref(0.0f);
        cli_print("  speed mode off (manual, iq=0)\r\n");
        return;
    }
    if (fault_is_active()) {
        cli_print("  error: fault latched (clrf first)\r\n");
        return;
    }
    arm_pos_stop();
    can_motion_release();
    float rpm = strtof(arg, NULL);
    foc_set_speed_ref(RPM_TO_OMEGA_E(rpm));
    cli_printf("  spd_ref set to %.1f RPM%s\r\n", rpm,
               foc_is_enabled() ? "" : "  (enable FOC with 'en')");
}

#if FOC_DEBUG_ENABLE
static void cmd_kp(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  kp = %.4f\r\n", foc_get_kp());
        return;
    }
    float val = strtof(arg, NULL);
    foc_set_kp(val);
    cli_printf("  kp set to %.4f\r\n", foc_get_kp());
}

static void cmd_ki(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  ki = %.4f\r\n", foc_get_ki());
        return;
    }
    float val = strtof(arg, NULL);
    foc_set_ki(val);
    cli_printf("  ki set to %.4f\r\n", foc_get_ki());
}

static void cmd_skp(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  skp = %.5f\r\n", foc_get_speed_kp());
        return;
    }
    float val = strtof(arg, NULL);
    foc_set_speed_kp(val);
    cli_printf("  skp set to %.5f\r\n", foc_get_speed_kp());
}

static void cmd_ski(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  ski = %.5f\r\n", foc_get_speed_ki());
        return;
    }
    float val = strtof(arg, NULL);
    foc_set_speed_ki(val);
    cli_printf("  ski set to %.5f\r\n", foc_get_speed_ki());
}

static void cmd_drv(void)
{
    char buf[OUT_BUF_SIZE];
    cli_print("  DRV8316 registers:\r\n");
    drv8316_dump_regs(buf, sizeof buf);
    cli_print(buf);
}
#endif

static void cmd_fault(void)
{
    char buf[OUT_BUF_SIZE];
    fault_describe(buf, sizeof buf);
    cli_print(buf);
}

static void cmd_clrf(void)
{
    fault_clear();
    cli_print("  faults cleared\r\n");
}

static void cmd_cal(void)
{
    if (arm_pos_is_active()) {
        cli_print("  error: arm position PID active (apos off first)\r\n");
        return;
    }
    if (foc_is_enabled()) {
        cli_print("  error: disable FOC first (dis)\r\n");
        return;
    }
#if FOC_DEBUG_ENABLE
    if (foc_is_rotor_voltage_mode()) {
        cli_print("  error: rotor voltage mode active (rv off)\r\n");
        return;
    }
#endif
    if (!foc_recalibrate()) {
        cli_print("  error: calibration timed out (no ADC conversions — check TIM1/ADC)\r\n");
        return;
    }
    cli_printf("  offsets: a=%.1f b=%.1f c=%.1f\r\n",
               foc_get_offset_a(), foc_get_offset_b(), foc_get_offset_c());
}

#if FOC_DEBUG_ENABLE
static void cmd_ctune(void)
{
    if (fault_is_active()) {
        cli_print("  error: fault latched (clrf first)\r\n");
        return;
    }
    if (foc_is_enabled()) {
        cli_print("  error: disable FOC first (dis)\r\n");
        return;
    }
    if (foc_is_rotor_voltage_mode()) {
        cli_print("  error: rotor voltage mode active (rv off)\r\n");
        return;
    }
    char buf[256];
    cli_print("  Current-loop autotune: rotor aligns + brief current pulses...\r\n");
    foc_tune_current(buf, sizeof buf);
    cli_print(buf);
}

static void cmd_stune(const char *arg)
{
    if (fault_is_active()) {
        cli_print("  error: fault latched (clrf first)\r\n");
        return;
    }
    if (foc_is_enabled()) {
        cli_print("  error: disable FOC first (dis)\r\n");
        return;
    }
    if (foc_is_rotor_voltage_mode()) {
        cli_print("  error: rotor voltage mode active (rv off)\r\n");
        return;
    }
    float rpm = (arg && *arg) ? strtof(arg, NULL) : TUNE_SPEED_RPM;
    char buf[256];
    cli_printf("  Speed-loop relay autotune @ %.0f RPM: motor will oscillate...\r\n", rpm);
    foc_tune_speed(RPM_TO_OMEGA_E(rpm), buf, sizeof buf);
    cli_print(buf);
}
#endif

static void cmd_hcal(void)
{
    if (arm_pos_is_active()) {
        cli_print("  error: arm position PID active (apos off first)\r\n");
        return;
    }
    if (fault_is_active()) {
        cli_print("  error: fault latched (clrf first)\r\n");
        return;
    }
    if (foc_is_enabled()) {
        cli_print("  error: disable FOC first (dis)\r\n");
        return;
    }
#if FOC_DEBUG_ENABLE
    if (foc_is_rotor_voltage_mode()) {
        cli_print("  error: rotor voltage mode active (rv off)\r\n");
        return;
    }
#endif
    char buf[640];
    cli_print("  Hall calibration: driving open-loop ~5 s (fwd+rev), keep rotor clear...\r\n");
    hall_calibrate(buf, sizeof buf);
    cli_print(buf);
}

#if FOC_DEBUG_ENABLE
static void cmd_hchk(void)
{
    if (fault_is_active()) {
        cli_print("  error: fault latched (clrf first)\r\n");
        return;
    }
    if (foc_is_enabled()) {
        cli_print("  error: disable FOC first (dis)\r\n");
        return;
    }
    if (foc_is_rotor_voltage_mode()) {
        cli_print("  error: rotor voltage mode active (rv off)\r\n");
        return;
    }
    char buf[640];
    cli_print("  Hall check: driving open-loop ~8 s (fwd+rev), scoring theta_hat...\r\n");
    hall_check(buf, sizeof buf);
    cli_print(buf);
}

static void cmd_bb(const char *arg)
{
#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE
    if (!arg || *arg == '\0') {
        static const char *names[] = { "idle", "armed", "triggered", "frozen" };
        uint32_t st = bbox_state();
        cli_printf("  black box: %s  samples=%lu/%u  (dump: tools/bbox/bbox_dump.py)\r\n",
                   (st < 4u) ? names[st] : "?", (unsigned long)bbox_count(),
                   (unsigned)BBOX_LEN);
        return;
    }
    if (!strcmp(arg, "arm"))       { bbox_arm();     cli_print("  black box armed\r\n"); }
    else if (!strcmp(arg, "trig")) { bbox_trigger(); cli_print("  black box trigger requested\r\n"); }
    else cli_print("  usage: bb [arm|trig]\r\n");
#else
    (void)arg;
    cli_print("  black box disabled (set FOC_BBOX_ENABLE=1)\r\n");
#endif
}
#endif

static void cmd_blk(const char *arg)
{
    if (!arg || *arg == '\0' || !strcmp(arg, "off")) {
        arm_pos_stop();
        bool was_on = foc_is_block_mode();
        if (was_on) foc_block_disable();
        cli_print(was_on ? "  block mode off\r\n" : "  block mode already off\r\n");
        return;
    }
    if (fault_is_active()) {
        cli_print("  error: fault latched (clrf first)\r\n");
        return;
    }
    if (foc_is_enabled()) {
        cli_print("  error: disable FOC first (dis)\r\n");
        return;
    }
#if FOC_DEBUG_ENABLE
    if (foc_is_rotor_voltage_mode()) {
        cli_print("  error: rotor voltage mode active (rv off)\r\n");
        return;
    }
#endif
    arm_pos_stop();
    float duty = strtof(arg, NULL);
    if (foc_is_block_mode()) {
        foc_block_set_duty(duty);
        cli_printf("  block duty set to %.3f %s\r\n",
                   (double)fabsf(duty), (duty >= 0.0f) ? "CCW" : "CW");
    } else {
        foc_block_enable(duty);
        float omega = fabsf(hall_get_omega_e());
        float rpm   = omega * (60.0f / M_TWOPI_F);
        cli_printf("  block mode on, duty=%.3f %s  (stale-speed=%.0f RPM)\r\n",
                   (double)fabsf(duty), (duty >= 0.0f) ? "CCW" : "CW", (double)rpm);
    }
}

#if FOC_DEBUG_ENABLE
static void cmd_hoff(const char *arg)
{
    if (!arg || *arg == '\0') {
        cli_printf("  hoff = %.4f rad\r\n", hall_get_angle_offset());
        return;
    }
    float val = strtof(arg, NULL);
    hall_set_angle_offset(val);
    cli_printf("  hoff set to %.4f rad\r\n", hall_get_angle_offset());
}

/* hpll [<kp> <ki>] — get/set Hall PLL gains (live, RAM only). */
static void cmd_hpll(const char *arg)
{
    if (arg && *arg) {
        char *end;
        float kp = strtof(arg, &end);
        float ki = hall_get_pll_ki();
        if (end) {
            while (*end == ' ' || *end == '\t') end++;
            if (*end) ki = strtof(end, NULL);
        }
        hall_set_pll(kp, ki);
    }
    cli_printf("  pll: kp=%.3f  ki=%.3f\r\n",
               (double)hall_get_pll_kp(), (double)hall_get_pll_ki());
}

/* fv <vd> <vq>  — open-loop voltage vector test (polarity check).
 * fv off        — exit force mode. */
static void cmd_fv(const char *arg)
{
    if (!arg || !strcmp(arg, "off")) {
        if (foc_is_enabled()) { cli_print("  fv: FOC is enabled — disable first (dis)\r\n"); return; }
        foc_force_end();
        cli_print("  force mode off\r\n");
        return;
    }
    if (foc_is_enabled()) { cli_print("  fv: disable FOC first (dis)\r\n"); return; }
    if (foc_is_rotor_voltage_mode()) { cli_print("  fv: rotor voltage mode active (rv off)\r\n"); return; }
    if (fault_is_active()) { cli_print("  fv: fault latched (clrf first)\r\n"); return; }

    char *end;
    float vd = strtof(arg, &end);
    float vq = 0.0f;
    if (end) {
        while (*end == ' ' || *end == '\t') end++;
        if (*end) vq = strtof(end, NULL);
    }

    /* foc_force_begin sets force_vd=vmag, force_vq=0, angle=0, MOE on.
     * If we're already in force mode, just update the voltages in place. */
    if (!foc_is_enabled()) {
        /* Re-entering force mode restarts the angle at 0; that's fine for
         * a stationary test where θe doesn't matter (just measuring polarity). */
        if (!foc_force_begin(vd)) {
            cli_print("  fv: conflicting drive mode active\r\n");
            return;
        }
        foc_force_set_vq(vq);
    }
    cli_printf("  force: vd=%.3f V  vq=%.3f V  theta=0\r\n", (double)vd, (double)vq);
}

/* rv <vd> <vq> — fixed voltage in the live rotor/Hall theta_e frame.
 * rv off       — exit rotor-frame voltage mode. */
static void cmd_rv(const char *arg)
{
    if (!arg || !strcmp(arg, "off")) {
        foc_rotor_voltage_end();
        cli_print("  rotor voltage mode off\r\n");
        return;
    }
    if (foc_is_enabled()) { cli_print("  rv: disable FOC first (dis)\r\n"); return; }
    if (fault_is_active()) { cli_print("  rv: fault latched (clrf first)\r\n"); return; }

    char *end;
    float vd = strtof(arg, &end);
    float vq = 0.0f;
    if (end) {
        while (*end == ' ' || *end == '\t') end++;
        if (*end) vq = strtof(end, NULL);
    }

    if (foc_is_rotor_voltage_mode()) {
        foc_rotor_voltage_set(vd, vq);
    } else if (!foc_rotor_voltage_begin(vd, vq)) {
        cli_print("  rv: conflicting drive mode active\r\n");
        return;
    }
    cli_printf("  rotor voltage: vd=%.3f V  vq=%.3f V  theta=hall\r\n",
               (double)vd, (double)vq);
}
#endif

static void cli_exec(char *line)
{
    /* Tokenise: first word = command, rest = argument */
    char *cmd = strtok(line, " \t");
    if (!cmd) return;
    char *arg = strtok(NULL, "");
    if (arg) {
        /* strip leading spaces */
        while (*arg == ' ' || *arg == '\t') arg++;
    }

    if      (!strcmp(cmd, "help") || !strcmp(cmd, "?")) cmd_help();
    else if (!strcmp(cmd, "status"))                    cmd_status();
    else if (!strcmp(cmd, "ia"))  cli_printf("  Ia=%.4f A\r\n", foc_get_ia());
    else if (!strcmp(cmd, "ib"))  cli_printf("  Ib=%.4f A\r\n", foc_get_ib());
    else if (!strcmp(cmd, "ic"))  cli_printf("  Ic=%.4f A\r\n", foc_get_ic());
    else if (!strcmp(cmd, "iabc"))                      cmd_iabc();
    else if (!strcmp(cmd, "idq"))                       cmd_idq();
#if FOC_DEBUG_ENABLE
    else if (!strcmp(cmd, "vout"))                      cmd_vout();
#endif
    else if (!strcmp(cmd, "adc"))                       cmd_adc();
    else if (!strcmp(cmd, "hall"))                      cmd_hall(arg);
    else if (!strcmp(cmd, "enc"))                       cmd_enc(arg);
    else if (!strcmp(cmd, "apos"))                      cmd_apos(arg);
    else if (!strcmp(cmd, "apk"))                       cmd_apk(arg);
    else if (!strcmp(cmd, "aki"))                       cmd_aki(arg);
    else if (!strcmp(cmd, "akd"))                       cmd_akd(arg);
    else if (!strcmp(cmd, "en")) {
        if (fault_is_active()) cli_print("  error: fault latched (clrf first)\r\n");
        else {
            foc_enable();
            cli_print(foc_is_enabled() ? "  FOC enabled\r\n"
                                      : "  error: direct drive mode active\r\n");
        }
    }
    else if (!strcmp(cmd, "dis")) { arm_pos_stop(); foc_disable(); cli_print("  FOC disabled\r\n"); }
    else if (!strcmp(cmd, "iq"))                        cmd_iq(arg);
    else if (!strcmp(cmd, "spd"))                       cmd_spd(arg);
#if FOC_DEBUG_ENABLE
    else if (!strcmp(cmd, "kp"))                        cmd_kp(arg);
    else if (!strcmp(cmd, "ki"))                        cmd_ki(arg);
    else if (!strcmp(cmd, "skp"))                       cmd_skp(arg);
    else if (!strcmp(cmd, "ski"))                       cmd_ski(arg);
    else if (!strcmp(cmd, "ctune"))                     cmd_ctune();
    else if (!strcmp(cmd, "stune"))                     cmd_stune(arg);
    else if (!strcmp(cmd, "drv"))                       cmd_drv();
#endif
    else if (!strcmp(cmd, "fault"))                     cmd_fault();
    else if (!strcmp(cmd, "clrf"))                      cmd_clrf();
    else if (!strcmp(cmd, "cal"))                       cmd_cal();
    else if (!strcmp(cmd, "hcal"))                      cmd_hcal();
#if FOC_DEBUG_ENABLE
    else if (!strcmp(cmd, "hchk"))                      cmd_hchk();
    else if (!strcmp(cmd, "bb"))                        cmd_bb(arg);
#endif
    else if (!strcmp(cmd, "blk"))                       cmd_blk(arg);
#if FOC_DEBUG_ENABLE
    else if (!strcmp(cmd, "hoff"))                      cmd_hoff(arg);
    else if (!strcmp(cmd, "hpll"))                      cmd_hpll(arg);
    else if (!strcmp(cmd, "fv"))                        cmd_fv(arg);
    else if (!strcmp(cmd, "rv"))                        cmd_rv(arg);
#endif
    else cli_printf("  unknown: '%s'  (try help)\r\n", cmd);
}

/* ------------------------------------------------------------------ */
/* Interrupt RX callback — one byte per interrupt. Accumulates into    */
/* s_accum_buf and dispatches only when CR or LF is received. Robust   */
/* against fast multi-char input (no DMA re-arm window to lose bytes). */
/* ------------------------------------------------------------------ */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    char c = (char)s_rx_byte;
    if (c == '\r' || c == '\n') {
        if (!s_cmd_ready && s_accum_len > 0) {
            memcpy(s_cmd_buf, s_accum_buf, s_accum_len);
            s_cmd_buf[s_accum_len] = '\0';
            s_accum_len = 0;
            s_cmd_ready = 1;
        }
    } else if ((c == '\b' || c == 0x7F) && s_accum_len > 0) {
        s_accum_len--;
    } else if (s_accum_len < CMD_BUF_SIZE - 1) {
        s_accum_buf[s_accum_len++] = c;
    }

    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

/* ------------------------------------------------------------------ */
/* Error callback — an uncleared ORE/FE/NE/PE wedges the RX path, so a */
/* single error after the first byte would silently kill the CLI. Clear */
/* the flags and re-arm reception so input keeps flowing.              */
/* ------------------------------------------------------------------ */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_FEF
                               | UART_CLEAR_NEF  | UART_CLEAR_PEF);

    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void cli_init(void)
{
    s_cmd_ready = 0;
    s_accum_len = 0;
    HAL_UART_Receive_IT(&huart3, &s_rx_byte, 1);
    cli_print("\r\nBLDC FOC ready. Type 'help' for commands.\r\n> ");
}

void cli_process(void)
{
    if (!s_cmd_ready) return;

    /* Make a local copy so the ISR can begin filling the next command */
    char line[CMD_BUF_SIZE];
    memcpy(line, s_cmd_buf, CMD_BUF_SIZE);
    s_cmd_ready = 0;

    cli_print(line);
    cli_print("\r\n");
    cli_exec(line);
    cli_print("> ");
}
