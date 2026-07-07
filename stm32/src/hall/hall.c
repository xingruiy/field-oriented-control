#include "hall/hall.h"
#include "control/foc.h"
#include "control/foc_math.h"
#include "fault/fault.h"
#include "common/settings.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

/* Per-state electrical-angle table, 1 pole pair. Defaults come from
 * HALL_SECTOR_ANGLE_INIT (settings.h); `hcal` overwrites this live RAM copy.
 * Index = (GPIOD->IDR >> 12) & 0x7 = bit0:HA(PD12) bit1:HB(PD13) bit2:HC(PD14).
 *
 * The CCW/CW labels below are the electrical-angle direction (angle increasing
 * = "CCW"). Physical rotation relates to it by HALL_PHYS_DIR_SIGN — on this
 * rotor the shaft turns CW as θe increases, so the reported direction/speed
 * (hall_get_dir / hall_get_omega_e) flip the sign to match reality.
 *
 * θe-increasing sequence: 3→2→6→4→5→1→3
 * θe-decreasing sequence: 3→1→5→4→6→2→3
 */
static float s_sector_angle[8] = HALL_SECTOR_ANGLE_INIT;

/* Expected next Hall state for each rotation direction.
 * Indexed by current Hall state [0..7]; 0 = invalid entry. RAM so that
 * hall_calibrate() can rebuild the order for non-standard sensor wiring. */
static uint8_t s_next_ccw[8] = { 0, 3, 6, 2, 5, 1, 4, 0 };
static uint8_t s_next_cw[8]  = { 0, 5, 3, 1, 6, 4, 2, 0 };

/* TIM4 runs in Hall-sensor interface mode (slave RESET): the counter resets to 0
 * on every Hall edge, CCR1 captures the inter-edge period, and CNT counts ticks
 * since the last edge. */
#define HALL_TICK_S         1.0e-5f   /* TIM4 tick period @ 100 kHz (must match
                                       * the CubeMX TIM4 prescaler: 200 MHz kernel
                                       * / (PSC+1=2000) = 100 kHz. See README §5) */
#define HALL_MIN_PERIOD     5u        /* ticks (50 µs) — reject contact bounce  */
#define HALL_STALE_MS       100u

static volatile float    s_angle_offset = HALL_ANGLE_OFFSET_RAD;
static volatile uint8_t  s_state;
static volatile uint8_t  s_sector;
static volatile float    s_omega_e;      /* signed: +CCW, −CW, rad/s */
static volatile int8_t   s_dir;          /* +1 CCW, −1 CW, 0 unknown */
static volatile uint32_t s_last_edge_ms;

/* Hall angle/speed observer: θ̂ integrated at the current-loop rate. On every
 * Hall edge the boundary angle softly corrects θ̂ (KP, no hard snap → continuous
 * angle) and the measured edge speed is low-passed into ω̂ (KI), so ω̂ tracks
 * true speed with no lag while shedding the per-edge steps. Edge data is handed
 * off from the TIM4 ISR via a sequence counter so all updates happen in one
 * context. */
static volatile float    s_theta_hat;    /* observed θe (rad, θe-frame)         */
static volatile float    s_omega_hat;    /* observed dθe/dt (rad/s, θe-frame)   */
static volatile float    s_edge_theta;   /* boundary angle of the last edge     */
static volatile float    s_edge_omega;   /* measured speed at the last edge     */
static volatile float    s_boundary[8][8]; /* boundary angle between sectors [from][to] */
static volatile uint32_t s_edge_seq;     /* incremented on every valid edge     */
static uint32_t          s_obs_seq;      /* last edge consumed by the observer  */

/* Edge-glitch diagnostics: shortest inter-edge period seen and a count of edges
 * arriving suspiciously close to HALL_MIN_PERIOD. A too-short period snaps ω̂ to
 * an enormous value in hall_observer_update() → angle runaway. */
#if FOC_DEBUG_ENABLE
#define HALL_GLITCH_PERIOD  (HALL_MIN_PERIOD + 5u)   /* ≤100 µs ≈ >200k RPM elec */
static volatile uint16_t s_min_period = 0xFFFFu;
static volatile uint32_t s_glitch_edges;

/* Per-edge innovation: wrap_pi(θ̂ − boundary) just before the hard snap — the
 * θ̂ interpolation error accumulated since the previous edge. Small at a clean
 * steady spin; one huge value = glitch ω̂ snap; a repeating per-sector pattern
 * = the 60°/edge speed bias on unequal sectors. Keyed by the sector entered. */
static volatile float    s_innov_last, s_innov_max;   /* rad; max keeps sign  */
static volatile float    s_innov_by_sector[8];
static volatile uint32_t s_innov_count;
#endif
static bool              s_was_stale = true;          /* skip 1st edge after hold */
static bool              s_need_speed_seed;           /* 2nd edge has valid period */

/* PLL gains (per-edge PI). Live-tunable via `hpll`; boot defaults in settings.h. */
static volatile float    s_pll_kp = HALL_PLL_KP;
static volatile float    s_pll_ki = HALL_PLL_KI;

static inline float wrap_pi(float e)
{
    while (e >  M_PI_F) e -= M_TWOPI_F;
    while (e < -M_PI_F) e += M_TWOPI_F;
    return e;
}

/* Geometric boundary (circular midpoint) between two sector-center angles —
 * the angle at which the Hall code flips. Uses the actual centers so unequal
 * sectors are respected. Returns a value in [0, 2π). */
static inline float circ_midpoint(float a, float b)
{
    float m = a + 0.5f * wrap_pi(b - a);
    m = fmodf(m, M_TWOPI_F);
    if (m < 0.0f) m += M_TWOPI_F;
    return m;
}

static inline int hall_is_stale(void);

void hall_init(void)
{
    uint8_t state   = (uint8_t)((GPIOD->IDR >> 12) & 0x7u);
    s_state         = state;
    s_sector        = state;
    s_last_edge_ms  = HAL_GetTick();
    s_omega_e       = 0.0f;
    s_dir           = 0;

    s_theta_hat     = s_sector_angle[state] + s_angle_offset;
    s_omega_hat     = 0.0f;
    s_edge_theta    = s_theta_hat;
    s_edge_omega    = 0.0f;
    s_edge_seq      = 0u;
    s_obs_seq       = 0u;

    /* Bootstrap boundary table from hardcoded sector centres (replaced by hcal) */
    for (uint8_t st = 1u; st <= 6u; st++) {
        uint8_t nxt = s_next_ccw[st];
        s_boundary[st][nxt] = circ_midpoint(s_sector_angle[st], s_sector_angle[nxt]);
        s_boundary[nxt][st] = s_boundary[st][nxt];
    }
}

void hall_update(void)
{
    /* Reset mode: CCR1 already holds the period (ticks) since the previous edge. */
    uint16_t period = (uint16_t)TIM4->CCR1;
    uint8_t  state  = (uint8_t)((GPIOD->IDR >> 12) & 0x7u);
    bool     was_stale = hall_is_stale();

    if (!was_stale && period < HALL_MIN_PERIOD) return;

#if FOC_DEBUG_ENABLE
    /* Glitch diagnostics: this period is about to set ω̂ via the hard snap. */
    if (!was_stale) {
        if (period < s_min_period) s_min_period = period;
        if (period < HALL_GLITCH_PERIOD) s_glitch_edges++;
    }
#endif

    /* Electrical-angle direction from the state transition: +1 when θe is
     * increasing (transition follows the CCW-electrical table), −1 decreasing.
     * This sign feeds the edge speed handed to the observer, so it must track
     * dθe/dt — not physical rotation. */
    int8_t edir = 0;
    if      (state == s_next_ccw[s_state]) edir = +1;
    else if (state == s_next_cw[s_state])  edir = -1;

    /* Publish atomically: the ADC current loop (priority 1) can preempt this
     * handler (priority 5) and read these fields via hall_get_theta_e(). Guard
     * the stores so it never sees a new sector paired with a stale ω.          */
    uint32_t now = HAL_GetTick();
    float omega = s_omega_e;
    if (edir != 0 && !was_stale) {
        /* ω_e [rad/s] = 2π / (6 edges/rev × period[s]), signed by θe direction */
        omega = (float)edir * M_TWOPI_F / (6.0f * (float)period * HALL_TICK_S);
    } else if (edir != 0) {
        omega = 0.0f;
    }

    uint8_t old_sector = s_sector;

    __disable_irq();
    s_state        = state;
    s_sector       = state;
    s_last_edge_ms = now;
    if (edir != 0) {
        /* s_dir is the physical rotation for display; map electrical→physical. */
        s_dir     = (int8_t)(HALL_PHYS_DIR_SIGN * edir);
        s_omega_e = omega;   /* θe-frame, raw edge speed */
        /* Hand off to the observer: the boundary just crossed (between the old
         * and new sector centers), the measured speed, and the edge interval. */
        s_edge_theta = s_boundary[old_sector][state];
        s_edge_omega = omega;
        s_edge_seq++;
    }
    __enable_irq();
}

static inline int hall_is_stale(void)
{
    return (HAL_GetTick() - s_last_edge_ms) > HALL_STALE_MS;
}

void hall_observer_update(float dt)
{
    /* Standstill: hold θ̂ at the sector centre and zero ω̂. Resync the edge
     * counter so the first edge after motion resumes with a clean snap. */
    if (hall_is_stale()) {
        s_theta_hat = s_sector_angle[s_sector] + s_angle_offset;
        s_omega_hat = 0.0f;
        s_obs_seq   = s_edge_seq;
        s_was_stale = true;
        s_need_speed_seed = false;
        return;
    }

    /* PLL: on every new Hall edge the boundary angle is a phase measurement.
     * e = wrap_pi(boundary − θ̂) is the innovation; correct θ̂ and ω̂ by KP·e / KI·e
     * instead of snapping, so both stay continuous and ω̂ smoothly tracks. */
    if (s_edge_seq != s_obs_seq) {
        s_obs_seq = s_edge_seq;
        float ref = s_edge_theta + s_angle_offset;
        if (s_was_stale) {
            /* First edge after a stale hold: the PLL integrator is empty and θ̂
             * was parked at the sector centre, so seed directly from the edge
             * rather than PI-correcting a half-sector error. The captured
             * period on this edge is stale/overflowed, so do not trust speed
             * until the next edge supplies a real inter-edge interval. */
            s_theta_hat = ref;
            s_omega_hat = 0.0f;
            s_was_stale = false;
            s_need_speed_seed = true;
        } else if (s_need_speed_seed) {
            /* Second edge after a stale hold: now the period is a real sector
             * transit time. Seed both θ̂ and ω̂ directly, then let the PLL take
             * over on subsequent edges. */
            s_theta_hat = ref;
            s_omega_hat = s_edge_omega;
            s_need_speed_seed = false;
        } else {
            float e = wrap_pi(ref - s_theta_hat);
#if FOC_DEBUG_ENABLE
            /* Innovation stat keeps its existing sign (θ̂ − ref = −e). */
            s_innov_last = -e;
            if (fabsf(e) > fabsf(s_innov_max)) s_innov_max = -e;
            s_innov_by_sector[s_sector] = -e;
            s_innov_count++;
#endif
            /* θ̂: soft phase correction (KP) → continuous angle, no snap.
             * ω̂: low-pass the directly-measured edge speed (KI = filter coeff
             * in (0,1]). Using the measurement — not a pure integral of e —
             * lets ω̂ track acceleration with no lag, so the outer speed loop
             * can lock instead of running away. KI→1 = old hard-snap speed. */
            s_theta_hat += s_pll_kp * e;
            s_omega_hat += s_pll_ki * (s_edge_omega - s_omega_hat);
        }
    }

    s_theta_hat += s_omega_hat * dt;
    s_theta_hat  = fmodf(s_theta_hat, M_TWOPI_F);
    if (s_theta_hat < 0.0f) s_theta_hat += M_TWOPI_F;
}

float hall_get_theta_e(void)
{
    return s_theta_hat;
}

/* Physical-frame signed speed (+CCW, −CW). Internal s_omega_hat is θe-frame; the
 * sign is mapped to physical rotation here for callers/CLI. */
float   hall_get_omega_e(void) { return hall_is_stale() ? 0.0f : (HALL_PHYS_DIR_SIGN * s_omega_hat); }
int8_t  hall_get_dir(void)     { return hall_is_stale() ? 0    : s_dir;     }
uint8_t hall_get_state(void)   { return s_state; }
uint8_t hall_get_sector(void)  { return s_sector; }

#if FOC_DEBUG_ENABLE
void  hall_set_angle_offset(float rad) { s_angle_offset = rad; }
float hall_get_angle_offset(void)      { return s_angle_offset; }

void  hall_set_pll(float kp, float ki) { s_pll_kp = kp; s_pll_ki = ki; }
float hall_get_pll_kp(void)            { return s_pll_kp; }
float hall_get_pll_ki(void)            { return s_pll_ki; }

/* Debug accessors — raw internals, no stale-gate/sign-mapping (see hall.h). */
float    hall_get_omega_hat(void)    { return s_omega_hat; }
uint16_t hall_get_min_period(void)   { return s_min_period; }
uint32_t hall_get_glitch_edges(void) { return s_glitch_edges; }
float    hall_get_innov_last(void)   { return s_innov_last; }
float    hall_get_innov_max(void)    { return s_innov_max; }
float    hall_get_innov_sector(uint8_t st) { return s_innov_by_sector[st & 7u]; }
uint32_t hall_get_innov_count(void)  { return s_innov_count; }

void hall_dbg_reset(void)
{
    s_min_period = 0xFFFFu;
    s_glitch_edges = 0u;
    s_innov_last = s_innov_max = 0.0f;
    s_innov_count = 0u;
    for (uint8_t st = 0u; st < 8u; st++) s_innov_by_sector[st] = 0.0f;
}
#endif

/* ------------------------------------------------------------------ */
/* Open-loop Hall angle calibration (`hcal` CLI command)               */
/* ------------------------------------------------------------------ */

#define HCAL_ALIGN_CURRENT_A   0.4f    /* hold current during alignment    */
#define HCAL_N_STEPS           180u    /* angle samples per electrical rev  */
#define HCAL_STEP_MS           20u     /* dwell at each angle               */
#define HCAL_ALIGN_MS          300u    /* initial settle before each sweep  */

typedef struct {
    float    sum_sin[8], sum_cos[8];   /* circular-mean accumulators per state */
    uint16_t count[8];
    uint8_t  next_ccw[8], next_cw[8];  /* observed transition order            */
    float    bnd_fwd[8][8], bnd_rev[8][8]; /* transition angle for each pair   */
} HcalAccum;

/* Returns NULL on success, or an abort-reason string. */
static const char *hcal_sweep(int dir, HcalAccum *acc)
{
    /* Settle at this pass's start angle before sampling */
    foc_force_set_angle(dir > 0 ? 0.0f : M_TWOPI_F);
    HAL_Delay(HCAL_ALIGN_MS);

    uint8_t prev = (uint8_t)((GPIOD->IDR >> 12) & 0x7u);

    for (uint32_t i = 0; i < HCAL_N_STEPS; i++) {
        float frac  = (float)i / (float)HCAL_N_STEPS;
        float theta = (dir > 0) ? (M_TWOPI_F * frac)
                                : (M_TWOPI_F * (1.0f - frac));
        foc_force_set_angle(theta);
        HAL_Delay(HCAL_STEP_MS);

        if (fault_is_active()) return "fault latched";
        if (fabsf(foc_get_ia()) > MOTOR_OC_TRIP_A ||
            fabsf(foc_get_ib()) > MOTOR_OC_TRIP_A ||
            fabsf(foc_get_ic()) > MOTOR_OC_TRIP_A) return "overcurrent";

        uint8_t state = (uint8_t)((GPIOD->IDR >> 12) & 0x7u);
        if (state == 0u || state == 7u) continue;   /* invalid Hall code */

        acc->sum_sin[state] += sinf(theta);
        acc->sum_cos[state] += cosf(theta);
        acc->count[state]++;

        if (state != prev && prev >= 1u && prev <= 6u) {
            if (dir > 0) {
                acc->next_ccw[prev] = state; acc->next_cw[state] = prev;
                acc->bnd_fwd[prev][state] = theta;
            } else {
                acc->next_cw[prev]  = state; acc->next_ccw[state] = prev;
                acc->bnd_rev[prev][state] = theta;
            }
        }
        prev = state;
    }
    return NULL;
}

bool hall_calibrate(char *report, size_t report_size)
{
    int n = 0;
    #define HCAL_APPEND(...) do { \
        if ((size_t)n < report_size) \
            n += snprintf(report + n, report_size - (size_t)n, __VA_ARGS__); \
    } while (0)

    /* vmag ≈ I_hold · Rs at standstill (no back-EMF) */
    if (!foc_force_begin(HCAL_ALIGN_CURRENT_A * MOTOR_RS_OHM)) {
        HCAL_APPEND("  hcal: disable FOC first (dis)\r\n");
        return false;
    }

    /* Forward (CCW) then reverse (CW) pass, sharing the accumulators so the
     * one-way lag/hysteresis bias averages out. */
    HcalAccum   acc = {0};
    const char *err = hcal_sweep(+1, &acc);
    if (!err) err   = hcal_sweep(-1, &acc);

    foc_force_end();

    if (err) {
        HCAL_APPEND("  hcal: aborted (%s)\r\n", err);
        return false;
    }

    /* Validate: all six states seen */
    char missing[16];
    int  mlen = 0;
    for (uint8_t st = 1u; st <= 6u; st++) {
        if (acc.count[st] == 0u)
            mlen += snprintf(missing + mlen, sizeof missing - (size_t)mlen, " %u", st);
    }
    if (mlen > 0) {
        HCAL_APPEND("  hcal: incomplete — states never seen:%s\r\n"
                    "        (check Hall wiring / raise drive current)\r\n", missing);
        return false;
    }

    /* Walk the CCW cycle from state 1; must visit 6 distinct states and close */
    uint8_t order[6];
    uint8_t cur      = 1u;
    bool    cycle_ok = true;
    for (int k = 0; k < 6 && cycle_ok; k++) {
        order[k] = cur;
        for (int j = 0; j < k; j++)
            if (order[j] == cur) cycle_ok = false;   /* repeated → not a 6-cycle */
        uint8_t nxt = acc.next_ccw[cur];
        if (nxt < 1u || nxt > 6u) cycle_ok = false;
        cur = nxt;
    }
    if (!cycle_ok || cur != 1u) {
        HCAL_APPEND("  hcal: inconsistent Hall sequence (noisy/ambiguous edges)\r\n");
        return false;
    }

    /* Compute per-state center angles (circular mean) */
    float angle[8] = {0};
    for (uint8_t st = 1u; st <= 6u; st++) {
        float a = atan2f(acc.sum_sin[st], acc.sum_cos[st]);
        if (a < 0.0f) a += M_TWOPI_F;
        angle[st] = a;
    }

    /* Compute boundary angles: average forward and reverse transition
     * angles for each sector pair to cancel hysteresis. */
    float boundary[8][8] = {{0}};
    for (uint8_t from = 1u; from <= 6u; from++) {
        uint8_t to = acc.next_ccw[from];
        if (to < 1u || to > 6u) continue;
        float fwd = acc.bnd_fwd[from][to];
        float rev = acc.bnd_rev[to][from];
        float b   = circ_midpoint(fwd, rev);
        boundary[from][to] = b;
        boundary[to][from] = b;
    }

    /* Apply atomically — the current loop ISR reads these tables */
    __disable_irq();
    for (uint8_t st = 1u; st <= 6u; st++) s_sector_angle[st] = angle[st];
    for (uint8_t st = 0u; st < 8u; st++) {
        s_next_ccw[st] = acc.next_ccw[st];
        s_next_cw[st]  = acc.next_cw[st];
    }
    for (uint8_t from = 1u; from <= 6u; from++)
        for (uint8_t to = 1u; to <= 6u; to++)
            s_boundary[from][to] = boundary[from][to];
    s_angle_offset = 0.0f;
    __enable_irq();

    /* Report */
    HCAL_APPEND("  hall calibration applied (RAM):\r\n");
    for (uint8_t st = 1u; st <= 6u; st++)
        HCAL_APPEND("    state %u : %6.1f deg  (n=%u)\r\n",
                    st, (double)(angle[st] * (180.0f / M_PI_F)), acc.count[st]);
    HCAL_APPEND("    CCW order:");
    for (int k = 0; k < 6; k++) HCAL_APPEND(" %u", order[k]);
    HCAL_APPEND(" -> %u\r\n", order[0]);
    HCAL_APPEND("    Boundaries:\r\n");
    for (uint8_t from = 1u; from <= 6u; from++) {
        uint8_t to = acc.next_ccw[from];
        HCAL_APPEND("      %u->%u : %6.1f deg  (fwd:%5.1f rev:%5.1f)\r\n",
                    from, to, (double)(boundary[from][to] * (180.0f / M_PI_F)),
                    (double)(acc.bnd_fwd[from][to] * (180.0f / M_PI_F)),
                    (double)(acc.bnd_rev[to][from] * (180.0f / M_PI_F)));
    }
    HCAL_APPEND("    hoff reset to 0 (RAM only; reboots to defaults)\r\n");

    #undef HCAL_APPEND
    return true;
}

/* ------------------------------------------------------------------ */
/* Hall angle check (`hchk` CLI command)                               */
/* ------------------------------------------------------------------ */

#if FOC_DEBUG_ENABLE
/* Ground-truth score of the live table+offset: sweep the forced open-loop
 * angle (the same drive hcal trusts) and accumulate err = wrap_pi(θ̂ − θ_cmd)
 * per Hall state as a circular mean, fwd and rev kept separate. At this sweep
 * speed the observer is stale-held at sector centres, so instantaneous err
 * sawtooths ±half-sector by design; the fwd/rev-averaged per-state mean is the
 * verdict (load-angle lag cancels). */
typedef struct {
    float    ss[2][8], sc[2][8];   /* err circular-mean accum [pass][state] */
    uint16_t cnt[2][8];
} HchkAccum;

static const char *hchk_sweep(int pass, int dir, HchkAccum *a)
{
    foc_force_set_angle(dir > 0 ? 0.0f : M_TWOPI_F);
    HAL_Delay(HCAL_ALIGN_MS);

    for (uint32_t i = 0; i < HCAL_N_STEPS; i++) {
        float frac  = (float)i / (float)HCAL_N_STEPS;
        float theta = (dir > 0) ? (M_TWOPI_F * frac)
                                : (M_TWOPI_F * (1.0f - frac));
        foc_force_set_angle(theta);
        HAL_Delay(HCAL_STEP_MS);

        if (fault_is_active()) return "fault latched";
        if (fabsf(foc_get_ia()) > MOTOR_OC_TRIP_A ||
            fabsf(foc_get_ib()) > MOTOR_OC_TRIP_A ||
            fabsf(foc_get_ic()) > MOTOR_OC_TRIP_A) return "overcurrent";

        uint8_t state = (uint8_t)((GPIOD->IDR >> 12) & 0x7u);
        if (state == 0u || state == 7u) continue;

        float err = wrap_pi(hall_get_theta_e() - theta);
        a->ss[pass][state] += sinf(err);
        a->sc[pass][state] += cosf(err);
        a->cnt[pass][state]++;
    }
    return NULL;
}

bool hall_check(char *report, size_t report_size)
{
    int n = 0;
    #define HCHK_APPEND(...) do { \
        if ((size_t)n < report_size) \
            n += snprintf(report + n, report_size - (size_t)n, __VA_ARGS__); \
    } while (0)

    if (!foc_force_begin(HCAL_ALIGN_CURRENT_A * MOTOR_RS_OHM)) {
        HCHK_APPEND("  hchk: disable FOC first (dis)\r\n");
        return false;
    }

    HchkAccum   acc = {0};
    const char *err = hchk_sweep(0, +1, &acc);
    if (!err) err   = hchk_sweep(1, -1, &acc);

    foc_force_end();

    if (err) {
        HCHK_APPEND("  hchk: aborted (%s)\r\n", err);
        return false;
    }

    const float r2d = 180.0f / M_PI_F;
    bool pass = true;
    HCHK_APPEND("  hall check (theta_hat vs forced angle, fwd+rev):\r\n");
    for (uint8_t st = 1u; st <= 6u; st++) {
        if (acc.cnt[0][st] == 0u || acc.cnt[1][st] == 0u) {
            HCHK_APPEND("    state %u : not seen in both directions\r\n", st);
            pass = false;
            continue;
        }
        float mf     = atan2f(acc.ss[0][st], acc.sc[0][st]);
        float mr     = atan2f(acc.ss[1][st], acc.sc[1][st]);
        float mean   = atan2f(acc.ss[0][st] + acc.ss[1][st],
                              acc.sc[0][st] + acc.sc[1][st]);
        float spread = wrap_pi(mf - mr);
        if (fabsf(mean) * r2d > HCHK_PASS_ERR_DEG) pass = false;
        HCHK_APPEND("    state %u : mean %+6.1f deg  fwd/rev spread %5.1f deg  (n=%u)\r\n",
                    st, (double)(mean * r2d), (double)(fabsf(spread) * r2d),
                    acc.cnt[0][st] + acc.cnt[1][st]);
    }
    HCHK_APPEND("  %s (per-state |mean| limit %.1f deg)\r\n",
                pass ? "PASS" : "FAIL", (double)HCHK_PASS_ERR_DEG);

    #undef HCHK_APPEND
    return pass;
}
#endif
