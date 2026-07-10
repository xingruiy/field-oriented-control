#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "common/settings.h"

/* Hall sensor state machine and electrical angle / speed estimation.
 *
 * Hardware:  TIM4 Hall sensor interface (slave RESET) mode, 100 kHz timebase
 *            (200 MHz kernel / (PSC 1999 + 1), set in bsp_tim4_init). The
 *            counter resets on every Hall edge: CCR1 = inter-edge period
 *            (ticks), CNT = ticks since last edge.
 *            PD12=HA, PD13=HB, PD14=HC — read directly from GPIOD.
 *
 * Motor:     1 pole pair → θ_electrical = θ_mechanical.
 *            6 Hall edges per revolution; each sector ≈ 60° electrical
 *            (true widths are calibrated by hcal). */

void    hall_init(void);

/* Called from TIM4 CC1 capture callback (priority 5). */
void    hall_update(void);

/* Advance the Hall PLL angle observer by dt seconds. Call once per current-loop
 * tick (priority 1) before hall_get_theta_e(); it integrates θ̂ and applies the
 * soft per-edge boundary correction. */
void    hall_observer_update(float dt);

/* Called from ADC current-loop ISR (priority 1).
 * Returns the observed θe in [0, 2π). */
float   hall_get_theta_e(void);

/* Signed electrical angular velocity in rad/s: +CCW, −CW; 0 when stopped/stale.
 * (CLI speed readouts show the magnitude and report direction separately.) */
float   hall_get_omega_e(void);

/* Hall 3-bit code (PD12..14) as latched on the last edge, for CLI diagnostics. */
uint8_t hall_get_state(void);

/* Live 3-bit Hall code read straight from the pins (no edge/ISR latency).
 * Valid codes are 1..6; 0/7 mean a disconnected or shorted sensor line. */
uint8_t hall_state_now(void);

/* Current sector = the Hall code (1..6) latched on the last edge. */
uint8_t hall_get_sector(void);

/* Last detected rotation direction: +1=CCW, -1=CW, 0=unknown/stopped. */
int8_t  hall_get_dir(void);

/* Runtime-adjustable electrical angle offset added to sector angle (radians).
 * Tune empirically: increase until motor spins smoothly, try ±π/6 first. */
#if FOC_DEBUG_ENABLE
void    hall_set_angle_offset(float rad);
float   hall_get_angle_offset(void);

/* Hall PLL gains (per-edge PI angle-tracking observer). Live-tune via `hpll`;
 * boot defaults are HALL_PLL_KP/KI in settings.h. */
void    hall_set_pll(float kp, float ki);
float   hall_get_pll_kp(void);
float   hall_get_pll_ki(void);
#endif

/* Debug accessors for SWV/ITM trace and Live Expressions. hall_get_omega_hat()
 * is the raw θe-frame observed speed (unlike hall_get_omega_e() it is not
 * stale-gated or physical-sign-mapped) so a runaway is visible as-is.
 * hall_get_min_period() / hall_get_glitch_edges() expose the edge-glitch
 * counters (see hall_update): a near-HALL_MIN_PERIOD edge snaps ω̂ enormous. */
#if FOC_DEBUG_ENABLE
float    hall_get_omega_hat(void);
uint16_t hall_get_min_period(void);
uint32_t hall_get_glitch_edges(void);
void     hall_dbg_reset(void);

/* Per-edge innovation: wrap_pi(θ̂ − boundary) captured just before each hard
 * snap — the θ̂ interpolation error accumulated over the previous sector.
 * last/max are signed radians (max keeps the sign of the largest |·|);
 * hall_get_innov_sector(st) is the last innovation on entering sector st.
 * Reset together with the glitch counters via hall_dbg_reset(). */
float    hall_get_innov_last(void);
float    hall_get_innov_max(void);
float    hall_get_innov_sector(uint8_t st);
uint32_t hall_get_innov_count(void);
#endif

/* Open-loop Hall angle calibration. Drives the rotor through one electrical
 * revolution with a forced voltage vector (via foc_force_*), measures the
 * center electrical angle of each Hall state and the rotation sequence, then
 * applies the result to the live sector/transition tables (RAM) and resets the
 * scalar angle offset to 0. Requires FOC disabled and no latched fault.
 * Writes a human-readable summary into report[]. Returns true on success. */
bool    hall_calibrate(char *report, size_t report_size);

/* Hall angle check (`hchk`): score the live table+offset against the forced
 * open-loop angle without modifying anything. Same drive/guards as
 * hall_calibrate. Writes a per-state report with a PASS/FAIL verdict
 * (per-state |circular mean error| < HCHK_PASS_ERR_DEG). Returns pass. */
#if FOC_DEBUG_ENABLE
bool    hall_check(char *report, size_t report_size);
#endif
