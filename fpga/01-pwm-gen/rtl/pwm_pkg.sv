// ============================================================================
// pwm_pkg.sv
//
//  Shared parameters for the 01-pwm-gen project. Compiled first by the
//  simulate/build flows, then imported where needed via `import pwm_pkg::*;`.
//
//  Operating point (Arty S7-50, 100 MHz oscillator):
//    PWM_PERIOD = F_CLK / F_PWM = 100e6 / 80e3 = 1250 counts
//    F_PWM      = 100e6 / 1250  = 80.000 kHz exactly
//    period     = 1250 counts   = 12.5 us
//
//  The period (1250) is not a power of two, so the counter/duty width PWM_W
//  is derived from it independently of any power-of-two resolution.
// ============================================================================

package pwm_pkg;

  parameter int unsigned F_CLK_HZ   = 100_000_000;            // board oscillator
  parameter int unsigned F_PWM_HZ   = 80_000;                 // target PWM rate
  parameter int unsigned PWM_PERIOD = F_CLK_HZ / F_PWM_HZ;    // 1250 counts
  parameter int unsigned PWM_TOP    = PWM_PERIOD - 1;         // 1249; counter wraps here
  parameter int unsigned PWM_W      = $clog2(PWM_PERIOD);     // 11 bits
  parameter int unsigned SW_W       = 4;                      // 4 slide switches -> 16 steps

endpackage : pwm_pkg
