#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "common/settings.h"

/* Field-oriented current (torque) control — single 40 kHz loop.
 *
 * Call order in main() (STM32H755, dual ADC injected-simultaneous):
 *   drv8316_init();
 *   hall_init();
 *   foc_init();                          <- starts TIM1, calibrates ADC offsets
 *   HAL_TIMEx_HallSensor_Start_IT(&htim4);
 *   HAL_ADCEx_InjectedStart(&hadc2);     <- slave ADC2 (phase B), no IT
 *   HAL_ADCEx_InjectedStart_IT(&hadc1);  <- master ADC1 starts current-loop ISR
 *   fault_init();
 *   cli_init();
 *   // foc_enable() is user-triggered via CLI "en" command
 */

void  foc_init(void);
void  foc_enable(void);
void  foc_disable(void);

/* ISR-safe immediate kill (MOE off + disable flag). Call from the nFAULT
 * EXTI handler; follow up with foc_disable() in thread context. */
void  foc_emergency_stop(void);

/* Set Iq reference in amperes. Sign encodes direction.
 * Clamped to ±MOTOR_CURRENT_LIMIT_A. Id_ref is always 0.
 * The live iq_ref slews toward this target at IQ_REF_SLEW_A_PER_S. */
void  foc_set_iq_ref(float iq_a);

/* Re-run zero-current ADC offset calibration (requires FOC disabled).
 * Returns false if the conversion poll timed out (no ADC triggers). */
bool  foc_recalibrate(void);

/* Forced-angle open-loop drive — used by Hall angle calibration.
 * Applies a stationary voltage vector (vmag along the commanded electrical
 * angle, vq=0); at standstill this pulls the rotor d-axis to that angle.
 * Bypasses the current PI loop. Mutually exclusive with the enabled FOC loop.
 *
 *   foc_force_begin()  refuses (returns false) if FOC is enabled; else turns
 *                      MOE on and starts driving at angle 0.
 *   foc_force_set_angle() updates the commanded angle (rad).
 *   foc_force_end()    stops driving (neutral duty, MOE off). */
bool  foc_force_begin(float vmag);
void  foc_force_set_angle(float theta);
void  foc_force_set_vq(float vq);
void  foc_force_end(void);

/* Rotor-frame fixed-voltage drive. Applies fixed vd/vq voltages in the live
 * Hall θe frame, bypassing current PI regulation. Mutually exclusive with FOC,
 * forced-angle drive, and 6-step block mode. */
#if FOC_DEBUG_ENABLE
bool  foc_rotor_voltage_begin(float vd, float vq);
void  foc_rotor_voltage_set(float vd, float vq);
void  foc_rotor_voltage_end(void);
bool  foc_is_rotor_voltage_mode(void);
#else
static inline bool foc_is_rotor_voltage_mode(void) { return false; }
#endif

/* Live current-loop PI gain tuning. Each setter drives both the d- and q-axis
 * controllers (axes share Rs/Ls, so gains are identical). RAM only — resets to
 * settings.h defaults on reboot. Safe to call while enabled. */
#if FOC_DEBUG_ENABLE
void  foc_set_kp(float kp);
void  foc_set_ki(float ki);
float foc_get_vd_cmd(void);
float foc_get_vq_cmd(void);
#endif

/* Outer speed loop. foc_set_speed_ref() takes a setpoint in electrical rad/s
 * (use RPM_TO_OMEGA_E) and engages speed mode: the speed PI then generates
 * iq_ref at 1 kHz inside the current-loop ISR. The active reference ramps from
 * the measured speed toward the setpoint at SPEED_REF_SLEW_RAD_S2. Any
 * foc_set_iq_ref() call exits speed mode (manual torque override). Requires
 * foc_enable() to drive the motor.
 * foc_speed_disable() reverts to manual mode and resets the speed integrator. */
void  foc_set_speed_ref(float omega_e);
void  foc_speed_disable(void);
#if FOC_DEBUG_ENABLE
void  foc_set_speed_kp(float kp);
void  foc_set_speed_ki(float ki);
#endif
bool  foc_is_speed_mode(void);
float foc_get_speed_ref(void);   /* commanded setpoint, electrical rad/s */
#if FOC_DEBUG_ENABLE
float foc_get_speed_kp(void);
float foc_get_speed_ki(void);
#endif

/* 6-step (block commutation) control. Maps Hall state directly to a fixed
 * commutation vector — two phases PWM-driven, one at 50% (virtual neutral).
 * Mutually exclusive with FOC and force_mode. Duty in [0, BLOCK_DUTY_MAX];
 * sign encodes direction (+CCW, −CW). */
void  foc_block_enable(float duty);
void  foc_block_disable(void);
void  foc_block_set_duty(float duty);
bool  foc_is_block_mode(void);

/* Autotune (blocking, thread-context — call from the CLI, not an ISR). Both
 * require FOC disabled and no latched fault, drive the motor themselves, leave
 * it disabled on return, and write a human-readable summary into report[].
 *
 * foc_tune_current(): identifies Rs/Ls (rotor aligns/twitches) and applies the
 *   current-loop gains Kp=Ls*wc, Ki=Rs*wc.
 * foc_tune_speed():   relay (Åström) autotune at omega_ref [elec rad/s] — the
 *   motor oscillates, then settles — and applies the speed-loop skp/ski.
 * Return true on success. */
#if FOC_DEBUG_ENABLE
bool  foc_tune_current(char *report, size_t n);
bool  foc_tune_speed(float omega_ref, char *report, size_t n);
#endif

/* Software overcurrent backstop (MOTOR_OC_TRIP_A). The 40 kHz loop latches
 * oc_trip and kills the bridge when any phase exceeds the threshold; fault_poll()
 * surfaces and clears it. foc_clear_oc_trip() is called from the fault layer. */
bool  foc_oc_tripped(void);
void  foc_clear_oc_trip(void);

/* Invalid-Hall-code backstop. While FOC / block / rotor-voltage mode is
 * driving, a Hall code of 0b000/0b111 persisting HALL_INVALID_TRIP_TICKS
 * kills the bridge and latches hall_trip; fault_poll() surfaces it. */
bool  foc_hall_tripped(void);
void  foc_clear_hall_trip(void);

/* State accessors for CLI diagnostics */
bool  foc_is_enabled(void);
float foc_get_iq_ref(void);      /* live (slew-ramped) reference */
float foc_get_iq_target(void);   /* commanded torque setpoint    */
#if FOC_DEBUG_ENABLE
float foc_get_kp(void);
float foc_get_ki(void);
#endif
float foc_get_ia(void);
float foc_get_ib(void);
float foc_get_ic(void);
float foc_get_id(void);
float foc_get_iq(void);
float foc_get_offset_a(void);
float foc_get_offset_b(void);
float foc_get_offset_c(void);
