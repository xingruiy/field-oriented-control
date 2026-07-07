#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Fault supervision for the DRV8316 power stage.
 *
 * Layers:
 *   1. nFAULT EXTI (PC6, EXTI9_5)  -> HAL_GPIO_EXTI_Callback() cuts PWM (MOE off)
 *      via foc_emergency_stop() and raises a pending flag. Never blocks, no SPI.
 *   2. fault_poll() in the main loop:
 *        - services the pending nFAULT: full foc_disable(), decode STATUS1/2 over
 *          SPI, print the cause, latch the fault.
 *        - every 100 ms reads IC_STATUS to catch soft conditions (e.g. overtemp
 *          warning) before they escalate to a latched shutdown.
 *   3. drv8316.c checks HAL_OK + OVR/MODF on every SPI transfer.
 *
 * The 40 kHz current loop handles immediate overcurrent detection;
 * this layer handles SPI decoding, reporting, and fault latching.
 */

void  fault_init(void);   /* reset state, sample initial nFAULT level */
void  fault_poll(void);   /* call from main loop (replaces drv8316_has_fault) */

bool  fault_is_active(void);   /* true while a fault is latched */
void  fault_clear(void);       /* clear DRV faults + software latch */

/* Format the latched fault (decoded STATUS bits + SPI error count) into buf.
 * Returns bytes written. */
int   fault_describe(char *buf, int len);
