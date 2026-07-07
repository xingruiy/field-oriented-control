#pragma once
#include <stdint.h>
#include <stdbool.h>

/* TMAG5273 I2C magnetic angle encoder (datasheets/tmag5273.pdf) monitoring an
 * EXTERNAL rotating object — independent of the motor/FOC loops.
 * Blocking I2C1 access from the main loop only (encoder_poll); never in ISRs. */

void     encoder_init(void);          /* probe + configure; safe if absent   */
void     encoder_poll(void);          /* call from while(1); self-paced      */
bool     encoder_is_ok(void);
float    encoder_get_angle_deg(void); /* wrapped [0, 360)                    */
float    encoder_get_total_deg(void); /* unwrapped: turns*360 + angle        */
int32_t  encoder_get_turns(void);
float    encoder_get_speed_dps(void); /* LPF-filtered deg/s, signed          */
uint8_t  encoder_get_magnitude(void); /* raw MAGNITUDE_RESULT (field check)  */
uint8_t  encoder_get_variant(void);   /* DEVICE_ID VER: 1=A1, 2=A2, 0=unknown */
uint32_t encoder_get_err_count(void);
void     encoder_reset_turns(void);
