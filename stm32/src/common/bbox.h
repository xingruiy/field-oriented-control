#pragma once
#include <stdint.h>
#include "common/settings.h"

/* In-RAM black-box recorder: 8 int16 channels logged every 40 kHz current-loop
 * tick into a 256 KB ring buffer (≈0.41 s of lossless history). Arm it, run the
 * test; a trigger (manual `bb trig`, fault/OC via foc_emergency_stop(), or
 * |ω̂| > BBOX_TRIP_OMEGA) records BBOX_POST_TRIG more samples then freezes, so
 * the capture holds both pre- and post-trigger history.
 *
 * Channels: 0 θ̂ (u16 = rad·65536/2π), 1 ω̂ (rad/s), 2 innovation (cdeg),
 *           3 Iq (mA), 4 Id (mA), 5 Iq_ref (mA), 6 Hall sector, 7 Ia (mA).
 *
 * Dump on the PC with tools/bbox/bbox_dump.py: it reads the g_bbox symbol over
 * ST-LINK (address from the build .map) and renders CSV + interactive HTML. */

enum { BBOX_IDLE = 0u, BBOX_ARMED, BBOX_TRIGGERED, BBOX_FROZEN };

#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE
void     bbox_arm(void);      /* reset + start recording                     */
void     bbox_trigger(void);  /* request freeze; safe from any context       */
uint32_t bbox_state(void);    /* BBOX_* */
uint32_t bbox_count(void);    /* samples recorded (saturates at BBOX_LEN)    */

/* Called once per current-loop tick, after hall_observer_update(). */
void     bbox_sample(float iq, float id, float iq_ref, float ia);
#else
static inline void bbox_arm(void) {}
static inline void bbox_trigger(void) {}
static inline uint32_t bbox_state(void) { return BBOX_IDLE; }
static inline uint32_t bbox_count(void) { return 0u; }
static inline void bbox_sample(float iq, float id, float iq_ref, float ia)
{
    (void)iq;
    (void)id;
    (void)iq_ref;
    (void)ia;
}
#endif
