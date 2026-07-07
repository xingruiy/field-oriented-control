#include "common/bbox.h"
#include "common/settings.h"

#if FOC_DEBUG_ENABLE && FOC_BBOX_ENABLE

#include "control/foc_math.h"
#include "control/foc.h"
#include "hall/hall.h"
#include <math.h>

#define BBOX_CH     8u
#define BBOX_MAGIC  0x31584242u   /* "BBX1" little-endian */

/* Header layout is decoded by tools/bbox/bbox_dump.py — keep in lockstep. */
typedef struct {
    uint32_t magic;
    volatile uint32_t state;      /* BBOX_*                                  */
    volatile uint32_t widx;       /* next slot to write                      */
    volatile uint32_t nsamp;      /* total written, saturates at BBOX_LEN    */
    volatile uint32_t trig_idx;   /* slot of the trigger sample              */
    uint32_t len, nch, tick_hz;   /* buffer geometry for the decoder         */
} BboxHeader;

typedef struct {
    BboxHeader hdr;
    int16_t    samp[BBOX_LEN][BBOX_CH];
} Bbox;

/* Global (not static) so the symbol appears in the .map for the dump script. */
Bbox g_bbox = { .hdr = { .magic = BBOX_MAGIC, .len = BBOX_LEN,
                         .nch = BBOX_CH, .tick_hz = 40000u } };

static volatile uint32_t s_trig_req;   /* set by bbox_trigger(), any context */
static uint32_t          s_post_left;  /* run-on countdown (ISR-only)        */

void bbox_arm(void)
{
    g_bbox.hdr.state = BBOX_IDLE;      /* stop sampling while resetting */
    g_bbox.hdr.widx = g_bbox.hdr.nsamp = g_bbox.hdr.trig_idx = 0u;
    s_trig_req = 0u;
    g_bbox.hdr.state = BBOX_ARMED;
}

void     bbox_trigger(void) { s_trig_req = 1u; }
uint32_t bbox_state(void)   { return g_bbox.hdr.state; }
uint32_t bbox_count(void)   { return g_bbox.hdr.nsamp; }

static inline int16_t sat16(float v)
{
    if (v >  32767.0f) return INT16_MAX;
    if (v < -32768.0f) return INT16_MIN;
    return (int16_t)lrintf(v);
}

void bbox_sample(float iq, float id, float iq_ref, float ia)
{
    uint32_t st = g_bbox.hdr.state;
    if (st != BBOX_ARMED && st != BBOX_TRIGGERED) return;

    float omega = hall_get_omega_hat();
    int16_t *s  = g_bbox.samp[g_bbox.hdr.widx];
    /* θ̂ as full-scale u16 turns: [0,2π) → [0,65536), wraps naturally. */
    s[0] = (int16_t)(uint16_t)lrintf(hall_get_theta_e() * (65536.0f / M_TWOPI_F));
    s[1] = sat16(omega);
    s[2] = sat16(hall_get_innov_last() * CAN_ANGLE_CDEG_SCALE);
    s[3] = sat16(iq     * CAN_CURRENT_SCALE);
    s[4] = sat16(id     * CAN_CURRENT_SCALE);
    s[5] = sat16(iq_ref * CAN_CURRENT_SCALE);
    s[6] = (int16_t)hall_get_sector();
    (void)ia;
    s[7] = sat16(foc_get_vq_cmd() * 1000.0f);

    g_bbox.hdr.widx = (g_bbox.hdr.widx + 1u) % BBOX_LEN;
    if (g_bbox.hdr.nsamp < BBOX_LEN) g_bbox.hdr.nsamp++;

    if (st == BBOX_ARMED) {
        if (fabsf(omega) > BBOX_TRIP_OMEGA) s_trig_req = 1u;
        if (s_trig_req) {
            g_bbox.hdr.trig_idx = (g_bbox.hdr.widx + BBOX_LEN - 1u) % BBOX_LEN;
            s_post_left         = BBOX_POST_TRIG;
            g_bbox.hdr.state    = BBOX_TRIGGERED;
        }
    } else if (--s_post_left == 0u) {
        g_bbox.hdr.state = BBOX_FROZEN;
    }
}

#endif
