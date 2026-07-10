#include "control/foc_math.h"
#include "common/settings.h"
#include <math.h>

void foc_clarke(float ia, float ib, float ic,
                float *alpha, float *beta)
{
    (void)ic;
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) / M_SQRT3_F;
}

void foc_park_sc(float alpha, float beta, float sin_t, float cos_t,
                 float *d, float *q)
{
    *d =  alpha * cos_t + beta * sin_t;
    *q = -alpha * sin_t + beta * cos_t;
}

void foc_inv_park_sc(float vd, float vq, float sin_t, float cos_t,
                     float *alpha, float *beta)
{
    *alpha = vd * cos_t - vq * sin_t;
    *beta  = vd * sin_t + vq * cos_t;
}

void foc_park(float alpha, float beta, float theta_e,
              float *d, float *q)
{
    foc_park_sc(alpha, beta, sinf(theta_e), cosf(theta_e), d, q);
}

void foc_inv_park(float vd, float vq, float theta_e,
                  float *alpha, float *beta)
{
    foc_inv_park_sc(vd, vq, sinf(theta_e), cosf(theta_e), alpha, beta);
}

static inline float fclampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Dead-time compensation term for one phase: ±DT_COMP_V in the direction of
 * the phase current, tapered linearly through zero below DT_COMP_I_TH_A so
 * current-sense noise cannot chatter the sign at zero crossings. */
static inline float dt_comp(float i)
{
    return fclampf(i * (DT_COMP_V / DT_COMP_I_TH_A), -DT_COMP_V, DT_COMP_V);
}

void foc_svm(float valpha, float vbeta, float vbus,
             float ia, float ib, float ic,
             uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3)
{
    float va =  valpha;
    float vb = (-valpha + M_SQRT3_F * vbeta) * 0.5f;
    float vc = (-valpha - M_SQRT3_F * vbeta) * 0.5f;

    /* Dead-time compensation: the bridge loses ~Vbus·t_dt·f_pwm of average
     * voltage opposing each phase current; add it back before modulation. */
    va += dt_comp(ia);
    vb += dt_comp(ib);
    vc += dt_comp(ic);

    /* Zero-sequence injection (min-max SVM) */
    float vmax = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
    float vmin = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
    float voff = -(vmax + vmin) * 0.5f;

    float half = vbus * 0.5f;

    float da = (va + voff + half) / vbus;
    float db = (vb + voff + half) / vbus;
    float dc = (vc + voff + half) / vbus;

    /* CCR = duty × ARR, clamped away from full-on/full-off */
    *ccr1 = (uint16_t)fclampf(da * (float)PWM_ARR, 1.0f, (float)(PWM_ARR - 1));
    *ccr2 = (uint16_t)fclampf(db * (float)PWM_ARR, 1.0f, (float)(PWM_ARR - 1));
    *ccr3 = (uint16_t)fclampf(dc * (float)PWM_ARR, 1.0f, (float)(PWM_ARR - 1));
}
