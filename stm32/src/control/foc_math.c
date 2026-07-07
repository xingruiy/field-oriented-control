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

void foc_park(float alpha, float beta, float theta_e,
              float *d, float *q)
{
    float c = cosf(theta_e);
    float s = sinf(theta_e);
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

void foc_inv_park(float vd, float vq, float theta_e,
                  float *alpha, float *beta)
{
    float c = cosf(theta_e);
    float s = sinf(theta_e);
    *alpha = vd * c - vq * s;
    *beta  = vd * s + vq * c;
}

static inline float fclampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void foc_svm(float valpha, float vbeta, float vbus,
             uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3)
{
    float va =  valpha;
    float vb = (-valpha + M_SQRT3_F * vbeta) * 0.5f;
    float vc = (-valpha - M_SQRT3_F * vbeta) * 0.5f;

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
