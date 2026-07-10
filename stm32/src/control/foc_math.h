#pragma once
#include <stdint.h>

#define M_PI_F          3.14159265358979323846f
#define M_TWOPI_F       6.28318530717958647692f
#define M_SQRT3_F       1.73205080756887729353f
#define M_PI_OVER_3_F   1.04719755119659774615f  /* π/3 = one Hall sector */

/* Clarke transform — amplitude-invariant, assumes Ia+Ib+Ic = 0 */
void foc_clarke(float ia, float ib, float ic,
                float *alpha, float *beta);

/* Park / inverse Park with caller-supplied sin/cos of θe. The 40 kHz loop
 * needs both transforms at the same angle — compute sinf/cosf once and share
 * (two libm transcendentals per tick instead of four). */
void foc_park_sc(float alpha, float beta, float sin_t, float cos_t,
                 float *d, float *q);
void foc_inv_park_sc(float vd, float vq, float sin_t, float cos_t,
                     float *alpha, float *beta);

/* Convenience wrappers that compute sin/cos internally (thread-context use). */
void foc_park(float alpha, float beta, float theta_e,
              float *d, float *q);
void foc_inv_park(float vd, float vq, float theta_e,
                  float *alpha, float *beta);

/* Min-max SVM — symmetric 7-segment equivalent — with per-phase dead-time
 * compensation (adds DT_COMP_V·sign(i_phase), tapered below DT_COMP_I_TH_A).
 * vbus: DC bus voltage [V]
 * valpha, vbeta: voltage vector [V]
 * ia, ib, ic: measured phase currents [A] for the dead-time term
 * ccr*: output TIM1 compare values in [1, PWM_ARR-1] */
void foc_svm(float valpha, float vbeta, float vbus,
             float ia, float ib, float ic,
             uint16_t *ccr1, uint16_t *ccr2, uint16_t *ccr3);
