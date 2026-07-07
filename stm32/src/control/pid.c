#include "control/pid.h"

void pid_init(PidState *s, float kp, float ki, float out_min, float out_max)
{
    s->kp      = kp;
    s->ki      = ki;
    s->out_min = out_min;
    s->out_max = out_max;
    s->integral = 0.0f;
}

void pid_reset(PidState *s)
{
    s->integral = 0.0f;
}

void pid_set_gains(PidState *s, float kp, float ki)
{
    /* Preserve the integrator so live gain tuning is bumpless (no torque step).
     * The integral is re-clamped against the new ki on the next pid_update().
     * Enable/disable paths still call pid_reset() explicitly when needed.      */
    s->kp = kp;
    s->ki = ki;
}

float pid_update(PidState *s, float error, float dt)
{
    s->integral += error * dt;

    float ilim_hi = (s->ki > 0.0f) ?  s->out_max / s->ki : 0.0f;
    float ilim_lo = (s->ki > 0.0f) ?  s->out_min / s->ki : 0.0f;
    if (s->integral > ilim_hi) s->integral = ilim_hi;
    if (s->integral < ilim_lo) s->integral = ilim_lo;

    float out = s->kp * error + s->ki * s->integral;
    if (out > s->out_max) {
        out = s->out_max;
        if (s->ki > 0.0f) s->integral = (out - s->kp * error) / s->ki;
    } else if (out < s->out_min) {
        out = s->out_min;
        if (s->ki > 0.0f) s->integral = (out - s->kp * error) / s->ki;
    }
    return out;
}
