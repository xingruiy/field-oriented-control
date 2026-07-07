#pragma once

typedef struct {
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
} PidState;

void  pid_init(PidState *s, float kp, float ki, float out_min, float out_max);
void  pid_reset(PidState *s);
void  pid_set_gains(PidState *s, float kp, float ki);
float pid_update(PidState *s, float error, float dt);
