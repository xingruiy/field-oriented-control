#include "arm/arm_pos.h"
#include "encoder/encoder.h"
#include "control/foc.h"
#include "control/foc_math.h"
#include "fault/fault.h"
#include "common/settings.h"
#include "main.h"
#include <math.h>

typedef struct {
    bool active;
    bool auto_enabled;
    ArmPosStatus status;
    float target_deg;
    float error_deg;
    float output_rpm;
    float kp, ki, kd;
    float integral;
    uint32_t last_ms;
} ArmPosState;

static ArmPosState s;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void clear_pid(void)
{
    s.integral = 0.0f;
    s.output_rpm = 0.0f;
    s.error_deg = s.target_deg - encoder_get_total_deg();
    s.last_ms = HAL_GetTick();
}

static void stop_drive(void)
{
    foc_speed_disable();
    foc_set_iq_ref(0.0f);
    if (s.auto_enabled) {
        foc_disable();
        s.auto_enabled = false;
    }
}

void arm_pos_init(void)
{
    s.active = false;
    s.auto_enabled = false;
    s.status = ARM_POS_IDLE;
    s.target_deg = 0.0f;
    s.error_deg = 0.0f;
    s.output_rpm = 0.0f;
    s.kp = ARM_POS_KP;
    s.ki = ARM_POS_KI;
    s.kd = ARM_POS_KD;
    s.integral = 0.0f;
    s.last_ms = HAL_GetTick();
}

bool arm_pos_set_target_deg(float target_deg)
{
    if (target_deg < ARM_POS_MIN_DEG || target_deg > ARM_POS_MAX_DEG) {
        s.status = ARM_POS_LIMIT_ERROR;
        s.active = false;
        s.target_deg = target_deg;
        s.error_deg = target_deg - encoder_get_total_deg();
        stop_drive();
        return false;
    }
    if (!encoder_is_ok()) {
        s.status = ARM_POS_ENCODER_ERROR;
        s.active = false;
        s.target_deg = target_deg;
        stop_drive();
        return false;
    }
    if (fault_is_active()) {
        s.status = ARM_POS_FAULT;
        s.active = false;
        s.target_deg = target_deg;
        stop_drive();
        return false;
    }

    bool was_enabled = foc_is_enabled();
    if (!was_enabled) {
        foc_enable();
        if (!foc_is_enabled()) {
            s.status = ARM_POS_DRIVE_ERROR;
            s.active = false;
            s.target_deg = target_deg;
            stop_drive();
            return false;
        }
    }

    s.target_deg = target_deg;
    s.auto_enabled = !was_enabled;
    s.active = true;
    s.status = ARM_POS_ACTIVE;
    clear_pid();
    return true;
}

void arm_pos_stop(void)
{
    s.active = false;
    s.status = ARM_POS_IDLE;
    s.output_rpm = 0.0f;
    s.integral = 0.0f;
    stop_drive();
}

void arm_pos_poll(void)
{
    if (!s.active) return;

    if (fault_is_active()) {
        s.active = false;
        s.status = ARM_POS_FAULT;
        stop_drive();
        return;
    }
    if (!encoder_is_ok()) {
        s.active = false;
        s.status = ARM_POS_ENCODER_ERROR;
        stop_drive();
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - s.last_ms;
    if (dt_ms < ENC_POLL_PERIOD_MS) return;
    s.last_ms = now;

    float dt = (float)dt_ms * 1.0e-3f;
    float cur_deg = encoder_get_total_deg();
    float err = s.target_deg - cur_deg;
    s.error_deg = err;

    s.integral += err * dt;
    if (s.ki != 0.0f) {
        float ilim = fabsf(ARM_POS_I_LIMIT_RPM / s.ki);
        s.integral = clampf(s.integral, -ilim, ilim);
    } else {
        s.integral = 0.0f;
    }

    float p = s.kp * err;
    float i = s.ki * s.integral;
    float d = -s.kd * encoder_get_speed_dps();
    float out = p + i + d;

    if (fabsf(err) <= ARM_POS_TOL_DEG) {
        s.status = ARM_POS_REACHED;
    } else {
        s.status = ARM_POS_ACTIVE;
    }

    out = clampf(out, -ARM_POS_MAX_RPM, ARM_POS_MAX_RPM);
    out *= (float)ARM_POS_DIR_SIGN;
    s.output_rpm = out;
    foc_set_speed_ref(RPM_TO_OMEGA_E(out));
}

bool arm_pos_is_active(void) { return s.active; }
ArmPosStatus arm_pos_get_status(void) { return s.status; }
float arm_pos_get_target_deg(void) { return s.target_deg; }
float arm_pos_get_error_deg(void) { return s.error_deg; }
float arm_pos_get_output_rpm(void) { return s.output_rpm; }
float arm_pos_get_kp(void) { return s.kp; }
float arm_pos_get_ki(void) { return s.ki; }
float arm_pos_get_kd(void) { return s.kd; }

void arm_pos_set_kp(float kp) { s.kp = (kp > 0.0f) ? kp : 0.0f; }
void arm_pos_set_ki(float ki) { s.ki = (ki > 0.0f) ? ki : 0.0f; }
void arm_pos_set_kd(float kd) { s.kd = (kd > 0.0f) ? kd : 0.0f; }

const char *arm_pos_status_str(ArmPosStatus st)
{
    switch (st) {
    case ARM_POS_IDLE:          return "idle";
    case ARM_POS_ACTIVE:        return "active";
    case ARM_POS_REACHED:       return "reached";
    case ARM_POS_ENCODER_ERROR: return "encoder-error";
    case ARM_POS_LIMIT_ERROR:   return "limit-error";
    case ARM_POS_FAULT:         return "fault";
    case ARM_POS_DRIVE_ERROR:   return "drive-error";
    default:                    return "?";
    }
}
