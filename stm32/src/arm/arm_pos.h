#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ARM_POS_IDLE = 0,
    ARM_POS_ACTIVE,
    ARM_POS_REACHED,
    ARM_POS_ENCODER_ERROR,
    ARM_POS_LIMIT_ERROR,
    ARM_POS_FAULT,
    ARM_POS_DRIVE_ERROR,
} ArmPosStatus;

void arm_pos_init(void);
bool arm_pos_set_target_deg(float target_deg);
void arm_pos_stop(void);
void arm_pos_poll(void);

bool arm_pos_is_active(void);
ArmPosStatus arm_pos_get_status(void);
const char *arm_pos_status_str(ArmPosStatus st);

float arm_pos_get_target_deg(void);
float arm_pos_get_error_deg(void);
float arm_pos_get_output_rpm(void);
float arm_pos_get_kp(void);
float arm_pos_get_ki(void);
float arm_pos_get_kd(void);
void  arm_pos_set_kp(float kp);
void  arm_pos_set_ki(float ki);
void  arm_pos_set_kd(float kd);
