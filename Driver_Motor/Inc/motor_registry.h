#ifndef R1_SUPERSTRUCTURE_MOTOR_REGISTRY_H
#define R1_SUPERSTRUCTURE_MOTOR_REGISTRY_H

#include "main.h"
#include "dji_3508_2006_motor.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"
#include "xiaomi_motor_ctrl.h"
#include "unitree_go_m8010_6_motor.h"
#include "blazer_foc_motor.h"

#define MOTOR_TOTAL_NUM (DJI_MOTOR_COUNT + DM_MOTOR_COUNT + XIAOMI_MOTOR_COUNT + UNITREE_GO_M8010_6_MOTOR_COUNT + BLAZER_FOC_MOTOR_COUNT)

typedef enum {
    MOTOR_TYPE_DJI = 0,
    MOTOR_TYPE_DAMIAO,
    MOTOR_TYPE_XIAOMI,
    MOTOR_TYPE_UNITREE_GO_M8010_6,
    MOTOR_TYPE_BLAZER_FOC,
} Motor_Type_e;

typedef struct {
    float angle;
    float speed;
    float torque;
    float temp;
} Motor_State_t;

typedef struct {
    float position_offset;
    float position_amplitude;
    float period_s;
    float phase_rad;
    uint32_t start_tick_ms;
} Motor_Sine_Profile_t;

typedef struct {
    float start_position;
    float target_position;
    float max_speed;
    float duration_s;
    uint32_t start_tick_ms;
    uint8_t active;
} Motor_Smooth_Goto_Profile_t;

typedef struct Motor_Class {
    Motor_Type_e type;
    void *instance;

    void (*init)(struct Motor_Class *self);
    void (*enable)(struct Motor_Class *self);
    void (*stop)(struct Motor_Class *self, uint8_t clear_error);
    void (*set_zero)(struct Motor_Class *self);
    void (*set_speed)(struct Motor_Class *self, float speed);
    void (*set_position)(struct Motor_Class *self, float position, float vel_limit);
    void (*set_mit)(struct Motor_Class *self, float position, float speed, float kp, float kd, float torque);
    void (*set_psi)(struct Motor_Class *self, float position, float speed, float current);
    void (*update_feedback)(struct Motor_Class *self, uint8_t *rx_data, uint32_t identifier);
    Motor_State_t (*get_state)(struct Motor_Class *self);
} Motor_Class_t;

void Motor_Registry_Init(void);
void Motor_All_Control_Loop(void);
void Motor_Feedback_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data);
void Motor_Enable(int motor_index);
void Motor_Stop(int motor_index, uint8_t clear_error);
void Motor_SetZero(int motor_index);
void Motor_SetMIT(int motor_index, float position, float speed, float kp, float kd, float torque);
void Motor_SetPSI(int motor_index, float position, float speed, float current);
void Motor_SineProfile_Init(Motor_Sine_Profile_t *profile,
                            float position_offset,
                            float position_amplitude,
                            float period_s,
                            float phase_rad);
void Motor_SineProfile_Reset(Motor_Sine_Profile_t *profile);
void Motor_SineProfile_Eval(const Motor_Sine_Profile_t *profile, float *position, float *speed);
void Motor_RunSineMIT(int motor_index,
                      const Motor_Sine_Profile_t *profile,
                      float kp,
                      float kd,
                      float torque_ff);
void Motor_SmoothGoto_Start(Motor_Smooth_Goto_Profile_t *profile,
                            int motor_index,
                            float target_position,
                            float max_speed);
void Motor_SmoothGoto_Reset(Motor_Smooth_Goto_Profile_t *profile);
uint8_t Motor_SmoothGoto_Eval(const Motor_Smooth_Goto_Profile_t *profile, float *position, float *speed);
uint8_t Motor_RunSmoothGotoMIT(int motor_index,
                               Motor_Smooth_Goto_Profile_t *profile,
                               float kp,
                               float kd,
                               float torque_ff);

extern Motor_Class_t g_motor_list[MOTOR_TOTAL_NUM];

#endif /* R1_SUPERSTRUCTURE_MOTOR_REGISTRY_H */
