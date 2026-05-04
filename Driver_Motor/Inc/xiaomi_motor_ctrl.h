#ifndef R1_SUPERSTRUCTURE_XIAOMI_MOTOR_CTRL_H
#define R1_SUPERSTRUCTURE_XIAOMI_MOTOR_CTRL_H

#include <stdbool.h>
#include "bsp_can.h"

typedef enum {
    XIAOMI_Motor1 = 0,
    XIAOMI_Motor2,
    XIAOMI_MOTOR_COUNT
} xiaomi_motor_num_e;

typedef enum {
    XIAOMI_Motor1_CAN_ID = 0x0A,
    XIAOMI_Motor2_CAN_ID = 0x02,
} xiaomi_motor_can_id_e;

typedef enum {
    XIAOMI_Motor1_FEEDBACK_ID = XIAOMI_Motor1_CAN_ID,
    XIAOMI_Motor2_FEEDBACK_ID = XIAOMI_Motor2_CAN_ID,
} xiaomi_motor_feedback_id_e;

typedef enum {
    XIAOMI_MASTER_CAN_ID = 0x00,
} xiaomi_motor_master_id_e;

typedef enum {
    XIAOMI_MODE_MOTION = 0,
    XIAOMI_MODE_POSITION = 1,
    XIAOMI_MODE_SPEED = 2,
    XIAOMI_MODE_CURRENT = 3,
} Xiaomi_Run_Mode_e;

typedef struct {
    float angle;
    float speed;
    float torque;
    float temp;
    uint8_t error_code;
    bool online;
} Xiaomi_Motor_Feedback_t;

typedef struct {
    Xiaomi_Run_Mode_e run_mode;
    Xiaomi_Run_Mode_e applied_mode;
    uint8_t mode_configured;
    float torque_set;
    float pos_set;
    float speed_set;
    float current_set;
    float kp_set;
    float kd_set;
    float speed_limit;
    uint8_t pending_cycles;
} Xiaomi_Motor_Control_t;

typedef struct {
    hcan_t *hcan;
    uint8_t can_id;
    uint8_t feedback_id;
    uint8_t master_id;
    Xiaomi_Motor_Control_t ctrl;
    Xiaomi_Motor_Feedback_t feedback;
} Xiaomi_Motor_t;

extern Xiaomi_Motor_t g_xiaomi_motor_registry[XIAOMI_MOTOR_COUNT];

void xiaomi_motor_init(void);
void xiaomi_motor_enable(Xiaomi_Motor_t *motor);
void xiaomi_motor_stop(Xiaomi_Motor_t *motor, uint8_t clear_error);
void xiaomi_motor_set_zero(Xiaomi_Motor_t *motor);
void xiaomi_motor_ctrl_send(Xiaomi_Motor_t *motor);
void xiaomi_motor_update_feedback(Xiaomi_Motor_t *motor, const uint8_t data[8], uint32_t identifier);
uint8_t xiaomi_motor_extract_feedback_id(uint32_t identifier);
uint8_t xiaomi_motor_extract_comm_type(uint32_t identifier);

#endif /* R1_SUPERSTRUCTURE_XIAOMI_MOTOR_CTRL_H */
