#include "xiaomi_motor_ctrl.h"

#include <math.h>
#include <string.h>
#include "cmsis_os2.h"
#include "fdcan.h"

#define XIAOMI_PI 3.14159265358979323846f

enum {
    XIAOMI_COMM_GET_ID = 0x00,
    XIAOMI_COMM_MOTION_CONTROL = 0x01,
    XIAOMI_COMM_MOTOR_REQUEST = 0x02,
    XIAOMI_COMM_MOTOR_ENABLE = 0x03,
    XIAOMI_COMM_MOTOR_STOP = 0x04,
    XIAOMI_COMM_SET_POS_ZERO = 0x06,
    XIAOMI_COMM_SET_CAN_ID = 0x07,
    XIAOMI_COMM_GET_SINGLE_PARAMETER = 0x11,
    XIAOMI_COMM_SET_SINGLE_PARAMETER = 0x12,
    XIAOMI_COMM_ERROR_FEEDBACK = 0x15,
};

enum {
    XIAOMI_PARAM_RUN_MODE = 0x7005,
    XIAOMI_PARAM_IQ_REF = 0x7006,
    XIAOMI_PARAM_SPD_REF = 0x700A,
    XIAOMI_PARAM_LOC_REF = 0x7016,
    XIAOMI_PARAM_LIMIT_SPD = 0x7017,
};

#define XIAOMI_MAX_POS_DEG 720.0f
#define XIAOMI_MIN_POS_DEG (-720.0f)
#define XIAOMI_MAX_SPEED_RAD 30.0f
#define XIAOMI_MIN_SPEED_RAD (-30.0f)
#define XIAOMI_MAX_TORQUE 12.0f
#define XIAOMI_MIN_TORQUE (-12.0f)
#define XIAOMI_MAX_KP 500.0f
#define XIAOMI_MIN_KP 0.0f
#define XIAOMI_MAX_KD 5.0f
#define XIAOMI_MIN_KD 0.0f
#define XIAOMI_TEMP_GAIN 0.1f
#define XIAOMI_FEEDBACK_ERROR_MASK 0x3FU
#define XIAOMI_FEEDBACK_ERROR_SHIFT 16U
#define XIAOMI_FEEDBACK_MODE_MASK 0x03U
#define XIAOMI_FEEDBACK_MODE_SHIFT 22U

static uint16_t xiaomi_float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;

    if (x > x_max) {
        x = x_max;
    } else if (x < x_min) {
        x = x_min;
    }

    return (uint16_t)((x - offset) * ((float)((1U << bits) - 1U)) / span);
}

static float xiaomi_uint_to_float(uint16_t x, float x_min, float x_max, int bits) {
    uint32_t span = (1UL << bits) - 1UL;
    float offset = x_max - x_min;
    return offset * (float)x / (float)span + x_min;
}

static uint32_t xiaomi_build_ext_id(uint8_t comm_type, uint16_t data_field, uint8_t master_id, uint8_t can_id) {
    return ((uint32_t)comm_type << 24) |
           ((uint32_t)data_field << 8) |
           ((uint32_t)master_id << 8) |
           (uint32_t)can_id;
}

static void xiaomi_pack_float_le(float value, uint8_t *dst) {
    union {
        float f;
        uint8_t b[4];
    } raw;

    raw.f = value;
    dst[0] = raw.b[0];
    dst[1] = raw.b[1];
    dst[2] = raw.b[2];
    dst[3] = raw.b[3];
}

static void xiaomi_send_ext(Xiaomi_Motor_t *motor, uint32_t identifier, uint8_t *data, uint32_t len) {
    if (motor == NULL || motor->hcan == NULL) {
        return;
    }

    fdcanx_send_ex_data(motor->hcan, identifier, data, len, CAN_ID_EXT);
}

static void xiaomi_set_mode(Xiaomi_Motor_t *motor, Xiaomi_Run_Mode_e mode) {
    uint8_t data[8] = {0};
    uint32_t identifier;

    if (motor == NULL) {
        return;
    }

    identifier = xiaomi_build_ext_id(XIAOMI_COMM_SET_SINGLE_PARAMETER, 0, motor->master_id, motor->can_id);
    data[0] = (uint8_t)(XIAOMI_PARAM_RUN_MODE & 0xFF);
    data[1] = (uint8_t)((XIAOMI_PARAM_RUN_MODE >> 8) & 0xFF);
    data[4] = (uint8_t)mode;
    xiaomi_send_ext(motor, identifier, data, sizeof(data));
}

static void xiaomi_set_parameter_float(Xiaomi_Motor_t *motor, uint16_t index, float value) {
    uint8_t data[8] = {0};
    uint32_t identifier;

    if (motor == NULL) {
        return;
    }

    identifier = xiaomi_build_ext_id(XIAOMI_COMM_SET_SINGLE_PARAMETER, 0, motor->master_id, motor->can_id);
    data[0] = (uint8_t)(index & 0xFF);
    data[1] = (uint8_t)((index >> 8) & 0xFF);
    xiaomi_pack_float_le(value, &data[4]);
    xiaomi_send_ext(motor, identifier, data, sizeof(data));
}

static void xiaomi_send_motion_control(Xiaomi_Motor_t *motor) {
    uint32_t identifier;
    uint8_t data[8];
    float pos_deg;
    uint16_t torque_u16;
    uint16_t pos_u16;
    uint16_t speed_u16;
    uint16_t kp_u16;
    uint16_t kd_u16;

    if (motor == NULL) {
        return;
    }

    pos_deg = motor->ctrl.pos_set * 180.0f / XIAOMI_PI;
    torque_u16 = xiaomi_float_to_uint(motor->ctrl.torque_set, XIAOMI_MIN_TORQUE, XIAOMI_MAX_TORQUE, 16);
    pos_u16 = xiaomi_float_to_uint(pos_deg, XIAOMI_MIN_POS_DEG, XIAOMI_MAX_POS_DEG, 16);
    speed_u16 = xiaomi_float_to_uint(motor->ctrl.speed_set, XIAOMI_MIN_SPEED_RAD, XIAOMI_MAX_SPEED_RAD, 16);
    kp_u16 = xiaomi_float_to_uint(motor->ctrl.kp_set, XIAOMI_MIN_KP, XIAOMI_MAX_KP, 16);
    kd_u16 = xiaomi_float_to_uint(motor->ctrl.kd_set, XIAOMI_MIN_KD, XIAOMI_MAX_KD, 16);

    identifier = ((uint32_t)XIAOMI_COMM_MOTION_CONTROL << 24) |
                 ((uint32_t)torque_u16 << 8) |
                 (uint32_t)motor->can_id;

    data[0] = (uint8_t)(pos_u16 >> 8);
    data[1] = (uint8_t)(pos_u16 & 0xFF);
    data[2] = (uint8_t)(speed_u16 >> 8);
    data[3] = (uint8_t)(speed_u16 & 0xFF);
    data[4] = (uint8_t)(kp_u16 >> 8);
    data[5] = (uint8_t)(kp_u16 & 0xFF);
    data[6] = (uint8_t)(kd_u16 >> 8);
    data[7] = (uint8_t)(kd_u16 & 0xFF);

    xiaomi_send_ext(motor, identifier, data, sizeof(data));
}

Xiaomi_Motor_t g_xiaomi_motor_registry[XIAOMI_MOTOR_COUNT] = {
    [XIAOMI_Motor1] = {
        .hcan = &hfdcan2,
        .can_id = XIAOMI_Motor1_CAN_ID,
        .feedback_id = XIAOMI_Motor1_FEEDBACK_ID,
        .master_id = XIAOMI_MASTER_CAN_ID,
    },
    [XIAOMI_Motor2] = {
        .hcan = &hfdcan2,
        .can_id = XIAOMI_Motor2_CAN_ID,
        .feedback_id = XIAOMI_Motor2_FEEDBACK_ID,
        .master_id = XIAOMI_MASTER_CAN_ID,
    },
};

void xiaomi_motor_init(void) {
    for (int i = 0; i < XIAOMI_MOTOR_COUNT; ++i) {
        Xiaomi_Motor_t *motor = &g_xiaomi_motor_registry[i];

        memset(&motor->feedback, 0, sizeof(motor->feedback));
        memset(&motor->ctrl, 0, sizeof(motor->ctrl));
        motor->ctrl.applied_mode = (Xiaomi_Run_Mode_e)0xFF;
        motor->ctrl.kp_set = 100.0f;
        motor->ctrl.kd_set = 5.0f;
        motor->ctrl.speed_limit = 5.0f;
        motor->ctrl.pending_cycles = 0;
    }
}

void xiaomi_motor_enable(Xiaomi_Motor_t *motor) {
    uint8_t data[8] = {0};

    if (motor == NULL) {
        return;
    }

    xiaomi_send_ext(motor,
                    xiaomi_build_ext_id(XIAOMI_COMM_MOTOR_ENABLE, 0, motor->master_id, motor->can_id),
                    data,
                    sizeof(data));
}

void xiaomi_motor_stop(Xiaomi_Motor_t *motor, uint8_t clear_error) {
    uint8_t data[8] = {0};

    if (motor == NULL) {
        return;
    }

    data[0] = clear_error;
    xiaomi_send_ext(motor,
                    xiaomi_build_ext_id(XIAOMI_COMM_MOTOR_STOP, 0, motor->master_id, motor->can_id),
                    data,
                    sizeof(data));
}

void xiaomi_motor_set_zero(Xiaomi_Motor_t *motor) {
    uint8_t data[8] = {0};

    if (motor == NULL) {
        return;
    }

    data[0] = 0x01;
    xiaomi_send_ext(motor,
                    xiaomi_build_ext_id(XIAOMI_COMM_SET_POS_ZERO, 0, motor->master_id, motor->can_id),
                    data,
                    sizeof(data));
}

void xiaomi_motor_ctrl_send(Xiaomi_Motor_t *motor) {
    if (motor == NULL) {
        return;
    }

    if (motor->ctrl.mode_configured == 0U) {
        return;
    }

    if (motor->ctrl.applied_mode != motor->ctrl.run_mode) {
        xiaomi_set_mode(motor, motor->ctrl.run_mode);
        motor->ctrl.applied_mode = motor->ctrl.run_mode;
        if (motor->ctrl.pending_cycles < 3U) {
            motor->ctrl.pending_cycles = 3U;
        }
    }

    switch (motor->ctrl.run_mode) {
        case XIAOMI_MODE_MOTION:
            xiaomi_send_motion_control(motor);
            break;

        case XIAOMI_MODE_POSITION:
            if (motor->ctrl.pending_cycles > 0U) {
                xiaomi_set_parameter_float(motor, XIAOMI_PARAM_LIMIT_SPD, motor->ctrl.speed_limit);
                xiaomi_set_parameter_float(motor, XIAOMI_PARAM_LOC_REF, motor->ctrl.pos_set);
                motor->ctrl.pending_cycles--;
            }
            break;

        case XIAOMI_MODE_SPEED:
            if (motor->ctrl.pending_cycles > 0U) {
                xiaomi_set_parameter_float(motor, XIAOMI_PARAM_SPD_REF, motor->ctrl.speed_set);
                motor->ctrl.pending_cycles--;
            }
            break;

        case XIAOMI_MODE_CURRENT:
            if (motor->ctrl.pending_cycles > 0U) {
                xiaomi_set_parameter_float(motor, XIAOMI_PARAM_IQ_REF, motor->ctrl.current_set);
                motor->ctrl.pending_cycles--;
            }
            break;
    }
}

void xiaomi_motor_update_feedback(Xiaomi_Motor_t *motor, const uint8_t data[8], uint32_t identifier) {
    uint16_t angle_raw;
    uint16_t speed_raw;
    uint16_t torque_raw;
    uint8_t comm_type;

    if (motor == NULL || data == NULL) {
        return;
    }

    comm_type = xiaomi_motor_extract_comm_type(identifier);
    if (comm_type == XIAOMI_COMM_ERROR_FEEDBACK) {
        motor->feedback.fault_code = (uint32_t)data[0] |
                                     ((uint32_t)data[1] << 8) |
                                     ((uint32_t)data[2] << 16) |
                                     ((uint32_t)data[3] << 24);
        motor->feedback.warning_code = (uint32_t)data[4] |
                                       ((uint32_t)data[5] << 8) |
                                       ((uint32_t)data[6] << 16) |
                                       ((uint32_t)data[7] << 24);
        motor->feedback.error_code = (uint8_t)(motor->feedback.fault_code & 0xFFU);
        motor->feedback.online = true;
        return;
    }

    if (comm_type != XIAOMI_COMM_MOTOR_REQUEST) {
        return;
    }

    angle_raw = (uint16_t)((data[0] << 8) | data[1]);
    speed_raw = (uint16_t)((data[2] << 8) | data[3]);
    torque_raw = (uint16_t)((data[4] << 8) | data[5]);

    motor->feedback.angle = xiaomi_uint_to_float(angle_raw, XIAOMI_MIN_POS_DEG, XIAOMI_MAX_POS_DEG, 16) * XIAOMI_PI / 180.0f;
    motor->feedback.speed = xiaomi_uint_to_float(speed_raw, XIAOMI_MIN_SPEED_RAD, XIAOMI_MAX_SPEED_RAD, 16);
    motor->feedback.torque = xiaomi_uint_to_float(torque_raw, XIAOMI_MIN_TORQUE, XIAOMI_MAX_TORQUE, 16);
    motor->feedback.temp = (float)((data[6] << 8) | data[7]) * XIAOMI_TEMP_GAIN;
    motor->feedback.error_code = (uint8_t)((identifier >> XIAOMI_FEEDBACK_ERROR_SHIFT) & XIAOMI_FEEDBACK_ERROR_MASK);
    motor->feedback.mode_state = (uint8_t)((identifier >> XIAOMI_FEEDBACK_MODE_SHIFT) & XIAOMI_FEEDBACK_MODE_MASK);
    motor->feedback.online = true;
}

uint8_t xiaomi_motor_extract_feedback_id(uint32_t identifier) {
    return (uint8_t)((identifier & 0xFFFFUL) >> 8);
}

uint8_t xiaomi_motor_extract_target_id(uint32_t identifier) {
    return (uint8_t)(identifier & 0xFFUL);
}

uint8_t xiaomi_motor_extract_comm_type(uint32_t identifier) {
    return (uint8_t)((identifier >> 24) & 0x1FUL);
}
