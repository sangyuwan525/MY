#include "blazer_foc_motor.h"

#include <string.h>
#include "bsp_can.h"
#include "fdcan.h"

#define BLAZER_FOC_DEFAULT_CAN (&hfdcan3)
#define BLAZER_FOC_READ_PARAM_COUNT 5U
#define BLAZER_FOC_RPM_PER_RPS 60.0f

/*
 * Blazer FOC device registry.
 *
 * The default entries assume:
 *   - ESC CAN node IDs = 0..3
 *   - ESCs are wired to FDCAN1
 *
 * If the OLED/USB menu changes the ESC CAN ID, update node_id here too.
 * If the ESC is moved to another CAN bus, update hcan here.
 */
Blazer_FOC_Motor_t g_blazer_foc_motor_registry[BLAZER_FOC_MOTOR_COUNT] = {
    [BLAZER_FOC_MOTOR1] = {
        .hcan = &hfdcan1,
        .node_id = 0x00U,
    },
    [BLAZER_FOC_MOTOR2] = {
        .hcan = &hfdcan1,
        .node_id = 0x01U,
    },
    [BLAZER_FOC_MOTOR3] = {
        .hcan = &hfdcan1,
        .node_id = 0x02U,
    },
    [BLAZER_FOC_MOTOR4] = {
        .hcan = &hfdcan1,
        .node_id = 0x03U,
    },
};

/*
 * Feedback polling list.
 *
 * Blazer FOC does not push all state automatically; the master reads one
 * parameter by sending the odd ID (param_id + 1), then the ESC replies with
 * the same odd ID and 4 bytes of float data. Polling one item per 1 ms control
 * loop keeps bus load small while still refreshing useful state.
 */
static const Blazer_FOC_ParamID_e k_blazer_foc_read_params[BLAZER_FOC_READ_PARAM_COUNT] = {
    BLAZER_FOC_PARAM_SPD_FILT,
    BLAZER_FOC_PARAM_IQ,
    BLAZER_FOC_PARAM_TEMP,
    BLAZER_FOC_PARAM_ERROR,
    BLAZER_FOC_PARAM_ENC_RAW,
};

/* Manual protocol: extended CAN ID = node_id << 8 | param_id. */
uint32_t Blazer_FOC_MakeID(uint8_t node_id, Blazer_FOC_ParamID_e param_id) {
    return (((uint32_t)node_id) << 8) | ((uint32_t)param_id);
}

/*
 * Keep the exact IEEE-754 float bit pattern.
 * Do not cast float to uint32_t numerically; that would destroy decimals and
 * signs. memcpy is used to avoid strict-aliasing trouble on embedded compilers.
 */
static uint32_t Blazer_FOC_FloatToBits(float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static float Blazer_FOC_BitsToFloat(uint32_t bits) {
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/*
 * The manual example turns float bits into uint8_t bytes by shifting the
 * uint32_t value. That yields high byte first on the CAN bus:
 * data[0] = bits[31:24], ... data[3] = bits[7:0].
 */
static void Blazer_FOC_PackFloat(float value, uint8_t data[4]) {
    uint32_t bits = Blazer_FOC_FloatToBits(value);

    data[0] = (uint8_t)((bits >> 24) & 0xFFU);
    data[1] = (uint8_t)((bits >> 16) & 0xFFU);
    data[2] = (uint8_t)((bits >> 8) & 0xFFU);
    data[3] = (uint8_t)(bits & 0xFFU);
}

static float Blazer_FOC_UnpackFloat(const uint8_t data[4]) {
    uint32_t bits = ((uint32_t)data[0] << 24) |
                    ((uint32_t)data[1] << 16) |
                    ((uint32_t)data[2] << 8) |
                    ((uint32_t)data[3]);

    return Blazer_FOC_BitsToFloat(bits);
}

/* Write a Blazer FOC parameter. Only even parameter IDs are writable. */
static uint8_t Blazer_FOC_WriteParam(Blazer_FOC_Motor_t *motor, Blazer_FOC_ParamID_e param_id, float value) {
    uint8_t data[4];

    if (motor == NULL || motor->hcan == NULL) {
        return 1U;
    }

    Blazer_FOC_PackFloat(value, data);
    return fdcanx_send_ex_data(motor->hcan, Blazer_FOC_MakeID(motor->node_id, param_id), data, 4U, CAN_ID_EXT);
}

/* Request a Blazer FOC parameter. Read requests use param_id + 1. */
static void Blazer_FOC_ReadParam(Blazer_FOC_Motor_t *motor, Blazer_FOC_ParamID_e param_id) {
    uint8_t data[4] = {0};

    if (motor == NULL || motor->hcan == NULL) {
        return;
    }

    (void)fdcanx_send_ex_data(motor->hcan,
                              Blazer_FOC_MakeID(motor->node_id, (Blazer_FOC_ParamID_e)((uint8_t)param_id + 1U)),
                              data,
                              4U,
                              CAN_ID_EXT);
}

void Blazer_FOC_Motor_Init(void) {
    for (uint32_t i = 0; i < BLAZER_FOC_MOTOR_COUNT; ++i) {
        Blazer_FOC_Motor_t *motor = &g_blazer_foc_motor_registry[i];

        memset(&motor->ctrl, 0, sizeof(motor->ctrl));
        memset(&motor->feedback, 0, sizeof(motor->feedback));
        motor->ctrl.mode = BLAZER_FOC_MODE_DISABLE;
        motor->ctrl.mode_pending = 1U;
        motor->ctrl.enabled = 1U;
        motor->read_cursor = 0U;
    }
}

/* Change only the local desired mode; Blazer_FOC_Control_Send() sends it. */
void Blazer_FOC_SetMode(Blazer_FOC_Motor_t *motor, Blazer_FOC_Mode_e mode) {
    if (motor == NULL) {
        return;
    }

    if (motor->ctrl.mode != mode) {
        motor->ctrl.mode = mode;
        motor->ctrl.mode_pending = 1U;
    }
}

/*
 * Firmware speed command in RPM, matching DJI's external API.
 * Blazer's CAN protocol uses mechanical revolutions per second, so the
 * conversion happens when sending/receiving protocol parameters.
 */
void Blazer_FOC_SetSpeed(Blazer_FOC_Motor_t *motor, float speed_rpm) {
    if (motor == NULL) {
        return;
    }

    motor->ctrl.speed_set = speed_rpm;
    motor->ctrl.setpoint_pending = 1U;
    Blazer_FOC_SetMode(motor, BLAZER_FOC_MODE_SPEED);
}

/* Current command in amperes; it selects current mode. */
void Blazer_FOC_SetCurrent(Blazer_FOC_Motor_t *motor, float current_a) {
    if (motor == NULL) {
        return;
    }

    motor->ctrl.current_set = current_a;
    motor->ctrl.setpoint_pending = 1U;
    Blazer_FOC_SetMode(motor, BLAZER_FOC_MODE_CURRENT);
}

/* Position command in mechanical revolutions; it selects position mode. */
void Blazer_FOC_SetPosition(Blazer_FOC_Motor_t *motor, float position_rev) {
    if (motor == NULL) {
        return;
    }

    motor->ctrl.position_set = position_rev;
    motor->ctrl.setpoint_pending = 1U;
    Blazer_FOC_SetMode(motor, BLAZER_FOC_MODE_POSITION);
}

/* Release the motor by switching Blazer FOC back to mode 0. */
void Blazer_FOC_Stop(Blazer_FOC_Motor_t *motor) {
    if (motor == NULL) {
        return;
    }

    motor->ctrl.speed_set = 0.0f;
    motor->ctrl.current_set = 0.0f;
    motor->ctrl.setpoint_pending = 1U;
    Blazer_FOC_SetMode(motor, BLAZER_FOC_MODE_DISABLE);
}

/*
 * Called by Motor_All_Control_Loop().
 *
 * For speed mode we resend spd_set every cycle. The manual states can_hb is a
 * heartbeat timeout in modes 1 and 2, so periodic setpoint refresh doubles as
 * the heartbeat that keeps the ESC enabled.
 */
void Blazer_FOC_Control_Send(Blazer_FOC_Motor_t *motor) {
    Blazer_FOC_ParamID_e read_param;

    if (motor == NULL || motor->ctrl.enabled == 0U) {
        return;
    }

    if (motor->ctrl.mode_pending != 0U) {
        if (Blazer_FOC_WriteParam(motor, BLAZER_FOC_PARAM_MODE, (float)motor->ctrl.mode) != 0U) {
            return;
        }
        motor->ctrl.mode_pending = 0U;
    }

    if (motor->ctrl.mode == BLAZER_FOC_MODE_SPEED) {
        if (Blazer_FOC_WriteParam(motor,
                                  BLAZER_FOC_PARAM_SPD_SET,
                                  motor->ctrl.speed_set / BLAZER_FOC_RPM_PER_RPS) != 0U) {
            return;
        }
        motor->ctrl.setpoint_pending = 0U;
    } else if (motor->ctrl.setpoint_pending != 0U && motor->ctrl.mode == BLAZER_FOC_MODE_CURRENT) {
        if (Blazer_FOC_WriteParam(motor, BLAZER_FOC_PARAM_I_SET, motor->ctrl.current_set) != 0U) {
            return;
        }
        motor->ctrl.setpoint_pending = 0U;
    } else if (motor->ctrl.setpoint_pending != 0U && motor->ctrl.mode == BLAZER_FOC_MODE_POSITION) {
        if (Blazer_FOC_WriteParam(motor, BLAZER_FOC_PARAM_POS_SET, motor->ctrl.position_set) != 0U) {
            return;
        }
        motor->ctrl.setpoint_pending = 0U;
    }

    read_param = k_blazer_foc_read_params[motor->read_cursor];
    Blazer_FOC_ReadParam(motor, read_param);
    motor->read_cursor = (uint8_t)((motor->read_cursor + 1U) % BLAZER_FOC_READ_PARAM_COUNT);
}

/*
 * Match only Blazer read replies for this registered node and CAN bus.
 * The queue layer already filtered likely motor frames; this is the final
 * per-device guard before parsing data.
 */
uint8_t Blazer_FOC_Match_Feedback(const Blazer_FOC_Motor_t *motor, FDCAN_HandleTypeDef *hfdcan, uint32_t identifier) {
    uint8_t node_id;
    uint8_t param_id;

    if (motor == NULL || motor->hcan != hfdcan) {
        return 0U;
    }

    node_id = (uint8_t)((identifier >> 8) & 0xFFU);
    param_id = (uint8_t)(identifier & 0xFFU);

    return (node_id == motor->node_id &&
            param_id <= ((uint8_t)BLAZER_FOC_PARAM_ERROR + 1U) &&
            (param_id & 0x01U) != 0U) ? 1U : 0U;
}

/* Parse a read reply and cache the state in engineering units. */
void Blazer_FOC_Update_Feedback(Blazer_FOC_Motor_t *motor, uint32_t identifier, const uint8_t data[4]) {
    uint8_t param_id;
    float value;

    if (motor == NULL || data == NULL) {
        return;
    }

    param_id = (uint8_t)(identifier & 0xFEU);
    value = Blazer_FOC_UnpackFloat(data);

    switch (param_id) {
        case BLAZER_FOC_PARAM_VBUS:
            motor->feedback.vbus = value;
            break;
        case BLAZER_FOC_PARAM_IBUS:
            motor->feedback.ibus = value;
            break;
        case BLAZER_FOC_PARAM_IQ:
            motor->feedback.iq = value;
            break;
        case BLAZER_FOC_PARAM_SPD_FILT:
            motor->feedback.speed = value * BLAZER_FOC_RPM_PER_RPS;
            break;
        case BLAZER_FOC_PARAM_ENC_RAW:
            motor->feedback.enc_raw = value;
            break;
        case BLAZER_FOC_PARAM_TEMP:
            motor->feedback.temp = value;
            break;
        case BLAZER_FOC_PARAM_ERROR:
            motor->feedback.error = value;
            break;
        default:
            break;
    }

    motor->feedback.last_rx_tick_ms = HAL_GetTick();
    motor->feedback.online = 1U;
}
