#include "unitree_go_m8010_6_motor.h"

#include <math.h>
#include <string.h>

#define UNITREE_GO_HEAD_0 0xFEU
#define UNITREE_GO_HEAD_1 0xEEU
#define UNITREE_GO_FB_HEAD_0 0xFDU
#define UNITREE_GO_FB_HEAD_1 0xEEU
#define UNITREE_GO_DEFAULT_GEAR_RATIO 6.33f
#define UNITREE_GO_DEFAULT_KD 0.01f
#define UNITREE_GO_DEFAULT_KP 0.05f
#define UNITREE_GO_TWO_PI 6.28318530717958647692f
#define UNITREE_GO_ZERO_CALIBRATE_CYCLES 50U

/*
 * 默认注册 1 台 GO-M8010-6：
 * UART4 -> RS485 收发器 -> 485 总线 -> 电机 ID 0。
 * 如果硬件接到了其它串口，改 huart。
 */
Unitree_GO_M8010_6_Motor_t g_unitree_go_m8010_6_motor_registry[UNITREE_GO_M8010_6_MOTOR_COUNT] = {
    [UNITREE_GO_M8010_6_Motor1] = {
        .huart = &huart4,
        .id = UNITREE_GO_M8010_6_Motor1_ID,
        .gear_ratio = UNITREE_GO_DEFAULT_GEAR_RATIO,
    },
    [UNITREE_GO_M8010_6_Motor2] = {
        .huart = &huart4,
        .id = UNITREE_GO_M8010_6_Motor2_ID,
        .gear_ratio = UNITREE_GO_DEFAULT_GEAR_RATIO,
    },
};

/* 限幅工具，避免打包时超过官方定点数格式允许范围。 */
static float unitree_clamp(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

/* 力矩使用 q8 格式：实际值 * 2^8。 */
static int16_t unitree_float_to_q8(float value) {
    return (int16_t)(unitree_clamp(value, -127.0f, 127.0f) * 256.0f);
}

/* 速度使用 q7 格式：实际值 * 2^7。 */
static int16_t unitree_float_to_q7(float value) {
    return (int16_t)(unitree_clamp(value, -255.0f, 255.0f) * 128.0f);
}

/* 位置使用 q15 格式：实际值 * 2^15。 */
static int32_t unitree_float_to_q15(float value) {
    return (int32_t)(unitree_clamp(value, -65535.0f, 65535.0f) * 32768.0f);
}

/* kp/kd 官方范围是 0.0-1.0，这里转成 q15 无符号数。 */
static uint16_t unitree_gain_to_q15(float value) {
    return (uint16_t)(unitree_clamp(value, 0.0f, 1.0f) * 32768.0f);
}

/* 按小端格式写入 16 位数据，匹配 SDK 结构体在小端 MCU/PC 上的内存布局。 */
static void unitree_put_u16_le(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/* 按小端格式写入 32 位数据。 */
static void unitree_put_u32_le(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/* 从反馈包里按小端格式读取 16 位数据。 */
static uint16_t unitree_get_u16_le(const uint8_t *src) {
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

/* 从反馈包里按小端格式读取 32 位数据。 */
static uint32_t unitree_get_u32_le(const uint8_t *src) {
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

/*
 * CRC-CCITT 校验，和宇树 SDK 使用的 crc_ccitt 逻辑一致。
 * 控制包发送前会计算前 15 字节的 CRC，并放到最后 2 字节。
 * 反馈包解析时也会先校验 CRC，校验失败则丢弃本包。
 */
static uint16_t unitree_crc_ccitt(uint16_t crc, const uint8_t *buffer, uint32_t len) {
    while (len-- > 0U) {
        crc ^= *buffer++;
        for (uint8_t i = 0; i < 8U; ++i) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1) ^ 0x8408U;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/*
 * 生成 GO-M8010-6 官方 17 字节控制包。
 *
 * 大注册表统一使用“输出端”的位置/速度单位：
 *   pos_set   = 输出端目标角度(rad)
 *   speed_set = 输出端目标速度(rad/s)
 *
 * 宇树官方例程里会把速度乘 queryGearRatio(GO_M8010_6)，说明协议包里使用的是
 * 电机端单位。因此这里统一在打包时乘 gear_ratio，调用者不要再手动乘减速比。
 *
 * 包格式来自 motor_msg_GO-M8010-6.h：
 *   head[2] + mode[1] + tor_des[2] + spd_des[2] + pos_des[4]
 *   + k_pos[2] + k_spd[2] + CRC16[2]
 */
static void unitree_go_build_packet(Unitree_GO_M8010_6_Motor_t *motor, uint8_t packet[UNITREE_GO_M8010_6_PACKET_LEN]) {
    float gear_ratio;
    float motor_speed_rev_s;
    float motor_position_rev;
    int16_t torque_q8;
    int16_t speed_q7;
    int32_t position_q15;
    uint16_t kp_q15;
    uint16_t kd_q15;
    uint16_t crc;

    memset(packet, 0, UNITREE_GO_M8010_6_PACKET_LEN);

    gear_ratio = (motor->gear_ratio > 0.0f) ? motor->gear_ratio : 1.0f;
    motor_speed_rev_s = motor->ctrl.speed_set * gear_ratio / UNITREE_GO_TWO_PI;
    motor_position_rev = motor->ctrl.pos_set * gear_ratio / UNITREE_GO_TWO_PI;
    torque_q8 = unitree_float_to_q8(motor->ctrl.torque_set);
    speed_q7 = unitree_float_to_q7(motor_speed_rev_s);
    position_q15 = unitree_float_to_q15(motor_position_rev);
    kp_q15 = unitree_gain_to_q15(motor->ctrl.kp_set);
    kd_q15 = unitree_gain_to_q15(motor->ctrl.kd_set);

    packet[0] = UNITREE_GO_HEAD_0;
    packet[1] = UNITREE_GO_HEAD_1;
    packet[2] = (uint8_t)((motor->id & 0x0FU) | (((uint8_t)motor->ctrl.mode & 0x07U) << 4));
    unitree_put_u16_le(&packet[3], (uint16_t)torque_q8);
    unitree_put_u16_le(&packet[5], (uint16_t)speed_q7);
    unitree_put_u32_le(&packet[7], (uint32_t)position_q15);
    unitree_put_u16_le(&packet[11], kp_q15);
    unitree_put_u16_le(&packet[13], kd_q15);

    crc = unitree_crc_ccitt(0, packet, UNITREE_GO_M8010_6_PACKET_LEN - 2U);
    unitree_put_u16_le(&packet[15], crc);
}

/*
 * 通过串口/RS485 发送一个完整宇树控制包。
 *
 * GO-M8010-6 官方例程就是直接通过串口发送 17 字节包。
 * 这里把发送函数做成 weak，后续如果你的 RS485 需要 DE/RE 方向控制，
 * 可以在别的文件里重新实现同名函数，先拉高发送使能，再 HAL_UART_Transmit。
 */
__weak uint8_t unitree_go_m8010_6_transport_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len) {
    if (huart == NULL || data == NULL) {
        return 1U;
    }

    return (HAL_UART_Transmit(huart, (uint8_t *)data, len, 2U) == HAL_OK) ? 0U : 1U;
}

/*
 * 初始化宇树电机运行时状态。
 *
 * 这里只清空控制量/反馈量，并设置一组保守的默认 FOC 增益。
 * 不会立即发送控制包；只有调用大注册表 set_speed/set_position/set_mit/set_psi
 * 设置目标后，mode_configured 才会置位，周期控制函数才会开始发包。
 */
void unitree_go_m8010_6_motor_init(void) {
    for (int i = 0; i < UNITREE_GO_M8010_6_MOTOR_COUNT; ++i) {
        Unitree_GO_M8010_6_Motor_t *motor = &g_unitree_go_m8010_6_motor_registry[i];

        memset(&motor->ctrl, 0, sizeof(motor->ctrl));
        memset(&motor->feedback, 0, sizeof(motor->feedback));
        motor->ctrl.mode = UNITREE_GO_M8010_6_MODE_FOC;
        motor->ctrl.kp_set = UNITREE_GO_DEFAULT_KP;
        motor->ctrl.kd_set = UNITREE_GO_DEFAULT_KD;
    }
}

/*
 * 周期控制发送函数。
 *
 * Motor_All_Control_Loop() 会周期调用这个函数。
 * 如果还没有通过注册表写入目标值，mode_configured 为 0，本函数直接返回；
 * 如果已经设置目标值，则生成 17 字节控制包，并通过串口/RS485 发出。
 */
void unitree_go_m8010_6_motor_ctrl_send(Unitree_GO_M8010_6_Motor_t *motor) {
    uint8_t packet[UNITREE_GO_M8010_6_PACKET_LEN];

    if (motor == NULL || motor->ctrl.mode_configured == 0U) {
        return;
    }

    unitree_go_build_packet(motor, packet);
    (void)unitree_go_m8010_6_transport_send(motor->huart, packet, UNITREE_GO_M8010_6_PACKET_LEN);

    if (motor->ctrl.zero_calibrate_cycles > 0U) {
        motor->ctrl.zero_calibrate_cycles--;
        if (motor->ctrl.zero_calibrate_cycles == 0U) {
            motor->ctrl.mode = UNITREE_GO_M8010_6_MODE_BRAKE;
            motor->ctrl.torque_set = 0.0f;
            motor->ctrl.speed_set = 0.0f;
            motor->ctrl.pos_set = 0.0f;
            motor->ctrl.kp_set = 0.0f;
            motor->ctrl.kd_set = 0.0f;
        }
    }
}

/*
 * 请求刹车/停止。
 *
 * 这里不会直接阻塞发送，而是把 mode 改成 BRAKE 并置位 mode_configured。
 * 随后 Motor_All_Control_Loop() 会按周期把刹车包发出去。
 */
void unitree_go_m8010_6_motor_stop(Unitree_GO_M8010_6_Motor_t *motor) {
    if (motor == NULL) {
        return;
    }

    motor->ctrl.mode = UNITREE_GO_M8010_6_MODE_BRAKE;
    motor->ctrl.mode_configured = 1U;
    motor->ctrl.torque_set = 0.0f;
    motor->ctrl.speed_set = 0.0f;
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.kp_set = 0.0f;
    motor->ctrl.kd_set = 0.0f;
    motor->ctrl.zero_calibrate_cycles = 0U;
}

void unitree_go_m8010_6_motor_set_zero(Unitree_GO_M8010_6_Motor_t *motor) {
    if (motor == NULL) {
        return;
    }

    motor->ctrl.mode = UNITREE_GO_M8010_6_MODE_CALIBRATE;
    motor->ctrl.mode_configured = 1U;
    motor->ctrl.torque_set = 0.0f;
    motor->ctrl.speed_set = 0.0f;
    motor->ctrl.pos_set = 0.0f;
    motor->ctrl.kp_set = 0.0f;
    motor->ctrl.kd_set = 0.0f;
    motor->ctrl.zero_calibrate_cycles = UNITREE_GO_ZERO_CALIBRATE_CYCLES;
}

/*
 * 解析 GO-M8010-6 反馈包。
 *
 * 反馈包长度为 16 字节。先校验 CRC，通过后再提取力矩、速度、位置、温度和错误码。
 * 协议反馈是电机端单位，写入注册表反馈前会除以 gear_ratio，转回输出端单位。
 *
 * 当前工程还没有把串口/RS485 收到的回包重新拼成 16 字节反馈包，
 * 所以后续如果你需要读反馈，可以在串口接收路径里拼包后调用本函数。
 */
void unitree_go_m8010_6_update_feedback(Unitree_GO_M8010_6_Motor_t *motor, const uint8_t data[UNITREE_GO_M8010_6_FB_PACKET_LEN]) {
    uint16_t crc;
    uint16_t packet_crc;
    float gear_ratio;

    if (motor == NULL || data == NULL) {
        return;
    }
    if (data[0] != UNITREE_GO_FB_HEAD_0 || data[1] != UNITREE_GO_FB_HEAD_1) {
        return;
    }
    if ((data[2] & 0x0FU) != (motor->id & 0x0FU)) {
        return;
    }

    crc = unitree_crc_ccitt(0, data, UNITREE_GO_M8010_6_FB_PACKET_LEN - 2U);
    packet_crc = unitree_get_u16_le(&data[14]);
    if (crc != packet_crc) {
        return;
    }

    gear_ratio = (motor->gear_ratio > 0.0f) ? motor->gear_ratio : 1.0f;
    motor->feedback.torque = (float)((int16_t)unitree_get_u16_le(&data[3])) / 256.0f;
    motor->feedback.speed = ((float)((int16_t)unitree_get_u16_le(&data[5])) / 256.0f) * UNITREE_GO_TWO_PI / gear_ratio;
    motor->feedback.angle = ((float)((int32_t)unitree_get_u32_le(&data[7])) / 32768.0f) * UNITREE_GO_TWO_PI / gear_ratio;
    motor->feedback.temp = (float)((int8_t)data[11]);
    motor->feedback.error_code = data[12] & 0x07U;
    motor->feedback.online = true;
}

uint8_t unitree_go_m8010_6_process_rx_bytes(const uint8_t *data, uint16_t len) {
    static uint8_t frame[UNITREE_GO_M8010_6_FB_PACKET_LEN];
    static uint8_t frame_len = 0U;
    uint8_t parsed_count = 0U;

    if (data == NULL) {
        return 0U;
    }

    for (uint16_t i = 0U; i < len; ++i) {
        uint8_t byte = data[i];

        if (frame_len == 0U) {
            if (byte == UNITREE_GO_FB_HEAD_0) {
                frame[frame_len++] = byte;
            }
            continue;
        }

        if (frame_len == 1U) {
            if (byte == UNITREE_GO_FB_HEAD_1) {
                frame[frame_len++] = byte;
            } else if (byte == UNITREE_GO_FB_HEAD_0) {
                frame[0] = byte;
                frame_len = 1U;
            } else {
                frame_len = 0U;
            }
            continue;
        }

        frame[frame_len++] = byte;
        if (frame_len >= UNITREE_GO_M8010_6_FB_PACKET_LEN) {
            uint8_t motor_id = frame[2] & 0x0FU;

            if (unitree_crc_ccitt(0, frame, UNITREE_GO_M8010_6_FB_PACKET_LEN - 2U) == unitree_get_u16_le(&frame[14])) {
                for (uint8_t motor_idx = 0U; motor_idx < UNITREE_GO_M8010_6_MOTOR_COUNT; ++motor_idx) {
                    Unitree_GO_M8010_6_Motor_t *motor = &g_unitree_go_m8010_6_motor_registry[motor_idx];
                    if ((motor->id & 0x0FU) == motor_id) {
                        unitree_go_m8010_6_update_feedback(motor, frame);
                        ++parsed_count;
                        break;
                    }
                }
            }

            frame_len = 0U;
        }
    }

    return parsed_count;
}
