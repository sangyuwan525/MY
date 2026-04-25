#ifndef R1_SUPERSTRUCTURE_UNITREE_GO_M8010_6_MOTOR_H
#define R1_SUPERSTRUCTURE_UNITREE_GO_M8010_6_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "usart.h"

/*
 * 宇树 GO-M8010-6 官方串口协议包长度。
 * 本工程通过串口/RS485 发送官方协议数据包：
 * 控制包 17 字节，反馈包 16 字节。
 */
#define UNITREE_GO_M8010_6_PACKET_LEN 17U
#define UNITREE_GO_M8010_6_FB_PACKET_LEN 16U

/* 本工程中注册了几台 GO-M8010-6 电机。继续扩展时在这里加 Motor2/Motor3。 */
typedef enum {
    UNITREE_GO_M8010_6_Motor1 = 0,
    UNITREE_GO_M8010_6_MOTOR_COUNT
} unitree_go_m8010_6_motor_num_e;

/*
 * 宇树协议包内部的电机 ID。
 * 这个 ID 是写进 17 字节控制包里的电机地址，不是 CAN ID。
 */
typedef enum {
    UNITREE_GO_M8010_6_Motor1_ID = 0x00,
} unitree_go_m8010_6_motor_id_e;

/*
 * 宇树协议包里的 mode 字段。
 * 正常控制统一使用 FOC；BRAKE 用于刹车/停止；CALIBRATE 一般不要在普通控制里调用。
 */
typedef enum {
    UNITREE_GO_M8010_6_MODE_BRAKE = 0,
    UNITREE_GO_M8010_6_MODE_FOC = 1,
    UNITREE_GO_M8010_6_MODE_CALIBRATE = 2,
} Unitree_GO_M8010_6_Mode_e;

/* 电机反馈值，统一成大注册表里的常用物理量。 */
typedef struct {
    float angle;       /* 输出端角度，单位 rad。 */
    float speed;       /* 输出端速度，单位 rad/s。 */
    float torque;      /* 电机反馈力矩，单位 Nm。 */
    float temp;        /* 电机温度，单位摄氏度。 */
    uint8_t error_code;/* 电机错误码，来自反馈包 MError 字段。 */
    bool online;       /* 收到并成功校验反馈包后置 true。 */
} Unitree_GO_M8010_6_Feedback_t;

/* 大注册表写入的控制目标，最终会被打包成宇树 17 字节控制包。 */
typedef struct {
    Unitree_GO_M8010_6_Mode_e mode;
    uint8_t mode_configured; /* 防止初始化后还没有目标值时就开始发控制包。 */
    float torque_set;        /* 前馈力矩，单位 Nm，对应官方 tau。 */
    float pos_set;           /* 输出端目标位置，单位 rad，对应注册表 position。 */
    float speed_set;         /* 输出端目标速度，单位 rad/s，对应注册表 speed。 */
    float kp_set;            /* 位置刚度，官方 GO 包范围 0.0-1.0。 */
    float kd_set;            /* 速度阻尼，官方 GO 包范围 0.0-1.0。 */
} Unitree_GO_M8010_6_Control_t;

/* 单台 GO-M8010-6 电机的完整注册信息和运行时状态。 */
typedef struct {
    UART_HandleTypeDef *huart; /* 连接 RS485 收发器的串口句柄。 */
    uint8_t id;              /* 宇树协议包内部的电机 ID。 */
    float gear_ratio;        /* 减速比：注册表使用输出端单位，打包时转成电机端单位。 */
    Unitree_GO_M8010_6_Control_t ctrl;
    Unitree_GO_M8010_6_Feedback_t feedback;
} Unitree_GO_M8010_6_Motor_t;

extern Unitree_GO_M8010_6_Motor_t g_unitree_go_m8010_6_motor_registry[UNITREE_GO_M8010_6_MOTOR_COUNT];

void unitree_go_m8010_6_motor_init(void);
void unitree_go_m8010_6_motor_ctrl_send(Unitree_GO_M8010_6_Motor_t *motor);
void unitree_go_m8010_6_motor_stop(Unitree_GO_M8010_6_Motor_t *motor);
void unitree_go_m8010_6_update_feedback(Unitree_GO_M8010_6_Motor_t *motor, const uint8_t data[UNITREE_GO_M8010_6_FB_PACKET_LEN]);
uint8_t unitree_go_m8010_6_transport_send(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);

#endif /* R1_SUPERSTRUCTURE_UNITREE_GO_M8010_6_MOTOR_H */
