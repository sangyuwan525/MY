//
// Created by 马皓然 on 2026/1/31.
//

#ifndef R1_SUPERSTRUCTURE_MOTOR_REGISTRY_H
#define R1_SUPERSTRUCTURE_MOTOR_REGISTRY_H
#include "main.h"
#include "dji_3508_2006_motor.h"
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"

#define MOTOR_TOTAL_NUM (DJI_MOTOR_COUNT + DM_MOTOR_COUNT)
#define DM_MOTOR1_CAN_ID 0x01
// 电机类型枚举
typedef enum {
    MOTOR_TYPE_DJI,
    MOTOR_TYPE_DAMIAO
} Motor_Type_e;

// 注意：这里只是为了统一格式，单位取决于底层驱动（DJI通常是编码器值，达妙是弧度）
typedef struct {
    float angle;    // 位置 (DJI: total_angle 累积编码器值 / DM: rad)
    float speed;    // 速度 (DJI: RPM / DM: rad/s)
    float torque;   // 转矩或电流 (DJI: current / DM: N.m)
    float temp;     // 温度
} Motor_State_t;

// 电机基类结构体
typedef struct Motor_Class {
    Motor_Type_e type;      // 电机品种
    void* instance;  // 存放指向真正结构体的指针 (Dji_Motor_t* 或 Damiao_Motor_t*)



    // --- 虚函数：不同的电机有不同的实现 ---
    void (*init)(struct Motor_Class* self);

    /**
    * @brief 速度控制模式
    * @param speed 目标速度 (DJI: RPM, DM: rad/s)
    */
    void (*set_speed)(struct Motor_Class* self, float speed);

    /**
     * @brief 位置控制模式
     * @param position   目标位置 (DJI: Encoder Count, DM: rad)
     * @param vel_limit  速度限制或前馈 (DJI: 可忽略或作为内环限幅, DM: 作为前馈速度或限幅)
     */
    void (*set_position)(struct Motor_Class* self, float position, float vel_limit);

    /**
     * @brief MIT控制模式（主要用于达妙电机）
     * @param position  目标位置，单位通常为 rad
     * @param speed     目标速度，单位通常为 rad/s
     * @param kp        位置环刚度系数（比例增益）
     * @param kd        速度环阻尼系数（微分增益）
     * @param torque    转矩前馈，单位通常为 N·m
     *
     * @note  MIT 模式本质上是一种组合控制：
     *        输出 = 位置项 + 速度项 + 转矩前馈
     *        常用于关节电机，实现“既给目标位置，又给动态柔顺性”的控制。
     *        DJI 电机一般不直接使用该模式，主要面向达妙电机。
     */
    void (*set_mit)(struct Motor_Class* self, float position, float speed, float kp, float kd, float torque);

    /**
     * @brief PSI控制模式（主要用于达妙电机）
     * @param position  目标位置，单位通常为 rad
     * @param speed     目标速度，单位通常为 rad/s
     * @param current   目标电流/力矩指令，具体单位取决于底层驱动定义
     *
     * @note  PSI 模式通常表示位置-速度-电流的复合控制接口：
     *        - position 提供目标位置
     *        - speed 提供速度约束或前馈
     *        - current 提供电流/力矩前馈
     *        适合需要同时约束位置响应和输出力度的场景。
     *        DJI 电机一般不直接使用该模式，主要面向达妙电机。
     */
    void (*set_psi)(struct Motor_Class* self, float position, float speed, float current);
    // 统一的反馈更新接口
    void (*update_feedback)(struct Motor_Class* self, uint8_t* rx_data);

    Motor_State_t (*get_state)(struct Motor_Class* self);
} Motor_Class_t;


// --- 外部接口 ---
void Motor_Registry_Init(void);
void Motor_All_Control_Loop(void);
void Motor_Feedback_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data);
void Motor_SetMIT(int motor_index, float position, float speed, float kp, float kd, float torque);
void Motor_SetPSI(int motor_index, float position, float speed, float current);
extern Motor_Class_t g_motor_list[MOTOR_TOTAL_NUM];
#endif //R1_SUPERSTRUCTURE_MOTOR_REGISTRY_H
