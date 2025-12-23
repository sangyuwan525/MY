//
// Created by 马皓然 on 2025/11/5.
//

#ifndef R1_CHASSIS_CHASSIS_DRIVER_H
#define R1_CHASSIS_CHASSIS_DRIVER_H

// --- 1. 底盘类型宏定义 ---
 #define CHASSIS_TYPE_DUOLUN
 //#define CHASSIS_TYPE_QUANXIANGLUN

// 注意:以下这些值目前纯数瞎给，需要根据实际底盘参数进行调整
#define WHEEL_NUM       4
#define SPEED_LIMIT_XY  12000.0f  // XY轴合速度限幅 (MM/S)
#define MOTOR_VEL_LIMIT 10000.0f  // 单个轮子转速限幅 (RPM 或自定义单位)
#define CHASSIS_RADIUS  289.91f   // 底盘有效半径 (MM)
#define WHEEL_CIRCUMFERENCE 314.16f // 轮子周长 (MM)
#define SQRT_2_INV      0.70710678f // 1/sqrt(2)
// --- 3. 数据结构 ---
// 用于存储每个轮子的目标值
typedef struct {
    float vel;           // 轮子期望速度（全向轮：转速，舵轮：线速度）
#ifdef CHASSIS_TYPE_DUOLUN
    float target_angle;  // 舵轮期望转向角 (仅舵轮需要)
#endif
} Wheel_Command_t;

void cha_remote(float vx, float vy, float vr);

#endif //R1_CHASSIS_CHASSIS_DRIVER_H