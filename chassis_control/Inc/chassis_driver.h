#ifndef R1_CHASSIS_CHASSIS_DRIVER_H
#define R1_CHASSIS_CHASSIS_DRIVER_H

#include "fdcan.h"
#include "stm32g4xx.h"
#include <stdio.h>
#include <string.h>
#include "global_motor_conf.h"

// --- 1. Chassis type ---
//#define CHASSIS_TYPE_DUOLUN
//#define CHASSIS_TYPE_QUANXIANGLUN
#define CHASSIS_TYPE_MECANUM_OMNI

// --- 2. CAN send type ---
//#define BUFFERS_SEND

// cha_remote input units: vx/vy = mm/s, vr = chassis yaw rpm.
#define WHEEL_NUM       4
#define SPEED_LIMIT_XY  3000.0f   // XY resultant speed limit (mm/s)
#define SPEED_LIMIT_R   5.0f      // yaw speed limit (rpm)
#define MOTOR_VEL_LIMIT 10000.0f  // single wheel motor speed limit (rpm)
#define CHASSIS_RADIUS  289.91f   // legacy effective chassis radius (mm)
#define CHASSIS_MECANUM_CENTER_DISTANCE 287.9f // rotation center to front mecanum wheels (mm)
#define CHASSIS_OMNI_CENTER_DISTANCE    325.0f // rotation center to rear omni wheels (mm)
#define MECANUM_WHEEL_DIAMETER 125.0f // front mecanum wheel diameter (mm)
#define OMNI_WHEEL_DIAMETER    120.0f // rear omni wheel diameter (mm)
#define MECANUM_WHEEL_CIRCUMFERENCE (3.1415926f * MECANUM_WHEEL_DIAMETER)
#define OMNI_WHEEL_CIRCUMFERENCE    (3.1415926f * OMNI_WHEEL_DIAMETER)
// 电机输出轴每转一圈时，车轮转过的圈数。
// 例如：从电机输出轴到车轮是 5:1 的减速比，表示电机输出轴转 5 圈，车轮转 1 圈，
// 那么这个值就是 1/5 = 0.2f。
#define MECANUM_WHEEL_REV_PER_MOTOR_REV 0.2162162f
#define OMNI_WHEEL_REV_PER_MOTOR_REV    0.2162162f
#define SQRT_2_INV      0.70710678f // 1/sqrt(2)
#define CHASSIS_YAW_MECANUM_COEFF (CHASSIS_MECANUM_CENTER_DISTANCE / SQRT_2_INV)
#define CHASSIS_YAW_OMNI_COEFF    CHASSIS_OMNI_CENTER_DISTANCE

typedef struct {
    float vel;           // target wheel motor speed (rpm)
#ifdef CHASSIS_TYPE_DUOLUN
    float target_angle;  // target steering angle for swerve chassis
#endif
} Wheel_Command_t;

void cha_remote(float vx, float vy, float vr);
void Chassis_Send_Swerve_Command(int i, float vel, float angle);

#endif // R1_CHASSIS_CHASSIS_DRIVER_H
