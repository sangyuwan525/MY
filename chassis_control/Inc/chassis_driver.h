#ifndef R1_CHASSIS_CHASSIS_DRIVER_H
#define R1_CHASSIS_CHASSIS_DRIVER_H

#include "fdcan.h"
#include "stm32g4xx.h"
#include <stdio.h>
#include <string.h>

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
#define CHASSIS_RADIUS  289.91f   // effective chassis radius (mm)
#define WHEEL_DIAMETER  200.0f    // wheel diameter (mm)
#define WHEEL_CIRCUMFERENCE 628.3185f // wheel circumference for 200 mm wheel (mm)
#define SQRT_2_INV      0.70710678f // 1/sqrt(2)
#define CHASSIS_YAW_MECANUM_COEFF (CHASSIS_RADIUS / SQRT_2_INV)
#define CHASSIS_YAW_OMNI_COEFF    CHASSIS_RADIUS

typedef struct {
    float vel;           // target wheel motor speed (rpm)
#ifdef CHASSIS_TYPE_DUOLUN
    float target_angle;  // target steering angle for swerve chassis
#endif
} Wheel_Command_t;

void cha_remote(float vx, float vy, float vr);
void Chassis_Send_Swerve_Command(int i, float vel, float angle);

#endif // R1_CHASSIS_CHASSIS_DRIVER_H
