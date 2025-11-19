//
// Created by 马皓然 on 2025/11/5.
//


#include "chassis_driver.h"
#include <math.h>
#define PI  3.1415926f
// --- 1. 全局数据实例 ---
// 用于存储轮子最终指令，由 speed_decompose 填充
static Wheel_Command_t wheel_data[WHEEL_NUM];
// --- 2. 运动学解算函数 (speed_decompose) ---
// 将函数名称规范化，并基于宏切换实现

#ifdef CHASSIS_TYPE_QUANXIANGLUN
// ==========================================================
// A. 全向轮运动学逆解算
// ==========================================================
/**
 * @brief  全向轮底盘的运动学逆解算。
 * @note  注意轮子个数，响应解算函数要跟着更改
 * 这里简化为：wi = +/-Vx +/- Vy +/- Vr
 */
static void speed_decompose_quanxianglun(int motor_id, float vx, float vy, float vr)
{
    float vel_r = vr * CHASSIS_RADIUS;

    // 简化公式：wi = Vr*R +/- 0.707*Vx +/- 0.707*Vy
    //电机转向正反说明：默认使整车逆时针旋转的电机转向为正，使整车顺时针旋转的电机转向为负
    //这里默认是标准正方形四角45度安装全向轮

    switch(motor_id)
    {
        case 0: // 前左 (FL)
            wheel_data[motor_id].vel = vel_r - SQRT_2_INV * vx - SQRT_2_INV * vy;
            break;
        case 1: // 后左 (RL)
            wheel_data[motor_id].vel = vel_r + SQRT_2_INV * vx - SQRT_2_INV * vy;
            break;
        case 2: // 前右 (FR)
            wheel_data[motor_id].vel = vel_r - SQRT_2_INV * vx + SQRT_2_INV * vy;
            break;
        case 3: // 后右 (RR)
            wheel_data[motor_id].vel = vel_r + SQRT_2_INV * vx + SQRT_2_INV * vy;
            break;
        default:
            break;
    }
}

#elif defined(CHASSIS_TYPE_DUOLUN)
// ==========================================================
// B. 四轮舵轮 (Swerve) 运动学逆解算
// ==========================================================
/**
 * @brief  四轮舵轮底盘的运动学逆解算辅助公式
 * @note   默认舵轮轮子与y轴同方向为角度0，y轴角度范围(-180,180]，转向x轴正半轴为正角
 */
float arctan2y(float x,float y)
{
    float theta = 0.0f;
    if(x == 0)
    {
        if( y >= 0 )
            theta = PI/2.0 ;
        else if(y < 0)
            theta = -PI/2.0;
    }
    else
    {
        theta= atanf(y/x);
        if( y < 0 && x < 0 )
            theta = theta - PI ;
        if( y >= 0 && x < 0 )
            theta = theta + PI ;
    }
    theta = 90.0 - theta / PI * 180.0 ;
    //theta范围 [-90,270]，所以下面判断中一种情况不存在，没必要判断，暂时注释掉
    while ( theta > 180.0 )
        theta -= 360.0 ;     //第三象限中情况
    while ( theta < -180.0 )
        theta += 360.0 ;
    return theta ;
}
/**
 * @brief  四轮舵轮底盘的运动学逆解算。
 * @note   舵轮解算公式：
 * Vx_i = Vx - Vr * Ri_y
 * Vy_i = Vy + Vr * Ri_x
 * wi = sqrt(Vx_i^2 + Vy_i^2)
 * theta_i = atan2(Vy_i, Vx_i)
 */
static void speed_decompose_duolun(int motor_id, float vx, float vy, float vr)
{
    // 1. 计算轮子i的期望速度分量 Vx_i, Vy_i
    float vx_i, vy_i; // 轮子i的期望速度分量 (线速度)
    // 假设所有轮子位于半径 CHASSIS_RADIUS 的正方形上
    float R = CHASSIS_RADIUS * SQRT_2_INV; // 简化 R
    switch (motor_id) {
        case 0://左前
            vx_i = vx - vr * R;
            vy_i = vy - vr * R;
            break;
        case 1://左后
            vx_i = vx + vr * R;
            vy_i = vy - vr * R;
            break;
        case 2://右前
            vx_i = vx - vr * R;
            vy_i = vy + vr * R;
            break;
        case 3://右后
            vx_i = vx + vr * R;
            vy_i = vy + vr * R;
            break;
        default:
            vx_i = 0.0f;
            vy_i = 0.0f;
            break;
    }
    // 2. 计算轮子速度大小和方向
    wheel_data[motor_id].vel = sqrtf(vx_i * vx_i + vy_i * vy_i);
    wheel_data[motor_id].target_angle = arctan2y(vx_i, vy_i);
    //注意一下这个函数返回的角度是与y轴正方向的夹角，范围是[-180,180]
    if (wheel_data[motor_id].vel > SPEED_LIMIT_XY) {
            wheel_data[motor_id].vel = SPEED_LIMIT_XY;
    }
}

#else
// 如果未定义底盘类型，则编译空函数
static void speed_decompose_unknown(int motor_id, float vx, float vy, float vr) {
    // 警告或错误处理
}
#endif


// --- 3. 主速度控制函数实现

void cha_remote(float vx, float vy, float vr)
{
    float velx, vely, vela;

    // 1. 速度限幅 (仅对 XY 合速度进行限幅，旋转速度单独处理)
    float abs_xy_spd_sq = vx * vx + vy * vy;

    if (abs_xy_spd_sq > SPEED_LIMIT_XY * SPEED_LIMIT_XY)
    {
        // 速度超过限幅，按比例缩小
        float ratio = SPEED_LIMIT_XY / sqrtf(abs_xy_spd_sq);
        velx = vx * ratio;
        vely = vy * ratio;
    }
    else
    {
        velx = vx;
        vely = vy;
    }
    vela = vr; // 旋转速度不与线速度联动限幅

    // 2. 运动学解算
    for (int i = 0; i < WHEEL_NUM; i++)
    {
#ifdef CHASSIS_TYPE_QUANXIANGLUN
        speed_decompose_quanxianglun(i, velx, vely, vela);
#elif defined(CHASSIS_TYPE_DUOLUN)
        speed_decompose_duolun(i, velx, vely, vela);
#else
        speed_decompose_unknown(i, velx, vely, vela);
#endif
    }
    // 3. 发送数据到电机
    // 假设有一个通用的发送函数：Change_Motor_Command(motor_id, speed, angle)
    for (int i = 0; i < WHEEL_NUM; i++)
    {
#ifdef CHASSIS_TYPE_MECANUM
        // 全向轮只需发送转速指令
        // 假设 Change_dji_speed 是发送电机转速的函数
        Change_dji_speed(i, wheel_data[i].vel);
#elif defined(CHASSIS_TYPE_SWERVE)
        // 舵轮需要发送转速和转向角
        Chassis_Send_Swerve_Command(i,
                                    wheel_data[i].vel,
                                    wheel_data[i].target_angle);
#endif
    }
}
