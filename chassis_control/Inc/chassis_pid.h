//
// Created by 马皓然 on 2025/11/2.
//

#ifndef R1_CHASSIS_CHASSIS_PID_H
#define R1_CHASSIS_CHASSIS_PID_H

#include "chassis_path.h"

/**
 * @brief PID 控制器结构体
 * @note 用于封装PID算法的参数和内部状态变量 (如积分项、上次误差)。
 */
typedef struct {
    // PID 参数
    float kp;             ///< 比例系数
    float ki;             ///< 积分系数
    float kd;             ///< 微分系数

    // 状态变量
    float integral_sum;   ///< 积分项累加值 (sumerr_ang_spd)
    float last_error;     ///< 上一次的误差 (pid_ang_last_err)

    // 限制与阈值
    float integral_limit; ///< 积分限幅值 (abs_limit_pid_angle_ill)
    float output_limit;   ///< 输出限幅值 (abs_limit_pid_angle)
    float integral_separate_thr; ///< 积分分离阈值 (0.15f)

} PID_Angle_t;
/**
 * @brief 2D 修正速度 PID 结构体
 * @note 用于封装路径跟踪中横向误差修正的 PID 参数和状态（X轴和Y轴）。
 */
typedef struct {
    // PID 参数
    float kp;             ///< 比例系数
    float ki;             ///< 积分系数
    float kd;             ///< 微分系数

    // 状态变量 (X/Y 轴)
    vec2 integral_sum;    ///< 积分项累加值 (X/Y)
    vec2 last_error;      ///< 上一次的误差 (X/Y)

    // 限制与阈值
    float integral_limit; ///< 积分限幅值 (abs_limit_correct_ill)
    float output_limit;   ///< 输出限幅值 (abs_limit_correct)

} PID_Correct_t;
/**
 * @brief 目标靠近PID结构体
 * @note 用于封装路径末端或定位阶段的位置PID参数和状态，该结构体用于靠近最后终点处的控制。
 */
typedef struct {
    // PID 参数
    float kp;             ///< 比例系数
    float ki;             ///< 积分系数
    float kd;             ///< 微分系数

    // 状态变量
    vec2 integral_sum;    ///< 积分项累加值 (X/Y)
    vec2 last_point;      ///< 上一次机器人坐标 (用于计算微分项)

    // 限制与阈值
    float integral_limit; ///< 积分限幅值 (abs_limit_kaojin_ill)
    float output_limit;   ///< 输出限幅值 (abs_limit_kaojin)
    float integral_separate_i_thr; ///< 积分项分离阈值 (13.0f)
    float min_direction_thr; ///< 方向向量模长最小值 (min)

} PID_Approaching_t;

extern PID_Angle_t chassis_yaw_pid;
extern PID_Correct_t chassis_correct_pid;
extern PID_Approaching_t chassis_kaojin_pid;


void PID_Angle_Init(PID_Angle_t *pid);
void PID_Correct_Init(PID_Correct_t *pid);
void PID_Approaching_Init(PID_Approaching_t *pid);
float PID_Angle_Calculate(PID_Angle_t *pid, float target_angle, float now_angle);
vec2 PID_Correct_Calculate(PID_Correct_t *pid, Point_struct now_point, Point_struct foot_point);
vec2 PID_Approaching_Calculate(PID_Approaching_t *pid, Point_struct now_point, Point_struct foot_point);
#endif //R1_CHASSIS_CHASSIS_PID_H