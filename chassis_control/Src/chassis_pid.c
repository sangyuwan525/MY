//
// Created by 马皓然 on 2025/11/2.
//
#include "chassis_pid.h"
#include "chassis_path.h"
#include <math.h>

PID_Angle_t chassis_yaw_pid;
PID_Correct_t chassis_correct_pid;
PID_Approaching_t chassis_kaojin_pid;




/**
 * @brief  初始化朝向角PID结构体参数。
 *
 * @param  pid 待初始化的PID结构体指针。
 *
 * @note   将原代码中的常量和阈值赋给结构体成员。
 * @date   2025/11/02
 */
void PID_Angle_Init(PID_Angle_t *pid)
{
    // 假设使用您之前代码中的参数值进行初始化
    pid->kp = 2500.545f;
    pid->ki = 1.0f;
    pid->kd = 0.0f;

    // 限制与阈值
    pid->output_limit = 4000.0f;        // abs_limit_pid_angle
    pid->integral_limit = 2500.0f;      // abs_limit_pid_angle_ill
    pid->integral_separate_thr = 0.15f; // 积分分离阈值

    // 状态变量初始化
    pid->integral_sum = 0.0f;
    pid->last_error = 0.0f;
}
/**
 * @brief  使用位置式PID计算朝向角控制输出（旋转速度）。
 *
 * @param  pid 待使用的PID结构体指针，包含参数和状态。
 * @param  target_angle 目标朝向角 (弧度)。
 * @param  now_angle 当前朝向角 (弧度)。
 *
 * @return float 计算得到的期望旋转速度。
 *
 * @note   包含角度差归一化（-PI到PI）、积分限幅和积分分离。
 * @author stm32小高手
 * @date   2025/11/02
 */
float PID_Angle_Calculate(PID_Angle_t *pid, float target_angle, float now_angle)
{
    float err;
    float set_rotate_spd; // 瞬时输出，无需 static

    // 1. 角度差计算
    err = target_angle - now_angle;

    // 2. 角度差归一化 (-PI 到 PI)
    // 假设 PI 宏已定义为 3.1415926535f
    if (err > pi) {
        err = err - 2.0f * pi;
    } else if (err < -1.0f * pi) {
        err = err + 2.0f * pi;
    }

    // 3. P 项计算
    set_rotate_spd = pid->kp * err;

    // 4. I 项计算和限幅
    // 积分分离判断：当误差小于阈值时才进行积分
    if (fabsf(err) < pid->integral_separate_thr) {
        // 累加积分项
        pid->integral_sum += pid->ki * err;
    }
    // 积分限幅
    if (pid->integral_sum > pid->integral_limit) {
        pid->integral_sum = pid->integral_limit;
    } else if (pid->integral_sum < -pid->integral_limit) {
        pid->integral_sum = -pid->integral_limit;
    }

    set_rotate_spd += pid->integral_sum;

    // 5. D 项计算
    set_rotate_spd += pid->kd * (err - pid->last_error);

    // 6. 更新历史误差 (为下一次计算 D 项做准备)
    pid->last_error = err;

    // 7. 输出限幅
    if (set_rotate_spd > pid->output_limit) {
        set_rotate_spd = pid->output_limit;
    } else if (set_rotate_spd < -pid->output_limit) {
        set_rotate_spd = -pid->output_limit;
    }

    return set_rotate_spd;
}
/**
 * @brief  初始化横向修正PID结构体参数。
 *
 * @param  pid 待初始化的PID结构体指针。
 *
 * @note   将原代码中的常量和阈值赋给结构体成员。
 * @date   2025/11/02
 */
void PID_Correct_Init(PID_Correct_t *pid)
{
    // 假设使用您之前代码中的参数值进行初始化
    pid->kp = 10.0f;
    pid->ki = 0.000005f;
    pid->kd = 0.0002f;

    // 限制与阈值
    pid->output_limit = 500.0f;        // abs_limit_correct
    pid->integral_limit = 200.0f;      // abs_limit_correct_ill

    // 状态变量初始化
    pid->integral_sum.x = 0.0f;
    pid->integral_sum.y = 0.0f;
    pid->last_error.x = 0.0f;
    pid->last_error.y = 0.0f;
}
/**
 * @brief  使用位置式PID计算横向修正速度（X/Y轴）。
 *
 * @param  pid 待使用的PID结构体指针，包含参数和状态。
 * @param  now_point 机器人当前的坐标点。
 * @param  foot_point 轨迹上的垂足（目标）坐标。
 *
 * @return vec2 计算得到的期望修正速度向量 (在世界坐标系下)。
 *
 * @note   该函数实现了 X 和 Y 轴的独立 PID，用于计算将机器人拉回到垂足点上的修正速度。
 * @author stm32小高手
 * @date   2025/11/02
 */
vec2 PID_Correct_Calculate(PID_Correct_t *pid, Point_struct now_point, Point_struct foot_point)
{
    vec2 current_err;
    vec2 spd; // 瞬时输出，无需 static

    // 1. 计算当前误差 (目标 - 当前)
    current_err.x = foot_point.x - now_point.x;
    current_err.y = foot_point.y - now_point.y;

    // 2. P 项计算
    spd.x = pid->kp * current_err.x;
    spd.y = pid->kp * current_err.y;

    // 3. I 项计算和限幅
    pid->integral_sum.x += pid->ki * current_err.x;
    pid->integral_sum.y += pid->ki * current_err.y;

    // I 项限幅 (X 轴)
    if (pid->integral_sum.x > pid->integral_limit) pid->integral_sum.x = pid->integral_limit;
    else if (pid->integral_sum.x < -pid->integral_limit) pid->integral_sum.x = -pid->integral_limit;

    // I 项限幅 (Y 轴)
    if (pid->integral_sum.y > pid->integral_limit) pid->integral_sum.y = pid->integral_limit;
    else if (pid->integral_sum.y < -pid->integral_limit) pid->integral_sum.y = -pid->integral_limit;

    // 累加 I 项
    spd.x += pid->integral_sum.x;
    spd.y += pid->integral_sum.y;

    // 4. D 项计算
    // ⚠ 原代码中的错误修正：D项计算应使用 current_err.x 和 current_err.y
    spd.x += pid->kd * (current_err.x - pid->last_error.x);
    spd.y += pid->kd * (current_err.y - pid->last_error.y);

    // 5. 更新历史误差
    pid->last_error = current_err;

    // 6. 输出限幅 (X 轴)
    if (spd.x > pid->output_limit) spd.x = pid->output_limit;
    else if (spd.x < -pid->output_limit) spd.x = -pid->output_limit;

    // 7. 输出限幅 (Y 轴)
    if (spd.y > pid->output_limit) spd.y = pid->output_limit;
    else if (spd.y < -pid->output_limit) spd.y = -pid->output_limit;

    return spd;
}
/**
 * @brief  初始化目标靠近PID结构体参数。
 *
 * @param  pid 待初始化的PID结构体指针。
 *
 * @note   将原代码中的常量和阈值赋给结构体成员。
 * @date   2025/11/02
 */
void PID_Approaching_Init(PID_Approaching_t *pid)
{
    pid->kp = 3.8f;
    pid->ki = 0.79f;
    pid->kd = 0.0f; // 注意：原代码中 kd=0.0，但 D 项逻辑是存在的

    // 限制与阈值
    pid->output_limit = 1000.0f;        // abs_limit_kaojin
    pid->integral_limit = 200.0f;       // abs_limit_kaojin_ill
    pid->integral_separate_i_thr = 13.0f; // 积分项分离阈值 (原代码中直接判断 sum_err_spd > 13)
    pid->min_direction_thr = 1e-7f;     // min (使用浮点数常量的更小值)

    // 状态变量初始化
    pid->integral_sum.x = 0.0f;
    pid->integral_sum.y = 0.0f;
    pid->last_point.x = 0.0f;
    pid->last_point.y = 0.0f;
}
/**
 * @brief  计算末端定位阶段的期望靠近速度（X/Y轴）。
 *
 * @param  pid 待使用的PID结构体指针，包含参数和状态。
 * @param  now_point 机器人当前的坐标点。
 * @param  foot_point 目标坐标点。
 *
 * @return vec2 计算得到的期望靠近速度向量 (在世界坐标系下)。
 *
 * @note   该函数实现了复杂的 PID 逻辑：积分项限幅、微分项基于位置变化、积分项分离，以及最终的速度矢量限幅。
 * @date   2025/11/02
 */
vec2 PID_Approaching_Calculate(PID_Approaching_t *pid, Point_struct now_point, Point_struct foot_point)
{
    vec2 current_err;
    vec2 spd; // 瞬时输出，无需 static
    float temp_spd_x, temp_spd_y;

    // 1. 计算当前误差 (目标 - 当前)
    current_err.x = foot_point.x - now_point.x;
    current_err.y = foot_point.y - now_point.y;

    // 2. I 项计算和 D 项计算 (注意：您原始代码中 D 项直接加到了积分项上，这是一种特殊的控制策略，这里保留其计算方式)

    // 积分累加
    pid->integral_sum.x += pid->ki * current_err.x;
    pid->integral_sum.y += pid->ki * current_err.y;

    // D 项作为速度反馈/阻尼项，被集成到 sum_err_spd 中（即不完全微分或速度反馈）
    // D项 = -Kd * (当前位置 - 上次位置) / dT，这里 dT 被隐藏在了 Kd 中
    //这里为了减少计算量复杂度，把原定的err-last_err直接化简为now_point-last_point
    pid->integral_sum.x -= pid->kd * (now_point.x - pid->last_point.x);
    pid->integral_sum.y -= pid->kd * (now_point.y - pid->last_point.y);

    // 3. 积分项限幅
    if (pid->integral_sum.x > pid->integral_limit) pid->integral_sum.x = pid->integral_limit;
    else if (pid->integral_sum.x < -pid->integral_limit) pid->integral_sum.x = -pid->integral_limit;

    if (pid->integral_sum.y > pid->integral_limit) pid->integral_sum.y = pid->integral_limit;
    else if (pid->integral_sum.y < -pid->integral_limit) pid->integral_sum.y = -pid->integral_limit;

    // 4. 积分分离判断和最终输出计算
    // 积分分离逻辑：当 sum_err_spd (积分项) 较小时才将其项贡献给输出
    float index_i_kaojin_x = (fabsf(pid->integral_sum.x) < pid->integral_separate_i_thr) ? 1.0f : 0.0f;
    float index_i_kaojin_y = (fabsf(pid->integral_sum.y) < pid->integral_separate_i_thr) ? 1.0f : 0.0f;

    // 最终速度计算：P项 + I项 (分离后)
    temp_spd_x = pid->kp * current_err.x + index_i_kaojin_x * pid->integral_sum.x;
    temp_spd_y = pid->kp * current_err.y + index_i_kaojin_y * pid->integral_sum.y;

    // 5. 速度矢量限幅 (确保 X/Y 轴速度比例与目标方向一致时，不超过总限幅)
    vec2 spd_dir;
    spd_dir.x = current_err.x;
    spd_dir.y = current_err.y;

    float dis_sq = spd_dir.x * spd_dir.x + spd_dir.y * spd_dir.y; // 距离的平方

    if (dis_sq < pid->min_direction_thr) {
        // 如果误差距离接近零，则速度为零
        spd.x = 0.0f;
        spd.y = 0.0f;
    } else {
        // 目标方向向量 (未归一化)
        float max_spd = pid->output_limit;

        // 简化限幅逻辑：确保总速度模长不超过 limit
        float current_spd_sq = temp_spd_x * temp_spd_x + temp_spd_y * temp_spd_y;

        if (current_spd_sq > max_spd * max_spd) {
            // 如果总速度超过限制，按比例缩小 X 和 Y 速度
            float ratio = max_spd / sqrtf(current_spd_sq);
            spd.x = temp_spd_x * ratio;
            spd.y = temp_spd_y * ratio;
        } else {
            spd.x = temp_spd_x;
            spd.y = temp_spd_y;
        }
    }

    // 6. 更新历史点
    pid->last_point.x = now_point.x;
    pid->last_point.y = now_point.y;

    return spd;
}