#ifndef R2_CHASSIS_LIFT_WALK_CONTROLLER_H
#define R2_CHASSIS_LIFT_WALK_CONTROLLER_H

#include <stdint.h>

#include "global_motor_conf.h"
#include "motor_registry.h"

#define LIFT_WALK_SUPPORT_COUNT 4U
#define LIFT_WALK_SIDE_COUNT 2U
#define LIFT_WALK_INVALID_MOTOR (-1)

/* 控制器返回状态。
 * OK 表示本周期解算成功；ABORT 表示姿态或机构几何已经不适合继续输出。
 */
typedef enum {
    LIFT_WALK_OK = 0,
    LIFT_WALK_ABORT_ROLL_LIMIT,
    LIFT_WALK_ABORT_PITCH_LIMIT,
    LIFT_WALK_ABORT_HEIGHT_LIMIT,
    LIFT_WALK_ABORT_ARM_RANGE,
    LIFT_WALK_ABORT_ARM_DEAD_ZONE,
} LiftWalk_Status_e;

/* 四个支撑点编号。
 * 前两个支撑点由宇树小臂承担，后两个支撑点由小米滑轨承担。
 */
typedef enum {
    LIFT_WALK_FRONT_LEFT = 0,
    LIFT_WALK_FRONT_RIGHT,
    LIFT_WALK_REAR_LEFT,
    LIFT_WALK_REAR_RIGHT,
} LiftWalk_Support_e;

typedef enum {
    LIFT_WALK_LEFT = 0,
    LIFT_WALK_RIGHT,
} LiftWalk_Side_e;

/* 支撑点在底盘坐标系下的位置。
 * x_mm: 前后方向，向前为正。
 * y_mm: 左右方向，向左为正。
 */
typedef struct {
    float x_mm;
    float y_mm;
} LiftWalk_Point_t;

typedef struct {
    /* 电机绑定：填写 global_motor_conf.h 里的全局电机编号。
     * 不使用某个执行器时，可以填 LIFT_WALK_INVALID_MOTOR。
     */
    int xiaomi_slider_motor[LIFT_WALK_SIDE_COUNT];
    int unitree_arm_motor[LIFT_WALK_SIDE_COUNT];
    int front_wheel_motor[LIFT_WALK_SIDE_COUNT];
    int rear_wheel_motor[LIFT_WALK_SIDE_COUNT];

    /* 四个底盘角/支撑点相对底盘中心的位置。
     * 用于把 roll/pitch 姿态修正转换成四个角的抬升高度差。
     */
    LiftWalk_Point_t support_pos[LIFT_WALK_SUPPORT_COUNT];

    /* 小米滑轨参数。
     * slider_pitch_mm_per_rev: 电机输出轴每转 1 圈，滑块沿导轨方向移动的距离。
     * slider_reduction_ratio: 电机到滑轨输入之间的传动比，直连填 1。
     * slider_motor_sign: 小米电机方向修正，方向反了填 -1。
     * slider_zero_rad: 底盘未抬升时的电机位置零点。
     * slider_min/max_rad: 滑轨位置软限位。
     * slider_vel_limit_rad_s: 位置模式速度限制。
     */
    float slider_pitch_mm_per_rev;
    float slider_reduction_ratio;
    float slider_motor_sign[LIFT_WALK_SIDE_COUNT];
    float slider_zero_rad[LIFT_WALK_SIDE_COUNT];
    float slider_min_rad[LIFT_WALK_SIDE_COUNT];
    float slider_max_rad[LIFT_WALK_SIDE_COUNT];
    float slider_vel_limit_rad_s;

    /* 宇树小臂几何和 MIT 参数。
     * arm_length_mm: 小臂转轴到末端轮子中心的距离。
     * arm_pivot_z_mm: 未抬升时小臂转轴距离地面的高度。
     * arm_zero_offset_rad: 小臂机构零位和数学模型角 phi 的偏置。
     * arm_motor_zero_rad: 底盘未抬升时宇树电机反馈的位置零点。
     * arm_motor_sign: 宇树电机方向修正，方向反了填 -1。
     * arm_min/max_rad: 小臂机构角软限位。
     * arm_dead_cos_min: 小臂接近竖直死区时的保护阈值。
     * arm_kp/kd/torque_ff: 下发给宇树 MIT 控制的参数。
     */
    float arm_length_mm[LIFT_WALK_SIDE_COUNT];
    float arm_pivot_z_mm[LIFT_WALK_SIDE_COUNT];
    float arm_zero_offset_rad[LIFT_WALK_SIDE_COUNT];
    float arm_motor_zero_rad[LIFT_WALK_SIDE_COUNT];
    float arm_motor_sign[LIFT_WALK_SIDE_COUNT];
    float arm_min_rad[LIFT_WALK_SIDE_COUNT];
    float arm_max_rad[LIFT_WALK_SIDE_COUNT];
    float arm_dead_cos_min;
    float arm_kp;
    float arm_kd;
    float arm_torque_ff;

    /* 轮速补偿参数。
     * front_wheel_radius_mm: 小臂末端达妙前轮半径。
     * rear_wheel_radius_mm: 后轮半径。
     * rear_wheel_drive_angle_rad: 后轮驱动方向相对 x 轴的角度。
     * front/rear_wheel_sign: 电机方向修正，方向反了填 -1。
     */
    float front_wheel_radius_mm;
    float rear_wheel_radius_mm;
    float rear_wheel_drive_angle_rad[LIFT_WALK_SIDE_COUNT];
    float front_wheel_speed_limit_rad_s;
    float rear_wheel_speed_limit_rpm;
    float front_wheel_accel_limit_rad_s2;
    float rear_wheel_accel_limit_rpm_s;
    float wheel_arm_comp_gain;
    float front_wheel_sign[LIFT_WALK_SIDE_COUNT];
    float rear_wheel_sign[LIFT_WALK_SIDE_COUNT];

    /* 姿态闭环参数。
     * roll/pitch 用于四点高度修正，yaw 用于行走时的差速修正。
     */
    float roll_kp;
    float roll_ki;
    float roll_kd;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float yaw_kp;
    float yaw_kd;

    /* 安全限幅。 */
    float attitude_cmd_limit_rad;
    float support_corr_limit_mm;
    float max_roll_rad;
    float max_pitch_rad;

    /* 抬升高度轨迹限制。 */
    float min_height_mm;
    float max_height_mm;
    float lift_vmax_mm_s;
    float lift_amax_mm_s2;
} LiftWalk_Config_t;

typedef struct {
    /* 抬升行走期望速度，单位 mm/s。
     * vy_mm_s: 正前方速度，沿底盘 y 轴正方向。
     * vx_mm_s: 抬升阶段暂不使用，底盘抬升时不做 x 方向横移。
     */
    float vx_mm_s;
    float vy_mm_s;

    /* 目标抬升高度，单位 mm。
     * 这里表示底盘相对未抬升状态的抬升量，不是轮心离地高度。
     */
    float target_height_mm;

    /* IMU 姿态角，单位 rad。 */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_ref_rad;

    /* IMU 角速度，单位 rad/s。 */
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;

    /* 控制周期，单位 s。 */
    float dt_s;

    /* 0: 只计算输出，便于调试观察；非 0: 计算后直接下发电机。 */
    uint8_t enable_motor_output;
} LiftWalk_Input_t;

typedef struct {
    /* 平滑后的高度轨迹。 */
    float height_ref_mm;
    float height_dot_ref_mm_s;

    /* 四个底盘角的抬升高度目标，顺序见 LiftWalk_Support_e。
     * 这里是相对未抬升状态的抬升量，不是轮心离地高度。
     */
    float support_z_mm[LIFT_WALK_SUPPORT_COUNT];

    /* 两个小米滑轨电机位置目标，单位 rad。 */
    float slider_motor_rad[LIFT_WALK_SIDE_COUNT];

    /* 小臂机构角和机构角速度。下发时会再换算成宇树电机角。 */
    float arm_theta_rad[LIFT_WALK_SIDE_COUNT];
    float arm_theta_dot_rad_s[LIFT_WALK_SIDE_COUNT];

    /* 小臂转动造成的末端轮心水平速度，用于前轮速度补偿。 */
    float arm_x_dot_mm_s[LIFT_WALK_SIDE_COUNT];

    /* 前后轮速度目标。
     * 前轮为轮子角速度 rad/s，下发时会再换算成达妙电机角速度。
     * 后轮为 Blazer FOC 转速 rpm。
     */
    float front_wheel_rad_s[LIFT_WALK_SIDE_COUNT];
    float rear_wheel_rpm[LIFT_WALK_SIDE_COUNT];

    /* 姿态闭环输出，便于调试方向和幅度。 */
    float roll_cmd_rad;
    float pitch_cmd_rad;
    float yaw_cmd_rad_s;

    LiftWalk_Status_e status;
} LiftWalk_Output_t;

typedef struct {
    LiftWalk_Config_t cfg;
    LiftWalk_Output_t out;

    float roll_i;
    float pitch_i;

    float last_height_target_mm;
    uint8_t lift_action_active;
    uint8_t lift_action_done;
    float lift_action_target_mm;

    uint8_t initialized;
} LiftWalk_Controller_t;

void LiftWalk_DefaultConfig(LiftWalk_Config_t *cfg);
void LiftWalk_Init(LiftWalk_Controller_t *ctrl, const LiftWalk_Config_t *cfg);
void LiftWalk_Reset(LiftWalk_Controller_t *ctrl, float current_height_mm);
LiftWalk_Status_e LiftWalk_Update(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in);

/* 执行一次“边走边抬升”动作。
 * 返回 0 表示动作仍在执行或保护退出；返回 1 表示到达目标高度并完成。
 */
uint8_t LiftWalk_RunLiftAction(LiftWalk_Controller_t *ctrl,
                               const LiftWalk_Input_t *in,
                               float vy_mm_s,
                               float start_height_mm,
                               float done_tolerance_mm);

void LiftWalk_Stop(LiftWalk_Controller_t *ctrl);
const LiftWalk_Output_t *LiftWalk_GetOutput(const LiftWalk_Controller_t *ctrl);

#endif /* R2_CHASSIS_LIFT_WALK_CONTROLLER_H */
