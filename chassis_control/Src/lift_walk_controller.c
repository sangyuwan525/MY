#include "lift_walk_controller.h"

#include <math.h>
#include <string.h>

#define LIFT_WALK_PI 3.14159265358979323846f
#define LIFT_WALK_TWO_PI 6.28318530717958647692f
#define LIFT_WALK_EPS 1.0e-6f

/* 本文件实现“边走边抬升”的协调控制。
 * 控制思路：
 * 1. 先把目标抬升高度 target_height_mm 变成平滑轨迹 height_ref_mm。
 * 2. 用 IMU 的 roll/pitch 做姿态闭环，把高度目标分配到四个支撑点。
 * 3. 后侧小米滑轨直接把支撑高度 z 转成电机角度。
 * 4. 前侧宇树小臂通过圆弧逆解，把支撑高度 z 转成小臂角度 theta。
 * 5. 前轮速度 = 底盘前进速度 + 小臂水平扫动速度补偿 + yaw 差速修正。
 * 6. 后轮速度 = 同一个底盘前进速度 + yaw 差速修正。
 *    这样前后轮都围绕同一个车体速度目标滚动，前轮只额外补偿小臂扫动。
 */

// 取浮点数绝对值。
static float lw_absf(float value) {
    return (value >= 0.0f) ? value : -value;
}

/* 通用限幅，所有位置/速度/姿态修正都走这里，避免输出突然越界。 */
//通用限幅函数。如果值超过最大值，就返回最大值；如果小于最小值，就返回最小值。
static float lw_clampf(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

/* 一阶斜坡限速。
 * 用于高度和高度速度，避免 target_height_mm 一变就直接阶跃到目标。
 * 当前值不能一下子跳到目标值，每个周期最多变化 max_step。
 */
static float lw_rate_limit(float current, float target, float max_step) {
    float delta = target - current;

    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

/* 全局电机编号检查。
 * 控制器支持把某个执行器配置成 LIFT_WALK_INVALID_MOTOR，此时不会下发对应命令。
 */
static uint8_t lw_motor_valid(int motor_index) {
    return (motor_index >= 0 && motor_index < MOTOR_TOTAL_NUM) ? 1U : 0U;
}

/* 统一位置接口：当前主要用于小米滑轨。
 * vel_limit 会继续传给电机适配层，由对应电机驱动解释单位。
* 检查电机编号是否合法；
* 检查这个电机有没有 set_position 函数；
* 如果有，就调用：
 */
static void lw_apply_position(int motor_index, float position, float vel_limit) {
    if (lw_motor_valid(motor_index) == 0U) {
        return;
    }
    if (g_motor_list[motor_index].set_position != NULL) {
        g_motor_list[motor_index].set_position(&g_motor_list[motor_index], position, vel_limit);
    }
}

/* 统一 MIT 接口：当前主要用于宇树小臂。
 * position/speed 是小臂关节角和角速度目标。
 */
static void lw_apply_mit(int motor_index, float position, float speed, float kp, float kd, float torque) {
    if (lw_motor_valid(motor_index) == 0U) {
        return;
    }
    if (g_motor_list[motor_index].set_mit != NULL) {
        g_motor_list[motor_index].set_mit(&g_motor_list[motor_index], position, speed, kp, kd, torque);
    }
}

/* 统一速度接口：当前主要用于两个前轮。 */
static void lw_apply_speed(int motor_index, float speed) {
    if (lw_motor_valid(motor_index) == 0U) {
        return;
    }
    if (g_motor_list[motor_index].set_speed != NULL) {
        g_motor_list[motor_index].set_speed(&g_motor_list[motor_index], speed);
    }
}

/* 把航向误差包到 [-pi, pi]，避免跨越 pi/-pi 时 yaw PID 突然跳变。 */
static float lw_wrap_pi(float angle) {
    while (angle > LIFT_WALK_PI) {
        angle -= LIFT_WALK_TWO_PI;
    }
    while (angle < -LIFT_WALK_PI) {
        angle += LIFT_WALK_TWO_PI;
    }
    return angle;
}

/* 小米滑轨高度 -> 电机角度。
 * 线性位移 z_mm / 丝杆导程 = 丝杆转数；
 * 丝杆转数 * 2pi * 减速比 = 电机角度增量。
 */
static float lw_slider_z_to_motor_rad(const LiftWalk_Config_t *cfg, uint8_t side, float z_mm) {
    float pitch = (cfg->slider_pitch_mm_per_rev > LIFT_WALK_EPS) ? cfg->slider_pitch_mm_per_rev : 1.0f;
    float motor_rad = cfg->slider_zero_rad[side] +
                      z_mm * LIFT_WALK_TWO_PI * cfg->slider_reduction_ratio / pitch;

    return lw_clampf(motor_rad, cfg->slider_min_rad[side], cfg->slider_max_rad[side]);
}

/* 宇树圆周小臂高度 -> 关节角度逆解。
 * 模型：
 *   z = z_pivot + L * sin(phi)
 *   x = L * cos(phi)
 *   theta = phi - zero_offset
 *
 * 同时计算：
 *   theta_dot = z_dot / (L * cos(phi))
 *   x_dot = -L * sin(phi) * theta_dot
 *
 * x_dot 后面用于前轮速度补偿。如果小臂转动导致轮心向前/向后扫动，
 * 前轮必须滚动配合，否则轮子会拖地并扰动底盘姿态。
 */
static LiftWalk_Status_e lw_arm_z_to_theta(const LiftWalk_Config_t *cfg,
                                           uint8_t side,
                                           float z_mm,
                                           float z_dot_mm_s,
                                           float *theta_rad,
                                           float *theta_dot_rad_s,
                                           float *x_dot_mm_s) {
    float length = cfg->arm_length_mm[side];
    float z_local;
    float sin_phi;
    float phi;
    float cos_phi;

    if (length <= LIFT_WALK_EPS) {
        return LIFT_WALK_ABORT_ARM_RANGE;
    }

    /* sin(phi) 超出 [-1, 1] 表示目标高度超过小臂几何可达范围。 */
    z_local = z_mm - cfg->arm_pivot_z_mm[side];
    sin_phi = z_local / length;
    if (sin_phi > 1.0f || sin_phi < -1.0f) {
        return LIFT_WALK_ABORT_ARM_RANGE;
    }

    sin_phi = lw_clampf(sin_phi, -1.0f, 1.0f);
    phi = asinf(sin_phi);
    cos_phi = cosf(phi);

    /* cos(phi) 太小表示接近竖直死点，少量高度变化需要巨大角速度，不适合继续抬升。 */
    if (lw_absf(cos_phi) < cfg->arm_dead_cos_min) {
        return LIFT_WALK_ABORT_ARM_DEAD_ZONE;
    }

    *theta_rad = lw_clampf(phi - cfg->arm_zero_offset_rad[side],
                           cfg->arm_min_rad[side],
                           cfg->arm_max_rad[side]);
    *theta_dot_rad_s = z_dot_mm_s / (length * cos_phi);
    *x_dot_mm_s = -length * sin_phi * (*theta_dot_rad_s);

    return LIFT_WALK_OK;
}

/* 抬升高度轨迹生成。
 * 这里不是完整 S 曲线，而是带加速度限制的梯形速度轨迹：
 * - 离目标较远时用 lift_vmax_mm_s；
 * - 按当前速度估算刹停距离，接近目标时把期望速度降到 0；
 * - 再用 lift_amax_mm_s2 限制速度变化。
 */
static void lw_step_height(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;
    float target = lw_clampf(in->target_height_mm, cfg->min_height_mm, cfg->max_height_mm);
    float dt = (in->dt_s > LIFT_WALK_EPS) ? in->dt_s : 0.001f;
    float desired_v;
    float stop_dist;
    float distance;
    float max_v_step;
    float max_h_step;

    distance = target - ctrl->out.height_ref_mm;
    stop_dist = (ctrl->out.height_dot_ref_mm_s * ctrl->out.height_dot_ref_mm_s) /
                (2.0f * cfg->lift_amax_mm_s2 + LIFT_WALK_EPS);

    if (lw_absf(distance) <= stop_dist) {
        desired_v = 0.0f;
    } else {
        desired_v = (distance >= 0.0f) ? cfg->lift_vmax_mm_s : -cfg->lift_vmax_mm_s;
    }

    max_v_step = cfg->lift_amax_mm_s2 * dt;
    ctrl->out.height_dot_ref_mm_s = lw_rate_limit(ctrl->out.height_dot_ref_mm_s, desired_v, max_v_step);

    max_h_step = lw_absf(ctrl->out.height_dot_ref_mm_s) * dt;
    ctrl->out.height_ref_mm = lw_rate_limit(ctrl->out.height_ref_mm, target, max_h_step);
    ctrl->last_height_target_mm = target;
}

void LiftWalk_DefaultConfig(LiftWalk_Config_t *cfg) {
    if (cfg == NULL) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    /* 默认电机绑定：
     * 两个小米作为后侧直线滑轨；
     * 两个宇树作为前侧圆周小臂；
     * DJI_YL/DJI_YR 作为前侧左右轮速度输出。
     */
    cfg->xiaomi_slider_motor[LIFT_WALK_LEFT] = XIAOMI_MOTOR1_G;
    cfg->xiaomi_slider_motor[LIFT_WALK_RIGHT] = XIAOMI_MOTOR2_G;
    cfg->unitree_arm_motor[LIFT_WALK_LEFT] = UNITREE_GO_M8010_6_MOTOR1_G;
    cfg->unitree_arm_motor[LIFT_WALK_RIGHT] = UNITREE_GO_M8010_6_MOTOR2_G;
    cfg->front_wheel_motor[LIFT_WALK_LEFT] = DM_JOINT_G;
    cfg->front_wheel_motor[LIFT_WALK_RIGHT] = DM_FRONT_RIGHT_G;
    cfg->rear_wheel_motor[LIFT_WALK_LEFT] = BLAZER_FOC_MOTOR1_G;
    cfg->rear_wheel_motor[LIFT_WALK_RIGHT] = BLAZER_FOC_MOTOR2_G;

    /* 四支撑点坐标只是占位值。
     * 实车调试时应改成支撑点相对底盘中心的真实 x/y 尺寸。
     */
    cfg->support_pos[LIFT_WALK_FRONT_LEFT] = (LiftWalk_Point_t){ .x_mm = 250.0f, .y_mm = 180.0f };
    cfg->support_pos[LIFT_WALK_FRONT_RIGHT] = (LiftWalk_Point_t){ .x_mm = 250.0f, .y_mm = -180.0f };
    cfg->support_pos[LIFT_WALK_REAR_LEFT] = (LiftWalk_Point_t){ .x_mm = -250.0f, .y_mm = 180.0f };
    cfg->support_pos[LIFT_WALK_REAR_RIGHT] = (LiftWalk_Point_t){ .x_mm = -250.0f, .y_mm = -180.0f };

    /* 滑轨默认参数是保守占位：
     * 丝杆导程、减速比、零点和软限位必须根据实际机构修正。
     */
    cfg->slider_pitch_mm_per_rev = 5.0f;
    cfg->slider_reduction_ratio = 1.0f;
    cfg->slider_min_rad[LIFT_WALK_LEFT] = -1000.0f;
    cfg->slider_min_rad[LIFT_WALK_RIGHT] = -1000.0f;
    cfg->slider_max_rad[LIFT_WALK_LEFT] = 1000.0f;
    cfg->slider_max_rad[LIFT_WALK_RIGHT] = 1000.0f;
    cfg->slider_vel_limit_rad_s = 2.0f;

    /* 小臂默认按 180 mm 长度估算，MIT 刚度较低，方便第一次上电调试。 */
    cfg->arm_length_mm[LIFT_WALK_LEFT] = 180.0f;
    cfg->arm_length_mm[LIFT_WALK_RIGHT] = 180.0f;
    cfg->arm_min_rad[LIFT_WALK_LEFT] = -1.3f;
    cfg->arm_min_rad[LIFT_WALK_RIGHT] = -1.3f;
    cfg->arm_max_rad[LIFT_WALK_LEFT] = 1.3f;
    cfg->arm_max_rad[LIFT_WALK_RIGHT] = 1.3f;
    cfg->arm_dead_cos_min = 0.2f;
    cfg->arm_kp = 0.08f;
    cfg->arm_kd = 0.01f;
    cfg->arm_torque_ff = 0.0f;

    /* 轮速补偿参数。
     * 前轮是小臂末端达妙，后轮是 Blazer FOC。
     * 如果抬升时车体被小臂拖拽，优先调 wheel_arm_comp_gain 和 front_wheel_sign。
     */
    cfg->front_wheel_radius_mm = 50.0f;
    cfg->rear_wheel_radius_mm = 50.0f;
    cfg->rear_wheel_drive_angle_rad[LIFT_WALK_LEFT] = LIFT_WALK_PI / 4.0f;
    cfg->rear_wheel_drive_angle_rad[LIFT_WALK_RIGHT] = -LIFT_WALK_PI / 4.0f;
    cfg->front_wheel_speed_limit_rad_s = 25.0f;
    cfg->rear_wheel_speed_limit_rpm = 3000.0f;
    cfg->wheel_arm_comp_gain = 1.0f;
    cfg->front_wheel_sign[LIFT_WALK_LEFT] = 1.0f;
    cfg->front_wheel_sign[LIFT_WALK_RIGHT] = 1.0f;
    cfg->rear_wheel_sign[LIFT_WALK_LEFT] = 1.0f;
    cfg->rear_wheel_sign[LIFT_WALK_RIGHT] = 1.0f;

    /* 姿态 PID 默认只给较小修正，避免第一次调试时四点高度大幅摆动。 */
    cfg->roll_kp = 0.8f;
    cfg->roll_ki = 0.0f;
    cfg->roll_kd = 0.06f;
    cfg->pitch_kp = 0.8f;
    cfg->pitch_ki = 0.0f;
    cfg->pitch_kd = 0.06f;
    cfg->yaw_kp = 0.8f;
    cfg->yaw_kd = 0.03f;

    /* 姿态和高度安全限制。
     * roll/pitch 约 0.17 rad，约等于 9.7 度，超过后会停止前轮输出。
     */
    cfg->attitude_cmd_limit_rad = 0.08f;
    cfg->support_corr_limit_mm = 25.0f;
    cfg->max_roll_rad = 0.17f;
    cfg->max_pitch_rad = 0.17f;

    cfg->min_height_mm = 0.0f;
    cfg->max_height_mm = 160.0f;
    cfg->lift_vmax_mm_s = 35.0f;
    cfg->lift_amax_mm_s2 = 100.0f;
}

/* 初始化控制器。
 * 注意：此函数不会自动下发电机命令，只是准备控制器内部状态。
 */
void LiftWalk_Init(LiftWalk_Controller_t *ctrl, const LiftWalk_Config_t *cfg) {
    if (ctrl == NULL) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    if (cfg != NULL) {
        ctrl->cfg = *cfg;
    } else {
        LiftWalk_DefaultConfig(&ctrl->cfg);
    }
    ctrl->out.status = LIFT_WALK_OK;
    ctrl->initialized = 1U;
}

/* 重置控制器状态。
 * 一般在进入“边走边抬升”状态机时调用，用当前高度作为轨迹起点。
 */
void LiftWalk_Reset(LiftWalk_Controller_t *ctrl, float current_height_mm) {
    if (ctrl == NULL) {
        return;
    }

    ctrl->roll_i = 0.0f;
    ctrl->pitch_i = 0.0f;
    ctrl->out.height_ref_mm = lw_clampf(current_height_mm, ctrl->cfg.min_height_mm, ctrl->cfg.max_height_mm);
    ctrl->out.height_dot_ref_mm_s = 0.0f;
    ctrl->last_height_target_mm = ctrl->out.height_ref_mm;
    ctrl->lift_action_active = 0U;
    ctrl->lift_action_done = 0U;
    ctrl->lift_action_target_mm = ctrl->out.height_ref_mm;
    ctrl->out.status = LIFT_WALK_OK;
}

/* 主控制函数，建议 5 ms 左右调用一次。
 *
 * 输出逻辑：
 * - enable_motor_output = 0: 只更新 ctrl->out，适合先打印观察。
 * - enable_motor_output = 1: 同时下发滑轨、小臂和前轮电机命令。
 */
LiftWalk_Status_e LiftWalk_Update(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in) {
    LiftWalk_Config_t *cfg;
    float dt;
    float roll_err;
    float pitch_err;
    float yaw_err;
    float rear_left_z;
    float rear_right_z;
    LiftWalk_Status_e arm_status;

    if (ctrl == NULL || in == NULL) {
        return LIFT_WALK_ABORT_HEIGHT_LIMIT;
    }
    if (ctrl->initialized == 0U) {
        LiftWalk_Init(ctrl, NULL);
    }

    cfg = &ctrl->cfg;
    dt = (in->dt_s > LIFT_WALK_EPS) ? in->dt_s : 0.001f;

    /* 先做姿态硬保护。姿态已经明显歪斜时，不继续计算新的抬升命令。 */
    if (lw_absf(in->roll_rad) > cfg->max_roll_rad) {
        ctrl->out.status = LIFT_WALK_ABORT_ROLL_LIMIT;
        LiftWalk_Stop(ctrl);
        return ctrl->out.status;
    }
    if (lw_absf(in->pitch_rad) > cfg->max_pitch_rad) {
        ctrl->out.status = LIFT_WALK_ABORT_PITCH_LIMIT;
        LiftWalk_Stop(ctrl);
        return ctrl->out.status;
    }

    /* 生成平滑高度轨迹，避免 target_height_mm 阶跃造成冲击。 */
    lw_step_height(ctrl, in);

    /* 姿态目标为 roll=0、pitch=0。
     * 这里用误差 = 目标 - 实测，所以直接取 -roll/-pitch。
     */
    roll_err = -in->roll_rad;
    pitch_err = -in->pitch_rad;
    ctrl->roll_i = lw_clampf(ctrl->roll_i + roll_err * dt, -0.2f, 0.2f);
    ctrl->pitch_i = lw_clampf(ctrl->pitch_i + pitch_err * dt, -0.2f, 0.2f);

    ctrl->out.roll_cmd_rad = cfg->roll_kp * roll_err +
                             cfg->roll_ki * ctrl->roll_i -
                             cfg->roll_kd * in->roll_rate_rad_s;
    ctrl->out.pitch_cmd_rad = cfg->pitch_kp * pitch_err +
                              cfg->pitch_ki * ctrl->pitch_i -
                              cfg->pitch_kd * in->pitch_rate_rad_s;
    ctrl->out.roll_cmd_rad = lw_clampf(ctrl->out.roll_cmd_rad,
                                       -cfg->attitude_cmd_limit_rad,
                                       cfg->attitude_cmd_limit_rad);
    ctrl->out.pitch_cmd_rad = lw_clampf(ctrl->out.pitch_cmd_rad,
                                        -cfg->attitude_cmd_limit_rad,
                                        cfg->attitude_cmd_limit_rad);

    /* yaw 只用于前轮左右差速修正，不直接参与四支撑点高度。 */
    yaw_err = lw_wrap_pi(in->yaw_ref_rad - in->yaw_rad);
    ctrl->out.yaw_cmd_rad_s = cfg->yaw_kp * yaw_err - cfg->yaw_kd * in->yaw_rate_rad_s;

    /* 把底盘姿态修正分配到四个支撑点。
     * 公式：
     *   z_i = height_ref + y_i * roll_cmd - x_i * pitch_cmd
     * 如果修正方向和实车相反，通常改 IMU 轴方向或调换这里的符号。
     */
    for (uint8_t i = 0U; i < LIFT_WALK_SUPPORT_COUNT; ++i) {
        float correction = cfg->support_pos[i].y_mm * ctrl->out.roll_cmd_rad -
                           cfg->support_pos[i].x_mm * ctrl->out.pitch_cmd_rad;
        correction = lw_clampf(correction, -cfg->support_corr_limit_mm, cfg->support_corr_limit_mm);
        ctrl->out.support_z_mm[i] = lw_clampf(ctrl->out.height_ref_mm + correction,
                                              cfg->min_height_mm,
                                              cfg->max_height_mm);
    }

    /* 后侧两个支撑点由小米滑轨承担，高度直接换算为滑轨电机位置。 */
    rear_left_z = ctrl->out.support_z_mm[LIFT_WALK_REAR_LEFT];
    rear_right_z = ctrl->out.support_z_mm[LIFT_WALK_REAR_RIGHT];
    ctrl->out.slider_motor_rad[LIFT_WALK_LEFT] = lw_slider_z_to_motor_rad(cfg, LIFT_WALK_LEFT, rear_left_z);
    ctrl->out.slider_motor_rad[LIFT_WALK_RIGHT] = lw_slider_z_to_motor_rad(cfg, LIFT_WALK_RIGHT, rear_right_z);

    /* 前侧两个支撑点由宇树小臂承担，需要圆弧逆解。
     * 任意一侧不可达或接近死点时，立即停止前轮输出并返回保护状态。
     */
    arm_status = lw_arm_z_to_theta(cfg,
                                   LIFT_WALK_LEFT,
                                   ctrl->out.support_z_mm[LIFT_WALK_FRONT_LEFT],
                                   ctrl->out.height_dot_ref_mm_s,
                                   &ctrl->out.arm_theta_rad[LIFT_WALK_LEFT],
                                   &ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_LEFT],
                                   &ctrl->out.arm_x_dot_mm_s[LIFT_WALK_LEFT]);
    if (arm_status != LIFT_WALK_OK) {
        ctrl->out.status = arm_status;
        LiftWalk_Stop(ctrl);
        return ctrl->out.status;
    }

    arm_status = lw_arm_z_to_theta(cfg,
                                   LIFT_WALK_RIGHT,
                                   ctrl->out.support_z_mm[LIFT_WALK_FRONT_RIGHT],
                                   ctrl->out.height_dot_ref_mm_s,
                                   &ctrl->out.arm_theta_rad[LIFT_WALK_RIGHT],
                                   &ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_RIGHT],
                                   &ctrl->out.arm_x_dot_mm_s[LIFT_WALK_RIGHT]);
    if (arm_status != LIFT_WALK_OK) {
        ctrl->out.status = arm_status;
        LiftWalk_Stop(ctrl);
        return ctrl->out.status;
    }

    /* 前后轮速度计算。
     * rear_linear = 底盘期望前进速度 + yaw 差速修正。
     * front_linear = rear_linear + 小臂水平扫动补偿。
     *
     * 这样后轮按底盘目标速度滚动，前轮由于安装在小臂末端，需要额外补偿轮心相对底盘的水平速度。
     * 从地面视角看，前后轮接地点的滚动速度会尽量接近同一个 vx_mm_s。
     */

    for (uint8_t side = 0U; side < LIFT_WALK_SIDE_COUNT; ++side) {
        /*
         * Blazer 后轮是 45 度全向轮：先计算后轮安装点的刚体速度，
         * 再把 vx/vy 投影到该轮子的驱动方向 alpha 上。
         * alpha=+45 度表示左前方向，alpha=-45 度表示右前方向。
         */
        uint8_t front_idx = (side == LIFT_WALK_LEFT) ? LIFT_WALK_FRONT_LEFT : LIFT_WALK_FRONT_RIGHT;
        uint8_t rear_idx = (side == LIFT_WALK_LEFT) ? LIFT_WALK_REAR_LEFT : LIFT_WALK_REAR_RIGHT;
        float yaw_rate = ctrl->out.yaw_cmd_rad_s;
        float front_vx_mm_s = in->vx_mm_s - yaw_rate * cfg->support_pos[front_idx].y_mm;
        float rear_vx_mm_s = in->vx_mm_s - yaw_rate * cfg->support_pos[rear_idx].y_mm;
        float rear_vy_mm_s = in->vy_mm_s + yaw_rate * cfg->support_pos[rear_idx].x_mm;
        float rear_drive_angle = cfg->rear_wheel_drive_angle_rad[side];
        float rear_drive_mm_s = rear_vx_mm_s * cosf(rear_drive_angle) +
                                rear_vy_mm_s * sinf(rear_drive_angle);
        float front_linear_mm_s = front_vx_mm_s +
                                  cfg->wheel_arm_comp_gain * ctrl->out.arm_x_dot_mm_s[side];
        float rear_circumference_mm = LIFT_WALK_TWO_PI * cfg->rear_wheel_radius_mm;
        float front_rad_s = 0.0f;
        float rear_rpm = 0.0f;

        if (cfg->front_wheel_radius_mm > LIFT_WALK_EPS) {
            front_rad_s = front_linear_mm_s / cfg->front_wheel_radius_mm;
        }
        if (rear_circumference_mm > LIFT_WALK_EPS) {
            rear_rpm = rear_drive_mm_s * 60.0f / rear_circumference_mm;
        }
        front_rad_s *= cfg->front_wheel_sign[side];
        rear_rpm *= cfg->rear_wheel_sign[side];
        ctrl->out.front_wheel_rad_s[side] = lw_clampf(front_rad_s,
                                                      -cfg->front_wheel_speed_limit_rad_s,
                                                      cfg->front_wheel_speed_limit_rad_s);
        ctrl->out.rear_wheel_rpm[side] = lw_clampf(rear_rpm,
                                                   -cfg->rear_wheel_speed_limit_rpm,
                                                   cfg->rear_wheel_speed_limit_rpm);
    }

    ctrl->out.status = LIFT_WALK_OK;

    /* 真正下发电机命令。
     * 调试初期建议先把 enable_motor_output 置 0，只观察 out 是否符合预期。
     */
    if (in->enable_motor_output != 0U) {
        lw_apply_position(cfg->xiaomi_slider_motor[LIFT_WALK_LEFT],
                          ctrl->out.slider_motor_rad[LIFT_WALK_LEFT],
                          cfg->slider_vel_limit_rad_s);
        lw_apply_position(cfg->xiaomi_slider_motor[LIFT_WALK_RIGHT],
                          ctrl->out.slider_motor_rad[LIFT_WALK_RIGHT],
                          cfg->slider_vel_limit_rad_s);
        lw_apply_mit(cfg->unitree_arm_motor[LIFT_WALK_LEFT],
                     ctrl->out.arm_theta_rad[LIFT_WALK_LEFT],
                     ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_LEFT],
                     cfg->arm_kp,
                     cfg->arm_kd,
                     cfg->arm_torque_ff);
        lw_apply_mit(cfg->unitree_arm_motor[LIFT_WALK_RIGHT],
                     ctrl->out.arm_theta_rad[LIFT_WALK_RIGHT],
                     ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_RIGHT],
                     cfg->arm_kp,
                     cfg->arm_kd,
                     cfg->arm_torque_ff);
        lw_apply_speed(cfg->front_wheel_motor[LIFT_WALK_LEFT], ctrl->out.front_wheel_rad_s[LIFT_WALK_LEFT]);
        lw_apply_speed(cfg->front_wheel_motor[LIFT_WALK_RIGHT], ctrl->out.front_wheel_rad_s[LIFT_WALK_RIGHT]);
        lw_apply_speed(cfg->rear_wheel_motor[LIFT_WALK_LEFT], ctrl->out.rear_wheel_rpm[LIFT_WALK_LEFT]);
        lw_apply_speed(cfg->rear_wheel_motor[LIFT_WALK_RIGHT], ctrl->out.rear_wheel_rpm[LIFT_WALK_RIGHT]);
    }

    return ctrl->out.status;
}

/* 停止函数目前只清零前轮速度，并冻结高度轨迹速度。
 * 小臂和滑轨不在这里强制回零，是为了避免保护触发时机构突然下坠或猛动。
 */
/* 底盘抬升动作：执行中返回 0，到达目标高度且速度降下来后返回 1。 */
uint8_t LiftWalk_RunLiftAction(LiftWalk_Controller_t *ctrl,
                               const LiftWalk_Input_t *in,
                               float vx_mm_s,
                               float start_height_mm,
                               float done_tolerance_mm) {
    LiftWalk_Input_t action_in;
    float target;
    float tolerance;
    LiftWalk_Status_e status;

    if (ctrl == NULL || in == NULL) {
        return 0U;
    }
    if (ctrl->initialized == 0U) {
        LiftWalk_Init(ctrl, NULL);
    }

    action_in = *in;
    action_in.vx_mm_s = vx_mm_s;

    target = lw_clampf(action_in.target_height_mm, ctrl->cfg.min_height_mm, ctrl->cfg.max_height_mm);
    tolerance = (done_tolerance_mm > LIFT_WALK_EPS) ? done_tolerance_mm : 2.0f;

    if (lw_absf(target - ctrl->lift_action_target_mm) > tolerance) {
        ctrl->lift_action_active = 0U;
        ctrl->lift_action_done = 0U;
    }

    if (ctrl->lift_action_done != 0U) {
        return 1U;
    }

    if (ctrl->lift_action_active == 0U) {
        LiftWalk_Reset(ctrl, start_height_mm);
        ctrl->lift_action_active = 1U;
        ctrl->lift_action_done = 0U;
        ctrl->lift_action_target_mm = target;
    }

    status = LiftWalk_Update(ctrl, &action_in);
    if (status != LIFT_WALK_OK) {
        ctrl->lift_action_active = 0U;
        ctrl->lift_action_done = 0U;
        return 0U;
    }

    if (lw_absf(ctrl->out.height_ref_mm - target) <= tolerance &&
        lw_absf(ctrl->out.height_dot_ref_mm_s) <= (ctrl->cfg.lift_amax_mm_s2 * action_in.dt_s + LIFT_WALK_EPS)) {
        ctrl->lift_action_active = 0U;
        ctrl->lift_action_done = 1U;
        return 1U;
    }

    return 0U;
}

/* 停止行走轮并冻结抬升动作状态；滑轨和小臂保持最后一次位置命令。 */
void LiftWalk_Stop(LiftWalk_Controller_t *ctrl) {
    if (ctrl == NULL) {
        return;
    }

    ctrl->out.height_dot_ref_mm_s = 0.0f;
    ctrl->out.front_wheel_rad_s[LIFT_WALK_LEFT] = 0.0f;
    ctrl->out.front_wheel_rad_s[LIFT_WALK_RIGHT] = 0.0f;
    ctrl->out.rear_wheel_rpm[LIFT_WALK_LEFT] = 0.0f;
    ctrl->out.rear_wheel_rpm[LIFT_WALK_RIGHT] = 0.0f;
    ctrl->lift_action_active = 0U;

    lw_apply_speed(ctrl->cfg.front_wheel_motor[LIFT_WALK_LEFT], 0.0f);
    lw_apply_speed(ctrl->cfg.front_wheel_motor[LIFT_WALK_RIGHT], 0.0f);
    lw_apply_speed(ctrl->cfg.rear_wheel_motor[LIFT_WALK_LEFT], 0.0f);
    lw_apply_speed(ctrl->cfg.rear_wheel_motor[LIFT_WALK_RIGHT], 0.0f);
}

/* 返回最近一次输出，供调试打印或上层状态机判断。 */
const LiftWalk_Output_t *LiftWalk_GetOutput(const LiftWalk_Controller_t *ctrl) {
    if (ctrl == NULL) {
        return NULL;
    }
    return &ctrl->out;
}
