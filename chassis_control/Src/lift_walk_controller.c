#include "lift_walk_controller.h"

#include <math.h>
#include <string.h>

// =========================
// 1. 机械硬参数
// =========================
// 这里放真实机械结构决定的常量：长度、轮径、减速比、滑轨安装角等。
// PID、零点、方向符号、限幅这类需要调车的量仍然放在 LiftWalk_Config_t 里。
#define LIFT_WALK_PI 3.14159265358979323846f
#define LIFT_WALK_TWO_PI 6.28318530717958647692f
#define LIFT_WALK_EPS 1.0e-6f

#define LIFT_WALK_UNITREE_MOTOR_RAD_PER_ARM_RAD (2.0f / 3.0f)
#define LIFT_WALK_ARM_LENGTH_MM 217.75f
#define LIFT_WALK_ARM_PIVOT_HEIGHT_MM 60.0f

#define LIFT_WALK_FRONT_WHEEL_DIAMETER_MM 83.0f
#define LIFT_WALK_FRONT_WHEEL_RADIUS_MM (LIFT_WALK_FRONT_WHEEL_DIAMETER_MM * 0.5f)
#define LIFT_WALK_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD (74.0f / 24.0f)
#define LIFT_WALK_REAR_WHEEL_RADIUS_MM 60.0f

#define LIFT_WALK_SLIDER_TRAVEL_MM_PER_MOTOR_REV 180.0f
#define LIFT_WALK_SLIDER_ANGLE_FROM_VERTICAL_RAD (7.5f * LIFT_WALK_PI / 180.0f)

// =========================
// 2. 基础数学工具
// =========================
static float lw_absf(float value) {
    return (value >= 0.0f) ? value : -value;
}

static float lw_clampf(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

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

// =========================
// 3. 电机下发包装
// =========================
static uint8_t lw_motor_valid(int motor_index) {
    return (motor_index >= 0 && motor_index < MOTOR_TOTAL_NUM) ? 1U : 0U;
}

static void lw_apply_position(int motor_index, float position, float vel_limit) {
    if (lw_motor_valid(motor_index) == 0U) {
        return;
    }
    if (g_motor_list[motor_index].set_position != NULL) {
        g_motor_list[motor_index].set_position(&g_motor_list[motor_index], position, vel_limit);
    }
}

static void lw_apply_mit(int motor_index, float position, float speed, float kp, float kd, float torque) {
    if (lw_motor_valid(motor_index) == 0U) {
        return;
    }
    if (g_motor_list[motor_index].set_mit != NULL) {
        g_motor_list[motor_index].set_mit(&g_motor_list[motor_index], position, speed, kp, kd, torque);
    }
}

static void lw_apply_speed(int motor_index, float speed) {
    if (lw_motor_valid(motor_index) == 0U) {
        return;
    }
    if (g_motor_list[motor_index].set_speed != NULL) {
        g_motor_list[motor_index].set_speed(&g_motor_list[motor_index], speed);
    }
}

// =========================
// 4. 机械量换算
// =========================
static float lw_arm_rad_to_unitree_motor_rad(const LiftWalk_Config_t *cfg, uint8_t side, float arm_rad) {
    return cfg->arm_motor_zero_rad[side] +
           cfg->arm_motor_sign[side] * arm_rad * LIFT_WALK_UNITREE_MOTOR_RAD_PER_ARM_RAD;
}

static float lw_arm_rad_s_to_unitree_motor_rad_s(const LiftWalk_Config_t *cfg, uint8_t side, float arm_rad_s) {
    return cfg->arm_motor_sign[side] * arm_rad_s * LIFT_WALK_UNITREE_MOTOR_RAD_PER_ARM_RAD;
}

static float lw_wheel_rad_s_to_dm_motor_rad_s(float wheel_rad_s) {
    return wheel_rad_s * LIFT_WALK_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD;
}

// lift_mm 是后侧底盘角相对未抬升状态的抬升量。
// 小米滑轨沿导轨方向运动，导轨相对竖直方向偏 7.5 度，所以竖直高度只占
// 导轨位移的 cos(7.5 deg)。
static float lw_slider_lift_to_motor_rad(const LiftWalk_Config_t *cfg, uint8_t side, float lift_mm) {
    float rail_mm_per_rev = (cfg->slider_pitch_mm_per_rev > LIFT_WALK_EPS) ?
                            cfg->slider_pitch_mm_per_rev :
                            1.0f;
    float vertical_mm_per_rev = rail_mm_per_rev * cosf(LIFT_WALK_SLIDER_ANGLE_FROM_VERTICAL_RAD);
    float motor_rad;

    if (vertical_mm_per_rev <= LIFT_WALK_EPS) {
        vertical_mm_per_rev = 1.0f;
    }

    motor_rad = cfg->slider_zero_rad[side] +
                cfg->slider_motor_sign[side] *
                lift_mm * LIFT_WALK_TWO_PI * cfg->slider_reduction_ratio / vertical_mm_per_rev;

    return lw_clampf(motor_rad, cfg->slider_min_rad[side], cfg->slider_max_rad[side]);
}

// 前侧底盘角抬升量 -> 小臂角度逆解。
// 几何模型：
//   pivot_height = pivot_height_0 + lift
//   wheel_center_height = wheel_radius
//   wheel_center_height = pivot_height + L * sin(phi)
//   theta = phi - zero_offset
// theta 表示小臂机构角。真正下发给宇树时才换算成宇树电机角，
// 这样 ctrl->out 里保留的是更直观的机构量，方便 RTT 打印调试。
static LiftWalk_Status_e lw_arm_lift_to_theta(const LiftWalk_Config_t *cfg,
                                              uint8_t side,
                                              float lift_mm,
                                              float lift_dot_mm_s,      //地盘抬升速度
                                              float *theta_rad,
                                              float *theta_dot_rad_s,   //小臂角速度
                                              float *x_dot_mm_s) {      //小臂末端轮心的水平速度
    float length = cfg->arm_length_mm[side];
    float pivot_height_mm;              //  转轴高度
    float wheel_center_height_mm;       //  小臂末端前轮中心高度
    float sin_phi;
    float phi;
    float cos_phi;

    if (length <= LIFT_WALK_EPS) {
        return LIFT_WALK_ABORT_ARM_RANGE;
    }

    pivot_height_mm = cfg->arm_pivot_z_mm[side] + lift_mm;
    wheel_center_height_mm = cfg->front_wheel_radius_mm;
    sin_phi = (wheel_center_height_mm - pivot_height_mm) / length;
    if (sin_phi > 1.0f || sin_phi < -1.0f) {
        return LIFT_WALK_ABORT_ARM_RANGE;
    }

    sin_phi = lw_clampf(sin_phi, -1.0f, 1.0f);
    phi = asinf(sin_phi);
    cos_phi = cosf(phi);

    // cos(phi) 过小说明小臂接近竖直死区：
    // 很小的高度变化会需要很大的关节角速度，所以这里直接保护退出。
    if (lw_absf(cos_phi) < cfg->arm_dead_cos_min) {
        return LIFT_WALK_ABORT_ARM_DEAD_ZONE;
    }

    *theta_rad = lw_clampf(phi - cfg->arm_zero_offset_rad[side],
                           cfg->arm_min_rad[side],
                           cfg->arm_max_rad[side]);
    /*这是根据抬升速度 lift_dot_mm_s，反算小臂应该转多快。
    几何关系是：
    wheel_center_height = pivot_height + length * sin(phi)
    轮子压在地面，wheel_center_height 基本不变；抬升时 pivot_height 变高，所以 phi 要变化。
    对时间求导：
    0 = lift_dot + length * cos(phi) * phi_dot
    所以：
    phi_dot = -lift_dot / (length * cos(phi))
    代码里的 theta_dot_rad_s 基本就是这个小臂角速度。
    它后面用于宇树 MIT 控制的速度前馈：
    lw_apply_mit(..., position, speed, kp, kd, torque)
    也就是说宇树不仅收到“转到哪个角度”，还收到“应该以多快速度过去”。*/
    *theta_dot_rad_s = -lift_dot_mm_s / (length * cos_phi);
    /*这是算小臂末端轮子中心在水平方向的速度。
    因为小臂末端轮心的水平位置可以近似写成：
    x = length * cos(phi)
    对时间求导：
    x_dot = -length * sin(phi) * phi_dot
    所以代码就是这个公式。
    它后面用于前轮达妙速度补偿：
    front_linear_mm_s =
        front_vx_mm_s + wheel_arm_comp_gain * arm_x_dot_mm_s;
    意思是：小臂转动时，前轮轮心会前后扫动。为了让轮子不要在地上拖拽，需要给前轮额外加一点速度补偿。*/
    *x_dot_mm_s = -length * sin_phi * (*theta_dot_rad_s);

    return LIFT_WALK_OK;
}

// =========================
// 5. 单步控制计算
// =========================
/*
  *检查当前底盘姿态角有没有超过安全范围。
  *如果 roll 或 pitch 太大，就认为车身姿态危险，立即停止控制输出。
 */
static LiftWalk_Status_e lw_check_attitude_limits(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;

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

    return LIFT_WALK_OK;
}

// 生成带速度/加速度限制的平滑高度轨迹。
// 这样 target_height_mm 突然变化时，滑轨和小臂不会直接跳到目标位置。
/*把用户输入的目标抬升高度 target_height_mm，转换成一个平滑变化的高度轨迹。
它不会让高度目标一下子跳变，而是按照最大速度 lift_vmax_mm_s 和最大加速度 lift_amax_mm_s2 慢慢逼近目标高度。*/
static void lw_step_height(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;
    float target = lw_clampf(in->target_height_mm, cfg->min_height_mm, cfg->max_height_mm);
    float dt = (in->dt_s > LIFT_WALK_EPS) ? in->dt_s : 0.001f;
    float distance = target - ctrl->out.height_ref_mm;
    float stop_dist = (ctrl->out.height_dot_ref_mm_s * ctrl->out.height_dot_ref_mm_s) /
                      (2.0f * cfg->lift_amax_mm_s2 + LIFT_WALK_EPS);
    float desired_v;
    float max_v_step;
    float max_h_step;

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

static void lw_update_attitude_pid(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in, float dt) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;
    float roll_err = -in->roll_rad;
    float pitch_err = -in->pitch_rad;

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

    ctrl->out.yaw_cmd_rad_s = 0.0f;
}

// 把底盘姿态修正量分配到四个底盘角抬升量。
// support_pos 是支撑点相对底盘中心的力臂，位置越远，同样姿态角产生的高度差越大。
static void lw_update_support_heights(LiftWalk_Controller_t *ctrl) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;

    for (uint8_t i = 0U; i < LIFT_WALK_SUPPORT_COUNT; ++i) {
        float correction = cfg->support_pos[i].y_mm * ctrl->out.roll_cmd_rad -
                           cfg->support_pos[i].x_mm * ctrl->out.pitch_cmd_rad;
        correction = lw_clampf(correction, -cfg->support_corr_limit_mm, cfg->support_corr_limit_mm);
        ctrl->out.support_z_mm[i] = lw_clampf(ctrl->out.height_ref_mm + correction,
                                              cfg->min_height_mm,
                                              cfg->max_height_mm);
    }
}

static void lw_update_slider_targets(LiftWalk_Controller_t *ctrl) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;

    ctrl->out.slider_motor_rad[LIFT_WALK_LEFT] =
        lw_slider_lift_to_motor_rad(cfg, LIFT_WALK_LEFT, ctrl->out.support_z_mm[LIFT_WALK_REAR_LEFT]);
    ctrl->out.slider_motor_rad[LIFT_WALK_RIGHT] =
        lw_slider_lift_to_motor_rad(cfg, LIFT_WALK_RIGHT, ctrl->out.support_z_mm[LIFT_WALK_REAR_RIGHT]);
}

static LiftWalk_Status_e lw_update_arm_targets(LiftWalk_Controller_t *ctrl) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;
    LiftWalk_Status_e status;

    status = lw_arm_lift_to_theta(cfg,
                                  LIFT_WALK_LEFT,
                                  ctrl->out.support_z_mm[LIFT_WALK_FRONT_LEFT],
                                  ctrl->out.height_dot_ref_mm_s,
                                  &ctrl->out.arm_theta_rad[LIFT_WALK_LEFT],
                                  &ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_LEFT],
                                  &ctrl->out.arm_x_dot_mm_s[LIFT_WALK_LEFT]);
    if (status != LIFT_WALK_OK) {
        ctrl->out.status = status;
        LiftWalk_Stop(ctrl);
        return status;
    }

    status = lw_arm_lift_to_theta(cfg,
                                  LIFT_WALK_RIGHT,
                                  ctrl->out.support_z_mm[LIFT_WALK_FRONT_RIGHT],
                                  ctrl->out.height_dot_ref_mm_s,
                                  &ctrl->out.arm_theta_rad[LIFT_WALK_RIGHT],
                                  &ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_RIGHT],
                                  &ctrl->out.arm_x_dot_mm_s[LIFT_WALK_RIGHT]);
    if (status != LIFT_WALK_OK) {
        ctrl->out.status = status;
        LiftWalk_Stop(ctrl);
        return status;
    }

    return LIFT_WALK_OK;
}

// 抬升行走时的轮速计算：
// 1. 前轮在小臂末端，小臂转动会带来轮心前后扫动，需要叠加补偿速度。
// 2. 后轮是 45 度全向轮，需要把底盘 vx/vy/yaw 投影到后轮驱动方向上。
static void lw_update_wheel_targets(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;
    float dt = (in->dt_s > LIFT_WALK_EPS) ? in->dt_s : 0.001f;
    float front_max_step = cfg->front_wheel_accel_limit_rad_s2 * dt;
    float rear_max_step = cfg->rear_wheel_accel_limit_rpm_s * dt;

    for (uint8_t side = 0U; side < LIFT_WALK_SIDE_COUNT; ++side) {
        float forward_y_mm_s = in->vy_mm_s;
        float rear_drive_angle = cfg->rear_wheel_drive_angle_rad[side];
        float rear_drive_mm_s = forward_y_mm_s * sinf(rear_drive_angle);
        float front_linear_mm_s = forward_y_mm_s +
                                  cfg->wheel_arm_comp_gain * ctrl->out.arm_x_dot_mm_s[side];
        float rear_circumference_mm = LIFT_WALK_TWO_PI * cfg->rear_wheel_radius_mm;     //  周长
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

        front_rad_s = lw_clampf(front_rad_s,
                                -cfg->front_wheel_speed_limit_rad_s,
                                cfg->front_wheel_speed_limit_rad_s);
        rear_rpm = lw_clampf(rear_rpm,
                             -cfg->rear_wheel_speed_limit_rpm,
                             cfg->rear_wheel_speed_limit_rpm);

        if (front_max_step > LIFT_WALK_EPS) {
            ctrl->out.front_wheel_rad_s[side] = lw_rate_limit(ctrl->out.front_wheel_rad_s[side],
                                                              front_rad_s,
                                                              front_max_step);
        } else {
            ctrl->out.front_wheel_rad_s[side] = front_rad_s;
        }

        if (rear_max_step > LIFT_WALK_EPS) {
            ctrl->out.rear_wheel_rpm[side] = lw_rate_limit(ctrl->out.rear_wheel_rpm[side],
                                                           rear_rpm,
                                                           rear_max_step);
        } else {
            ctrl->out.rear_wheel_rpm[side] = rear_rpm;
        }
    }
}

static void lw_apply_motor_outputs(LiftWalk_Controller_t *ctrl) {
    LiftWalk_Config_t *cfg = &ctrl->cfg;

    lw_apply_position(cfg->xiaomi_slider_motor[LIFT_WALK_LEFT],
                      ctrl->out.slider_motor_rad[LIFT_WALK_LEFT],
                      cfg->slider_vel_limit_rad_s);
    lw_apply_position(cfg->xiaomi_slider_motor[LIFT_WALK_RIGHT],
                      ctrl->out.slider_motor_rad[LIFT_WALK_RIGHT],
                      cfg->slider_vel_limit_rad_s);

    lw_apply_mit(cfg->unitree_arm_motor[LIFT_WALK_LEFT],
                 lw_arm_rad_to_unitree_motor_rad(cfg, LIFT_WALK_LEFT, ctrl->out.arm_theta_rad[LIFT_WALK_LEFT]),
                 lw_arm_rad_s_to_unitree_motor_rad_s(cfg, LIFT_WALK_LEFT, ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_LEFT]),
                 cfg->arm_kp,
                 cfg->arm_kd,
                 cfg->arm_torque_ff);
    lw_apply_mit(cfg->unitree_arm_motor[LIFT_WALK_RIGHT],
                 lw_arm_rad_to_unitree_motor_rad(cfg, LIFT_WALK_RIGHT, ctrl->out.arm_theta_rad[LIFT_WALK_RIGHT]),
                 lw_arm_rad_s_to_unitree_motor_rad_s(cfg, LIFT_WALK_RIGHT, ctrl->out.arm_theta_dot_rad_s[LIFT_WALK_RIGHT]),
                 cfg->arm_kp,
                 cfg->arm_kd,
                 cfg->arm_torque_ff);

    lw_apply_speed(cfg->front_wheel_motor[LIFT_WALK_LEFT],
                   lw_wheel_rad_s_to_dm_motor_rad_s(ctrl->out.front_wheel_rad_s[LIFT_WALK_LEFT]));
    lw_apply_speed(cfg->front_wheel_motor[LIFT_WALK_RIGHT],
                   lw_wheel_rad_s_to_dm_motor_rad_s(ctrl->out.front_wheel_rad_s[LIFT_WALK_RIGHT]));
    lw_apply_speed(cfg->rear_wheel_motor[LIFT_WALK_LEFT], ctrl->out.rear_wheel_rpm[LIFT_WALK_LEFT]);
    lw_apply_speed(cfg->rear_wheel_motor[LIFT_WALK_RIGHT], ctrl->out.rear_wheel_rpm[LIFT_WALK_RIGHT]);
}

// =========================
// 6. 对外接口
// =========================
void LiftWalk_DefaultConfig(LiftWalk_Config_t *cfg) {
    if (cfg == NULL) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));

    // 电机绑定。如果 global_motor_conf.h 里的全局电机编号变化，这里也要同步修改。
    cfg->xiaomi_slider_motor[LIFT_WALK_LEFT] = XIAOMI_MOTOR1_G;
    cfg->xiaomi_slider_motor[LIFT_WALK_RIGHT] = XIAOMI_MOTOR2_G;
    cfg->unitree_arm_motor[LIFT_WALK_LEFT] = UNITREE_GO_M8010_6_MOTOR1_G;
    cfg->unitree_arm_motor[LIFT_WALK_RIGHT] = UNITREE_GO_M8010_6_MOTOR2_G;
    cfg->front_wheel_motor[LIFT_WALK_LEFT] = DM_JOINT_G;
    cfg->front_wheel_motor[LIFT_WALK_RIGHT] = DM_FRONT_RIGHT_G;
    cfg->rear_wheel_motor[LIFT_WALK_LEFT] = BLAZER_FOC_MOTOR1_G;
    cfg->rear_wheel_motor[LIFT_WALK_RIGHT] = BLAZER_FOC_MOTOR2_G;

    // 四个支撑点相对底盘中心的坐标。
    // x：向前为正；y：向左为正。
    // 这些值决定 roll/pitch 姿态修正会被分配成多大的四点高度差。
    cfg->support_pos[LIFT_WALK_FRONT_LEFT] = (LiftWalk_Point_t){ .x_mm = 250.0f, .y_mm = 180.0f };
    cfg->support_pos[LIFT_WALK_FRONT_RIGHT] = (LiftWalk_Point_t){ .x_mm = 250.0f, .y_mm = -180.0f };
    cfg->support_pos[LIFT_WALK_REAR_LEFT] = (LiftWalk_Point_t){ .x_mm = -250.0f, .y_mm = 180.0f };
    cfg->support_pos[LIFT_WALK_REAR_RIGHT] = (LiftWalk_Point_t){ .x_mm = -250.0f, .y_mm = -180.0f };

    // 小米滑轨参数。
    // 默认含义：小米电机输出轴转 1 圈，滑块沿导轨方向移动 180 mm。
    cfg->slider_pitch_mm_per_rev = LIFT_WALK_SLIDER_TRAVEL_MM_PER_MOTOR_REV;
    cfg->slider_reduction_ratio = 1.0f;
    cfg->slider_motor_sign[LIFT_WALK_LEFT] = 1.0f;
    cfg->slider_motor_sign[LIFT_WALK_RIGHT] = 1.0f;
    cfg->slider_min_rad[LIFT_WALK_LEFT] = -1000.0f;
    cfg->slider_min_rad[LIFT_WALK_RIGHT] = -1000.0f;
    cfg->slider_max_rad[LIFT_WALK_LEFT] = 1000.0f;
    cfg->slider_max_rad[LIFT_WALK_RIGHT] = 1000.0f;
    cfg->slider_vel_limit_rad_s = 2.0f;

    // 宇树小臂几何参数和 MIT 控制参数。
    cfg->front_wheel_radius_mm = LIFT_WALK_FRONT_WHEEL_RADIUS_MM;
    cfg->arm_length_mm[LIFT_WALK_LEFT] = LIFT_WALK_ARM_LENGTH_MM;
    cfg->arm_length_mm[LIFT_WALK_RIGHT] = LIFT_WALK_ARM_LENGTH_MM;
    cfg->arm_pivot_z_mm[LIFT_WALK_LEFT] = LIFT_WALK_ARM_PIVOT_HEIGHT_MM;
    cfg->arm_pivot_z_mm[LIFT_WALK_RIGHT] = LIFT_WALK_ARM_PIVOT_HEIGHT_MM;
    cfg->arm_zero_offset_rad[LIFT_WALK_LEFT] =
        asinf((cfg->front_wheel_radius_mm - cfg->arm_pivot_z_mm[LIFT_WALK_LEFT]) /
              cfg->arm_length_mm[LIFT_WALK_LEFT]);
    cfg->arm_zero_offset_rad[LIFT_WALK_RIGHT] =
        asinf((cfg->front_wheel_radius_mm - cfg->arm_pivot_z_mm[LIFT_WALK_RIGHT]) /
              cfg->arm_length_mm[LIFT_WALK_RIGHT]);
    cfg->arm_motor_zero_rad[LIFT_WALK_LEFT] = 0.0f;
    cfg->arm_motor_zero_rad[LIFT_WALK_RIGHT] = 0.0f;
    cfg->arm_motor_sign[LIFT_WALK_LEFT] = 1.0f;
    cfg->arm_motor_sign[LIFT_WALK_RIGHT] = 1.0f;
    cfg->arm_min_rad[LIFT_WALK_LEFT] = -1.5f;
    cfg->arm_min_rad[LIFT_WALK_RIGHT] = -1.5f;
    cfg->arm_max_rad[LIFT_WALK_LEFT] = 1.3f;
    cfg->arm_max_rad[LIFT_WALK_RIGHT] = 1.3f;
    cfg->arm_dead_cos_min = 0.01f;
    cfg->arm_kp = 0.08f;
    cfg->arm_kd = 0.01f;
    cfg->arm_torque_ff = 0.0f;

    // 轮速补偿参数。
    // front/rear_wheel_sign 是方向校准量；实车方向反了就把对应项改成 -1。
    cfg->rear_wheel_radius_mm = LIFT_WALK_REAR_WHEEL_RADIUS_MM;
    cfg->rear_wheel_drive_angle_rad[LIFT_WALK_LEFT] = LIFT_WALK_PI / 4.0f;
    cfg->rear_wheel_drive_angle_rad[LIFT_WALK_RIGHT] = -LIFT_WALK_PI / 4.0f;
    cfg->front_wheel_speed_limit_rad_s = 25.0f;
    cfg->rear_wheel_speed_limit_rpm = 3000.0f;
    cfg->front_wheel_accel_limit_rad_s2 = 15.0f;
    cfg->rear_wheel_accel_limit_rpm_s = 600.0f;
    cfg->wheel_arm_comp_gain = 1.0f;
    cfg->front_wheel_sign[LIFT_WALK_LEFT] = 1.0f;
    cfg->front_wheel_sign[LIFT_WALK_RIGHT] = 1.0f;
    cfg->rear_wheel_sign[LIFT_WALK_LEFT] = 1.0f;
    cfg->rear_wheel_sign[LIFT_WALK_RIGHT] = 1.0f;

    // 姿态修正 PID。
    // 初值偏保守，建议先确认四个支撑点方向和几何量正确，再逐步加大增益。
    // cfg->roll_kp = 0.8f;
    // cfg->roll_ki = 0.0f;
    // cfg->roll_kd = 0.06f;
    // cfg->pitch_kp = 0.8f;
    // cfg->pitch_ki = 0.0f;
    // cfg->pitch_kd = 0.06f;
    // cfg->yaw_kp = 0.8f;
    // cfg->yaw_kd = 0.03f;
    cfg->roll_kp = 0.0f;
    cfg->roll_ki = 0.0f;
    cfg->roll_kd = 0.0f;
    cfg->pitch_kp = 0.0f;
    cfg->pitch_ki = 0.0f;
    cfg->pitch_kd = 0.0f;
    cfg->yaw_kp = 0.0f;
    cfg->yaw_kd = 0.0f;

    cfg->attitude_cmd_limit_rad = 0.08f;
    cfg->support_corr_limit_mm = 25.0f;
    cfg->max_roll_rad = 0.17f;
    cfg->max_pitch_rad = 0.17f;

    cfg->min_height_mm = 0.0f;
    cfg->max_height_mm = 198.0f;
    cfg->lift_vmax_mm_s = 10.0f;
    cfg->lift_amax_mm_s2 = 20.0f;
}

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

LiftWalk_Status_e LiftWalk_Update(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in) {
    float dt;
    LiftWalk_Status_e status;

    if (ctrl == NULL || in == NULL) {
        return LIFT_WALK_ABORT_HEIGHT_LIMIT;
    }
    if (ctrl->initialized == 0U) {
        LiftWalk_Init(ctrl, NULL);
    }

    dt = (in->dt_s > LIFT_WALK_EPS) ? in->dt_s : 0.001f;

    status = lw_check_attitude_limits(ctrl, in);
    if (status != LIFT_WALK_OK) {
        return status;
    }

    lw_step_height(ctrl, in);
    lw_update_attitude_pid(ctrl, in, dt);
    lw_update_support_heights(ctrl);
    lw_update_slider_targets(ctrl);

    status = lw_update_arm_targets(ctrl);
    if (status != LIFT_WALK_OK) {
        return status;
    }

    lw_update_wheel_targets(ctrl, in);
    ctrl->out.status = LIFT_WALK_OK;

    if (in->enable_motor_output != 0U) {
        lw_apply_motor_outputs(ctrl);
    }

    return ctrl->out.status;
}

uint8_t LiftWalk_RunLiftAction(LiftWalk_Controller_t *ctrl,
                               const LiftWalk_Input_t *in,
                               float vy_mm_s,
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
    action_in.vx_mm_s = 0.0f;
    action_in.vy_mm_s = vy_mm_s;

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

const LiftWalk_Output_t *LiftWalk_GetOutput(const LiftWalk_Controller_t *ctrl) {
    if (ctrl == NULL) {
        return NULL;
    }
    return &ctrl->out;
}
