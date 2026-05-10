#ifndef R2_CHASSIS_LIFT_WALK_CONTROLLER_H
#define R2_CHASSIS_LIFT_WALK_CONTROLLER_H

#include <stdint.h>
#include "motor_registry.h"
#include "global_motor_conf.h"

#define LIFT_WALK_SUPPORT_COUNT 4U  //表示四个支撑点：前左、前右、后左、后右
#define LIFT_WALK_SIDE_COUNT 2U     //表示左右两侧：左侧、右侧。
#define LIFT_WALK_INVALID_MOTOR (-1)     // 表示无效电机编号。cfg.rear_wheel_motor[LIFT_WALK_LEFT] = LIFT_WALK_INVALID_MOTOR;表示不控制这个轮子

/* 控制器状态/保护原因。
 * LIFT_WALK_OK 表示本周期解算成功；ABORT_* 表示姿态或机构几何已经超出安全范围。
 */
typedef enum {
    LIFT_WALK_OK = 0,   //正常
    LIFT_WALK_ABORT_ROLL_LIMIT,     //roll 角过大，触发保护
    LIFT_WALK_ABORT_PITCH_LIMIT,    //pitch 角过大，触发保护
    LIFT_WALK_ABORT_HEIGHT_LIMIT,   //高度或输入异常
    LIFT_WALK_ABORT_ARM_RANGE,      //小臂目标高度超出几何可达范围
    LIFT_WALK_ABORT_ARM_DEAD_ZONE,  //小臂接近死点，不适合继续控制
} LiftWalk_Status_e;

/* 四个支撑点的逻辑编号。
 * 前两个支撑点由宇树小臂实现，后两个支撑点由小米直线滑轨实现。
 */
typedef enum {
    LIFT_WALK_FRONT_LEFT = 0,
    LIFT_WALK_FRONT_RIGHT,
    LIFT_WALK_REAR_LEFT,
    LIFT_WALK_REAR_RIGHT,
} LiftWalk_Support_e;

/* 左右侧编号，用于成对配置滑轨、小臂和前轮。 */
typedef enum {
    LIFT_WALK_LEFT = 0,
    LIFT_WALK_RIGHT,
} LiftWalk_Side_e;

/* 支撑点在底盘坐标系下的位置。
 * x_mm: 前后方向，向前为正。
 * y_mm: 左右方向，向左为正。
 */
typedef struct {
    float x_mm;     //前后方向，向前为正
    float y_mm;     //左右方向，向左为正
} LiftWalk_Point_t;

typedef struct {
    /* 电机绑定。
     * xiaomi_slider_motor: 后侧左右两个小米滑轨电机的全局电机编号。
     * unitree_arm_motor: 前侧左右两个宇树小臂电机的全局电机编号。
     * front_wheel_motor: 小臂末端两个前轮达妙电机的全局电机编号。
     * rear_wheel_motor: 抬升阶段后轮 Blazer FOC 的全局电机编号。
     */
    int xiaomi_slider_motor[LIFT_WALK_SIDE_COUNT];
    int unitree_arm_motor[LIFT_WALK_SIDE_COUNT];
    int front_wheel_motor[LIFT_WALK_SIDE_COUNT];
    int rear_wheel_motor[LIFT_WALK_SIDE_COUNT];

    /* 四个支撑点相对底盘中心的位置，用于把 roll/pitch 姿态误差分配成四点高度修正。 */
    LiftWalk_Point_t support_pos[LIFT_WALK_SUPPORT_COUNT];

    /* 小米滑轨几何参数。
     * slider_pitch_mm_per_rev: 丝杆每转一圈对应的直线位移，单位 mm/rev。
     * slider_reduction_ratio: 电机到丝杆之间的减速比；直连时填 1。
     * slider_zero_rad: 底盘未抬升时对应的电机零位。
     * slider_min/max_rad: 软限位，防止目标位置超过滑轨行程。
     * slider_vel_limit_rad_s: 位置模式速度限制。
     */
    float slider_pitch_mm_per_rev;      //丝杆转一圈，滑块移动多少毫米
    float slider_reduction_ratio;
    float slider_zero_rad[LIFT_WALK_SIDE_COUNT];
    float slider_min_rad[LIFT_WALK_SIDE_COUNT];
    float slider_max_rad[LIFT_WALK_SIDE_COUNT];
    float slider_vel_limit_rad_s;       //小米电机位置模式的速度限制。

    /* 宇树小臂几何和 MIT 参数。
     * arm_length_mm: 小臂关节中心到轮/支撑点的长度。
     * arm_pivot_z_mm: 关节中心相对支撑高度零点的竖直偏置。
     * arm_zero_offset_rad: 机械零位与模型角 phi 的偏置，模型为 z = L * sin(phi)。
     * arm_min/max_rad: 关节软限位。
     * arm_dead_cos_min: cos(phi) 过小时接近机构死点，高度对角度不敏感，直接中止。
     * arm_kp/kd/torque_ff: 下发给宇树 MIT 控制的参数。
     */
    float arm_length_mm[LIFT_WALK_SIDE_COUNT];      //小臂关节中心 到 轮子/支撑点 的距离
    float arm_pivot_z_mm[LIFT_WALK_SIDE_COUNT];     //小臂关节中心相对支撑高度零点的竖直偏移
    float arm_zero_offset_rad[LIFT_WALK_SIDE_COUNT];    //机械零位和数学模型角度 phi 的偏置，如果你的电机 0 rad 不等于小臂水平位置，就要靠这个参数修正
    float arm_min_rad[LIFT_WALK_SIDE_COUNT];        //反解出来的角度超过范围，会被限幅
    float arm_max_rad[LIFT_WALK_SIDE_COUNT];
    float arm_dead_cos_min;
    float arm_kp;
    float arm_kd;
    float arm_torque_ff;

    /* 驱动轮速度补偿参数。
     * front_wheel_radius_mm: 小臂末端达妙前轮半径。
     * rear_wheel_radius_mm: Blazer FOC 后轮半径。
     * front_wheel_speed_limit_rad_s: 达妙前轮角速度软限幅，单位 rad/s。
     * rear_wheel_speed_limit_rpm: Blazer 后轮速度软限幅，单位 rpm。
     * wheel_arm_comp_gain: 小臂水平扫动速度补偿系数，实车可从 0.3 慢慢调到 1.0。
     * front/rear_wheel_sign: 前后左右轮方向修正，方向反时填 -1。
     */
    float front_wheel_radius_mm;
    float rear_wheel_radius_mm;
    /*
     * 后轮全向轮的驱动方向角，单位 rad。
     * 坐标约定：x 为底盘前进方向，y 为底盘左侧方向；
     * 0 表示轮子的有效驱动方向沿 x 正方向，+pi/4 表示朝左前 45 度，
     * -pi/4 表示朝右前 45 度。45 度安装的 Blazer 后轮必须用这个角度做速度投影。
     */
    float rear_wheel_drive_angle_rad[LIFT_WALK_SIDE_COUNT];     //后轮全向轮的驱动方向角。
    float front_wheel_speed_limit_rad_s;    //前轮角速度限幅，单位 rad/s。
    float rear_wheel_speed_limit_rpm;       //后轮转速限幅，单位 rpm。
    float wheel_arm_comp_gain;              //小臂水平扫动速度补偿系数
    float front_wheel_sign[LIFT_WALK_SIDE_COUNT];
    float rear_wheel_sign[LIFT_WALK_SIDE_COUNT];

    /* 姿态闭环参数。
     * roll/pitch 用来修正四个支撑点高度，yaw 用来修正左右前轮差速。
     */
    float roll_kp;
    float roll_ki;
    float roll_kd;
    float pitch_kp;
    float pitch_ki;
    float pitch_kd;
    float yaw_kp;
    float yaw_kd;

    /* 安全限幅。
     * attitude_cmd_limit_rad: 姿态 PID 输出角度限幅。
     * support_corr_limit_mm: 单个支撑点高度修正限幅。
     * max_roll/max_pitch_rad: 实测姿态超过该值时停止输出。
     */
    float attitude_cmd_limit_rad;
    float support_corr_limit_mm;
    float max_roll_rad;
    float max_pitch_rad;

    /* 抬升轨迹限制。
     * 控制器会把 target_height_mm 变成带速度/加速度约束的 height_ref_mm。
     */
    float min_height_mm;    //最低允许高度。
    float max_height_mm;    //最高允许高度。
    float lift_vmax_mm_s;
    float lift_amax_mm_s2;
} LiftWalk_Config_t;

typedef struct {
    /* 期望底盘前进速度，单位 mm/s。抬升时前轮速度会叠加小臂扫动补偿。 */
    float vx_mm_s;
    /*
     * 底盘期望横移速度，单位 mm/s；正值表示向机体左侧移动。
     * 如果爬楼梯阶段只要求直行，把这个量填 0 即可。
     * 45 度 Blazer 全向后轮会同时使用 vx/vy/yaw 做投影。
     */
    float vy_mm_s;
    /* 目标抬升高度，单位 mm。控制器内部会做平滑限速。 */
    float target_height_mm;
    /* IMU 姿态角，单位 rad。roll/pitch 用于保持水平，yaw 用于保持航向。 */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_ref_rad;
    /* IMU 角速度，单位 rad/s，用于姿态 D 项阻尼。 */
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    /* 控制周期，单位 s。 */
    float dt_s;
    /* 置 1 时直接下发电机命令；置 0 时只计算 out，适合先观察调试。 */
    uint8_t enable_motor_output;
} LiftWalk_Input_t;

typedef struct {
    /* 平滑后的抬升高度和速度。 */
    float height_ref_mm;
    float height_dot_ref_mm_s;
    /* 四个支撑点高度目标，顺序见 LiftWalk_Support_e。 */
    float support_z_mm[LIFT_WALK_SUPPORT_COUNT];
    /* 两个小米滑轨电机位置目标，单位 rad。 */
    float slider_motor_rad[LIFT_WALK_SIDE_COUNT];
    /* 两个宇树小臂 MIT 位置/速度目标。 */
    float arm_theta_rad[LIFT_WALK_SIDE_COUNT];
    /* 两个宇树小臂角速度目标，单位 rad/s。 */
    float arm_theta_dot_rad_s[LIFT_WALK_SIDE_COUNT];
    /* 小臂转动导致轮心相对底盘的水平速度，用于前轮速度补偿。 */
    float arm_x_dot_mm_s[LIFT_WALK_SIDE_COUNT];
    /* 两个达妙前轮最终速度目标，单位 rad/s，已经叠加小臂扫动补偿。 */
    float front_wheel_rad_s[LIFT_WALK_SIDE_COUNT];
    /* 两个 Blazer FOC 后轮最终速度目标，单位 rpm，与前轮使用同一底盘速度基准。 */
    float rear_wheel_rpm[LIFT_WALK_SIDE_COUNT];
    /* 姿态闭环输出，调试时可以观察这些量是否方向正确。 */
    float roll_cmd_rad;
    float pitch_cmd_rad;
    float yaw_cmd_rad_s;
    LiftWalk_Status_e status;
} LiftWalk_Output_t;

typedef struct {
    /* 控制器持有配置、上一次输出和积分项。 */
    LiftWalk_Config_t cfg;
    LiftWalk_Output_t out;
    float roll_i;   //roll PID 的积分项
    float pitch_i;  //pitch PID 的积分项。用于长期消除 pitch 静态误差。
    float last_height_target_mm;        //上一次目标高度。
    uint8_t lift_action_active;         //表示抬升动作是否正在执行。
    uint8_t lift_action_done;           //表示抬升动作是否完成。
    float lift_action_target_mm;        //当前抬升动作的目标高度。
    uint8_t initialized;                //是否已经初始化。
} LiftWalk_Controller_t;

/* 填充一套可编译运行的默认参数。
 * 注意：默认几何只用于占位，实车必须按真实机构修改。
 */
void LiftWalk_DefaultConfig(LiftWalk_Config_t *cfg);
/* 初始化控制器；cfg 为 NULL 时使用 LiftWalk_DefaultConfig。 */
void LiftWalk_Init(LiftWalk_Controller_t *ctrl, const LiftWalk_Config_t *cfg);
/* 重置轨迹和积分项，current_height_mm 应填当前实际/估计抬升高度。 */
void LiftWalk_Reset(LiftWalk_Controller_t *ctrl, float current_height_mm);
/* 单周期更新：计算四支撑点、小臂、滑轨和前轮命令，必要时直接下发电机。 */
LiftWalk_Status_e LiftWalk_Update(LiftWalk_Controller_t *ctrl, const LiftWalk_Input_t *in);
/* 底盘抬升动作：执行中返回 0，到达目标高度且速度降下来后返回 1。 */
uint8_t LiftWalk_RunLiftAction(LiftWalk_Controller_t *ctrl,
                               const LiftWalk_Input_t *in,
                               float vx_mm_s,
                               float start_height_mm,
                               float done_tolerance_mm);
/* 停止前轮并冻结抬升速度；用于保护触发或外部急停。 */
void LiftWalk_Stop(LiftWalk_Controller_t *ctrl);
/* 获取最近一次输出，便于 RTT/调试打印。 */
const LiftWalk_Output_t *LiftWalk_GetOutput(const LiftWalk_Controller_t *ctrl);

#endif /* R2_CHASSIS_LIFT_WALK_CONTROLLER_H */
