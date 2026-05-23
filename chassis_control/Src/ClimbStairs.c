//
// Created by lcf on 2025/12/1.
//

#include "../Inc/ClimbStairs.h"
#include <stdlib.h>
#include "gpio.h"
#include "chassis_driver.h"
#include "locator_driver.h"
#include "chassis_path.h"
#include "chassis_pid.h"
#include "path_plan.h"
#include "debug.h"
#include "Hfsm.h"
#include "lift_walk_controller.h"

#define PI 3.1415926
#define CLIMB_LIFT_TARGET_HEIGHT_MM 198.0f
#define CLIMB_LIFT_APPROACH_HEIGHT_MM 0.0f
#define CLIMB_LIFT_FORWARD_MM_S 200.0f
#define CLIMB_LIFT_START_HEIGHT_MM 0.0f
#define CLIMB_LIFT_DONE_TOLERANCE_MM 2.0f
#define CLIMB_LIFT_DT_S 0.01f
#define CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S CLIMB_LIFT_FORWARD_MM_S
#define CLIMB_FRONT_ARM_DEPLOY_LEFT_OFFSET_RAD 0.80f
#define CLIMB_FRONT_ARM_DEPLOY_RIGHT_OFFSET_RAD 0.80f
#define CLIMB_FRONT_ARM_MOVE_MAX_RAD_S 0.60f
#define CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD 0.03f
#define CLIMB_FRONT_ARM_DONE_SPEED_RAD_S 0.08f
#define CLIMB_FRONT_ARM_KP 0.08f
#define CLIMB_FRONT_ARM_KD 0.01f
#define CLIMB_FRONT_ARM_TORQUE_FF 0.0f
#define CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD 0.05f
#define CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S 0.0f
#define CLIMB_FRONT_WHEEL_RADIUS_MM 41.5f
#define CLIMB_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD (74.0f / 24.0f)
#define CLIMB_REAR_WHEEL_RADIUS_MM 60.0f
#define  ForestEdge 100  //  梅林边界
int climb_cnt = 0;
int down_cnt = 0;

// pb11 碰撞开关，碰到为 1
// pb10 后光电开关，常亮为 1
// pc1  前光电开关，常亮为 1，抬起灯

// R2 出发点为原点下台阶坐标
// pos stairs_center[13]={
//     {0,0,0},
//     {2780,3340,400},{1600,3340,200},{400,3340,400},
//     {2780,4540,200},{1600,4540,400},{400,4540,600},
//     {2780,5720,400},{1600,5720,600},{400,5720,400},
//     {2780,6900,200},{1600,6900,400},{400,6900,400}
// };

// 自定义原点下台阶坐标，偏置为 2780，1860
// pos stairs_center[12]={
//     //{0,0,0},
//     {2780,3340,400},{-1180,1480,200},{-2380,1480,400},
//     {2780,4540,200},{1600,4540,400},{400,4540,600},
//     {2780,5720,400},{1600,5720,600},{400,5720,400},
//     {2780,6900,200},{1600,6900,400},{400,6900,400}
// };

//
pos stairs_center[15]={
    //{0,0,0},
    {2690,3290,400},{1490,3290,200},{290,3290,400},
    {2690,4490,200},{1490,4490,400},{290,4490,600},
    {2690,5690,400},{1490,5690,600},{290,5690,400},
    {2690,6890,200},{1490,6890,400},{290,6890,200},
    {2690,8090,0},  {0,0,0},        {290,8090,0}
};

Point_struct entry_point[3] = {{2690,2090},{1490,2090},{290,2090}};

// 状态变量
Climb_State_e current_climb_state = CLIMB_IDLE;
Move_State_e current_move_state = MOVE_IDLE;
Down_State_e current_down_state = DOWN_IDLE;
uint32_t step_start_time = 0; // 用于计时延时步骤

static LiftWalk_Controller_t climb_lift_ctrl;
static LiftWalk_Input_t climb_lift_in;
static uint8_t climb_lift_inited = 0U;
static uint8_t climb_front_arm_pose_captured = 0U;
static uint8_t climb_front_arm_move_started = 0U;
static float climb_front_arm_retract_rad[LIFT_WALK_SIDE_COUNT] = {0.0f, 0.0f};
static float climb_front_arm_target_rad[LIFT_WALK_SIDE_COUNT] = {0.0f, 0.0f};

static float ClimbLift_GetMotorAngle(int motor_index)
{
    Motor_State_t state;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return 0.0f;
    }
    if (g_motor_list[motor_index].get_state == NULL) {
        return 0.0f;
    }

    state = g_motor_list[motor_index].get_state(&g_motor_list[motor_index]);
    return state.angle;
}

static float ClimbLift_GetMotorSpeed(int motor_index)
{
    Motor_State_t state;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return 0.0f;
    }
    if (g_motor_list[motor_index].get_state == NULL) {
        return 0.0f;
    }

    state = g_motor_list[motor_index].get_state(&g_motor_list[motor_index]);
    return state.speed;
}

static void ClimbMotor_SetSpeed(int motor_index, float speed)
{
    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }
    if (g_motor_list[motor_index].set_speed != NULL) {
        g_motor_list[motor_index].set_speed(&g_motor_list[motor_index], speed);
    }
}

static void ClimbMotor_SetPosition(int motor_index, float position, float vel_limit)
{
    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }
    if (g_motor_list[motor_index].set_position != NULL) {
        g_motor_list[motor_index].set_position(&g_motor_list[motor_index], position, vel_limit);
    }
}

static void ClimbMotor_SetMIT(int motor_index, float target_rad, float kp, float kd, float torque_ff)
{
    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }
    if (g_motor_list[motor_index].set_mit != NULL) {
        g_motor_list[motor_index].set_mit(&g_motor_list[motor_index],
                                          target_rad,
                                          0.0f,
                                          kp,
                                          kd,
                                          torque_ff);
    }
}

static void ClimbFrontWheel_SetLinearSpeed(float vy_mm_s)
{
    float front_motor_rad_s = (vy_mm_s / CLIMB_FRONT_WHEEL_RADIUS_MM) *
                              CLIMB_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD;

    ClimbMotor_SetSpeed(DM_FRONT_LEFT_G, front_motor_rad_s);
    ClimbMotor_SetSpeed(DM_FRONT_RIGHT_G, front_motor_rad_s);
}

static void ClimbDrive_SetForwardSpeed(float vy_mm_s)
{
    float rear_rpm = vy_mm_s * 0.70710678f * 60.0f /
                     (2.0f * PI * CLIMB_REAR_WHEEL_RADIUS_MM);

    ClimbFrontWheel_SetLinearSpeed(vy_mm_s);
    ClimbMotor_SetSpeed(BLAZER_FOC_MOTOR1_G, rear_rpm);
    ClimbMotor_SetSpeed(BLAZER_FOC_MOTOR2_G, -rear_rpm);
}

static void ClimbDrive_Stop(void)
{
    ClimbDrive_SetForwardSpeed(0.0f);
}

static void ClimbLift_InitOnce(void)
{
    LiftWalk_Config_t cfg;

    if (climb_lift_inited != 0U) {
        return;
    }

    LiftWalk_DefaultConfig(&cfg);

    cfg.slider_zero_rad[LIFT_WALK_LEFT] =
        ClimbLift_GetMotorAngle(cfg.xiaomi_slider_motor[LIFT_WALK_LEFT]);
    cfg.slider_zero_rad[LIFT_WALK_RIGHT] =
        ClimbLift_GetMotorAngle(cfg.xiaomi_slider_motor[LIFT_WALK_RIGHT]);
    cfg.arm_motor_zero_rad[LIFT_WALK_LEFT] =
        ClimbLift_GetMotorAngle(cfg.unitree_arm_motor[LIFT_WALK_LEFT]);
    cfg.arm_motor_zero_rad[LIFT_WALK_RIGHT] =
        ClimbLift_GetMotorAngle(cfg.unitree_arm_motor[LIFT_WALK_RIGHT]);

    /* First climb integration: lift four corners equally without IMU correction. */
    cfg.roll_kp = 0.0f;
    cfg.roll_ki = 0.0f;
    cfg.roll_kd = 0.0f;
    cfg.pitch_kp = 0.0f;
    cfg.pitch_ki = 0.0f;
    cfg.pitch_kd = 0.0f;
    cfg.yaw_kp = 0.0f;
    cfg.yaw_kd = 0.0f;

    LiftWalk_Init(&climb_lift_ctrl, &cfg);
    climb_lift_inited = 1U;
}

static void ClimbLift_Reset(void)
{
    if (climb_lift_inited != 0U) {
        LiftWalk_Stop(&climb_lift_ctrl);
    }
    climb_lift_inited = 0U;
}

static uint8_t ClimbLift_RunStep(void)
{
    ClimbLift_InitOnce();

    climb_lift_in.vx_mm_s = 0.0f;
    climb_lift_in.vy_mm_s = CLIMB_LIFT_FORWARD_MM_S;
    climb_lift_in.target_height_mm = CLIMB_LIFT_TARGET_HEIGHT_MM;

    climb_lift_in.roll_rad = 0.0f;
    climb_lift_in.pitch_rad = 0.0f;
    climb_lift_in.yaw_rad = 0.0f;
    climb_lift_in.yaw_ref_rad = 0.0f;
    climb_lift_in.roll_rate_rad_s = 0.0f;
    climb_lift_in.pitch_rate_rad_s = 0.0f;
    climb_lift_in.yaw_rate_rad_s = 0.0f;

    climb_lift_in.dt_s = CLIMB_LIFT_DT_S;
    climb_lift_in.enable_motor_output = 1U;

    return LiftWalk_RunLiftAction(&climb_lift_ctrl,
                                  &climb_lift_in,
                                  CLIMB_LIFT_FORWARD_MM_S,
                                  CLIMB_LIFT_START_HEIGHT_MM,
                                  CLIMB_LIFT_DONE_TOLERANCE_MM);
}

static void ClimbLift_UpdateTarget(float target_height_mm, float vy_mm_s)
{
    if (climb_lift_inited == 0U) {
        return;
    }

    climb_lift_in.vx_mm_s = 0.0f;
    climb_lift_in.vy_mm_s = vy_mm_s;
    climb_lift_in.target_height_mm = target_height_mm;

    climb_lift_in.roll_rad = 0.0f;
    climb_lift_in.pitch_rad = 0.0f;
    climb_lift_in.yaw_rad = 0.0f;
    climb_lift_in.yaw_ref_rad = 0.0f;
    climb_lift_in.roll_rate_rad_s = 0.0f;
    climb_lift_in.pitch_rate_rad_s = 0.0f;
    climb_lift_in.yaw_rate_rad_s = 0.0f;

    climb_lift_in.dt_s = CLIMB_LIFT_DT_S;
    climb_lift_in.enable_motor_output = 1U;

    (void)LiftWalk_Update(&climb_lift_ctrl, &climb_lift_in);
}

static void ClimbLift_DriveForward(void)
{
    ClimbLift_UpdateTarget(CLIMB_LIFT_TARGET_HEIGHT_MM, CLIMB_LIFT_FORWARD_MM_S);
}

static void ClimbLift_HoldRearSliderAndDrive(float vy_mm_s)
{
    if (climb_lift_inited == 0U) {
        ClimbDrive_SetForwardSpeed(vy_mm_s);
        return;
    }

    ClimbMotor_SetPosition(climb_lift_ctrl.cfg.xiaomi_slider_motor[LIFT_WALK_LEFT],
                           climb_lift_ctrl.out.slider_motor_rad[LIFT_WALK_LEFT],
                           climb_lift_ctrl.cfg.slider_vel_limit_rad_s);
    ClimbMotor_SetPosition(climb_lift_ctrl.cfg.xiaomi_slider_motor[LIFT_WALK_RIGHT],
                           climb_lift_ctrl.out.slider_motor_rad[LIFT_WALK_RIGHT],
                           climb_lift_ctrl.cfg.slider_vel_limit_rad_s);
    ClimbDrive_SetForwardSpeed(vy_mm_s);
}

static uint8_t ClimbLift_RetractRearSlider(float vy_mm_s)
{
    float left_err;
    float right_err;

    if (climb_lift_inited == 0U) {
        ClimbDrive_SetForwardSpeed(vy_mm_s);
        return 1U;
    }

    ClimbMotor_SetPosition(climb_lift_ctrl.cfg.xiaomi_slider_motor[LIFT_WALK_LEFT],
                           climb_lift_ctrl.cfg.slider_zero_rad[LIFT_WALK_LEFT],
                           climb_lift_ctrl.cfg.slider_vel_limit_rad_s);
    ClimbMotor_SetPosition(climb_lift_ctrl.cfg.xiaomi_slider_motor[LIFT_WALK_RIGHT],
                           climb_lift_ctrl.cfg.slider_zero_rad[LIFT_WALK_RIGHT],
                           climb_lift_ctrl.cfg.slider_vel_limit_rad_s);
    ClimbDrive_SetForwardSpeed(vy_mm_s);

    left_err = fabsf(ClimbLift_GetMotorAngle(climb_lift_ctrl.cfg.xiaomi_slider_motor[LIFT_WALK_LEFT]) -
                     climb_lift_ctrl.cfg.slider_zero_rad[LIFT_WALK_LEFT]);
    right_err = fabsf(ClimbLift_GetMotorAngle(climb_lift_ctrl.cfg.xiaomi_slider_motor[LIFT_WALK_RIGHT]) -
                      climb_lift_ctrl.cfg.slider_zero_rad[LIFT_WALK_RIGHT]);

    return (left_err <= CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD &&
            right_err <= CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD) ? 1U : 0U;
}

bool is_motor_cplt(int motor_id,int dis)
{
    return abs(Get_dji_information(motor_id).total_angle-dis)<100;
}
//距离转换为编码数
int DisToEncoder(float dis,int motor_id)
{
    float reduction_ratio[5]={19};//电机减速比
    float gear_ratio[5]={1};//电机传动比
    float fix_k[5]={1};//修正系数

    return (int)(dis*reduction_ratio[motor_id]*gear_ratio[motor_id]*fix_k[motor_id]);
}

void motor_move(int encoder_counts,int motor_id)
{
    int lower_lim[5]={};
    int upper_lim[5]={};
    //int now_loc=Get_dji_information(motor_id).angle;
    if (encoder_counts<lower_lim[motor_id]) encoder_counts=lower_lim[motor_id];
    if (encoder_counts>upper_lim[motor_id]) encoder_counts=upper_lim[motor_id];
    int send_loc=encoder_counts;
    Change_dji_loc(motor_id,send_loc);
}

// 判断上楼梯时是否走到台阶中心边缘，用于判断是否需要把后轮升上去
bool is_on_stair_edge(int stair_id,int face)
{
    int center_threshold=20;//距离中心轴线的偏置阈值，单位mm
    int edge_threshold=50;//距离台阶边缘的阈值，单位mm

    switch (face)
    {
        case 0://之前590
            if ( fabsf(lcResult.y-stairs_center[stair_id].y+290)<edge_threshold) return true;
            break;
        case 1:
            if (fabsf(lcResult.x-stairs_center[stair_id].x+290)<edge_threshold) return true;
            break;
        case 2:
            if (fabsf(lcResult.x-stairs_center[stair_id].x-290)<edge_threshold) return true;
            break;
        case 3:
            if (fabsf(lcResult.y-stairs_center[stair_id].y-290)<edge_threshold) return true;
            break;
        default:
            return false;
    }
    return false;
}

//判断上楼梯时是否走到台阶中心
bool is_on_stair_center(int stair_id)
{
    int center_threshold=20;//距离中心轴线的偏置阈值，单位mm

    if (fabsf(lcResult.x-stairs_center[stair_id].x)<center_threshold && fabsf(lcResult.y-stairs_center[stair_id].y)<center_threshold)
        return true;
    else return false;
}

//获得台阶边缘坐标,用于上台阶前靠近台阶边缘
Point_struct get_stair_edge(int stair_id,int face)
{
    Point_struct end_point;
    switch (face)
    {
    case 0:
        end_point.x = stairs_center[stair_id].x;
        end_point.y = stairs_center[stair_id].y - (590+250);
        break;
    case 1:
        end_point.x = stairs_center[stair_id].x - (590+250);
        end_point.y = stairs_center[stair_id].y;
        break;
    case 2:
        end_point.x = stairs_center[stair_id].x + (590+250);
        end_point.y = stairs_center[stair_id].y;
        break;
    case 3:
        end_point.x = stairs_center[stair_id].x;
        end_point.y = stairs_center[stair_id].y + (590+250);
        break;
    default:
        end_point.x = lcResult.x;
        end_point.y = lcResult.y;
    }
    return end_point;
}

// 朝向角对应角度
float face_angle(int face)
{
    float tmp=0.0f;
    if (face==0) tmp=0.0f;//往y轴正方向上楼梯
    else if (face==1) tmp=-1.57079632679f;//往x轴正方向上楼梯
    else if (face==2) tmp=1.57079632679f;//往x轴负方向上楼梯
    else if (face==3) tmp= 3.14159265359f;//往y轴负方向上楼梯
    return tmp;
}

// 获取对应朝向
int get_face(int curr_id, int target_id)
{
    int face = 0;
    int curr_r=curr_id/COLS,curr_c=curr_id%COLS;
    int target_r=target_id/COLS,target_c=target_id%COLS;
    if (curr_id == ENTRY_NODE || target_id == EXIT_NODE)  face = 0;
    else if (curr_r-target_r==1&&curr_c-target_c==0) {
        face = 3;
    }else if (curr_r-target_r==-1&&curr_c-target_c==0) {
        face = 0;
    }else if (curr_r-target_r==0&&curr_c-target_c==-1) {
        face = 2;
    }else if (curr_r-target_r==0&&curr_c-target_c==1) {
        face = 1;
    }
    return face;
}

// 运动靠近目标点
void move_approach(Point_struct now_point,Point_struct end_point,float now_pos,float vr)
{
    vec2 adjust_spd_world = PID_Approaching_Calculate(&chassis_kaojin_pid, now_point, end_point);
    // 速度转换到车身局部坐标系
    vec2 spd_local_temp = change_world_to_local(adjust_spd_world, now_pos);
    const float close_limit = 1000.0f;
    float max_spd=spd_local_temp.x>spd_local_temp.y?spd_local_temp.x:spd_local_temp.y;
    float min_spd=spd_local_temp.x<spd_local_temp.y?spd_local_temp.x:spd_local_temp.y;
    float rating=1.0;
    if (max_spd>close_limit) rating=close_limit/max_spd;
    if (min_spd<-close_limit) rating=-close_limit/min_spd;

    spd_local_temp.x*=rating;
    spd_local_temp.y*=rating;
    //RTT_Printf("vx:%f,vy:%f\n",spd_local_temp.x, spd_local_temp.y);
    cha_remote(spd_local_temp.x, spd_local_temp.y, vr); // 输出末端调整速度
    //RTT_Printf("spdx=%f, spdy=%f, vr=%f\n", spd_local_temp.x, spd_local_temp.y, vr);
}

// 移动回方格中心
int Move_back_to_Center(int stair_id)
{
    Point_struct now_point = {lcResult.x, lcResult.y}; // 机器人当前坐标点
    Point_struct end_point = {stairs_center[stair_id].x,stairs_center[stair_id].y};
    if (is_on_stair_center((stair_id))) {
        cha_remote(0,0,0);
        return 1;
    }else {
        move_approach(now_point,end_point,lcResult.r,0);
    }
    return 0;
}

// 爬楼梯的函数，高度200mm
/**
 * @brief 新电机组爬楼控制函数
 * @return int 状态反馈：0-正在爬升，1-爬升完成并到位
 */
int ClimbStairs(int curr_id, int stair_id)
{
    int face = get_face(curr_id, stair_id);

    if (current_climb_state == CLIMB_IDLE)
    {
        ClimbLift_Reset();
        cha_remote(0.0f, 0.0f, 0.0f);
        ClimbDrive_Stop();
        climb_front_arm_pose_captured = 0U;
        climb_front_arm_move_started = 0U;
        current_climb_state = CLIMB_STEP1_FRONT_ARM_DEPLOY;
        return 0;
    }

    switch (current_climb_state)
    {
        case CLIMB_IDLE:
        {
            cha_remote(0.0f, 0.0f, 0.0f);
            ClimbDrive_Stop();
            break;
        }

        case CLIMB_STEP1_FRONT_ARM_DEPLOY:
        {
            float vr = PID_Angle_Calculate(&chassis_yaw_pid, face_angle(face), lcResult.r);
            float left_err;
            float right_err;
            float left_speed;
            float right_speed;

            cha_remote(0.0f, CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S, vr);
            ClimbFrontWheel_SetLinearSpeed(CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S);

            if (climb_front_arm_pose_captured == 0U) {
                climb_front_arm_retract_rad[LIFT_WALK_LEFT] =
                    ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR1_G);
                climb_front_arm_retract_rad[LIFT_WALK_RIGHT] =
                    ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR2_G);
                climb_front_arm_pose_captured = 1U;
            }

            if (climb_front_arm_move_started == 0U) {
                climb_front_arm_target_rad[LIFT_WALK_LEFT] =
                    climb_front_arm_retract_rad[LIFT_WALK_LEFT] + CLIMB_FRONT_ARM_DEPLOY_LEFT_OFFSET_RAD;
                climb_front_arm_target_rad[LIFT_WALK_RIGHT] =
                    climb_front_arm_retract_rad[LIFT_WALK_RIGHT] + CLIMB_FRONT_ARM_DEPLOY_RIGHT_OFFSET_RAD;

                Motor_StartSmoothGotoMIT(UNITREE_GO_M8010_6_MOTOR1_G,
                                         climb_front_arm_target_rad[LIFT_WALK_LEFT],
                                         CLIMB_FRONT_ARM_MOVE_MAX_RAD_S,
                                         CLIMB_FRONT_ARM_KP,
                                         CLIMB_FRONT_ARM_KD,
                                         CLIMB_FRONT_ARM_TORQUE_FF);
                Motor_StartSmoothGotoMIT(UNITREE_GO_M8010_6_MOTOR2_G,
                                         climb_front_arm_target_rad[LIFT_WALK_RIGHT],
                                         CLIMB_FRONT_ARM_MOVE_MAX_RAD_S,
                                         CLIMB_FRONT_ARM_KP,
                                         CLIMB_FRONT_ARM_KD,
                                         CLIMB_FRONT_ARM_TORQUE_FF);
                climb_front_arm_move_started = 1U;
            }

            left_err = fabsf(ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR1_G) -
                             climb_front_arm_target_rad[LIFT_WALK_LEFT]);
            right_err = fabsf(ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR2_G) -
                              climb_front_arm_target_rad[LIFT_WALK_RIGHT]);
            left_speed = fabsf(ClimbLift_GetMotorSpeed(UNITREE_GO_M8010_6_MOTOR1_G));
            right_speed = fabsf(ClimbLift_GetMotorSpeed(UNITREE_GO_M8010_6_MOTOR2_G));

            if (left_err <= CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD &&
                right_err <= CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD &&
                left_speed <= CLIMB_FRONT_ARM_DONE_SPEED_RAD_S &&
                right_speed <= CLIMB_FRONT_ARM_DONE_SPEED_RAD_S) {
                climb_front_arm_move_started = 0U;
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR1_G,
                                  climb_front_arm_target_rad[LIFT_WALK_LEFT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR2_G,
                                  climb_front_arm_target_rad[LIFT_WALK_RIGHT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
                ClimbLift_Reset();
                cha_remote(0.0f, 0.0f, 0.0f);
                (void)ClimbLift_RunStep();
                current_climb_state = CLIMB_STEP2_LIFT_AND_FORWARD;
            }
            break;
        }

        case CLIMB_STEP2_LIFT_AND_FORWARD:
        {
            if (ClimbLift_RunStep() != 0U) {
                ClimbLift_DriveForward();
            }

            if (is_on_stair_edge(stair_id, face)) {
                climb_front_arm_move_started = 0U;
                current_climb_state = CLIMB_STEP3_FRONT_ARM_RETRACT;
            }
            break;
        }

        case CLIMB_STEP3_FRONT_ARM_RETRACT:
        {
            float left_err;
            float right_err;
            float left_speed;
            float right_speed;

            ClimbLift_HoldRearSliderAndDrive(CLIMB_LIFT_FORWARD_MM_S);

            if (climb_front_arm_pose_captured == 0U) {
                climb_front_arm_retract_rad[LIFT_WALK_LEFT] =
                    ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR1_G);
                climb_front_arm_retract_rad[LIFT_WALK_RIGHT] =
                    ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR2_G);
                climb_front_arm_pose_captured = 1U;
            }

            if (climb_front_arm_move_started == 0U) {
                climb_front_arm_target_rad[LIFT_WALK_LEFT] =
                    climb_front_arm_retract_rad[LIFT_WALK_LEFT];
                climb_front_arm_target_rad[LIFT_WALK_RIGHT] =
                    climb_front_arm_retract_rad[LIFT_WALK_RIGHT];

                Motor_StartSmoothGotoMIT(UNITREE_GO_M8010_6_MOTOR1_G,
                                         climb_front_arm_target_rad[LIFT_WALK_LEFT],
                                         CLIMB_FRONT_ARM_MOVE_MAX_RAD_S,
                                         CLIMB_FRONT_ARM_KP,
                                         CLIMB_FRONT_ARM_KD,
                                         CLIMB_FRONT_ARM_TORQUE_FF);
                Motor_StartSmoothGotoMIT(UNITREE_GO_M8010_6_MOTOR2_G,
                                         climb_front_arm_target_rad[LIFT_WALK_RIGHT],
                                         CLIMB_FRONT_ARM_MOVE_MAX_RAD_S,
                                         CLIMB_FRONT_ARM_KP,
                                         CLIMB_FRONT_ARM_KD,
                                         CLIMB_FRONT_ARM_TORQUE_FF);
                climb_front_arm_move_started = 1U;
            }

            left_err = fabsf(ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR1_G) -
                             climb_front_arm_target_rad[LIFT_WALK_LEFT]);
            right_err = fabsf(ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR2_G) -
                              climb_front_arm_target_rad[LIFT_WALK_RIGHT]);
            left_speed = fabsf(ClimbLift_GetMotorSpeed(UNITREE_GO_M8010_6_MOTOR1_G));
            right_speed = fabsf(ClimbLift_GetMotorSpeed(UNITREE_GO_M8010_6_MOTOR2_G));

            if (left_err <= CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD &&
                right_err <= CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD &&
                left_speed <= CLIMB_FRONT_ARM_DONE_SPEED_RAD_S &&
                right_speed <= CLIMB_FRONT_ARM_DONE_SPEED_RAD_S) {
                climb_front_arm_move_started = 0U;
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR1_G,
                                  climb_front_arm_target_rad[LIFT_WALK_LEFT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR2_G,
                                  climb_front_arm_target_rad[LIFT_WALK_RIGHT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
                current_climb_state = CLIMB_STEP4_REAR_SLIDER_RETRACT;
            }
            break;
        }

        case CLIMB_STEP4_REAR_SLIDER_RETRACT:
        {
            if (climb_front_arm_pose_captured != 0U) {
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR1_G,
                                  climb_front_arm_retract_rad[LIFT_WALK_LEFT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR2_G,
                                  climb_front_arm_retract_rad[LIFT_WALK_RIGHT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
            }
            if (ClimbLift_RetractRearSlider(CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S) != 0U) {
                current_climb_state = CLIMB_STEP5_CENTER_FORWARD;
            }
            break;
        }

        case CLIMB_STEP5_CENTER_FORWARD:
        {
            if (climb_front_arm_pose_captured != 0U) {
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR1_G,
                                  climb_front_arm_retract_rad[LIFT_WALK_LEFT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
                ClimbMotor_SetMIT(UNITREE_GO_M8010_6_MOTOR2_G,
                                  climb_front_arm_retract_rad[LIFT_WALK_RIGHT],
                                  CLIMB_FRONT_ARM_KP,
                                  CLIMB_FRONT_ARM_KD,
                                  CLIMB_FRONT_ARM_TORQUE_FF);
            }
            if (is_on_stair_center(stair_id)) {
                ClimbDrive_Stop();
                current_climb_state = CLIMB_COMPLETE;
            } else {
                ClimbDrive_SetForwardSpeed(CLIMB_LIFT_FORWARD_MM_S);
            }
            break;
        }

        case CLIMB_COMPLETE:
        {
            ClimbLift_Reset();
            cha_remote(0.0f, 0.0f, 0.0f);
            ClimbDrive_Stop();
            climb_front_arm_pose_captured = 0U;
            climb_front_arm_move_started = 0U;
            current_climb_state = CLIMB_IDLE;
            climb_cnt = 0;
            return 1;
        }

        default:
            ClimbLift_Reset();
            cha_remote(0.0f, 0.0f, 0.0f);
            ClimbDrive_Stop();
            climb_front_arm_pose_captured = 0U;
            climb_front_arm_move_started = 0U;
            current_climb_state = CLIMB_IDLE;
            break;
    }

    return 0;
}

//控制R2走向台阶边缘
/**
 * @brief 移动到目标格子边缘控制函数
 * @return int 状态反馈：0-正在移动，1-到位
 */
int Move_to_Edge(int curr_id, int stair_id)
{
    int face = get_face(curr_id,stair_id);
    // 假设按下 rc_engineer_data.button10_is_climb_trigger 是触发一键攀爬的按钮
    if ( current_move_state == MOVE_IDLE)
    {
        // 触发一键攀爬，开始第一步
        //Extend_Cylinder(); // 在开始之前先伸长气缸 (对应原图步骤2)
            //得出上楼梯的方向
            // if (fabsf(lcResult.r-0)<0.1) face=0;//往y轴正方向上楼梯
            // else if (fabsf(lcResult.r-4.71)<0.1) face=1;//往x轴正方向上楼梯
            // else if (fabsf(lcResult.r-1.57)<0.1) face=2;//往x轴负方向上楼梯
            // else if (fabsf(lcResult.r-3.14)<0.1) face=3;//往y轴负方向上楼梯
            // else face=4;

        current_move_state = MOVE_STEP1_FRONT_UP;
        return 0;
    }

    switch (current_move_state)
    {
        case MOVE_IDLE:
        {
            // 保持空闲，等待触发
             Change_dji_loc(DJI_M_CLIMB_LF,-climb_front_up);
             Change_dji_loc(DJI_M_CLIMB_RF,climb_front_up);
            Change_dji_loc(DJI_M_CLIMB_RB,-climb_behind_up);
            Change_dji_loc(DJI_M_CLIMB_LB,climb_behind_up);
            break;
        }

        // --- 步骤 1：前侧抬升 ---
        case MOVE_STEP1_FRONT_UP:
        {
            // 前轮抬到200平齐，后轮触地 (原图步骤2 + 原按钮)
            Change_dji_loc(DJI_M_CLIMB_LF,-climb_front_up);
            Change_dji_loc(DJI_M_CLIMB_RF,climb_front_up);
            Change_dji_loc(DJI_M_CLIMB_RB,-climb_behind_up);
            Change_dji_loc(DJI_M_CLIMB_LB,climb_behind_up);

            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_SET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_SET);
            // 判断电机是否到达目标位置 (或等待气缸伸长)
            // 假设我们使用一个简单的延时来等待气缸伸长完成
            // if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up))
            if (is_motor_cplt(DJI_M_CLIMB_LF,-climb_front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,climb_front_up))//&&climb_cnt == 2)
            {
                current_move_state = MOVE_STEP2_BASE_FORWARD;
            }
            break;
        }

        // --- 步骤 2：底盘向前移动 ---
        case MOVE_STEP2_BASE_FORWARD:
        {
            // 底盘向前移动，前轮搭在台子上 (原图步骤3)
            // 计算靠近速度 (世界坐标系)，使用全局靠近PID实例
            float now_pos = lcResult.r;                        // 机器人当前朝向角
            float vr = PID_Angle_Calculate(&chassis_yaw_pid, face_angle(face), now_pos);
            //RTT_Printf("face=%d  face_angle=%f\n",face,face_angle(face));
            if (fabsf(lcResult.r-face_angle(face))<0.05f)
            {
                cha_remote(0,500,vr);
                if (g_robot_ctx.plan.entry_grab == 1) {
                    if (is_on_stair_edge(stair_id,face))
                    {
                        // 停止向前移动
                        Change_dji_loc(DJI_M_CLIMB_RB,0);
                        Change_dji_loc(DJI_M_CLIMB_LB,0);
                        cha_remote(0,0,0);
                        current_move_state = MOVE_COMPLETE;
                    }
                }else if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)||HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)) {
                    // 停止向前移动
                    // Change_dji_loc(DJI_M_CLIMB_RB,2000);
                    // Change_dji_loc(DJI_M_CLIMB_LB,-2000);
                    cha_remote(0,0,0);
                    current_move_state = MOVE_COMPLETE;
                }
            }else
            {
                cha_remote(0, 0, vr);
            }
            break;
        }
        case MOVE_COMPLETE:
        {
            current_move_state = MOVE_IDLE;
            climb_cnt = 0;
            return 1;
            break;
        }

        default:
            current_move_state = MOVE_IDLE;
            break;
    }
    return 0;
}


//下楼梯的函数
int DownStairs(int curr_id, int stair_id)
{
    int face = get_face(curr_id,stair_id);
    // 假设按下 rc_engineer_data.button10_is_climb_trigger 是触发一键攀爬的按钮
    if ( current_down_state == DOWN_IDLE)
    {
        // 触发一键攀爬，开始第一步
        //Extend_Cylinder(); // 在开始之前先伸长气缸 (对应原图步骤2)
        current_down_state = DOWN_STEP1_BASE_FORWARD;
    }

    switch (current_down_state)
    {
        case DOWN_IDLE:
        {
            // 保持空闲，等待触发
            Change_dji_loc(DJI_M_CLIMB_LF,-100000);
            Change_dji_loc(DJI_M_CLIMB_RF,100000);
            Change_dji_loc(DJI_M_CLIMB_RB,-100000);
            Change_dji_loc(DJI_M_CLIMB_LB,100000);
            break;
        }

        // --- 步骤 1：底盘向前移动 ---
        case DOWN_STEP1_BASE_FORWARD:
                {
                    // 底盘向前移动，前轮搭在台子上 (原图步骤3)
                    // 计算靠近速度 (世界坐标系)，使用全局靠近PID实例
                    float now_pos = lcResult.r;                        // 机器人当前朝向角
                    float vr = PID_Angle_Calculate(&chassis_yaw_pid,face_angle(face),now_pos);
                    if (fabsf(lcResult.r-face_angle(face))<0.05f)
                    {
                        cha_remote(0,500,vr);
                        if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)) {
                            RTT_Printf("1111111\n");
                            cha_remote(0,0,0);
                            current_down_state = DOWN_STEP2_FRONT_DOWN;
                        }
                    }else
                    {
                        cha_remote(0, 0, vr);
                    }
                    break;
                }

        // --- 步骤 2：前侧下降 ---
        case DOWN_STEP2_FRONT_DOWN:
        {
            // 前轮抬到200平齐，后轮触地 (原图步骤2 + 原按钮)
            Change_dji_loc(DJI_M_CLIMB_LF,-back_up);
            Change_dji_loc(DJI_M_CLIMB_RF,back_up);
            Change_dji_loc(DJI_M_CLIMB_RB,25000);
            Change_dji_loc(DJI_M_CLIMB_LB,-25000);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_SET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_SET);
            // 判断电机是否到达目标位置 (或等待气缸伸长)
            // 假设我们使用一个简单的延时来等待气缸伸长完成
            // if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up))
            if (is_motor_cplt(DJI_M_CLIMB_LF,-back_up)&&is_motor_cplt(DJI_M_CLIMB_RF,back_up))//&&climb_cnt == 2)
            {
                current_down_state = DOWN_STEP3_REAR_FORWARD;
            }
            break;
        }

        // --- 步骤 3：2006往前走 ---
        case DOWN_STEP3_REAR_FORWARD:
        {
            // 2006推动底盘向前运动，让后轮也上台阶 (原图步骤6 + 原按钮)
            //cha_remote(0,500,0);

            if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10))
            {
                // 停止向前移动
                //cha_remote(0,0,0);

                current_down_state = DOWN_STEP4_DROP_DOWN;
            }
            break;

        }

        // --- 步骤 4：车身下降 ---
        case DOWN_STEP4_DROP_DOWN:
        {
            // 四个3508一起抬升底盘，将车身向上抬 (原按钮)
            // 此处抬升需要一个时间来完成，因为是速度控制或目标位置很远

            // 发送平滑处理后的期望位置
            Change_dji_loc(DJI_M_CLIMB_LF, 0);
            Change_dji_loc(DJI_M_CLIMB_RF, 0);
            Change_dji_loc(DJI_M_CLIMB_LB, climb_front_up);
            Change_dji_loc(DJI_M_CLIMB_RB, -climb_front_up);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_RESET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_RESET);

            if (is_motor_cplt(DJI_M_CLIMB_LF,0)&&is_motor_cplt(DJI_M_CLIMB_RF,0)
                &&is_motor_cplt(DJI_M_CLIMB_LB,climb_front_up)&&is_motor_cplt(DJI_M_CLIMB_RB,-climb_front_up))//&&climb_cnt == 4)
            {
                printf("进入下一步\n");
                current_down_state = DOWN_STEP5_BASE_FORWARD;
            }
            break;
        }

        // --- 步骤 5：电机归位 ---
        case DOWN_STEP5_BASE_FORWARD:
        {
            Point_struct now_point = {lcResult.x, lcResult.y}; // 机器人当前坐标点
            float now_pos = lcResult.r;                        // 机器人当前朝向角
            Point_struct end_point ={stairs_center[stair_id].x,stairs_center[stair_id].y};

            move_approach(now_point,end_point,now_pos,0);

            if (is_on_stair_center(stair_id))
            {
                cha_remote(0,0,0);
                current_down_state = DOWN_COMPLETE;
            }
            break;
        }

        case DOWN_COMPLETE:
        {
            current_down_state = DOWN_IDLE;
            down_cnt = 0;
            return 1;
            break;
        }

        default:
            current_down_state = DOWN_IDLE;
            break;
    }
    return 0;
}

// 200的台阶，可与ClimbStairs合并
// void UpStairs(void)
// {
//     //初始状态：前后均抬升一点（？）
//     //第一步，气缸伸长，前侧抬升
//     //第二步，底盘往前走，前侧放在台阶上，后侧放在地面上 upstairs_front_up upstairs_back_down
//     // LeftBack:599219
//     // RightBack:-565085
//     // LeftFront:-5087
//     // RightFront:-12602
//     //第三步，收气缸
//     //第四步，后侧2006推动
//     //第五步，收回
//     // 假设按下 rc_engineer_data.button10_is_climb_trigger 是触发一键攀爬的按钮
//     if ( current_climb_state == CLIMB_IDLE)
//     {
//         // 触发一键攀爬，开始第一步
//         //Extend_Cylinder(); // 在开始之前先伸长气缸 (对应原图步骤2)
//         if (climb_cnt == 1)
//         current_climb_state = CLIMB_STEP1_FRONT_UP;
//     }
//
//     switch (current_climb_state)
//     {
//         case CLIMB_IDLE:
//         {
//             // 保持空闲，等待触发
//              Change_dji_loc(DJI_M_CLIMB_LF,-climb_front_up);
//              Change_dji_loc(DJI_M_CLIMB_RF,climb_front_up);
//             Change_dji_loc(DJI_M_CLIMB_RB,100000);
//             Change_dji_loc(DJI_M_CLIMB_LB,-100000);
//             break;
//         }
//
//         // --- 步骤 1：前侧抬升，气缸抬升 ---
//         case CLIMB_STEP1_FRONT_UP:
//         {
//             // 前轮抬到200平齐，后轮触地 (原图步骤2 + 原按钮)
//             Change_dji_loc(DJI_M_CLIMB_LF,-climb_front_up);
//             Change_dji_loc(DJI_M_CLIMB_RF,climb_front_up);
//             Change_dji_loc(DJI_M_CLIMB_RB,0);
//             Change_dji_loc(DJI_M_CLIMB_LB,0);
//             HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_SET);
//             HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_SET);
//             // 判断电机是否到达目标位置 (或等待气缸伸长)
//             // 假设我们使用一个简单的延时来等待气缸伸长完成
//             // if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up))
//             if (is_motor_cplt(DJI_M_CLIMB_LF,-climb_front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,climb_front_up))//&&climb_cnt == 2)
//             {
//                 current_climb_state = CLIMB_STEP2_BASE_FORWARD;
//             }
//             break;
//         }
//
//         // --- 步骤 2：底盘向前移动 ---
//         case CLIMB_STEP2_BASE_FORWARD:
//         {
//             // 底盘向前移动，前轮搭在台子上 (原图步骤3)
//             cha_remote(0,1000,0);
//             if (fabsf(lcResult.y-ForestEdge)<10 || climb_cnt == 2)
//             {
//                 // 停止向前移动
//                 cha_remote(0,0,0);
//
//                 current_climb_state = CLIMB_STEP3_LIFT_UP;
//             }
//             break;
//         }
//
//         // --- 步骤 3：3508抬升车身 ---
//         case CLIMB_STEP3_LIFT_UP:
//         {
//             // 四个3508一起抬升底盘，将车身向上抬 (原按钮)
//             // 此处抬升需要一个时间来完成，因为是速度控制或目标位置很远
//             Change_dji_loc(DJI_M_CLIMB_LF,-front_up2);
//             Change_dji_loc(DJI_M_CLIMB_RF,front_up2);
//             Change_dji_loc(DJI_M_CLIMB_LB,back_up);
//             Change_dji_loc(DJI_M_CLIMB_RB,-back_up);
//             // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_RESET);
//             // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_RESET);
//
//             if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up2)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up2)
//                 &&is_motor_cplt(DJI_M_CLIMB_LB,back_up)&&is_motor_cplt(DJI_M_CLIMB_RB,-back_up))//&&climb_cnt == 4)
//             {
//                 current_climb_state = CLIMB_STEP4_REAR_FORWARD;
//             }
//             break;
//         }
//
//         // --- 步骤 4：后侧2006推动 ---
//         case CLIMB_STEP4_REAR_FORWARD:
//         {
//             // 2006推动底盘向前运动，让后轮也上台阶 (原图步骤6 + 原按钮)
//             Change_dji_speed(DJI_2006_L, -8000);
//             Change_dji_speed(DJI_2006_R, 8000);
//
//             if (fabsf(lcResult.y+800-ForestEdge)<10 || climb_cnt == 3)
//             {
//                  // 停止向前移动
//                 Change_dji_speed(DJI_2006_L, 0);
//                 Change_dji_speed(DJI_2006_R, 0);
//
//                 current_climb_state = CLIMB_STEP5_RESET_ALL;
//             }
//             break;
//         }
//
//         // --- 步骤 6：电机归位 ---
//         case CLIMB_STEP5_RESET_ALL:
//         {
//             // 四个3508归位 (原图步骤7 + 原按钮)
//             Change_dji_loc(DJI_M_CLIMB_LF,-300000);
//             Change_dji_loc(DJI_M_CLIMB_RF,300000);
//             Change_dji_loc(DJI_M_CLIMB_LB,-100000);
//             Change_dji_loc(DJI_M_CLIMB_RB,100000);
//
//             // 假设归位需要 TARGET_HOME_LOC 运行时间
//             if (is_motor_cplt(DJI_M_CLIMB_LF,-300000)&&is_motor_cplt(DJI_M_CLIMB_RF,300000)
//                 &&is_motor_cplt(DJI_M_CLIMB_LB,-100000)&&is_motor_cplt(DJI_M_CLIMB_RB,100000))
//             {
//                 current_climb_state = CLIMB_COMPLETE;
//             }
//             break;
//         }
//
//         case CLIMB_COMPLETE:
//         {
//             current_climb_state = CLIMB_IDLE;
//             climb_cnt = 0;
//             break;
//         }
//
//         default:
//             current_climb_state = CLIMB_IDLE;
//             break;
//     }
// }

//580
//320 (320 580)  (-1480,1180)-> (-1180,1480)
