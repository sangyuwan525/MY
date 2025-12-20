//
// Created by lcf on 2025/12/1.
//

#include "../Inc/ClimbStairs.h"

#include <stdlib.h>
#include "gpio.h"
#include "chassis_driver.h"
#include "locator_driver.h"

#define  ForestEdge 100  //  梅林边界
int climb_cnt = 0;
int down_cnt = 0;

// 状态变量
Climb_State_e current_climb_state = CLIMB_IDLE;
Down_State_e current_down_state = DOWN_IDLE;
uint32_t step_start_time = 0; // 用于计时延时步骤

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
    int   lower_lim[5]={};
    int upper_lim[5]={};
    //int now_loc=Get_dji_information(motor_id).angle;
    if (encoder_counts<lower_lim[motor_id]) encoder_counts=lower_lim[motor_id];
    if (encoder_counts>upper_lim[motor_id]) encoder_counts=upper_lim[motor_id];
    int send_loc=encoder_counts;
    Change_dji_loc(motor_id,send_loc);
}

//电机零点在哪，往哪为上?
void ClimbStairs(void)
{
    // 假设按下 rc_engineer_data.button10_is_climb_trigger 是触发一键攀爬的按钮
    if ( current_climb_state == CLIMB_IDLE)
    {
        // 触发一键攀爬，开始第一步
        //Extend_Cylinder(); // 在开始之前先伸长气缸 (对应原图步骤2)
        if (climb_cnt == 1)
        current_climb_state = CLIMB_STEP1_FRONT_UP;
    }

    switch (current_climb_state)
    {
        case CLIMB_IDLE:
        {
            // 保持空闲，等待触发
             Change_dji_loc(DJI_M_CLIMB_LF,-front_up);
             Change_dji_loc(DJI_M_CLIMB_RF,front_up);
            Change_dji_loc(DJI_M_CLIMB_RB,100000);
            Change_dji_loc(DJI_M_CLIMB_LB,-100000);
            break;
        }

        // --- 步骤 1：前侧抬升 ---
        case CLIMB_STEP1_FRONT_UP:
        {
            // 前轮抬到200平齐，后轮触地 (原图步骤2 + 原按钮1)
            Change_dji_loc(DJI_M_CLIMB_LF,-front_up);
            Change_dji_loc(DJI_M_CLIMB_RF,front_up);
            Change_dji_loc(DJI_M_CLIMB_RB,0);
            Change_dji_loc(DJI_M_CLIMB_LB,0);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_SET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_SET);
            // 判断电机是否到达目标位置 (或等待气缸伸长)
            // 假设我们使用一个简单的延时来等待气缸伸长完成
            // if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up))
            if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up))//&&climb_cnt == 2)
            {
                current_climb_state = CLIMB_STEP2_BASE_FORWARD;
            }
            break;
        }

        // --- 步骤 2：底盘向前移动 ---
        case CLIMB_STEP2_BASE_FORWARD:
        {
            // 底盘向前移动，前轮搭在台子上 (原图步骤3)
            cha_remote(0,1000,0);
            if (fabsf(lcResult.y-ForestEdge)<10 || climb_cnt == 2)
            {
                // 停止向前移动
                cha_remote(0,0,0);

                current_climb_state = CLIMB_STEP3_LIFT_UP;
            }
            break;
        }

        // --- 步骤 3：3508抬升车身 ---
        case CLIMB_STEP3_LIFT_UP:
        {
            // 四个3508一起抬升底盘，将车身向上抬 (原按钮5)
            // 此处抬升需要一个时间来完成，因为是速度控制或目标位置很远
            Change_dji_loc(DJI_M_CLIMB_LF,-front_up2);
            Change_dji_loc(DJI_M_CLIMB_RF,front_up2);
            Change_dji_loc(DJI_M_CLIMB_LB,back_up);
            Change_dji_loc(DJI_M_CLIMB_RB,-back_up);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_RESET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_RESET);

            if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up2)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up2)
                &&is_motor_cplt(DJI_M_CLIMB_LB,back_up)&&is_motor_cplt(DJI_M_CLIMB_RB,-back_up))//&&climb_cnt == 4)
            {
                current_climb_state = CLIMB_STEP4_REAR_FORWARD;
            }
            break;
        }

        // --- 步骤 4：后侧2006推动 ---
        case CLIMB_STEP4_REAR_FORWARD:
        {
            // 2006推动底盘向前运动，让后轮也上台阶 (原图步骤6 + 原按钮3)
            Change_dji_speed(DJI_2006_L, -8000);
            Change_dji_speed(DJI_2006_R, 8000);

            if (fabsf(lcResult.y+800-ForestEdge)<10 || climb_cnt == 3)
            {
                 // 停止向前移动
                Change_dji_speed(DJI_2006_L, 0);
                Change_dji_speed(DJI_2006_R, 0);

                current_climb_state = CLIMB_STEP5_RESET_ALL;
            }
            break;
        }

        // --- 步骤 6：电机归位 ---
        case CLIMB_STEP5_RESET_ALL:
        {
            // 四个3508归位 (原图步骤7 + 原按钮2)
            Change_dji_loc(DJI_M_CLIMB_LF,-300000);
            Change_dji_loc(DJI_M_CLIMB_RF,300000);
            Change_dji_loc(DJI_M_CLIMB_LB,-100000);
            Change_dji_loc(DJI_M_CLIMB_RB,100000);

            // 假设归位需要 TARGET_HOME_LOC 运行时间
            if (is_motor_cplt(DJI_M_CLIMB_LF,-300000)&&is_motor_cplt(DJI_M_CLIMB_RF,300000)
                &&is_motor_cplt(DJI_M_CLIMB_LB,-100000)&&is_motor_cplt(DJI_M_CLIMB_RB,100000))
            {
                current_climb_state = CLIMB_COMPLETE;
            }
            break;
        }

        case CLIMB_COMPLETE:
        {
            current_climb_state = CLIMB_IDLE;
            climb_cnt = 0;
            break;
        }

        default:
            current_climb_state = CLIMB_IDLE;
            break;
    }
}
//下楼梯的函数
void DownStairs(void)
{
    // 假设按下 rc_engineer_data.button10_is_climb_trigger 是触发一键攀爬的按钮
    if ( current_down_state == DOWN_IDLE)
    {
        // 触发一键攀爬，开始第一步
        //Extend_Cylinder(); // 在开始之前先伸长气缸 (对应原图步骤2)
        if (down_cnt == 1)
        current_down_state = DOWN_STEP1_BASE_FORWARD;
    }

    switch (current_down_state)
    {
        case DOWN_IDLE:
        {
            // 保持空闲，等待触发
            Change_dji_loc(DJI_M_CLIMB_LF,-100000);
            Change_dji_loc(DJI_M_CLIMB_RF,100000);
            Change_dji_loc(DJI_M_CLIMB_RB,100000);
            Change_dji_loc(DJI_M_CLIMB_LB,-100000);
            break;
        }

        // --- 步骤 1：底盘向前移动 ---
        case DOWN_STEP1_BASE_FORWARD:
                {
                    // 底盘向前移动，前轮搭在台子上 (原图步骤3)
                    cha_remote(0,200,0);
                    if (fabsf(lcResult.y-ForestEdge)<10 || down_cnt == 2)
                    {
                        // 停止向前移动
                        cha_remote(0,0,0);

                        current_down_state = DOWN_STEP2_FRONT_DOWN;
                    }
                    break;
                }

        // --- 步骤 2：前侧下降 ---
        case DOWN_STEP2_FRONT_DOWN:
        {
            // 前轮抬到200平齐，后轮触地 (原图步骤2 + 原按钮1)
            Change_dji_loc(DJI_M_CLIMB_LF,back_up+30000);
            Change_dji_loc(DJI_M_CLIMB_RF,-back_up-30000);
            Change_dji_loc(DJI_M_CLIMB_RB,-30000);
            Change_dji_loc(DJI_M_CLIMB_LB,30000);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_SET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_SET);
            // 判断电机是否到达目标位置 (或等待气缸伸长)
            // 假设我们使用一个简单的延时来等待气缸伸长完成
            // if (is_motor_cplt(DJI_M_CLIMB_LF,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RF,front_up))
            if (is_motor_cplt(DJI_M_CLIMB_LF,back_up+30000)&&is_motor_cplt(DJI_M_CLIMB_RF,-back_up-30000))//&&climb_cnt == 2)
            {
                current_down_state = DOWN_STEP3_REAR_FORWARD;
            }
            break;
        }

        // --- 步骤 3：2006往前走 ---
        case DOWN_STEP3_REAR_FORWARD:
        {
            // 2006推动底盘向前运动，让后轮也上台阶 (原图步骤6 + 原按钮3)
            //cha_remote(0,500,0);
            Change_dji_speed(DJI_2006_L, -2000);
            Change_dji_speed(DJI_2006_R, 2000);

            if (fabsf(lcResult.y+800-ForestEdge)<10 || down_cnt == 3)
            {
                // 停止向前移动
                Change_dji_speed(DJI_2006_L, 0);
                Change_dji_speed(DJI_2006_R, 0);
                //cha_remote(0,0,0);

                current_down_state = DOWN_STEP4_DROP_DOWN;
            }
            break;

        }

        // --- 步骤 4：车身下降 ---
        case DOWN_STEP4_DROP_DOWN:
        {
            // 四个3508一起抬升底盘，将车身向上抬 (原按钮5)
            // 此处抬升需要一个时间来完成，因为是速度控制或目标位置很远
            Change_dji_loc(DJI_M_CLIMB_LF,0);
            Change_dji_loc(DJI_M_CLIMB_RF,0);
            Change_dji_loc(DJI_M_CLIMB_LB,-front_up);
            Change_dji_loc(DJI_M_CLIMB_RB,front_up);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN1,GPIO_PIN_RESET);
            // HAL_GPIO_WritePin(CYLINDER_GPIO_PORT,CYLINDER_PIN2,GPIO_PIN_RESET);

            if (is_motor_cplt(DJI_M_CLIMB_LF,0)&&is_motor_cplt(DJI_M_CLIMB_RF,0)
                &&is_motor_cplt(DJI_M_CLIMB_LB,-front_up)&&is_motor_cplt(DJI_M_CLIMB_RB,front_up))//&&climb_cnt == 4)
            {
                current_down_state = DOWN_STEP5_BASE_FORWARD;
            }
            break;
        }

        // --- 步骤 5：电机归位 ---
        case DOWN_STEP5_BASE_FORWARD:
        {
            cha_remote(0,100,0);

                if (fabsf(lcResult.y+800-ForestEdge)<10 || down_cnt == 4)
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
            break;
        }

        default:
            current_down_state = DOWN_IDLE;
            break;
    }
}

//上400的台阶，可与ClimbStairs合并
void UpStairs(void)
{
    //初始状态 前后均抬升一点（？）
    //第一步，气缸伸长，前侧抬升
    //第二步，底盘往前走，前侧放在台阶上，后侧放在地面上 upstairs_front_up upstairs_back_down
    // LeftBack:599219
    // RightBack:-565085
    // LeftFront:-5087
    // RightFront:-12602
    //第三步，收气缸
    //第四步，后侧2006走
    //第五步，收回
}