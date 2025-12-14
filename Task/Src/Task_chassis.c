//
// Created by 马皓然 on 2025/11/27.
//
#include "remote_driver.h"
#include "chassis_driver.h"
#include "Task_chassis.h"
#include "dji_3508_2006_motor.h"
#include "ClimbStairs.h"

int chassis_control_cnt;


void StartTask_chassis(void *argument)
{
    /* USER CODE BEGIN StartTask_chassis */
    remote_engineer_t rc_engineer_data;
    chassis_control_cnt=0;
    /* Infinite loop */
    for(;;)
    {
        if (chassis_control_cnt>=20)//失联超过200ms,速度衰减
        {
            rc_engineer_data.vx*=0.95;
            rc_engineer_data.vy*=0.95;
            rc_engineer_data.vw*=0.95;
            cha_remote(rc_engineer_data.vx,
                       rc_engineer_data.vy,
                       rc_engineer_data.vw);
            //cha_remote(0,0,0);
        }else
        {
            chassis_control_cnt++;
            // 使用 Remote_GetEngineerData 确保在互斥量保护下安全读取
            if (Remote_GetEngineerData(&rc_engineer_data) == pdPASS)
            {
                // 模式 2 为手动模式
                if (rc_engineer_data.mode == CHASSIS_MODE_MANUAL)
                {
                    // 将遥控器工程量速度 (vx, vy, vw) 传入底盘驱动
                    cha_remote(rc_engineer_data.vx,
                               rc_engineer_data.vy,
                               rc_engineer_data.vw);
                }
                else // 其他模式 (待机/自动)，底盘速度清零
                {
                    // 停止底盘，发送 (0, 0, 0) 指令
                    cha_remote(0.0f, 0.0f, 0.0f);
                }
                //前3508抬升
                if (rc_engineer_data.button1 == 1)
                {
                    // 按钮1被按下，前面两个3508抬升
                    // Change_dji_loc(DJI_M_CLIMB_LF,-front_up);
                    // Change_dji_loc(DJI_M_CLIMB_RF,front_up);
                    //气缸测试 收
                    HAL_GPIO_WritePin(valve_port,valve_pin_l,0);
                    HAL_GPIO_WritePin(valve_port,valve_pin_r,0);
                }
                //
                if (rc_engineer_data.button2 == 1)
                {
                    // 按钮2被按下，四个3508归位
                    //Change_dji_loc(4,-front_up2);
                    //Change_dji_loc(5,front_up2);
                    // Change_dji_loc(DJI_M_CLIMB_LF,0);
                    // Change_dji_loc(DJI_M_CLIMB_RF,0);
                    // Change_dji_loc(DJI_M_CLIMB_LB,0);
                    // Change_dji_loc(DJI_M_CLIMB_RB,0);
                    Change_dji_loc(DJI_M_CLIMB_LF,-10000);
                    Change_dji_loc(DJI_M_CLIMB_RF,10000);
                    Change_dji_loc(DJI_M_CLIMB_LB,-10000);
                    Change_dji_loc(DJI_M_CLIMB_RB,10000);
                }
                if (rc_engineer_data.button3 == 1)
                {
                    // 按钮3被按下，2006推动底盘向前运动
                    // Change_dji_speed(DJI_2006_L,2500);
                    // Change_dji_speed(DJI_2006_R,-2500);
                    //气缸测试 放
                    HAL_GPIO_WritePin(valve_port,valve_pin_l,1);
                    HAL_GPIO_WritePin(valve_port,valve_pin_r,1);
                }
                else if (rc_engineer_data.button4 == 1)
                {
                    // 按钮4被按下，下楼梯时2006向相反方向运动
                    Change_dji_speed(DJI_2006_L,-2500);
                    Change_dji_speed(DJI_2006_R,2500);
                }
               else
                {
                    Change_dji_speed(DJI_2006_L,0);
                    Change_dji_speed(DJI_2006_R,0);
                }
                if (rc_engineer_data.button5 == 1)
                {
                    // 按钮5被按下，四个3508一起抬升底盘
                    //Change_dji_loc(6,back_up);
                    //Change_dji_loc(4,-front_up2);
                    //Change_dji_loc(5,front_up2);
                    Change_dji_loc(DJI_M_CLIMB_LF,-front_up2);
                    Change_dji_loc(DJI_M_CLIMB_RF,front_up2);
                    Change_dji_loc(DJI_M_CLIMB_LB,back_up);
                    Change_dji_loc(DJI_M_CLIMB_RB,-back_up);
                }
                if (rc_engineer_data.button6 == 1)
                {
                    // 按钮3被按下，一起抬升
                    //Change_dji_loc(6,back_up);
                    //Change_dji_loc(4,-front_up2);
                    //Change_dji_loc(5,front_up2);
                    Change_dji_loc(DJI_M_CLIMB_LF,-front_up);
                    Change_dji_loc(DJI_M_CLIMB_RF,front_up);
                    Change_dji_loc(DJI_M_CLIMB_LB,0);
                    Change_dji_loc(DJI_M_CLIMB_RB,0);
                }
            }
            else
            {
                // 如果获取数据失败（互斥量争夺失败），可以考虑错误处理或跳过本次循环
                // rc_engineer_data.vx*=0.9;
                // rc_engineer_data.vy*=0.9;
                // rc_engineer_data.vw*=0.9;
                // cha_remote(rc_engineer_data.vx,
                //            rc_engineer_data.vy,
                //            rc_engineer_data.vw);
                cha_remote(0,0,0);
            }
            //Change_dji_speed(0,1000);
            // 任务延时，保证任务周期性运行，例如每 10ms 运行一次
        }
        osDelay(CHASSIS_TASK_PERIOD);
    }
    /* USER CODE END StartTask_chassis */
}