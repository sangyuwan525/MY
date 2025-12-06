//
// Created by 马皓然 on 2025/11/27.
//
#include "remote_driver.h"
#include "chassis_driver.h"
#include "Task_chassis.h"
#include "dji_3508_2006_motor.h"

#define front_up 300000

#define back_up 285000
#define front_up2 0


void StartTask_chassis(void *argument)
{
    /* USER CODE BEGIN StartTask_chassis */
    remote_engineer_t rc_engineer_data;
    /* Infinite loop */
    for(;;)
    {
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
                // 按钮1被按下，执行相应操作
                Change_dji_loc(4,-front_up);
                Change_dji_loc(5,front_up);

            }
            //
            if (rc_engineer_data.button2 == 1)
            {
                // 按钮2被按下，一起抬升
                //Change_dji_loc(4,-front_up2);
                //Change_dji_loc(5,front_up2);
                Change_dji_loc(6,0);
            }
            if (rc_engineer_data.button3 == 1)
            {
                // 按钮3被按下，一起抬升
                Change_dji_loc(DJI_M_CHASSIS_B,0);
            }
            if (rc_engineer_data.button4 == 1)
            {
                // 按钮3被按下，一起抬升
                Change_dji_speed(7,-1000);
                Change_dji_speed(8,1000);
            }else
            {
                Change_dji_speed(7,0);
                Change_dji_speed(8,0);
            }
            if (rc_engineer_data.button5 == 1)
            {
                // 按钮3被按下，一起抬升
                Change_dji_loc(6,back_up);
                Change_dji_loc(4,-front_up2);
                Change_dji_loc(5,front_up2);
            }
            int number1=Get_dji_information(6).total_angle;
            int number2=Get_dji_information(5).total_angle;

        }
        else
        {
            // 如果获取数据失败（互斥量争夺失败），可以考虑错误处理或跳过本次循环
        }
        //Change_dji_speed(0,1000);
        // 任务延时，保证任务周期性运行，例如每 10ms 运行一次
        osDelay(CHASSIS_TASK_PERIOD);
    }
    /* USER CODE END StartTask_chassis */
}