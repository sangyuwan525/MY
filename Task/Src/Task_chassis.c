//
// Created by 马皓然 on 2025/11/27.
//
#include "remote_driver.h"
#include "chassis_driver.h"
#include "Task_chassis.h"



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
            if (rc_engineer_data.mode == 2)
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
        }
        else
        {
            // 如果获取数据失败（互斥量争夺失败），可以考虑错误处理或跳过本次循环
        }
        // 任务延时，保证任务周期性运行，例如每 10ms 运行一次
        osDelay(CHASSIS_TASK_PERIOD);
    }
    /* USER CODE END StartTask_chassis */
}