//
// Created by 马皓然 on 2025/11/6.
//

#include <stdio.h>
#include "cmsis_os2.h"
#include "Task_Printf.h"
#include "chassis_path.h"
#include "ClimbStairs.h"

#include "cmsis_gcc.h"
#include "locator_driver.h"

void StartTask_Printf(void *argument)
{
    /* USER CODE BEGIN StartTask_Printf */
    remote_engineer_t chassis_cmd; // 用于接收工程量数据的局部变量
    char message[100];
    /* Infinite loop */
    for(;;)
    {
        if (Remote_GetEngineerData(&chassis_cmd) == pdPASS) {
            float desired_vx = chassis_cmd.vx;
            float desired_vy = chassis_cmd.vy;
            float desired_vw = chassis_cmd.vw;
            // sprintf(message,"desired_vx: %.2f, desired_vy: %.2f, desired_vw: %.2f\r\n",desired_vx, desired_vy, desired_vw);
            // printf("%s",message);
            // printf("desired_vx = %f\n", 1000*desired_vx);
        }
        //printf("x66 y55 z66\n");
        printf("%.1f,",lcResult.x);
        printf("%.1f,",lcResult.y);
        printf("%.3f,",lcResult.r);
        printf("%.3f\n",test_angle);
        printf("state:%d,cnt:%d\n",current_climb_state,climb_cnt);
        printf("edge:%d\n",is_on_stair_edge(2,0));
        osDelay(500);
    }
    /* USER CODE END StartTask_Printf */
}
