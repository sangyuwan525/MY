//
// Created by 马皓然 on 2025/11/6.
//

#include <stdio.h>
#include "cmsis_os2.h"
#include "Task_Printf.h"

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
        uint32_t primask_bit = __get_PRIMASK();
        //printf("x=%f\n\r",lcResult.x);
        //printf("y=%f\n\r",lcResult.y);
        printf("yaw=%f\n\r",lcResult.r);
        __set_PRIMASK(primask_bit);
        osDelay(500);
    }
    /* USER CODE END StartTask_Printf */
}
