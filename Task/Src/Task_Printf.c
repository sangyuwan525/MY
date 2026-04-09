//
// Created by 马皓然 on 2025/11/6.
//

#include <stdio.h>
#include "cmsis_os2.h"
#include "Task_Printf.h"
#include "chassis_path.h"
#include "ClimbStairs.h"
#include "debug.h"
#include "cmsis_gcc.h"
#include "locator_driver.h"
#include "Hfsm.h"
#include "Task_chassis.h"
#include "remote_driver.h"
#include "stm32g4xx_hal.h"  // 根据你的MCU型号选择对应的头文件

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
        // printf("x=%.1f,",lcResult.x);
        // printf("y=%.1f,",lcResult.y);
        // printf("r=%.3f\n",lcResult.r);
        //RTT_Printf("vx=%f  vy=%f\n",remote_engineer.vx,remote_engineer.vy);
        //RTT_Printf("test_cnt%d,state%d,cnt%d\n",climb_test_cnt,current_climb_state,climb_cnt);
        //printf("开关:%d\n",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11));
        //printf("top:%d, mc:%d, mf:%d, cf:%d\n",g_robot_ctx.current_top_state,g_robot_ctx.sub_state.mc,g_robot_ctx.sub_state.mf,g_robot_ctx.sub_state.cf);
        //printf("MC_flag:%d,MF_flag:%d,CF_flag:%d\n",MC_flag,MF_flag,CF_flag);
        //printf("cid=%d,tid=%d,r2_taken=%d,c_step=%d,path_len=%d\n",g_robot_ctx.current_stair_id,g_robot_ctx.target_stair_id,g_robot_ctx.already_taken,g_robot_ctx.current_step,g_robot_ctx.plan.path_len);
        RTT_Printf("pb10 %d,pb11 %d\n",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11));
        RTT_Printf("pc0 %d,pc1 %d\n",HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1));
        //RTT_Printf("face %d\n",get_face(1,0));
        osDelay(1000);
    }
    /* USER CODE END StartTask_Printf */
}
