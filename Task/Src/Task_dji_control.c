//
// Created by 马皓然 on 2025/11/27.
//
#include "remote_driver.h"
#include "chassis_driver.h"
#include "Task_chassis.h"
#include "Task_dji_control.h"
#include "dji_3508_2006_motor.h"
#include "global_motor_conf.h"
#include "motor_registry.h"
#include "queue.h"

void StartTask_dji(void *argument)
{
    /* USER CODE BEGIN StartTask_dji */
    TickType_t xLastWakeTime;
    Motor_Rx_Queue_t rx_msg_tmp;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    //osDelay(100);
     Motor_Registry_Init();

    g_motor_list[BLAZER_FOC_MOTOR1_G].set_speed(
    &g_motor_list[BLAZER_FOC_MOTOR1_G],
    5.0f   // 5 r/s = 300 RPM
);

//     g_motor_list[UNITREE_GO_M8010_6_MOTOR1_G].set_speed(
//     &g_motor_list[UNITREE_GO_M8010_6_MOTOR1_G],
//     0.08f
// );
//     g_motor_list[UNITREE_GO_M8010_6_MOTOR1_G].set_mit(
//     &g_motor_list[UNITREE_GO_M8010_6_MOTOR1_G],
//     0.0f, -1.0f, 0.0f, 0.01f, 0.0f
// );
    // 2. 再测位置保持/位置运动
    g_motor_list[UNITREE_GO_M8010_6_MOTOR1_G].set_mit(
        &g_motor_list[UNITREE_GO_M8010_6_MOTOR1_G],
        0.5f, 0.0f, 0.05f, 0.01f, 0.0f
    );

    g_motor_list[XIAOMI_MOTOR1_G].set_mit(&g_motor_list[XIAOMI_MOTOR1_G], 10.0f,0.0f,2.0f,0.1f,0.0f);

    g_motor_list[DM_JOINT_G].set_mit(&g_motor_list[DM_JOINT_G], 10.0f,0.0f,2.0f,0.1f,0.0f);
    // g_dm_motor_registry[DM_Motor1].ctrl.mode = pos_mode;
    // dm_motor_enable(&g_dm_motor_registry[DM_Motor1]);
    // g_motor_list[DM_JOINT_G].set_position(&g_motor_list[DM_JOINT_G], 1.0f, 2.0f);

    // g_dm_motor_registry[DM_Motor1].ctrl.mode = mit_mode;
    // dm_motor_enable(&g_dm_motor_registry[DM_Motor1]);
    //g_dm_motor_registry[DM_Motor1].ctrl.mode = pos_mode;
    //dm_motor_enable(&g_dm_motor_registry[DM_Motor1]);
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {
        //Motor_Registry_Init();
        // g_dm_motor_registry[DM_Motor1].ctrl.mode = mit_mode;
        // dm_motor_enable(&g_dm_motor_registry[DM_Motor1]);
         //g_motor_list[DM_JOINT_G].set_mit(&g_motor_list[DM_JOINT_G], 10.0f,0.0f,2.0f,0.1f,0.0f);
        //g_motor_list[DM_JOINT_G].set_position(&g_motor_list[DM_JOINT_G], 10.0f, 2.0f);
        //Motor_SetMIT(DM_JOINT_G, 5.0f, 0.0f, 2.0f, 0.1f, 0.0f);
        while (xQueueReceive((QueueHandle_t)motorRxQueueHandle, &rx_msg_tmp, 0) == pdPASS) // 0表示不等待
        {
            Motor_Feedback_Dispatch(rx_msg_tmp.hfdcan, rx_msg_tmp.id, rx_msg_tmp.data);
        }

        Motor_All_Control_Loop();

        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
    /* USER CODE END StartTask_dji */
}
