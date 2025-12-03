//
// Created by 马皓然 on 2025/11/27.
//
#include "remote_driver.h"
#include "chassis_driver.h"
#include "Task_chassis.h"
#include "Task_dji_control.h"
#include "dji_3508_2006_motor.h"
#include "queue.h"

void StartTask_dji(void *argument)
{
    /* USER CODE BEGIN StartTask_dji */
    TickType_t xLastWakeTime;
    Motor_Rx_Queue_t rx_msg_tmp;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {
        while (xQueueReceive((QueueHandle_t)motorRxQueueHandle, &rx_msg_tmp, 0) == pdPASS) // 0表示不等待
        {
            Dji_Motor_Update_Status(rx_msg_tmp.hcan, rx_msg_tmp.motor_id, rx_msg_tmp.rx_data);
        }

        Dji_3508_all_motor_control();

        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
    /* USER CODE END StartTask_dji */
}