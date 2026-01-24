//
// Created by Lenovo on 2026/1/24.
//
#include "Task_locator_recv.h"
#include "remote_driver.h"
#include "Task_dji_control.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "locator_driver.h"


void StartTask_locator_recv(void *argument)
{
    /* USER CODE BEGIN StartTask_locator_recv */
    /* Infinite loop */
    Locator_Rx_Queue_t rx_msg_tmp;
    for(;;)
    {
       while (xQueueReceive((QueueHandle_t)(locatorQueue_x_yHandle), &rx_msg_tmp, 0) == pdPASS) // 依次处理队列中的所有数据，直到清空
            {
               analysis_locator_X_Y(&lcResult,  &rx_msg_tmp); //调用locator_driver中的数据解析函数，并把数据存入lcResult中
            }
        while (xQueueReceive((QueueHandle_t)(locatorQueue_z_rHandle), &rx_msg_tmp, 0) == pdPASS) // 依次处理队列中的所有数据，直到清空
            {
               analysis_locator_Z_R(&lcResult,  &rx_msg_tmp); //调用locator_driver中的数据解析函数，并把数据存入lcResult中
            }

        osDelay(20);
    }
    /* USER CODE END StartTask_locator_recv */
}
