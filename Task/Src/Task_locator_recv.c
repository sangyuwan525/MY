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
    (void)argument;

    Locator_Rx_Queue_t rx_msg_tmp;

    for (;;) {
        /* locatorQueue_x_yHandle keeps its old name, but now carries 0x12 x/y/yaw data. */
        while (xQueueReceive((QueueHandle_t)locatorQueue_x_yHandle, &rx_msg_tmp, 0) == pdPASS) {
            analysis_locator_X_Y(&lcResult, &rx_msg_tmp);
        }
        /* locatorQueue_z_rHandle keeps its old name, but now carries 0x100 laser data. */
        while (xQueueReceive((QueueHandle_t)locatorQueue_z_rHandle, &rx_msg_tmp, 0) == pdPASS) {
            analysis_locator_laser(&lcResult, &rx_msg_tmp);
        }

        osDelay(20);
    }
}
