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

static Motor_Smooth_Goto_Profile_t xiaomi_profile;

typedef enum {
    XIAOMI_MOVE_WAIT_FEEDBACK = 0,
    XIAOMI_MOVE_RUNNING,
    XIAOMI_MOVE_DONE,
} Xiaomi_Move_State_e;

static Xiaomi_Move_State_e xiaomi_move_state = XIAOMI_MOVE_WAIT_FEEDBACK;

void StartTask_dji(void *argument)
{
    /* USER CODE BEGIN StartTask_dji */
    TickType_t xLastWakeTime;
    Motor_Rx_Queue_t rx_msg_tmp;
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    //osDelay(100);
     Motor_Registry_Init();

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

        switch (xiaomi_move_state) {
            case XIAOMI_MOVE_WAIT_FEEDBACK:
                if (g_xiaomi_motor_registry[XIAOMI_Motor1].feedback.online == true) {
                    Motor_SmoothGoto_Start(&xiaomi_profile,
                                           XIAOMI_MOTOR1_G,
                                           g_xiaomi_motor_registry[XIAOMI_Motor1].feedback.angle + 5.0f,
                                           2.0f);
                    xiaomi_move_state = XIAOMI_MOVE_RUNNING;
                }
                break;

            case XIAOMI_MOVE_RUNNING:
                if (Motor_RunSmoothGotoMIT(XIAOMI_MOTOR1_G, &xiaomi_profile, 50.0f, 1.0f, 0.0f) == 0U) {
                    xiaomi_move_state = XIAOMI_MOVE_DONE;
                }
                break;

            case XIAOMI_MOVE_DONE:
            default:
                break;
        }

        Motor_All_Control_Loop();

        xLastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil( &xLastWakeTime, xFrequency);
    }
    /* USER CODE END StartTask_dji */
}
