/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Task_command.h"
#include "dji_3508_2006_motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_chassis */
osThreadId_t Task_chassisHandle;
const osThreadAttr_t Task_chassis_attributes = {
  .name = "Task_chassis",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for Task_LED */
osThreadId_t Task_LEDHandle;
const osThreadAttr_t Task_LED_attributes = {
  .name = "Task_LED",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Task_Printf */
osThreadId_t Task_PrintfHandle;
const osThreadAttr_t Task_Printf_attributes = {
  .name = "Task_Printf",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for Taskcommand */
osThreadId_t TaskcommandHandle;
const osThreadAttr_t Taskcommand_attributes = {
  .name = "Taskcommand",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for Task_dji */
osThreadId_t Task_djiHandle;
const osThreadAttr_t Task_dji_attributes = {
  .name = "Task_dji",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for remote_queue */
osMessageQueueId_t remote_queueHandle;
const osMessageQueueAttr_t remote_queue_attributes = {
  .name = "remote_queue"
};
/* Definitions for motorRxQueue */
osMessageQueueId_t motorRxQueueHandle;
const osMessageQueueAttr_t motorRxQueue_attributes = {
  .name = "motorRxQueue"
};
/* Definitions for rc_mutex */
osMutexId_t rc_mutexHandle;
const osMutexAttr_t rc_mutex_attributes = {
  .name = "rc_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_chassis(void *argument);
void StartTask_LED(void *argument);
void StartTask_Printf(void *argument);
void StartTaskcommand(void *argument);
void StartTask_dji(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of rc_mutex */
  rc_mutexHandle = osMutexNew(&rc_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of remote_queue */
  remote_queueHandle = osMessageQueueNew (16, sizeof(UartRxMessage_t), &remote_queue_attributes);

  /* creation of motorRxQueue */
  motorRxQueueHandle = osMessageQueueNew (16, sizeof(Motor_Rx_Queue_t), &motorRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_chassis */
  Task_chassisHandle = osThreadNew(StartTask_chassis, NULL, &Task_chassis_attributes);

  /* creation of Task_LED */
  Task_LEDHandle = osThreadNew(StartTask_LED, NULL, &Task_LED_attributes);

  /* creation of Task_Printf */
  Task_PrintfHandle = osThreadNew(StartTask_Printf, NULL, &Task_Printf_attributes);

  /* creation of Taskcommand */
  TaskcommandHandle = osThreadNew(StartTaskcommand, NULL, &Taskcommand_attributes);

  /* creation of Task_dji */
  Task_djiHandle = osThreadNew(StartTask_dji, NULL, &Task_dji_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_chassis */
/**
  * @brief  Function implementing the Task_chassis thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_chassis */
__weak void StartTask_chassis(void *argument)
{
  /* USER CODE BEGIN StartTask_chassis */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_chassis */
}

/* USER CODE BEGIN Header_StartTask_LED */
/**
* @brief Function implementing the Task_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_LED */
__weak void StartTask_LED(void *argument)
{
  /* USER CODE BEGIN StartTask_LED */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_LED */
}

/* USER CODE BEGIN Header_StartTask_Printf */
/**
* @brief Function implementing the Task_Printf thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_Printf */
__weak void StartTask_Printf(void *argument)
{
  /* USER CODE BEGIN StartTask_Printf */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_Printf */
}

/* USER CODE BEGIN Header_StartTaskcommand */
/**
* @brief Function implementing the Taskcommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskcommand */
__weak void StartTaskcommand(void *argument)
{
  /* USER CODE BEGIN StartTaskcommand */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskcommand */
}

/* USER CODE BEGIN Header_StartTask_dji */
/**
* @brief Function implementing the Task_dji thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_dji */
__weak void StartTask_dji(void *argument)
{
  /* USER CODE BEGIN StartTask_dji */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_dji */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

