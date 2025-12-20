//
// Created by 马皓然 on 2025/11/6.
//
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "Task_LED.h"
void StartTask_LED(void *argument)
{
    /* USER CODE BEGIN StartTask_LED */
    /* Infinite loop */
    for(;;)
    {
        // HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
        // HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);
        // HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);
        // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9);
        // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
        // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
        // HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
        // HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
        //printf("now_status:%d")
        osDelay(500);
    }
    /* USER CODE END StartTask_LED */
}