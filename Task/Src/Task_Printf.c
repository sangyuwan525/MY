//
// Created by 马皓然 on 2025/11/6.
//

#include <stdio.h>
#include "cmsis_os.h"
void StartTask_Printf(void *argument)
{
    /* USER CODE BEGIN StartTask_Printf */
    /* Infinite loop */
    for(;;)
    {
        printf("hello");
        osDelay(500);
    }
    /* USER CODE END StartTask_Printf */
}