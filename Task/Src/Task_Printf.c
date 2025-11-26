//
// Created by 马皓然 on 2025/11/6.
//

#include <stdio.h>
#include "cmsis_os.h"
#include "Task_Printf.h"
void StartTask_Printf(void *argument)
{
    /* USER CODE BEGIN StartTask_Printf */
    /* Infinite loop */
    for(;;)
    {
        printf("x=%d y=%d cir=%d sw1=%d sw2=%d button1=%d\r\n",rc.ch1,rc.ch2,rc.cir,rc.sw1,rc.sw2,rc.button1);
        osDelay(50);
    }
    /* USER CODE END StartTask_Printf */
}