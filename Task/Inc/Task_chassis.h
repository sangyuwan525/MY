//
// Created by 马皓然 on 2025/11/27.
//

#ifndef R1_CHASSIS_TASK_CHASSIS_H
#define R1_CHASSIS_TASK_CHASSIS_H
#include <stdio.h>
#include "cmsis_os2.h"
/* Definitions ---------------------------------------------------------------*/
// 任务循环延时时间，例如 10ms
#define CHASSIS_TASK_PERIOD 10

/* Structs -------------------------------------------------------------------*/

/* Global Variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
/**************外部接口begin**************/
void Chassis_Task(void *argument);
/**************外部接口end**************/
#endif //R1_CHASSIS_TASK_CHASSIS_H