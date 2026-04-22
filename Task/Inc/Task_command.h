//
// Created by 马皓然 on 2025/11/20.
//

#ifndef R1_CHASSIS_TASK_COMMAND_H
#define R1_CHASSIS_TASK_COMMAND_H

#include "main.h"
#include <string.h>

#include "cmsis_os2.h"
#define COMMAND_LENGTH 10// 指令长度
extern uint8_t remote_Buffer[10];
extern osMessageQueueId_t remote_queueHandle;
/* Structs -------------------------------------------------------------------*/
// 用于在中断和任务之间安全传递数据的结构体
typedef struct {
    uint8_t data[COMMAND_LENGTH];
    uint16_t size;
} UartRxMessage_t;

uint8_t Command_Write(uint8_t *data, uint8_t length);

uint8_t Command_GetCommand(uint8_t *command);

#endif //R1_CHASSIS_TASK_COMMAND_H
