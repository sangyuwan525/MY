//
// Created by 马皓然 on 2025/11/20.
//

#ifndef R1_CHASSIS_TASK_COMMAND_H
#define R1_CHASSIS_TASK_COMMAND_H

#include "main.h"
#include <string.h>
extern uint8_t readBuffer[10];

uint8_t Command_Write(uint8_t *data, uint8_t length);

uint8_t Command_GetCommand(uint8_t *command);

#endif //R1_CHASSIS_TASK_COMMAND_H