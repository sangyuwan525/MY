//
// Created by 马皓然 on 2025/11/20.
//
#include "Task_command.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "remote_driver.h"
#include "usart.h"


// 指令长度
#define COMMAND_LENGTH 10
// 循环缓冲区大小
#define BUFFER_SIZE 128
// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
uint8_t readIndex = 0;
// 循环缓冲区写索引
uint8_t writeIndex = 0;
// 存放指令的数组
uint8_t command[20];



//串口空闲中断接收数组
uint8_t remote_Buffer[10];

/**
* @brief 增加读索引
* @param length 要增加的长度
*/
static void Command_AddReadIndex(uint8_t length) {
    readIndex += length;
    readIndex %= BUFFER_SIZE;
}

/**
* @brief 读取第i位数据 超过缓存区长度自动循环
* @param i 要读取的数据索引
*/

static uint8_t Command_Read(uint8_t i) {
    uint8_t index = i % BUFFER_SIZE;
    return buffer[index];
}

/**
* @brief 计算未处理的数据长度
* @return 未处理的数据长度
* @retval 0 缓冲区为空
* @retval 1~BUFFER_SIZE-1 未处理的数据长度
* @retval BUFFER_SIZE 缓冲区已满
*/
//uint8_t Command_GetLength() {
//  // 读索引等于写索引时，缓冲区为空
//  if (readIndex == writeIndex) {
//    return 0;
//  }
//  // 如果缓冲区已满,返回BUFFER_SIZE
//  if (writeIndex + 1 == readIndex || (writeIndex == BUFFER_SIZE - 1 && readIndex == 0)) {
//    return BUFFER_SIZE;
//  }
//  // 如果缓冲区未满,返回未处理的数据长度
//  if (readIndex <= writeIndex) {
//    return writeIndex - readIndex;
//  } else {
//    return BUFFER_SIZE - readIndex + writeIndex;
//  }
//}

static uint8_t Command_GetLength() {
    return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
}


/**
* @brief 计算缓冲区剩余空间
* @return 剩余空间
* @retval 0 缓冲区已满
* @retval 1~BUFFER_SIZE-1 剩余空间
* @retval BUFFER_SIZE 缓冲区为空
*/
uint8_t Command_GetRemain() {
    return BUFFER_SIZE - Command_GetLength();
}

/**
* @brief 向缓冲区写入数据
* @param data 要写入的数据指针
* @param length 要写入的数据长度
* @return 写入的数据长度
*/
uint8_t Command_Write(uint8_t *data, uint8_t length) {
    // 如果缓冲区不足 则不写入数据 返回0
    if (Command_GetRemain() < length) {
        return 0;
    }
    // 使用memcpy函数将数据写入缓冲区
    if (writeIndex + length < BUFFER_SIZE) {
        memcpy(buffer + writeIndex, data, length);
        writeIndex += length;
    } else {
        uint8_t firstLength = BUFFER_SIZE - writeIndex;
        memcpy(buffer + writeIndex, data, firstLength);
        memcpy(buffer, data + firstLength, length - firstLength);
        writeIndex = length - firstLength;
    }
    return length;
}

/**
* @brief 尝试获取一条指令
* @param command 指令存放指针
* @return 获取的指令长度
* @retval 0 没有获取到指令
*/
uint8_t Command_GetCommand(uint8_t *command) {
    // 寻找完整指令
    while (1) {
        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (Command_GetLength() < 4) {
        return 0;
        }
        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(readIndex) != 0xAA) {
        Command_AddReadIndex(1);
        continue;
        }
        // 如果校验和不正确 则跳过 重新开始寻找
        uint8_t sum = 0;
        for (uint8_t i = 0; i < COMMAND_LENGTH - 1; i++) {
        sum += Command_Read(readIndex + i);
        }
        if (sum != Command_Read(readIndex + COMMAND_LENGTH - 1)) {
        Command_AddReadIndex(1);
        continue;
        }
        // 如果找到完整指令 则将指令写入command 返回指令长度
        for (uint8_t i = 0; i < COMMAND_LENGTH; i++) {
        command[i] = Command_Read(readIndex + i);
        }
        Command_AddReadIndex(COMMAND_LENGTH);
        return 1;
    }
}
/*---------------------------------------------------------------------------*/
/* USER CODE BEGIN Header_StartTaskcommand */
/**  * @brief  Function implementing the Taskcommand thread.
  * @param  argument: Not used
  * @retval None
  */


void StartTaskcommand(void *argument)
{
    /* USER CODE BEGIN StartTaskcommand */
    /* Infinite loop */
    for(;;)
    {
        if (Command_GetCommand(command) != 0) {
            // 处理指令内容，目前还没写
            printf("Command Yes\n");
            code_unzipread(command);
        }
        osDelay(10);
    }
    /* USER CODE END StartTaskcommand */
}


/* USER CODE END Header_StartTaskcommand */


// 串口接收完成回调函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART5) {
        // 将接收到的数据写入缓冲区
        Command_Write(remote_Buffer, Size);
        // 重新开启串口空闲中断接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_Buffer, sizeof(remote_Buffer));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}