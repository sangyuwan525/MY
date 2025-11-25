//
// Created by 马皓然 on 2025/11/20.
//
#include "Task_command.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "remote_driver.h"
#include "usart.h"

/* Definitions ---------------------------------------------------------------*/
#define COMMAND_LENGTH 10// 指令长度
#define BUFFER_SIZE 128// 循环缓冲区大小
#define COMMAND_HEADER 0x61//数据帧帧头
#define CRC_DATA_LENGTH (COMMAND_LENGTH - 2) // 参与CRC校验的数据长度
/* Structs -------------------------------------------------------------------*/
// 16位CRC循环校验码表，多项式 \text{0x1021}、初始值 \text{0xFFFF} 且高位优先的
const uint16_t CRC_16_Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

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
 * @brief 使用查表法计算 CRC-16
 * @param data 要计算的数据指针
 * @param length 数据字节数
 * @return 计算得到的 CRC-16 校验码
 */
uint16_t CRC_16_TableDriven(const uint8_t *data, uint8_t length) {
    // 初始值 0xFFFF
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < length; i++) {
        uint8_t current_byte = data[i];

        // 1. 计算索引：(CRC高8位) 异或 (当前数据字节)
        uint8_t index = (uint8_t)(crc >> 8) ^ current_byte;

        // 2. 更新 CRC：(CRC低8位左移8位) 异或 (查表结果)
        crc = (crc << 8) ^ CRC_16_Table[index];
    }

    return crc;
}

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
        if (Command_GetLength() < COMMAND_LENGTH) {
        return 0;
        }
        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(readIndex) != COMMAND_HEADER) {
        Command_AddReadIndex(1);
        continue;
        }
        // 计算CRC校验
        uint8_t crc_data[CRC_DATA_LENGTH];
        for (uint8_t i = 0; i < CRC_DATA_LENGTH; i++) {
            crc_data[i] = Command_Read(readIndex + i);
        }

        uint16_t calculated_crc = CRC_16_TableDriven(crc_data, CRC_DATA_LENGTH);
        uint16_t received_crc = ((uint16_t)Command_Read(readIndex + COMMAND_LENGTH - 2) << 8) |
                                ((uint16_t)Command_Read(readIndex + COMMAND_LENGTH - 1));

        if (calculated_crc != received_crc) {
            Command_AddReadIndex(1); // 跳过一个字节继续寻找包头
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
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,remote_Buffer,sizeof(remote_Buffer));
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

//串口错误处理函数
void HAL_UART_ErrorCallback( UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef ret=HAL_ERROR;
    huart->RxState = HAL_UART_STATE_READY;
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_PE | UART_FLAG_FE | UART_FLAG_ORE | UART_FLAG_NE);
    if (huart == &huart5){
        ret=HAL_UARTEx_ReceiveToIdle_DMA(&huart5,remote_Buffer,sizeof(remote_Buffer));
        if(ret!=HAL_OK){
            printf("ErrorCB Uart4 IT Enable Failed:%d",ret);
        }
    }
}