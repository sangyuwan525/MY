//
// Created by 91818 on 2025/11/30.
//

#ifndef R1_SUPERSTRUCTURE_BSP_CAN_H
#define R1_SUPERSTRUCTURE_BSP_CAN_H

#include "fdcan.h"
#include "stm32g4xx.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "queue.h"
#define hcan_t FDCAN_HandleTypeDef
// FDCAN 最大负载是 64 字节
typedef struct {
    uint32_t id;       // 电机id
    uint8_t  len;      // 数据长度
    uint8_t  data[64]; // 支持 FDCAN 长帧
    FDCAN_HandleTypeDef *hfdcan; // 标记数据来自哪个CAN口(可选，方便调试)
} can_msg_t;

typedef can_msg_t Motor_Rx_Queue_t;

// FDCAN 类型
typedef enum
{
    CAN_ID_STD = 0,
    CAN_ID_EXT = 1
} CAN_Id_Type_e;

// 外部引用的队列句柄
extern osMessageQueueId_t motor_rx_queueHandle;

// 初始化：配置过滤器并启动
void bsp_can_init(osMessageQueueId_t motor_q, osMessageQueueId_t chassis_q);

// 发送标准帧 (针对 DJI 电机)
uint8_t bsp_can_send_std_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *Txdata, uint8_t len, CAN_Id_Type_e id_type);

// 接受 FDCAN 帧 (针对底盘，如果需要)
uint8_t bsp_can_rev_fd_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len, CAN_Id_Type_e id_type);


uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
#endif //R1_SUPERSTRUCTURE_BSP_CAN_H
