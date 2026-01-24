//
// Created by 马皓然 on 2025/11/5.
//

#ifndef R1_CHASSIS_LOCATOR_DRIVER_H
#define R1_CHASSIS_LOCATOR_DRIVER_H
#include <stdint.h>

#include "cmsis_os2.h"
/**
 * @brief 激光雷达定位结果结构体
 */
typedef struct LocatorResult
{
    float x, y, r;   ///< 位置和朝向 (世界坐标系)
    float vx, vy, vr; ///< 速度分量
    float pitch, roll,yaw; ///< 俯仰角、横滚角、偏航角
} Locator_Result_t;

typedef struct LocatorRxQueue {
    uint32_t msg_identifier;
    uint8_t rx_data[8];
}Locator_Rx_Queue_t;

extern osMessageQueueId_t  locatorQueue_x_yHandle;//用于存放原始数据的X和Y的队列
extern osMessageQueueId_t  locatorQueue_z_rHandle;//用于存放原始数据的Z和yaw的队列
extern Locator_Result_t lcResult;
void analysis_locator_X_Y(Locator_Result_t* lcResult, const Locator_Rx_Queue_t* rx_msg_tmp);
void analysis_locator_Z_R(Locator_Result_t* lcResult, const Locator_Rx_Queue_t* rx_msg_tmp);
#endif //R1_CHASSIS_LOCATOR_DRIVER_H