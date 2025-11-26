//
// Created by 马皓然 on 2025/11/6.
//

#ifndef R1_CHASSIS_REMOTE_DRIVER_H
#define R1_CHASSIS_REMOTE_DRIVER_H
#include <stdint.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
/*-- Remote control data structure --*/
typedef struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;

    uint8_t sw1;
    uint8_t sw2;
    uint8_t sw3;
    uint8_t sw4;
    uint8_t sw5;

    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
    uint8_t button4;
    uint8_t button5;
    uint8_t button6;

    int16_t cir;
} rc_info_t;

// 遥控器数据工程量结构体
typedef struct {
    float vx;       // 底盘X方向速度 (m/s)，归一化处理后的摇杆数据
    float vy;       // 底盘Y方向速度 (m/s)，归一化处理后的摇杆数据
    float vw;       // 底盘旋转角速度 (rad/s)，归一化处理后的旋钮数据
    uint8_t mode;   // 机器人工作模式（由开关状态 sw1, sw2 等组合决定）
    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
    uint8_t button4;
    uint8_t button5;
    uint8_t button6;
} remote_engineer_t;


extern rc_info_t rc;
extern remote_engineer_t remote_engineer;
extern osMutexId_t rc_mutexHandle;
/*-- Remote control data unpacking function --*/
void code_unzipread(uint8_t *code);
void Remote_Data_Convert(const rc_info_t *rc_data, remote_engineer_t *engineer_data);
//以上两个函数仅用于内部调用
BaseType_t Remote_GetEngineerData(remote_engineer_t *engineer_data);
//获取遥控器工程量数据，线程安全
#endif //R1_CHASSIS_REMOTE_DRIVER_H