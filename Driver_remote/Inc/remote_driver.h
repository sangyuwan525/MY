//
// Created by 马皓然 on 2025/11/6.
//

#ifndef R1_CHASSIS_REMOTE_DRIVER_H
#define R1_CHASSIS_REMOTE_DRIVER_H
#include <stdint.h>
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
extern rc_info_t rc;
#endif //R1_CHASSIS_REMOTE_DRIVER_H