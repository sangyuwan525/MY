#ifndef R1_CHASSIS_REMOTE_DRIVER_H
#define R1_CHASSIS_REMOTE_DRIVER_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include <tgmath.h>

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

typedef enum {
    CHASSIS_MODE_STANDBY = 0,
    CHASSIS_MODE_AUTO    = 1,
    CHASSIS_MODE_MANUAL  = 2,
    CHASSIS_MODE_TEST    = 3
} chassis_mode_e;

typedef enum {
    CLIMB_MODE = 0,
    DOWN_MODE  = 1,
    UP_MODE
} test_mode_e;

typedef struct {
    float vx;       // chassis X speed (mm/s)
    float vy;       // chassis Y speed (mm/s)
    float vw;       // chassis yaw speed (rpm)
    chassis_mode_e mode;
    test_mode_e test_mode;
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

void code_unzipread(uint8_t *code);
void Remote_Data_Convert(const rc_info_t *rc_data, remote_engineer_t *engineer_data);
BaseType_t Remote_GetEngineerData(remote_engineer_t *engineer_data);

#endif // R1_CHASSIS_REMOTE_DRIVER_H
