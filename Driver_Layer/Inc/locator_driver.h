#ifndef R1_CHASSIS_LOCATOR_DRIVER_H
#define R1_CHASSIS_LOCATOR_DRIVER_H

#include <stdint.h>
#include "cmsis_os2.h"

typedef struct LocatorResult {
    float x, y, r;
    float vx, vy, vr;
    float pitch, roll, yaw;
    float laser_current_1;
    float laser_current_2;
} Locator_Result_t;

typedef struct LocatorRxQueue {
    uint32_t msg_identifier;
    uint8_t data_len;
    uint8_t rx_data[16];
} Locator_Rx_Queue_t;

/* Legacy queue names: x_y now carries 0x12 x/y/yaw; z_r now carries 0x100/0x101 laser data. */
extern osMessageQueueId_t locatorQueue_x_yHandle;
extern osMessageQueueId_t locatorQueue_z_rHandle;
extern Locator_Result_t lcResult;

void analysis_locator_X_Y(Locator_Result_t *lcResult, const Locator_Rx_Queue_t *rx_msg_tmp);
void analysis_locator_laser(Locator_Result_t *lcResult, const Locator_Rx_Queue_t *rx_msg_tmp);

#endif
