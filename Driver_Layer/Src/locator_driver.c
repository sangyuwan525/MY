#include "locator_driver.h"

#include <string.h>

#define LOCATOR_CAN_ID_POSE 0x12U
#define LOCATOR_CAN_ID_LASER_1 0x100U
#define LOCATOR_CAN_ID_LASER_2 0x101U

Locator_Result_t lcResult = {0};

static float Locator_ReadFloatLE(const uint8_t *data)
{
    uint32_t raw;
    float value;

    raw = ((uint32_t)data[0]) |
          ((uint32_t)data[1] << 8) |
          ((uint32_t)data[2] << 16) |
          ((uint32_t)data[3] << 24);
    memcpy(&value, &raw, sizeof(value));
    return value;
}

void analysis_locator_X_Y(Locator_Result_t *lcResult, const Locator_Rx_Queue_t *rx_msg_tmp)
{
    if (lcResult == NULL || rx_msg_tmp == NULL) {
        return;
    }

    if (rx_msg_tmp->msg_identifier != LOCATOR_CAN_ID_POSE || rx_msg_tmp->data_len < 16U) {
        return;
    }

    lcResult->x = 1000.0f * Locator_ReadFloatLE(&rx_msg_tmp->rx_data[0]);
    lcResult->y = 1000.0f * Locator_ReadFloatLE(&rx_msg_tmp->rx_data[4]);
    lcResult->r = Locator_ReadFloatLE(&rx_msg_tmp->rx_data[12]);
}

void analysis_locator_laser(Locator_Result_t *lcResult, const Locator_Rx_Queue_t *rx_msg_tmp)
{
    float laser_current;

    if (lcResult == NULL || rx_msg_tmp == NULL) {
        return;
    }

    if (rx_msg_tmp->data_len < 4U) {
        return;
    }

    laser_current = Locator_ReadFloatLE(&rx_msg_tmp->rx_data[0]);

    if (rx_msg_tmp->msg_identifier == LOCATOR_CAN_ID_LASER_1) {
        lcResult->laser_current = laser_current;
        lcResult->laser_current_1 = laser_current;
    } else if (rx_msg_tmp->msg_identifier == LOCATOR_CAN_ID_LASER_2) {
        lcResult->laser_current_2 = laser_current;
    }
}
