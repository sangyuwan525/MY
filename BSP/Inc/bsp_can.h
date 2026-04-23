#ifndef R1_SUPERSTRUCTURE_BSP_CAN_H
#define R1_SUPERSTRUCTURE_BSP_CAN_H

#include "fdcan.h"
#include "stm32g4xx.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "queue.h"

#define hcan_t FDCAN_HandleTypeDef

typedef struct {
    uint32_t id;
    uint32_t id_type;
    uint8_t len;
    uint8_t data[64];
    FDCAN_HandleTypeDef *hfdcan;
} can_msg_t;

typedef can_msg_t Motor_Rx_Queue_t;

typedef enum {
    CAN_ID_STD = 0,
    CAN_ID_EXT = 1
} CAN_Id_Type_e;

void bsp_can_init(osMessageQueueId_t motor_q, osMessageQueueId_t chassis_q);

uint8_t bsp_can_send_std_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *Txdata, uint8_t len, CAN_Id_Type_e id_type);
uint8_t bsp_can_rev_fd_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len, CAN_Id_Type_e id_type);

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_send_ex_data(hcan_t *hfdcan, uint32_t id, uint8_t *data, uint32_t len, CAN_Id_Type_e id_type);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);

#endif /* R1_SUPERSTRUCTURE_BSP_CAN_H */
