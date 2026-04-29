#include "bsp_can.h"

#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"
#include "locator_driver.h"

#define BLAZER_FOC_NODE_ID_MAX 0x07U
#define BLAZER_FOC_PARAM_ID_MAX 0x47U
#define LOCATOR_CAN_ID_X_Y 0x12U
#define LOCATOR_CAN_ID_Z_R 0x13U
#define LOCATOR_CAN_ID_LASER 0x100U

static osMessageQueueId_t g_motor_queue = NULL;
static osMessageQueueId_t g_chassis_queue = NULL;

static bool Is_Locator_Rx_Message(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *rx_header) {
    if (hfdcan != &hfdcan3 || rx_header == NULL || rx_header->IdType != FDCAN_STANDARD_ID) {
        return false;
    }

    return (rx_header->Identifier == LOCATOR_CAN_ID_X_Y ||
            rx_header->Identifier == LOCATOR_CAN_ID_Z_R ||
            rx_header->Identifier == LOCATOR_CAN_ID_LASER);
}

static void FDCAN_Filter_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_assignment, CAN_Id_Type_e id_type) {
    (void)id_type;
    (void)HAL_FDCAN_ConfigGlobalFilter(
        hfdcan,
        fifo_assignment == FDCAN_FILTER_TO_RXFIFO0 ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_ACCEPT_IN_RX_FIFO1,
        fifo_assignment == FDCAN_FILTER_TO_RXFIFO0 ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_ACCEPT_IN_RX_FIFO1,
        FDCAN_FILTER_REJECT,
        FDCAN_FILTER_REJECT);
    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0);
}

static uint8_t FDCAN_DlcToBytes(uint32_t dlc) {
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 8;
    }
}

static bool Is_Motor_Rx_Message(const FDCAN_RxHeaderTypeDef *rx_header) {
    uint8_t comm_type;
    uint8_t blazer_node_id;
    uint8_t blazer_param_id;

    if (rx_header == NULL) {
        return false;
    }

    if (rx_header->IdType == FDCAN_STANDARD_ID) {
        return ((rx_header->Identifier >= 0x201U && rx_header->Identifier <= 0x208U) ||
                (rx_header->Identifier == 0x000U));
    }

    comm_type = (uint8_t)((rx_header->Identifier >> 24) & 0x1FU);
    if (comm_type == 0x02U || comm_type == 0x15U) {
        return true;
    }

    /*
     * Blazer FOC uses extended frames with:
     *   ID = node_id << 8 | param_id
     * Read replies use odd param_id values, and valid node IDs are 0..7.
     * Keep this narrow so unrelated extended-frame chassis messages are not
     * accidentally routed into the motor feedback queue.
     */
    blazer_node_id = (uint8_t)((rx_header->Identifier >> 8) & 0xFFU);
    blazer_param_id = (uint8_t)(rx_header->Identifier & 0xFFU);
    return ((rx_header->Identifier & 0xFFFF0000U) == 0U &&
            blazer_node_id <= BLAZER_FOC_NODE_ID_MAX &&
            blazer_param_id <= BLAZER_FOC_PARAM_ID_MAX &&
            (blazer_param_id & 0x01U) != 0U);
}

static uint8_t fdcanx_send_impl(hcan_t *hfdcan, uint32_t id, uint8_t *data, uint32_t len, CAN_Id_Type_e id_type) {
    FDCAN_TxHeaderTypeDef tx_header = {0};

    tx_header.Identifier = id;
    tx_header.IdType = (id_type == CAN_ID_EXT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;

    if (len <= 8U) {
        tx_header.DataLength = len;
    } else if (len == 12U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_12;
    } else if (len == 16U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_16;
    } else if (len == 20U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_20;
    } else if (len == 24U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_24;
    } else if (len == 32U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_32;
    } else if (len == 48U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_48;
    } else if (len == 64U) {
        tx_header.DataLength = FDCAN_DLC_BYTES_64;
    } else {
        return 1;
    }

    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = (len <= 8U) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data) != HAL_OK) {
        return 1;
    }
    return 0;
}

static void Process_Rx_Message(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
    can_msg_t msg;
    Locator_Rx_Queue_t locator_msg;
    uint8_t locator_len;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    bool is_motor_msg;

    while (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data) == HAL_OK) {
        if (Is_Locator_Rx_Message(hfdcan, &rx_header)) {
            memset(&locator_msg, 0, sizeof(locator_msg));
            locator_msg.msg_identifier = rx_header.Identifier;
            locator_len = FDCAN_DlcToBytes(rx_header.DataLength);
            if (locator_len > sizeof(locator_msg.rx_data)) {
                locator_len = sizeof(locator_msg.rx_data);
            }
            if (locator_len > 0U) {
                memcpy(locator_msg.rx_data, rx_data, locator_len);
            }

            if (rx_header.Identifier == LOCATOR_CAN_ID_X_Y && locatorQueue_x_yHandle != NULL) {
                xQueueSendFromISR(locatorQueue_x_yHandle, &locator_msg, &xHigherPriorityTaskWoken);
            } else if ((rx_header.Identifier == LOCATOR_CAN_ID_Z_R ||
                        rx_header.Identifier == LOCATOR_CAN_ID_LASER) &&
                       locatorQueue_z_rHandle != NULL) {
                xQueueSendFromISR(locatorQueue_z_rHandle, &locator_msg, &xHigherPriorityTaskWoken);
            }
            continue;
        }

        memset(&msg, 0, sizeof(msg));
        msg.id = rx_header.Identifier;
        msg.id_type = rx_header.IdType;
        msg.hfdcan = hfdcan;
        msg.len = FDCAN_DlcToBytes(rx_header.DataLength);

        if (msg.len > 0U) {
            memcpy(msg.data, rx_data, msg.len);
        }

        is_motor_msg = Is_Motor_Rx_Message(&rx_header);
        if (is_motor_msg) {
            if (g_motor_queue != NULL) {
                xQueueSendFromISR(g_motor_queue, &msg, &xHigherPriorityTaskWoken);
            }
        } else {
            if (g_chassis_queue != NULL) {
                xQueueSendFromISR(g_chassis_queue, &msg, &xHigherPriorityTaskWoken);
            }
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void bsp_can_start(FDCAN_HandleTypeDef *hfdcan) {
    uint32_t fifo = (hfdcan == &hfdcan1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;

    FDCAN_Filter_Config(hfdcan, fifo, CAN_ID_STD);
    HAL_FDCAN_ActivateNotification(hfdcan,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                   0);
    HAL_FDCAN_Start(hfdcan);
}

void bsp_can_init(osMessageQueueId_t motor_q, osMessageQueueId_t chassis_q) {
    g_motor_queue = motor_q;
    g_chassis_queue = chassis_q;

    bsp_can_start(&hfdcan1);
    bsp_can_start(&hfdcan2);
    bsp_can_start(&hfdcan3);
}

uint8_t bsp_can_send_std_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *Txdata, uint8_t len, CAN_Id_Type_e id_type) {
    FDCAN_TxHeaderTypeDef tx_header = {0};

    tx_header.Identifier = id;
    tx_header.IdType = (id_type == CAN_ID_EXT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = len;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Txdata) != HAL_OK) {
        return 1;
    }
    return 0;
}

uint8_t bsp_can_rev_fd_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len, CAN_Id_Type_e id_type) {
    return bsp_can_send_std_msg(hfdcan, id, data, len, id_type);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    (void)ErrorStatusITs;

    if (hfdcan->Instance == FDCAN1) {
        MX_FDCAN1_Init();
    } else if (hfdcan->Instance == FDCAN2) {
        MX_FDCAN2_Init();
    } else if (hfdcan->Instance == FDCAN3) {
        MX_FDCAN3_Init();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        Process_Rx_Message(hfdcan, FDCAN_RX_FIFO0);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        Process_Rx_Message(hfdcan, FDCAN_RX_FIFO1);
    }
}

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len) {
    return fdcanx_send_impl(hfdcan, id, data, len, CAN_ID_STD);
}

uint8_t fdcanx_send_ex_data(hcan_t *hfdcan, uint32_t id, uint8_t *data, uint32_t len, CAN_Id_Type_e id_type) {
    return fdcanx_send_impl(hfdcan, id, data, len, id_type);
}

uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t len;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, buf) == HAL_OK) {
        *rec_id = (uint16_t)rx_header.Identifier;
        len = FDCAN_DlcToBytes(rx_header.DataLength);
        return len;
    }
    return 0;
}
