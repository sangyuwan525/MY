//
// Created by 91818 on 2025/11/30.
//
#include "bsp_can.h"
#include "string.h"
//#include "basic.h"
#include "cmsis_os.h"
#include "SEGGER_RTT.h"

//模块内部的静态变量，用来存句柄
static osMessageQueueId_t g_can_rx_queue = NULL;
static osMessageQueueId_t g_motor_queue = NULL;
static osMessageQueueId_t g_chassis_queue = NULL;


// --- 内部函数：配置过滤器 ---
/**
 * @brief 配置 FDCAN 接收过滤器
 * @param hfdcan
 * @param fifo_assignment 挂载在哪个FIFO上：FDCAN_FILTER_TO_RXFIFO0 或 FDCAN_FILTER_TO_RXFIFO1
 * @return void
 */
static void FDCAN_Filter_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo_assignment, CAN_Id_Type_e id_type) {
	FDCAN_FilterTypeDef sFilterConfig;

	// 配置接收过滤器：接收所有 ID (0x00000000 - 0x1FFFFFFF)
    if (id_type)
    {
        sFilterConfig.IdType       = FDCAN_EXTENDED_ID;
    }
    else
    {
        sFilterConfig.IdType       = FDCAN_STANDARD_ID;
    }
	sFilterConfig.FilterIndex      = 0;
	sFilterConfig.FilterType       = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig     = fifo_assignment; // FDCAN_FILTER_TO_RXFIFO0 或 FIFO1
	sFilterConfig.FilterID1        = 0x00000000;
	sFilterConfig.FilterID2        = 0x1FFFFFFF;

	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK) {
		// Error_Handler(); // 建议加上你的错误处理
	}

	// 配置全局过滤器：拒绝不匹配的(其实上面已经匹配所有了)，远程帧也拒绝
	if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
									 fifo_assignment == FDCAN_FILTER_TO_RXFIFO0 ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_ACCEPT_IN_RX_FIFO1,
									 fifo_assignment == FDCAN_FILTER_TO_RXFIFO0 ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_ACCEPT_IN_RX_FIFO1,
									 FDCAN_FILTER_REJECT,
									 FDCAN_FILTER_REJECT) != HAL_OK) {
		// Error_Handler();
									 }
    if(HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        // Error_Handler();
    }
}

/**
 * @brief 启动指定的 FDCAN 接口
 * @param hfdcan 指向 FDCAN 句柄
 * @return void
 * @note FDCAN1 用 FIFO0, 其他用 FIFO1
 */
void bsp_can_start(FDCAN_HandleTypeDef *hfdcan) {

    uint32_t fifo = (hfdcan == &hfdcan1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    uint32_t it_flag = (hfdcan == &hfdcan1) ? FDCAN_IT_RX_FIFO0_NEW_MESSAGE : FDCAN_IT_RX_FIFO1_NEW_MESSAGE;

    FDCAN_Filter_Config(hfdcan, fifo, CAN_ID_STD);
    HAL_FDCAN_ActivateNotification(hfdcan, it_flag, 0);
    HAL_FDCAN_Start(hfdcan);
}

// --- 外部接口：初始化所有 CAN ---
/**
 * @brief 初始化所有 CAN 接口
 * @param queue_handle 消息队列句柄，用于接收 CAN 消息,在上层task调用时传入
 * @return void
 * @note 配置过滤器，激活中断，启动 CAN
 */
void bsp_can_init(osMessageQueueId_t motor_q, osMessageQueueId_t chassis_q) {

    g_motor_queue = motor_q;
    g_chassis_queue = chassis_q;
    // 1. 配置 CAN1 -> FIFO 0
    bsp_can_start(&hfdcan1);//给电机

    // 2. 配置 CAN2 -> FIFO 1
    bsp_can_start(&hfdcan2);//给电机

    // 3. 配置 CAN3 -> FIFO 1
    bsp_can_start(&hfdcan3);
}

// --- 发送标准帧 (Classic CAN) ---
// 适用于 DJI 电机 (3508/2006/6020)
/**
 * @brief 发送标准帧 (Classic CAN)
 * @param hfdcan 指向 FDCAN 句柄
 * @param id 帧 ID
 * @param Txdata 指向数据缓冲区
 * @param len 数据长度 (最大 8 字节)
 * @param EXTflag = 1 使用扩展帧，= 0 使用标准帧
 * @return 0 成功，1 失败
 */
uint8_t bsp_can_send_std_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *Txdata, uint8_t len, CAN_Id_Type_e id_type) {

    FDCAN_TxHeaderTypeDef TxMessage;

    TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

    if(id_type)
        TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
    else
        TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */

    TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
    TxMessage.DataLength 			= len;					/* 设置数据长度 */
    TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
    TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
    TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;	/* FDCAN格式 */
    TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
    TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */


    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, Txdata) != HAL_OK) {
        return 1; // 发送满或出错
    }
    return 0;
}

/**
 * @brief FDCAN 错误状态回调函数
 * @param hfdcan 指向 FDCAN 句柄
 * @param ErrorStatusITs 错误状态中断标志
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    //__HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
    if(hfdcan->Instance == FDCAN1)
    {
        MX_FDCAN1_Init();
    }
    else if(hfdcan->Instance == FDCAN2)
    {
        MX_FDCAN2_Init();
    }
    else if(hfdcan->Instance == FDCAN3)
    {
        MX_FDCAN3_Init();
    }
    else
    {

    }
}

// --- 发送 FDCAN 帧 ---(应该是接收，要改)
// 适用于 底盘通信 (如果底盘用了 CAN FD 加速)
/**
 * @brief 发送 FDCAN 帧 (CAN FD)
 * @param hfdcan 指向 FDCAN 句柄
 * @param id 帧 ID
 * @param Txdata 指向数据缓冲区
 * @param len 数据长度 (最大 64 字节)
 * @return 0 成功，1 失败
 */
uint8_t bsp_can_send_fd_msg(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *Txdata, uint8_t len, CAN_Id_Type_e id_type) {

    FDCAN_TxHeaderTypeDef TxMessage;

    TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

    if(id_type)
        TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
    else
        TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */

    TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
    TxMessage.DataLength 			= len;					/* 设置数据长度 */
    TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
    TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
    TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;	/* FDCAN格式 */
    TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
    TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */


    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessage, Txdata) != HAL_OK) {
        return 1;
    }
    return 0;
}

// --- 内部工具：将 FDCAN 的 DLC 枚举转换为实际字节长度 ---
/**
 * @brief 将 FDCAN 的 DLC 枚举转换为实际字节长度
 * @param dlc FDCAN_DLC_BYTES_xxx 枚举值
 * @return 实际字节长度
 */
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
    default: return 8; // 默认防错
    }
}

// --- 通用接收处理函数 ---
/**
 * @brief 处理接收到的 FDCAN 消息
 * @param hfdcan 指向 FDCAN 句柄
 * @param fifo 指定从哪个 FIFO 读取 (FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1)
 */
static void Process_Rx_Message(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64]; // 临时缓冲区，最大支持64字节
    can_msg_t msg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 循环读取，直到 FIFO 为空 (防止高频数据堆积)
    while (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &RxHeader, RxData) == HAL_OK)
    {
        // 1. 记录基本信息
        msg.id = RxHeader.Identifier;
        msg.hfdcan = hfdcan; // 关键！记录了数据是从哪个 CAN 口来的

        // 2. 转换并记录长度 (实现 FDCAN 底盘长数据支持)
        msg.len = FDCAN_DlcToBytes(RxHeader.DataLength);

        // 3. 拷贝数据 (只拷贝有效长度，效率更高)
        // 注意：这里用了 memcpy，需包含 string.h
        // 如果数据量小，也可以用 for 循环
        if (msg.len > 0) {
            memcpy(msg.data, RxData, msg.len);
        }


        // 4. 推送至队列
        // 如果队列满了，直接丢弃新数据(0 delay)，保证中断绝对不卡死
        // --- 关键修改开始 ---

        if ((msg.id >= 0x201 && msg.id <= 0x208)||(msg.id == 0x0)) {//dm 电机反馈报文的id号

            if (g_motor_queue != NULL) {
                xQueueSendFromISR(g_motor_queue, &msg, &xHigherPriorityTaskWoken);
            }
        }
        else
        	{
            if (g_chassis_queue != NULL) {
                xQueueSendFromISR(g_chassis_queue, &msg, &xHigherPriorityTaskWoken);
            }
        }
    }

    // 如果唤醒了高优先级任务（如 Dispatch 任务），进行上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// FIFO0 回调 (通常给 CAN1)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        Process_Rx_Message(hfdcan, FDCAN_RX_FIFO0);
    }
}

// FIFO1 回调 (通常给 CAN2, CAN3)
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
        Process_Rx_Message(hfdcan, FDCAN_RX_FIFO1);

    }
}

/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;

	if(len<=8)
		pTxHeader.DataLength = len;
	else if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	else if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	else if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	else if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	else if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	else if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	else if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;

    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_OFF;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;

	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK)
		return 1;//发送
	return 0;
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;

	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_12)
			len = 12;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16)
			len = 16;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20)
			len = 20;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24)
			len = 24;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32)
			len = 32;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48)
			len = 48;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64)
			len = 64;

		return len;//接收数据
	}
	return 0;
}