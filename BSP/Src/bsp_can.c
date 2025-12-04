#include "bsp_can.h"
// #include "CanCommand.h"
// #include "control.h"
// #include "ak80_motor.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

uint8_t rece_can=0;
extern int jump_flag;
float distance_can_r1=0;
float distance_can_basket=0;
	
/**************内部宏定义与重命名begin**************/

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/

/**************内部变量与函数end**************/

/**************外部接口begin**************/
uint8_t can_data_num_g=0;//结构体数组can_database_g的大小，在Hash_table_init(void)中更新
Can_Data can_database_g[]={//通信表，odrive和vesc的依赖项。新加ID时，在ID_NUMDEF中定义相应ID的意义
		//Data_type						Data_ID						 *Data_ptr                                   					Data_length  			*MenuFunc			Channel			Fifo_num
		{WRITE_ONLY,      	vesc_motor1,         (uint8_t*)(&vesc_content_transform[1].u8_data),       4,          NULL,        3,		FDCAN_RX_FIFO0},
		{WRITE_ONLY,      	vesc_motor2,         (uint8_t*)(&vesc_content_transform[2].u8_data),       4,          NULL,        3,		FDCAN_RX_FIFO0},
    	{WRITE_ONLY,      	vesc_motor3,         (uint8_t*)(&vesc_content_transform[3].u8_data),       4,          NULL,        3,		FDCAN_RX_FIFO0}
};
uint16_t hash_table[1000]={999};
/**************外部接口end**************/
/**************FDCAN接收过滤器配置begin**************/
/*note FDCAN1使用FIFO0接收，FDCAN2使用FIFO1接收，FDCAN3使用FIFO0接收*/

void FDCAN1_RxFilter_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}

	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO0中断
	{
		Error_Handler();//进入硬件错误
	}

	HAL_FDCAN_Start(&hfdcan1);
}

void FDCAN2_RxFilter_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}

	if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO1中断
	{
		Error_Handler();//进入硬件错误
	}

	HAL_FDCAN_Start(&hfdcan2);
}

void FDCAN3_RxFilter_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}

	if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO0中断
	{
		Error_Handler();//进入硬件错误
	}

	HAL_FDCAN_Start(&hfdcan3);
}
/*
@brief FDCAN1发送函数
@param
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
uint8_t FDCAN1_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

	if(EXTflag)
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

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}

/*
@brief FDCAN2发送函数
@param
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
uint8_t FDCAN2_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

	if(EXTflag)
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

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}

/*
@brief FDCAN3发送函数
@param
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
uint8_t FDCAN3_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

	if(EXTflag)
		TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
	else
		TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */

	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= len;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;	/* FDCAN格式/传统CAN */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}
//错误重启函数
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

/*
1.函数功能：初始化通信hash表
2.入参：none
3.返回值：none
4.用法及调用要求：在使用vesc和odrive之前调用此函数
5.其它：
*/
void Hash_table_init(void){
	int i;
	can_data_num_g = sizeof(can_database_g) / sizeof(can_database_g[0]);
	for(i=0;i<1000;i++){
		hash_table[i] = 999;
	}
	for(i=0;i<can_data_num_g;i++){
		hash_table[can_database_g[i].Data_ID] = i;
	}
}

#if 1
// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)//dji电机
// {
// 	FDCAN_RxHeaderTypeDef rx_header;
// 	uint8_t rx_data[8];
// 	BaseType_t xHigherPriorityTaskWoken = pdFALSE; // 用于在中断退出时请求调度
// 	if(hfdcan==&hfdcan1)
// 	{
// 		//static int cnt[8]={0};
// 		// static uint32_t database;
// 		// HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
// 		while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
// 		{
// 			if(rx_header.Identifier>=CAN_3508_M1_ID&&rx_header.Identifier<=CAN_3508_M4_ID)
// 			{
// 				Motor_Rx_Queue_t rx_msg;
// 				rx_msg.motor_id = rx_header.Identifier;
// 				// 使用 memcpy 拷贝原始数据，中断中必须避免直接赋值大型结构体
// 				memcpy(rx_msg.rx_data, rx_data, 8);
// 				if (xQueueSendFromISR(motorRxQueueHandle, &rx_msg, &xHigherPriorityTaskWoken) != pdPASS)
// 				{
// 					// TODO: 队列已满，数据丢失。此处可添加日志记录或计数器。
// 				}
// 			}
// 		}
// 		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// 		// if(rx_header.Identifier>=CAN_3508_M5_ID&&rx_header.Identifier<=CAN_3508_M8_ID) {
// 		// 	switch (rx_header.Identifier){
// 		// 		 case CAN_3508_M5_ID:
// 		// 		 case CAN_3508_M6_ID:
// 		// 		 case CAN_3508_M7_ID:
// 		// 		 case CAN_3508_M8_ID: {
// 		// 			 Dji_3508_last_four_motor_control(rx_header.Identifier - CAN_3508_M1_ID, rx_data);
// 		// 			 break;
// 		// 		 }
// 		// 		default:
// 		// 			break;
// 		// 	 }
// 		// }
// 	}
//     if(hfdcan==&hfdcan3)
// 	{
// 		// static int cnt[8]={0};
//     	while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
//     	{
//     		if(rx_header.Identifier>=CAN_3508_M1_ID&&rx_header.Identifier<=CAN_3508_M4_ID)
//     		{
//     			Motor_Rx_Queue_t rx_msg;
//     			rx_msg.motor_id = rx_header.Identifier;
//     			// 使用 memcpy 拷贝原始数据，中断中必须避免直接赋值大型结构体
//     			memcpy(rx_msg.rx_data, rx_data, 8);
//     			if (xQueueSendFromISR(motorRxQueueHandle, &rx_msg, &xHigherPriorityTaskWoken) != pdPASS)
//     			{
//     				// TODO: 队列已满，数据丢失。此处可添加日志记录或计数器。
//     			}
//     		}
//     	}
//     	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
// 		// HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
// 		// if(rx_header.Identifier>=CAN_3508_M1_ID&&rx_header.Identifier<=CAN_3508_M4_ID){
// 		// 	switch (rx_header.Identifier) {
// 		// 		case CAN_3508_M1_ID:
// 		// 		case CAN_3508_M2_ID:
// 		// 		case CAN_3508_M3_ID:
// 		// 		case CAN_3508_M4_ID: {
// 		// 			Dji_3508_first_four_motor_control(rx_header.Identifier - CAN_3508_M1_ID, rx_data);
// 		// 			break;
// 		// 		}
// 		// 		default:
// 		// 			break;
// 		// 	}
// 		// }
// 	}
// }
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//if (hfdcan==&hfdcan3) printf("6");

	while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
	{
		// 确保 ID 在我们关注的电机ID范围内 (0x201 - 0x208)
		if(rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M8_ID)
		{
			Motor_Rx_Queue_t rx_msg;
			rx_msg.hcan = hfdcan; // 保存当前的 CAN 句柄
			rx_msg.motor_id = rx_header.Identifier;
			memcpy(rx_msg.rx_data, rx_data, 8);

			if (xQueueSendFromISR(motorRxQueueHandle, &rx_msg, &xHigherPriorityTaskWoken) != pdPASS)
			{
				// 队列已满
			}
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	// if(hfdcan==&hfdcan2)
	// {
	// 		// static int cnt[8]={0};
	// 	FDCAN_RxHeaderTypeDef rx_header;
 //    	uint8_t rx_data[8];
 //    	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
	// }
	FDCAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	//if (hfdcan==&hfdcan3) printf("6");

	while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)
	{
		// 确保 ID 在我们关注的电机ID范围内 (0x201 - 0x208)
		if(rx_header.Identifier >= CAN_3508_M1_ID && rx_header.Identifier <= CAN_3508_M8_ID)
		{
			Motor_Rx_Queue_t rx_msg;
			rx_msg.hcan = hfdcan; // 保存当前的 CAN 句柄
			rx_msg.motor_id = rx_header.Identifier;
			memcpy(rx_msg.rx_data, rx_data, 8);

			if (xQueueSendFromISR(motorRxQueueHandle, &rx_msg, &xHigherPriorityTaskWoken) != pdPASS)
			{
				// 队列已满
			}
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void send_message(uint32_t id,uint8_t data)
{
    static uint8_t data_byte[1];
    data_byte[0] = data;

    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = id; //这里的0x22
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    // 发送数据
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data_byte) != HAL_OK) {
	    // 错误处理
	    printf("no sb!\n");
    }
}
#endif
