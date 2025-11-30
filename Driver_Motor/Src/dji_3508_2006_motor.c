#include "FreeRTOS.h"
#include "dji_3508_2006_motor.h"
#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "pid.h"
#include "math.h"

#include "task.h"
#include <stdbool.h>


//3508电流范围   -16384-16384
//2006电流范围   -10000-10000

/**************内部宏定义与重命名begin**************/

//#define ALL_Send_Flag //收到所有电机报文后发送一条报文的标志

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
////将用6号电机代替9号电机
static int set_spd_s[9]={0,0,0,0,0,0,0,0,0};
int set_loc_s[9]={0,0,0,0,0,0,0,0,0};
static int control_flag_first4=1;
static int control_flag_last4=1;
static int control_flag=1;


static int mode_s[9]={
	SPEED_MODE,
	SPEED_MODE,
	SPEED_MODE,
	SPEED_MODE,
	GROUP_MODE,
	GROUP_MODE,
	GROUP_MODE,
	LOC_MODE,
	LOC_MODE
};
const uint8_t SYNC_GROUP_IDS[MAX_SYNC_MOTORS_PER_GROUP] = {4, 5, 6, 0, 0, 0, 0, 0}; // 示例：ID 1, 3, 5, 7 同步

static motor_measure_t motor_inf[9]={0};/*3508电机参数*/
static void Get_total_angle(motor_measure_t *p);
static void Can_dji_3508_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


void Dji_3508_first_four_motor_control(int i,uint8_t rx_data[8]);
void Dji_3508_last_four_motor_control(int i,uint8_t rx_data[8]);

static bool motor_enabled[8] = {true, true, true, true, true, true, true, true};
/**************内部变量与函数end**************/


/**************外部接口begin**************/
void Change_dji_speed(int motor_id,int target_spd);
void Change_dji_loc(int motor_id,int target_loc);
motor_measure_t Get_dji_information(int motor_id);
void Discontrol_dji_motor(void);
void Recontrol_dji_motor(void);

void Set_motor_enabled(int motor_id, bool enabled);
void Dji_3508_all_motor_control(void); // 新增：集成 PID 计算和发送函数
/**************外部接口end**************/
/**
 * @brief  计算一个整数的绝对值
 *
 * @param  x 待计算的整数
 *
 * @return int 绝对值
 *
 * @note   用于 Get_total_angle 中
 */
int Basic_int_abs(int x){/*绝对值*/
	return x>=0 ? x:-x;
}

/*
1.函数功能：请求dji电机信息，包括速度、位置等
2.入参：电机ID
3.返回值：电机信息结构体
4.用法及调用要求：
5.其它：
*/
motor_measure_t Get_dji_information(int motor_id){
	motor_measure_t temp_info;
	taskENTER_CRITICAL();
	{
		temp_info = motor_inf[motor_id];
	}
	taskEXIT_CRITICAL();
	return temp_info;

}
/*
1.函数功能：设定dji电机的速度大小
2.入参：电机ID，speed（RPM）
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_dji_speed(int motor_id,int target_spd){
	set_spd_s[motor_id]=-1*target_spd;//注意该代码为调整电机方向有*-1
}
/*
1.函数功能：设定dji电机的位置
2.入参：电机ID，整数值，编码器相关
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_dji_loc(int motor_id,int target_loc){
	set_loc_s[motor_id]=target_loc;
}
/*
1.函数功能：是否控制3508电机
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Discontrol_dji_motor(void){
	control_flag=0;
}
void Recontrol_dji_motor(void){
	control_flag=1;
}
/*
1.函数功能：得到dji电调的电机信息返回值
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：宏函数
*/
#define Get_motor_measure(ptr, data)                                    \
    {                                                                  \
        (ptr)->last_angle = (ptr)->angle;                                   \
        (ptr)->angle = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

#ifdef ALL_Send_Flag
		int can1_send_flag[8]={0};//收到所有电机报文后发送一条报文的标志
#endif
/*
1.函数功能：得到dji电机的总转程
2.入参：
3.返回值：无
4.用法及调用要求：
5.其它：
*/
static void Get_total_angle(motor_measure_t *p){


		int res1, res2, delta;
		if(p->angle < p->last_angle){			//可能的情况
			res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
			res2 = p->angle - p->last_angle;				//反转	delta=-
		}else{	//angle > last
			res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
			res2 = p->angle - p->last_angle;				//正转	delta +
		}
		//不管正反转，肯定是转的角度小的那个是真的
		if(Basic_int_abs(res1)<Basic_int_abs(res2))
			delta = res1;
		else
			delta = res2;

		p->total_angle += delta;
		p->last_angle = p->angle;
}
/*
1.函数功能：发送四个dji电机的电流值
2.入参：
3.返回值：无
4.用法及调用要求：注意修改发送通道为CAN1或CAN2
5.其它：
*/
static void Can_dji_3508_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
	static FDCAN_TxHeaderTypeDef  	first_four_motor_tx_message;
	static uint8_t              	first_four_motor_can_send_data[8];

	first_four_motor_tx_message.Identifier 	=	all_response_id;
	first_four_motor_tx_message.IdType		=	FDCAN_STANDARD_ID;
	first_four_motor_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
	first_four_motor_tx_message.DataLength 	= 	0x08;
	first_four_motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	first_four_motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
	first_four_motor_tx_message.FDFormat = FDCAN_FD_CAN;
	first_four_motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	first_four_motor_tx_message.MessageMarker = 0;

    first_four_motor_can_send_data[0] = motor1 >> 8;
    first_four_motor_can_send_data[1] = motor1;
    first_four_motor_can_send_data[2] = motor2 >> 8;
    first_four_motor_can_send_data[3] = motor2;
    first_four_motor_can_send_data[4] = motor3 >> 8;
    first_four_motor_can_send_data[5] = motor3;
    first_four_motor_can_send_data[6] = motor4 >> 8;
    first_four_motor_can_send_data[7] = motor4;

	if(hcan==&hfdcan3){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &first_four_motor_tx_message, first_four_motor_can_send_data);
	}
	else if(hcan==&hfdcan1){
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &first_four_motor_tx_message, first_four_motor_can_send_data);
	}
	else if(hcan==&hfdcan2){
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &first_four_motor_tx_message, first_four_motor_can_send_data);
	}
}

/*
1.函数功能：pid计算出电机此次应当输出的电流值并发送
2.入参：报文的header，报文数据域内容
3.返回值：无
4.用法及调用要求：在dji电机的CAN的接收回调里调用此函数
5.其它：
*/

//can3接前四号电机
void Dji_3508_first_four_motor_control(int i,uint8_t rx_data[8])//   0<=i<=3
{
	Get_motor_measure(&motor_inf[i], rx_data);//得到电机信息
	if (motor_inf[i].first == 0)
	{
		motor_inf[i].first = 1;
		motor_inf[i].last_angle=motor_inf[i].angle;
	}
	Get_total_angle(&motor_inf[i]);
	if(mode_s[i]==LOC_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].loc,motor_inf[i].total_angle,set_loc_s[i]);//利用电机反馈的速度和位置信息做pid计算
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,motor_3508_pid_g[i].loc.now_out);
	}
	else if(mode_s[i]==SPEED_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,set_spd_s[i]);
	}
	if(control_flag==0)
	{
		for(int cnt=0 ; cnt<4 ; cnt++)
		{
			motor_3508_pid_g[cnt].spd.now_out=0;
		}
	}
	Can_dji_3508_motor_send(&hfdcan3,CAN_FIRST_FOUR_MOTOR_ALL_ID,
				(int16_t)motor_3508_pid_g[0].spd.now_out,   //将PID的计算结果通过CAN发到电机
		(int16_t)motor_3508_pid_g[1].spd.now_out,
		(int16_t)motor_3508_pid_g[2].spd.now_out,
		(int16_t)motor_3508_pid_g[3].spd.now_out);
}

//can1接后四号电机
void Dji_3508_last_four_motor_control(int i,uint8_t rx_data[8])//4<=i<=7
{
	Get_motor_measure(&motor_inf[i], rx_data);//得到电机信息
	if (motor_inf[i].first == 0)
	{
		motor_inf[i].first = 1;
		motor_inf[i].last_angle=motor_inf[i].angle;
	}
	Get_total_angle(&motor_inf[i]);
	if(mode_s[i]==LOC_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].loc,motor_inf[i].total_angle,set_loc_s[i]);//利用电机反馈的速度和位置信息做pid计算
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,motor_3508_pid_g[i].loc.now_out);
	}
	else if(mode_s[i]==SPEED_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,set_spd_s[i]);
	}
	if(control_flag==0)
	{
		for(int cnt=4 ; cnt<8 ; cnt++)
		{
			motor_3508_pid_g[cnt].spd.now_out=0;
		}
	}
	Can_dji_3508_motor_send(&hfdcan1,CAN_LAST_FOUR_MOTOR_ALL_ID,
		(int16_t)motor_3508_pid_g[4].spd.now_out,   //将PID的计算结果通过CAN发到电机
		(int16_t)motor_3508_pid_g[5].spd.now_out,
		(int16_t)motor_3508_pid_g[6].spd.now_out,
		(int16_t)motor_3508_pid_g[7].spd.now_out);
}

//一下两个函数为can2同时接前四号和后四号电机使用，同时可控制9号电机，用6号代替（i=5，灯闪六下）
void Dji_3508_last_motor_control(int i,uint8_t rx_data[8])//4<=i<=7
{
	Get_motor_measure(&motor_inf[i], rx_data);//得到电机信息

	if (motor_inf[i].first == 0)
	{
		motor_inf[i].first = 1;
		motor_inf[i].last_angle=motor_inf[i].angle;
	}
	Get_total_angle(&motor_inf[i]);
	if(mode_s[i]==LOC_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].loc,motor_inf[i].total_angle,set_loc_s[i]);//利用电机反馈的速度和位置信息做pid计算
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,motor_3508_pid_g[i].loc.now_out);
	}
	else if(mode_s[i]==SPEED_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,set_spd_s[i]);
	}
	if(i==7 && motor_inf[7].temperate>70)
	{
		motor_3508_pid_g[7].spd.now_out=0;
	}
	if(control_flag==0)
	{
		for(int cnt=4 ; cnt<9 ; cnt++)
		{
			motor_3508_pid_g[cnt].spd.now_out=0;
		}
	}
	Can_dji_3508_motor_send(&hfdcan2,CAN_LAST_FOUR_MOTOR_ALL_ID,
		(int16_t)motor_3508_pid_g[4].spd.now_out,   //将PID的计算结果通过CAN发到电机
		(int16_t)motor_3508_pid_g[8].spd.now_out,
		(int16_t)motor_3508_pid_g[6].spd.now_out,
		(int16_t)motor_3508_pid_g[7].spd.now_out);
}

void Dji_3508_first_motor_control(int i,uint8_t rx_data[8])
{
	Get_motor_measure(&motor_inf[i], rx_data);//得到电机信息
	if (motor_inf[i].first == 0)
	{
		motor_inf[i].first = 1;
		motor_inf[i].last_angle=motor_inf[i].angle;
	}
	Get_total_angle(&motor_inf[i]);
	if(mode_s[i]==LOC_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].loc,motor_inf[i].total_angle,set_loc_s[i]);//利用电机反馈的速度和位置信息做pid计算
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,motor_3508_pid_g[i].loc.now_out);
	}
	else if(mode_s[i]==SPEED_MODE)
	{
		Pid_incremental_cal(&motor_3508_pid_g[i].spd,motor_inf[i].speed_rpm,set_spd_s[i]);
	}
	if(control_flag==0)
	{
		for(int cnt=0 ; cnt<4 ; cnt++)
		{
			motor_3508_pid_g[cnt].spd.now_out=0;
		}
	}
	Can_dji_3508_motor_send(&hfdcan2,CAN_FIRST_FOUR_MOTOR_ALL_ID,
		(int16_t)motor_3508_pid_g[0].spd.now_out,   //将PID的计算结果通过CAN发到电机
		(int16_t)motor_3508_pid_g[1].spd.now_out,
		(int16_t)motor_3508_pid_g[2].spd.now_out,
		(int16_t)motor_3508_pid_g[3].spd.now_out);
}
/**
 * @brief  计算当前同步群组内所有电机的平均总角度
 *
 * @param  无
 *
 * @return float 同步群组的平均总角度
 *
 * @note   仅遍历 SYNC_GROUP_IDS 数组中定义的、且处于 GROUP_MODE 的电机。
 * @see    SYNC_GROUP_IDS
 */
static float Calculate_Group_Average_Angle(void)
{
	int32_t total_sum = 0;
	int count = 0;
	for (int i = 0; i < MAX_SYNC_MOTORS_PER_GROUP && SYNC_GROUP_IDS[i] != 0; i++)
	{
		uint8_t motor_id = SYNC_GROUP_IDS[i];

		// 1. 检查 ID 是否在有效范围 (0-7)
		// 2. 检查该电机是否被设置为 GROUP_MODE
		// 3. 检查电机是否已接收到第一帧数据 (确保 total_angle 有效)
		if (motor_id < 8 && mode_s[motor_id] == GROUP_MODE && motor_inf[motor_id].first == 1)
		{
			total_sum += motor_inf[motor_id].total_angle;
			count++;
		}
	}

	if (count > 0)
	{
		return (float)total_sum / count;
	}
	return 0.0f; // 如果组内没有有效电机，返回 0
}



void Dji_Motor_Update_Status(uint32_t id, uint8_t *data)
{
	int index = -1;
	// 1. 根据 CAN ID 匹配数组索引 (0-7)
	// CAN_3508_M1_ID 是 0x201
	if (id >= CAN_3508_M1_ID && id <= CAN_3508_M4_ID) {
		index = id - CAN_3508_M1_ID; // 0 ~ 3
	}
	else if (id >= CAN_3508_M5_ID && id <= CAN_3508_M8_ID) {
		index = id - CAN_3508_M5_ID + 4; // 4 ~ 7
	}
	else {
		return; // ID 不在范围内，直接退出
	}
	Get_motor_measure(&motor_inf[index], data);
	// 3. 处理上电第一帧数据的初始化逻辑
	if (motor_inf[index].first == 0)
	{
		motor_inf[index].first = 1;
		motor_inf[index].last_angle = motor_inf[index].angle;
		motor_inf[index].total_angle = 0;
	}
	// 4. 计算多圈绝对角度
	Get_total_angle(&motor_inf[index]);
}

// /*
// 1.函数功能：集成 PID 计算后发送所有电机电流，一共八个电机，前四个一条 CAN 报文，后四个一条 CAN 报文
// 2.入参：无
// 3.返回值：无
// 4.用法及调用要求：在任务中定期调用，假设电机信息已通过回调更新
// 5.其它：未启用的电机不进行 PID 计算，直接设为 0
// */
// void Dji_3508_all_motor_control(void) {
//     // 计算前四个电机
//     for (int i = 0; i < 4; i++) {
//         if (motor_enabled[i]) {
//             if (mode_s[i] == LOC_MODE) {
//                 Pid_incremental_cal(&motor_3508_pid_g[i].loc, motor_inf[i].total_angle, set_loc_s[i]);
//                 Pid_incremental_cal(&motor_3508_pid_g[i].spd, motor_inf[i].speed_rpm, motor_3508_pid_g[i].loc.now_out);
//             } else if (mode_s[i] == SPEED_MODE) {
//                 Pid_incremental_cal(&motor_3508_pid_g[i].spd, motor_inf[i].speed_rpm, set_spd_s[i]);
//             }
//         } else {
//             motor_3508_pid_g[i].spd.now_out = 0;
//         }
//     }
//
//     // 如果全局控制关闭，设为 0
//     if (control_flag == 0) {
//         for (int i = 0; i < 4; i++) {
//             motor_3508_pid_g[i].spd.now_out = 0;
//         }
//     }
// 	if(control_flag_first4){
//     // 发送前四个电机
//     Can_dji_3508_motor_send(&hfdcan1, CAN_FIRST_FOUR_MOTOR_ALL_ID,
//         (int16_t)motor_3508_pid_g[0].spd.now_out,
//         (int16_t)motor_3508_pid_g[1].spd.now_out,
//         (int16_t)motor_3508_pid_g[2].spd.now_out,
//         (int16_t)motor_3508_pid_g[3].spd.now_out);}
//
//     // 计算后四个电机
//     for (int i = 4; i < 8; i++) {
//         if (motor_enabled[i]) {
//             if (mode_s[i] == LOC_MODE) {
//                 Pid_incremental_cal(&motor_3508_pid_g[i].loc, motor_inf[i].total_angle, set_loc_s[i]);
//                 Pid_incremental_cal(&motor_3508_pid_g[i].spd, motor_inf[i].speed_rpm, motor_3508_pid_g[i].loc.now_out);
//             } else if (mode_s[i] == SPEED_MODE) {
//                 Pid_incremental_cal(&motor_3508_pid_g[i].spd, motor_inf[i].speed_rpm, set_spd_s[i]);
//             }
//         } else {
//             motor_3508_pid_g[i].spd.now_out = 0;
//         }
//     }
//
//     // 如果全局控制关闭，设为 0
//     if (control_flag == 0) {
//         for (int i = 4; i < 8; i++) {
//             motor_3508_pid_g[i].spd.now_out = 0;
//         }
//     }
// 	if(control_flag_last4){
//     // 发送后四个电机
//     Can_dji_3508_motor_send(&hfdcan1, CAN_LAST_FOUR_MOTOR_ALL_ID,
//         (int16_t)motor_3508_pid_g[4].spd.now_out,
//         (int16_t)motor_3508_pid_g[5].spd.now_out,
//         (int16_t)motor_3508_pid_g[6].spd.now_out,
//         (int16_t)motor_3508_pid_g[7].spd.now_out);}
// }
/**
 * @brief  集成 PID 计算和发送所有电机电流
 *
 * @param  无
 *
 * @return 无
 *
 * @note   该函数在 FreeRTOS 任务中定期调用，负责执行所有控制算法和CAN发送。
 * @see    Calculate_Group_Average_Angle
 */
void Dji_3508_all_motor_control(void) {
    float average_angle = 0.0f;
    bool group_mode_active = false;
    for (int i = 0; i < 8; i++) {
        if (mode_s[i] == GROUP_MODE) {
            group_mode_active = true;
            break;
        }
    }

    if (group_mode_active) {
        average_angle = Calculate_Group_Average_Angle();
    }

    for (int i = 0; i < 8; i++) {
        if (motor_enabled[i] && motor_inf[i].first == 1) {

            if (mode_s[i] == LOC_MODE)
            {
                // LOC_MODE (普通位置-速度双环)
                Pid_incremental_cal(&motor_3508_pid_g[i].loc, motor_inf[i].total_angle, set_loc_s[i]);
                Pid_incremental_cal(&motor_3508_pid_g[i].spd, motor_inf[i].speed_rpm, motor_3508_pid_g[i].loc.now_out);
            }
            else if (mode_s[i] == SPEED_MODE)
            {
                // SPEED_MODE (纯速度环)
                Pid_incremental_cal(&motor_3508_pid_g[i].spd, motor_inf[i].speed_rpm, set_spd_s[i]);
            }
            else if (mode_s[i] == GROUP_MODE)
            {
                // GROUP_MODE (群组同步模式)
                Pid_incremental_cal(&motor_3508_pid_g[i].loc, motor_inf[i].total_angle, set_loc_s[i]);
                Pid_incremental_cal(&motor_3508_pid_g[i].diff,
                                    (float)motor_inf[i].total_angle, // 实际值 (A_current)
                                    average_angle);                  // 目标值 (A_avg)
                Pid_incremental_cal(&motor_3508_pid_g[i].spd,
                                    motor_inf[i].speed_rpm,
                                    (motor_3508_pid_g[i].loc.now_out - motor_3508_pid_g[i].diff.now_out));
            }
        }
        else
        {
            motor_3508_pid_g[i].spd.now_out = 0;
        }
    }
    if (control_flag == 0) {
        for (int i = 0; i < 8; i++) {
            motor_3508_pid_g[i].spd.now_out = 0;
        }
    }
    // FDCAN3/CAN1: 发送前四个电机 (0-3)
    Can_dji_3508_motor_send(&hfdcan1, CAN_FIRST_FOUR_MOTOR_ALL_ID,
        (int16_t)motor_3508_pid_g[0].spd.now_out,
        (int16_t)motor_3508_pid_g[1].spd.now_out,
        (int16_t)motor_3508_pid_g[2].spd.now_out,
        (int16_t)motor_3508_pid_g[3].spd.now_out);

    // FDCAN1/CAN2: 发送后四个电机 (4-7)
    // 注意：假设 hfdcan1 是发送后四号电机的句柄，请根据实际硬件连接调整 hcan 句柄（如 hfdcan1/hfdcan2）。
    Can_dji_3508_motor_send(&hfdcan1, CAN_LAST_FOUR_MOTOR_ALL_ID,
        (int16_t)motor_3508_pid_g[4].spd.now_out,
        (int16_t)motor_3508_pid_g[5].spd.now_out,
        (int16_t)motor_3508_pid_g[6].spd.now_out,
        (int16_t)motor_3508_pid_g[7].spd.now_out);
}