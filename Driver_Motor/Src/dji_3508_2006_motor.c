#include "FreeRTOS.h"
#include "dji_3508_2006_motor.h"
#include "main.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "pid.h"
#include "math.h"
#include "task.h"
#include <stdbool.h>
#include <string.h>


//3508电流范围   -16384-16384
//2006电流范围   -10000-10000

/**************内部宏定义与重命名begin**************/

//#define ALL_Send_Flag //收到所有电机报文后发送一条报文的标志

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
static int control_flag=1;
const uint8_t SYNC_GROUP_IDS[MAX_SYNC_MOTORS_PER_GROUP] = {4, 5, 6, 0, 0, 0, 0, 0}; // 示例：ID 1, 3, 5, 7 同步
Can_Tx_Buffer_t g_can_tx_buffers[MAX_CAN_HANDLES];
static void Get_total_angle(motor_measure_t *p);
static void Can_dji_3508_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
static bool Is_Climb_Motor(int motor_id);
static float Clampf(float value, float min_value, float max_value);
static int16_t Get_Gravity_FF_Current(const Dji_Motor_t *motor, int motor_id);
static float Shape_Climb_Spd_Target(float raw_spd_target, int motor_id);

/* Gravity feedforward config for 4 climb 3508 motors only.
 * sign: current direction that means "lifting up" (+1/-1).
 * hold: hold current near target; up/down: extra bias by move direction.
 */
static const int8_t g_climb_ff_sign[DJI_MOTOR_COUNT] = {
	0, 0, 0, 0,
	-1, 1, 1, -1,
	0, 0
};
static const int16_t g_climb_ff_hold[DJI_MOTOR_COUNT] = {
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0
};
static const int16_t g_climb_ff_up[DJI_MOTOR_COUNT] = {
	0, 0, 0, 0,
	2900, 2900, 400, 400,
	0, 0
};
static const int16_t g_climb_ff_down[DJI_MOTOR_COUNT] = {
	0, 0, 0, 0,
	4000, 4000, 600, 600,
	0, 0
};
static const int32_t g_climb_loc_hold_deadband = 180;

/*
 * 下降速度整形参数（只用于四个抬升电机的 LOC/GROUP 模式）：
 * - down_limit: 下降速度上限（按“向上轴”定义，绝对值越小下降越慢）。
 * - up_limit  : 上抬速度上限。
 * - slew      : 目标速度斜率限制，避免前后轴突变。
 *
 * 这里前轴 down_limit 更小（更慢），后轴更大（更快），
 * 目的是减小“前轴先触地导致零位抬高”的趋势。
 */
static const float g_climb_down_spd_limit_rpm[DJI_MOTOR_COUNT] = {
	0, 0, 0, 0,
	3500.0f, 3500.0f, 3500.0f, 3500.0f,
	0, 0
};
static const float g_climb_up_spd_limit_rpm[DJI_MOTOR_COUNT] = {
	0, 0, 0, 0,
	200.0f, 200.0f, 200.0f, 200.0f,
	0, 0
};
static const float g_climb_spd_slew_up_rpm_per_cycle = 100.0f;
static const float g_climb_spd_slew_down_rpm_per_cycle = 50.0f;
static float g_climb_spd_target_last[DJI_MOTOR_COUNT] = {0};

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
 * @brief 统一的DJI电机注册表
 * @note  电机数量为 DJI_MOTOR_COUNT。配置必须在这里明确定义，包括嵌入的 PID 参数。
 */
Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT] =
{
    // ------------------------------------------------------------------------
    // 索引 0: DJI_M_CHASSIS_LF - CAN1 - ID 0x201
    // ------------------------------------------------------------------------
    [DJI_M_CHASSIS_LF] = {
        .hcan_tx          = &hfdcan1,
        .can_rx_id        = CAN_3508_M1_ID,
        .can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 0,
        .target_spd       = 0,
        .control_mode     = SPEED_MODE,
        .is_enabled       = true,
    },

    // ------------------------------------------------------------------------
    // 索引 1: DJI_M_CHASSIS_LB - CAN1 - ID 0x202
    // ------------------------------------------------------------------------
    [DJI_M_CHASSIS_LB] = {
        .hcan_tx          = &hfdcan1,
        .can_rx_id        = CAN_3508_M2_ID,
        .can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 1,
        .target_spd       = 0,
        .control_mode     = SPEED_MODE,
        .is_enabled       = true,
    },

	// ------------------------------------------------------------------------
	// 索引 2: DJI_M_CHASSIS_RF - CAN1 - ID 0x203
	// ------------------------------------------------------------------------
	[DJI_M_CHASSIS_RF] = {
    	.hcan_tx          = &hfdcan1,
		.can_rx_id        = CAN_3508_M3_ID,
		.can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 2,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

	// ------------------------------------------------------------------------
	// 索引 3: DJI_M_CHASSIS_RB - CAN1 - ID 0x204
	// ------------------------------------------------------------------------
	[DJI_M_CHASSIS_RB] = {
    	.hcan_tx          = &hfdcan1,
		.can_rx_id        = CAN_3508_M4_ID,
		.can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 3,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

	// ------------------------------------------------------------------------
	// 索引 4: DJI_M_CLIMB_LF - CAN1 - ID 0x205
	// ------------------------------------------------------------------------
	[DJI_M_CLIMB_LF] = {
    	.hcan_tx          = &hfdcan1,
		.can_rx_id        = CAN_3508_M5_ID,
		.can_tx_header_id = CAN_LAST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 0,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

	// ------------------------------------------------------------------------
	// 索引 5: DJI_M_CLIMB_RF - CAN1 - ID 0x206
	// ------------------------------------------------------------------------
	[DJI_M_CLIMB_RF] = {
    	.hcan_tx          = &hfdcan2,
		.can_rx_id        = CAN_3508_M6_ID,
		.can_tx_header_id = CAN_LAST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 1,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

	// ------------------------------------------------------------------------
	// 索引 6: DJI_M_CLIMB_LB - CAN1 - ID 0x204
	// ------------------------------------------------------------------------
	[DJI_M_CLIMB_LB] = {
    	.hcan_tx          = &hfdcan2,
		.can_rx_id        = CAN_3508_M7_ID,
		.can_tx_header_id = CAN_LAST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 2,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

	// ------------------------------------------------------------------------
	// 索引 7: DJI_M_CLIMB_RB - CAN1 - ID 0x204
	// ------------------------------------------------------------------------
	[DJI_M_CLIMB_RB] = {
    		.hcan_tx          = &hfdcan2,
			.can_rx_id        = CAN_3508_M8_ID,
			.can_tx_header_id = CAN_LAST_FOUR_MOTOR_ALL_ID,
			.tx_index         = 3,
			.target_spd       = 0,
			.control_mode     = SPEED_MODE,
			.is_enabled       = true,
		},

	// ------------------------------------------------------------------------
	// 索引 8: DJI_M_CHASSIS_LF - CAN1 - ID 0x201
	// ------------------------------------------------------------------------
	[DJI_2006_L] = {
    	.hcan_tx          = &hfdcan2,
		.can_rx_id        = CAN_3508_M1_ID,
		.can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 0,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

	// // // ------------------------------------------------------------------------
	// // // 索引 9: DJI_M_CHASSIS_LF - CAN1 - ID 0x201
	// // // ------------------------------------------------------------------------
	[DJI_2006_R] = {
    	.hcan_tx          = &hfdcan2,
		.can_rx_id        = CAN_3508_M2_ID,
		.can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
		.tx_index         = 1,
		.target_spd       = 0,
		.control_mode     = SPEED_MODE,
		.is_enabled       = true,
	},

    // ... 更多电机实例 ...
};

Can_Tx_Buffer_t g_can_tx_buffers[MAX_CAN_HANDLES] = {
	{.hcan = &hfdcan1, .need_to_send = false}, // Index 0: FDCAN1
	{.hcan = &hfdcan2, .need_to_send = false}, // Index 1: FDCAN2
	{.hcan = &hfdcan3, .need_to_send = false}  // Index 2: FDCAN3
};
/**
 * @brief  初始化DJI电机注册表中的所有配置。
 *
 * @param  None
 *
 * @return None
 *
 * @note   1. 调用外部 Pid_parameter_init() 初始化 motor_3508_pid_g 数组。
 * @note   2. 将初始化好的 PID 参数拷贝到注册表中的相应电机实例。
 * @see    Pid_parameter_init
 * @date   2025-12-01
 */
void Dji_Motor_Registry_Init(void)
{
	Pid_parameter_init();
	g_can_tx_buffers[0].hcan = &hfdcan1;
	g_can_tx_buffers[1].hcan = &hfdcan2;
	g_can_tx_buffers[2].hcan = &hfdcan3;
	for (int i = 0; i < DJI_MOTOR_COUNT; i++)
	{
		Dji_Motor_t *motor = &g_dji_motor_registry[i];

		// 初始化通用状态
		motor->feedback.first = 0;
		motor->current_set = 0;
		motor->is_online = false;
		// 假设 motor_3508_pid_g 的索引 i 对应注册表中的电机实例 i
		if (i < DJI_MOTOR_COUNT) // 确保不越界访问 motor_3508_pid_g 数组 (大小为 9)
		{
			memcpy(&motor->pid_params,
				   &motor_3508_pid_g[i],
				   sizeof(motor_pid_parameter));
		}
	}
}

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

/**
1.函数功能：请求dji电机信息，包括速度、位置等
2.入参：电机ID
3.返回值：电机信息结构体
4.用法及调用要求：
5.其它：
*/
motor_measure_t Get_dji_information(int motor_id){
	motor_measure_t temp_info = {0};
	if (motor_id < DJI_MOTOR_COUNT)
	{
		taskENTER_CRITICAL();
		{
			temp_info = g_dji_motor_registry[motor_id].feedback;
		}
		taskEXIT_CRITICAL();
	}
	return temp_info;
}

/**
1.函数功能：设定dji电机的速度大小
2.入参：电机ID，speed（RPM）
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_dji_speed(int motor_id,int target_spd){
	if (motor_id < DJI_MOTOR_COUNT)
	{
		g_dji_motor_registry[motor_id].target_spd = -1 * target_spd; // 注意该代码为调整电机方向有*-1
		g_dji_motor_registry[motor_id].control_mode = SPEED_MODE; // 自动切换模式
	}
}
/**
1.函数功能：设定dji电机的位置
2.入参：电机ID，整数值，编码器相关
3.返回值：无
4.用法及调用要求：
5.其它：
*/
void Change_dji_loc(int motor_id,int target_loc){
	if (motor_id < DJI_MOTOR_COUNT)
	{
		// ✅ 修改注册表中的目标位置
		g_dji_motor_registry[motor_id].target_loc = target_loc;
		g_dji_motor_registry[motor_id].control_mode = LOC_MODE; // 自动切换模式
	}
}
/**
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
/**
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
/**
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

		if (motor_id < DJI_MOTOR_COUNT)
		{
			Dji_Motor_t *motor = &g_dji_motor_registry[motor_id];

			// 1. 检查该电机是否被设置为 GROUP_MODE
			// 2. 检查电机是否已接收到第一帧数据
			if (motor->control_mode == GROUP_MODE && motor->feedback.first == 1)
			{
				total_sum += motor->feedback.total_angle; // ✅ 使用注册表反馈
				count++;
			}
		}
	}

	if (count > 0)
	{
		return (float)total_sum / count;
	}
	else {
		return 0.0f;
	}
}

/* Enable gravity feedforward only for lift-axis motors. */
static bool Is_Climb_Motor(int motor_id)
{
	return (motor_id == DJI_M_CLIMB_LF ||
			motor_id == DJI_M_CLIMB_RF ||
			motor_id == DJI_M_CLIMB_LB ||
			motor_id == DJI_M_CLIMB_RB);
}

/* Clamp helper to keep final current inside PID configured bounds. */
static float Clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

/* Compute gravity FF current for climb motors by loc error and sign. */
static int16_t Get_Gravity_FF_Current(const Dji_Motor_t *motor, int motor_id)
{
	int32_t loc_err = motor->target_loc - motor->feedback.total_angle;
	int32_t abs_err = (loc_err >= 0) ? loc_err : (-loc_err);
	int32_t ff_mag = 0;
	int8_t sign = g_climb_ff_sign[motor_id];

	if (sign == 0) {
		return 0;
	}

	if (abs_err <= g_climb_loc_hold_deadband) {
		ff_mag = g_climb_ff_hold[motor_id];
	} else if ((loc_err * sign) > 0) {
		ff_mag = g_climb_ff_up[motor_id];
	} else {
		ff_mag = g_climb_ff_down[motor_id];
	}

	return (int16_t)(sign * ff_mag);
}

/*
 * 抬升轴速度整形（不做触地冻结/自动回零）：
 * 1) 先按前后轴分别限速（尤其下降限速）。
 * 2) 再做斜率限制，减小命令突变引起的结构弹性回弹。
 *
 * 输入输出单位都为 rpm，对应 speed PID 的目标转速。
 */
static float Shape_Climb_Spd_Target(float raw_spd_target, int motor_id)
{
	float desired = raw_spd_target;
	float shaped = desired;
	float last = g_climb_spd_target_last[motor_id];
	int8_t sign = g_climb_ff_sign[motor_id];

	if (sign == 0) {
		return raw_spd_target;
	}

	/* 统一到“向上轴”做判断，正值=上抬，负值=下降。 */
	{
		float up_axis_cmd = desired * sign;
		if (up_axis_cmd >= 0.0f) {
			up_axis_cmd = Clampf(up_axis_cmd, 0.0f, g_climb_up_spd_limit_rpm[motor_id]);
		} else {
			float down_mag = Clampf(-up_axis_cmd, 0.0f, g_climb_down_spd_limit_rpm[motor_id]);
			up_axis_cmd = -down_mag;
		}
		desired = up_axis_cmd * sign;
	}

	/* 斜率限制：每个控制周期只允许有限变化。 */
	{
		float delta = desired - last;
		delta = Clampf(delta, -g_climb_spd_slew_down_rpm_per_cycle, g_climb_spd_slew_up_rpm_per_cycle);
		shaped = last + delta;
	}

	g_climb_spd_target_last[motor_id] = shaped;
	return shaped;
}

void Dji_Motor_Update_Status(FDCAN_HandleTypeDef* hcan_rx, uint32_t id, uint8_t *data)
{
	int index = -1;
	for (int i = 0; i < DJI_MOTOR_COUNT; i++)
	{
		if (g_dji_motor_registry[i].hcan_tx == hcan_rx && g_dji_motor_registry[i].can_rx_id == id)
		{
			index = i;
			break; // 找到唯一匹配的电机
		}
	}
	if (index == -1) {
		// 未找到匹配的电机实例 (可能是未配置的 ID 或 CAN 句柄不匹配)
		return;
	}
	// 找到匹配的电机实例
	Dji_Motor_t *motor = &g_dji_motor_registry[index];
	taskENTER_CRITICAL();
	{
		//  解析原始 CAN 数据并填充 feedback 结构体
		Get_motor_measure(&motor->feedback, data);

		// 处理上电第一帧数据的初始化逻辑 (仅执行一次)
		if (motor->feedback.first == 0)
		{
			motor->feedback.first = 1;
			motor->feedback.last_angle = motor->feedback.angle;
			motor->feedback.total_angle = 0; // 首次启动，总角度归零
			motor->is_online = true;         // 首次收到信号，标记为在线
		}
		// 计算多圈绝对角度 (total_angle, total_round_cnt)
		Get_total_angle(&motor->feedback);
		// 更新在线状态 (每次收到报文都更新计数/标记)
		motor->is_online = true; // 每次收到都重置在线标记（如果采用超时机制，还需要一个计数器）
	}
	taskEXIT_CRITICAL();
}

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

	for (int i = 0; i < MAX_CAN_HANDLES; i++) {
		memset(g_can_tx_buffers[i].currents_0x200, 0, sizeof(g_can_tx_buffers[i].currents_0x200));
		memset(g_can_tx_buffers[i].currents_0x1FF, 0, sizeof(g_can_tx_buffers[i].currents_0x1FF));
		g_can_tx_buffers[i].need_to_send = false; // 每次控制循环前重置发送标记
	}
    // 遍历注册表，检查 GROUP_MODE 是否激活
    for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
        if (g_dji_motor_registry[i].control_mode == GROUP_MODE) {
            group_mode_active = true;
            break;
        }
    }
    if (group_mode_active) {
        average_angle = Calculate_Group_Average_Angle();
    }

	taskENTER_CRITICAL();
    for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
        Dji_Motor_t *motor = &g_dji_motor_registry[i];

    	int can_index = -1;
    	if (motor->hcan_tx == &hfdcan1) can_index = FDCNA1;
    	else if (motor->hcan_tx == &hfdcan2) can_index = FDCNA2;
    	else if (motor->hcan_tx == &hfdcan3) can_index = FDCNA3;

        if (motor->is_enabled && motor->feedback.first == 1  && can_index != -1) { // 检查是否启用且已接收第一帧

            // 模式控制
            if (Is_Climb_Motor(i) &&
                !(motor->control_mode == LOC_MODE || motor->control_mode == GROUP_MODE))
            {
                g_climb_spd_target_last[i] = 0.0f;
            }

            if (motor->control_mode == LOC_MODE)
            {
                // LOC_MODE (普通位置-速度双环)
                Pid_incremental_cal(&motor->pid_params.loc,
                                    (ElemType)motor->feedback.total_angle,
                                    (ElemType)motor->target_loc); // ✅ 使用注册表目标

                {
                    ElemType spd_target = motor->pid_params.loc.now_out;
                    if (Is_Climb_Motor(i)) {
                        spd_target = (ElemType)Shape_Climb_Spd_Target((float)spd_target, i);
                    }
                    Pid_incremental_cal(&motor->pid_params.spd,
                                        (ElemType)motor->feedback.speed_rpm,
                                        spd_target);
                }
            }
            else if (motor->control_mode == SPEED_MODE)
            {
                // SPEED_MODE (纯速度环)
                Pid_incremental_cal(&motor->pid_params.spd,
                                    (ElemType)motor->feedback.speed_rpm,
                                    (ElemType)motor->target_spd); // ✅ 使用注册表目标
            }
            else if (motor->control_mode == GROUP_MODE)
            {
                // GROUP_MODE (群组同步模式 - 位置环输出为速度环目标)
                // 1. 位置环 (外环)：控制电机跟踪平均位置 (可用于大范围运动)
                Pid_incremental_cal(&motor->pid_params.loc,
                                    (ElemType)motor->feedback.total_angle,
                                    (ElemType)motor->target_loc); // ✅ 使用注册表目标

                // 2. 同步差值环 (差值环)：控制电机修正与平均值的偏差
                Pid_incremental_cal(&motor->pid_params.diff,
                                    (ElemType)motor->feedback.total_angle, // 实际值 (A_current)
                                    average_angle);                  // 目标值 (A_avg)

                // 3. 速度环 (内环)：目标速度 = 位置环目标 - 同步差值环修正
                {
                    ElemType spd_target = (motor->pid_params.loc.now_out - motor->pid_params.diff.now_out);
                    if (Is_Climb_Motor(i)) {
                        spd_target = (ElemType)Shape_Climb_Spd_Target((float)spd_target, i);
                    }
                    Pid_incremental_cal(&motor->pid_params.spd,
                                        (ElemType)motor->feedback.speed_rpm,
                                        spd_target);
                }
            }

            // 最终电流值
            // Final command: PID output + gravity FF (lift motors in LOC/GROUP) + clamp.
            {
                float current_cmd = motor->pid_params.spd.now_out;

                if (Is_Climb_Motor(i) &&
                    (motor->control_mode == LOC_MODE || motor->control_mode == GROUP_MODE))
                {
                    current_cmd += (float)Get_Gravity_FF_Current(motor, i);
                }

                current_cmd = Clampf(current_cmd,
                                     motor->pid_params.spd.out_limit_down,
                                     motor->pid_params.spd.out_limit_up);
                motor->current_set = (int16_t)current_cmd;
            }
        }
        else
        {
            motor->current_set = 0;
            if (Is_Climb_Motor(i)) {
                g_climb_spd_target_last[i] = 0.0f;
            }
        }

        // 全局控制关闭，清零
        if (control_flag == 0) {
            motor->current_set = 0;
        }

        // 填充 CAN 发送数组
    	if (can_index != -1) {
    		if (motor->can_tx_header_id == CAN_FIRST_FOUR_MOTOR_ALL_ID && motor->tx_index < 4) {
    			g_can_tx_buffers[can_index].currents_0x200[motor->tx_index] = motor->current_set;
    			g_can_tx_buffers[can_index].need_to_send = true; // 标记该 CAN 句柄需要发送
    		} else if (motor->can_tx_header_id == CAN_LAST_FOUR_MOTOR_ALL_ID && motor->tx_index < 4) {
    			g_can_tx_buffers[can_index].currents_0x1FF[motor->tx_index] = motor->current_set;
    			g_can_tx_buffers[can_index].need_to_send = true; // 标记该 CAN 句柄需要发送
    		}
    	}
    }
	taskEXIT_CRITICAL();
    // **发送 CAN 报文**
	for (int i = 0; i < MAX_CAN_HANDLES; i++) {
		Can_Tx_Buffer_t *buffer = &g_can_tx_buffers[i];
		if (buffer->need_to_send) {
			// 发送 0x200 报文 (M1-M4)
			Can_dji_3508_motor_send(
				buffer->hcan,
				CAN_FIRST_FOUR_MOTOR_ALL_ID,
				buffer->currents_0x200[0],
				buffer->currents_0x200[1],
				buffer->currents_0x200[2],
				buffer->currents_0x200[3]
			);
			// 发送 0x1FF 报文 (M5-M8)
			Can_dji_3508_motor_send(
				buffer->hcan,
				CAN_LAST_FOUR_MOTOR_ALL_ID,
				buffer->currents_0x1FF[0],
				buffer->currents_0x1FF[1],
				buffer->currents_0x1FF[2],
				buffer->currents_0x1FF[3]
			);
		}
	}
}
