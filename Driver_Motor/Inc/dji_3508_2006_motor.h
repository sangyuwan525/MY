#ifndef DJI_3508_2006_MOTOR_H
#define DJI_3508_2006_MOTOR_H

/**********************************************
依赖文件：basic.c中的数学函数
					pid.c中的pid参数及pid计算
					can_database.c中的回调
**********************************************/


#include "main.h"
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>
#include "bsp_can.h"

enum
{
	SPEED_MODE = 0,
	LOC_MODE = 1,
	GROUP_MODE = 2,
};

enum {
	FDCNA1=0,
	FDCNA2,
	FDCNA3,
};

typedef enum
{
	DJI_YL = 0,		// 上下(Y轴)左边电机	0
	DJI_YR,			// 上下(Y轴)右边电机	1
	DJI_XL,			// 前后(X轴)左边电机	2
	DJI_XR,			// 前后(X轴)右边电机	3
	DJI_JOINT1_3508,// 机械臂关节1		4
	DJI_JOINT2_2006,// 机械臂关节2		5
	DJI_MOTOR_COUNT // 电机总数			6
} Dji_MotorID_e;

typedef struct {
	FDCAN_HandleTypeDef *hcan;
	int16_t currents_0x200[4];
	int16_t currents_0x1FF[4];
	bool need_to_send;
} Can_Tx_Buffer_t;

typedef enum{
	CAN_FIRST_FOUR_MOTOR_ALL_ID = 0x200,//dji系列电机号
	CAN_LAST_FOUR_MOTOR_ALL_ID = 0x1FF,

	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
	CAN_3508_M5_ID = 0x205,
	CAN_2006_M6_ID = 0x206,
	CAN_2006_M7_ID = 0x207,
	CAN_3508_M8_ID = 0x208,
} can_msg_id_e;

typedef struct {
	FDCAN_HandleTypeDef *hcan;
	int16_t  curr_200[4];  // 0x200 的4个电机电流
	int16_t  curr_1ff[4];  // 0x1FF 的4个电机电流
	bool     send_200;     // 是否需要发送标志
	bool     send_1ff;
} Dji_Send_Buffer_t;

#define MAX_CAN_HANDLES 3
//电机反馈结构体
typedef struct{
	uint16_t	angle;
	int16_t		speed_rpm;
	int16_t		given_current;
	uint8_t		temperate;
	uint16_t	last_angle;
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
	uint16_t	first;		//用于判断电调是否为第一次发送信号
} motor_measure_t;

// typedef struct {
// 	pid_incremental_struct spd;
// 	pid_incremental_struct loc;
// 	pid_incremental_struct diff;
// }motor_pid_parameter;

// 电机注册表结构体
typedef struct
{
	//硬件配置(用于CAN发送和接收匹配)
	FDCAN_HandleTypeDef*	hcan_tx;			// CAN发送句柄：例如 &hfdcan1
	uint32_t				can_rx_id;			// CAN接收ID：0x201 - 0x208
	uint32_t				can_tx_header_id;	// CAN发送报文ID：0x200 或 0x1FF
	uint8_t					tx_index;			// 报文中的索引：0-3 (对应报文数据 0-7)
	//控制目标
	float					target_loc;			// 目标位置
	float					target_spd;			// 目标速度 (RPM)
	uint8_t					control_mode;		// 控制模式：SPEED_MODE/LOC_MODE/GROUP_MODE
	//状态反馈
	motor_measure_t			feedback;			// 电机反馈信息
	//控制器
	motor_pid_parameter		pid_params;
	//运行时状态
	int16_t					current_set;		// 最终计算出的电流控制量
	bool					is_enabled;			// 是否启用控制
	bool					is_online;			// 是否在线 (通过反馈消息计数判断)

	int						sync_group_id;		//所属同步组ID
	int32_t					average_loc;		//当前组的平均值
} Dji_Motor_t;

#define TOTAL_SYNC_GROUPS 2           // 总共有几组 (X轴一组, Y轴一组)
#define MAX_MOTORS_PER_GROUP 2        // 每组最多几个电机

extern Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT];
// --- 函数接口  ---

// 初始化电机对象
void Dji_Motor_Init(motor_measure_t *motor, uint32_t id, uint8_t mode);
void Dji_Motor_Update(Dji_Motor_t *motor, uint8_t *rx_data);
void Dji_Motor_Update_Status(FDCAN_HandleTypeDef *hcan, uint32_t identifier, uint8_t *rx_data);
void Dji_Motor_Calc(Dji_Motor_t *motor);
void Dji_Motor_SetLoc(Dji_MotorID_e id, float location);
void Dji_Motor_SetSpeed(Dji_MotorID_e id, float speed);
void Dji_Motor_SetGroupLoc(int group_id, float target_loc);
motor_measure_t Get_dji_information(int motor_id);

void Change_dji_loc(int motor_id, float location);
void Change_dji_speed(int motor_id, float speed);

void Discontrol_dji_motor(void);
void Recontrol_dji_motor(void);

void Dji_3508_all_motor_control(void);
void Dji_Motor_Registry_Init(void);

#define DJI_M_CLIMB_LF DJI_YL
#define DJI_M_CLIMB_RF DJI_YR
#define DJI_M_CLIMB_LB DJI_XL
#define DJI_M_CLIMB_RB DJI_XR
#define DJI_2006_L     DJI_JOINT1_3508
#define DJI_2006_R     DJI_JOINT2_2006

#endif
