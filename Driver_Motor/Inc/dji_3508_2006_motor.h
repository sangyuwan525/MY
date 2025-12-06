#ifndef DJI_3508_2006_MOTOR_H
#define DJI_3508_2006_MOTOR_H

/**********************************************
pid.c中的pid参数及pid计算
can_database.c中的回调
**********************************************/

#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include "pid.h"

// #define SPEED_MODE 0
// #define LOC_MODE 1

enum {
	SPEED_MODE = 0,
	LOC_MODE = 1,
	GROUP_MODE = 2,
};

enum {
	FDCNA1=0,
	FDCNA2,
	FDCNA3,
};

// 统一的电机索引ID：使用枚举来映射到注册表数组的索引，增加可读性
typedef enum
{
	DJI_M_CHASSIS_LF = 0, // 左前底盘电机 (假设索引 0)
	DJI_M_CHASSIS_LB,	  // 左后底盘电机 (假设索引 1)
	DJI_M_CHASSIS_RF,     // 右前底盘电机 (假设索引 2)
	DJI_M_CHASSIS_RB,     // 右后底盘电机 (假设索引 3)
	DJI_M_CLIMB_LF,	//4
	DJI_M_CLIMB_RF,//5
	DJI_M_CLIMB_LB,//6
	DJI_M_CLIMB_RB,
	DJI_2006_L,//
	DJI_2006_R,//
	DJI_MOTOR_COUNT
} Dji_MotorID_e;

typedef struct {
	FDCAN_HandleTypeDef *hcan;
	int16_t currents_0x200[4];
	int16_t currents_0x1FF[4];
	bool need_to_send;
} Can_Tx_Buffer_t;



/* CAN send and receive ID */
typedef enum{
	CAN_FIRST_FOUR_MOTOR_ALL_ID = 0x200,//dji系列电机号
	CAN_LAST_FOUR_MOTOR_ALL_ID = 0x1FF,

	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	CAN_3508_M3_ID = 0x203,
	CAN_3508_M4_ID = 0x204,
	CAN_3508_M5_ID = 0x205,
	CAN_3508_M6_ID = 0x206,
	CAN_3508_M7_ID = 0x207,
	CAN_3508_M8_ID = 0x208,
} can_msg_id_e;
// =========================================================================
// !!!!!! 用户配置区：电机同步组配置 !!!!!!
// =========================================================================
// 定义电机同步组：将所有需要保持同步的电机ID放入此数组中。
// 例如：如果要让电机ID 1, 3, 5, 7同步，则配置为 {1, 3, 5, 7, 0}
// 数组末尾必须以 0 结束，0 不会参与同步计算。
#define MAX_SYNC_MOTORS_PER_GROUP 8
extern const uint8_t SYNC_GROUP_IDS[MAX_SYNC_MOTORS_PER_GROUP];
#define MAX_CAN_HANDLES 3
// 电机反馈结构体
typedef struct{
	uint16_t angle;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	int16_t last_angle;
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint16_t	fited_angle;
	uint32_t	msg_cnt;
	uint16_t  first  ; //用于判断电调是否为第一次发送信号
} motor_measure_t;
// 定义一个结构体，用于在中断和任务之间传递电机反馈数据
typedef struct
{
	uint32_t motor_id;          // 电机ID (CAN ID)
	uint8_t  rx_data[8];        // 原始CAN数据
	FDCAN_HandleTypeDef* hcan;  // 指示报文来自哪个 CAN 口
} Motor_Rx_Queue_t;

// **新的电机注册表结构体**
typedef struct
{
	// 硬件配置 (用于CAN发送和接收匹配)
	FDCAN_HandleTypeDef* hcan_tx;       // CAN发送句柄：例如 &hfdcan1
	uint32_t             can_rx_id;     // CAN接收ID：0x201 - 0x208
	uint32_t             can_tx_header_id; // CAN发送报文ID：0x200 或 0x1FF
	uint8_t              tx_index;      // 报文中的索引：0-3 (对应报文数据 0-7)
	// 控制目标
	int32_t              target_loc;    // 目标位置
	int32_t              target_spd;    // 目标速度 (RPM)
	uint8_t              control_mode;  // 控制模式：SPEED_MODE/LOC_MODE/GROUP_MODE
	// 状态反馈
	motor_measure_t      feedback;      // 电机反馈信息
	// 控制器
	motor_pid_parameter  pid_params;
	// 运行时状态
	int16_t              current_set;   // 最终计算出的电流控制量
	bool                 is_enabled;    // 是否启用控制
	bool                 is_online;     // 是否在线 (通过反馈消息计数判断)
} Dji_Motor_t;

/**************USER_begin**************/
extern int set_loc_s[9];
extern osMessageQueueId_t motorRxQueueHandle;
// 声明统一的电机注册表
extern motor_pid_parameter motor_3508_pid_g[DJI_MOTOR_COUNT];
extern Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT];
// static motor_measure_t motor_inf[9];/*3508电机参数*/
void Dji_3508_first_four_motor_control(int i,uint8_t rx_data[8]);//使用can3
void Dji_3508_last_four_motor_control(int i,uint8_t rx_data[8]);//使用can1

void Dji_3508_first_motor_control(int i,uint8_t rx_data[8]);//使用can2，此处can2可同时控制前四号电机和后四号电机，且可控制9号电机，用6号代替（i=5，灯闪六下）
void Dji_3508_last_motor_control(int i,uint8_t rx_data[8]);

void Change_dji_speed(int motor_id,int target_spd);
void Change_dji_loc(int motor_id,int target_loc);
motor_measure_t Get_dji_information(int motor_id);
void Discontrol_dji_motor(void);
void Recontrol_dji_motor(void);

void Dji_Motor_Registry_Init(void);
void Dji_3508_all_motor_control(void);
void Dji_Motor_Update_Status(FDCAN_HandleTypeDef* hcan_rx, uint32_t id, uint8_t *data);
/**************USER_end**************/

#endif
