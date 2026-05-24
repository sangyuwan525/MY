#ifndef __DM_MOTOR_CTRL_H__
#define __DM_MOTOR_CTRL_H__
#include "main.h"
#include "dm_motor_drv.h"
typedef enum
{
	DM_Motor1 = 0,
    DM_Motor2,
 //    Motor3,
 //    Motor4,
 //    Motor5,
 //    Motor6,
	// Motor7,
	// Motor8,
	// Motor9,
	// Motor10,
	DM_MOTOR_COUNT
} dm_motor_num;

typedef enum
{
	DM_Motor1_CAN_ID = 0x01,
	DM_Motor2_CAN_ID = 0x02,
} dm_motor_can_id_e;

typedef enum
{
	DM_Motor1_MST_ID = 0x11,
	DM_Motor2_MST_ID = 0x12,
} dm_motor_mst_id_e;

typedef union
{
	float f_val;
	uint32_t u_val;
	uint8_t b_val[4];

}float_type_u;

extern int8_t motor_id;

extern uint32_t motor1_data_sent;
extern uint32_t motor2_data_sent;
extern uint32_t motor3_data_sent;
extern uint32_t motor4_data_sent;

extern Damiao_Motor_t g_dm_motor_registry[DM_MOTOR_COUNT];



void dm_motor_init(void);

void read_all_motor_data(Damiao_Motor_t *motor);
void receive_motor_data(Damiao_Motor_t *motor, uint8_t *data);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);

#endif /* __DM_MOTOR_CTRL_H__ */
