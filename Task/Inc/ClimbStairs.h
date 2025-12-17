//
// Created by lcf on 2025/12/1.
//

#ifndef R2_CHASSIS_CLIMBSTAIRS_H
#define R2_CHASSIS_CLIMBSTAIRS_H

#include "gpio.h"
#include "dji_3508_2006_motor.h"
#include "tgmath.h"

//气缸电磁阀端口定义
#define valve_port GPIOC
#define valve_pin_l GPIO_PIN_4//左气缸 sync
#define valve_pin_r GPIO_PIN_5//右气缸 sync

//上下台阶所用的5个电机
#define front_left_motor_id 0//3508 sync
#define front_right_motor_id 1//3508 sync
#define back_motor_id 2//3508
#define wheel_left_motor_id 3//2006 sync
#define wheel_right_motor_id 4//2006 sync

// 攀爬状态枚举
typedef enum
{
    CLIMB_IDLE = 0,             // 初始/空闲状态
    CLIMB_STEP1_FRONT_UP,       // 第一步：前侧3508抬升 (对应原按钮1)
    CLIMB_STEP2_BASE_FORWARD,   // 第二步：底盘向前移动 (对应原图步骤3)
    CLIMB_STEP3_LIFT_UP,        // 第三步：3508抬升车身 (对应原按钮5的变体)
    CLIMB_STEP4_REAR_FORWARD,   // 第四步：后侧2006推动 (对应原按钮3)
    CLIMB_STEP5_RESET_ALL,      // 第五步：所有电机归位 (对应原按钮2)
    CLIMB_COMPLETE              // 攀爬完成
} Climb_State_e;

//信号量（标志位）
//int upstairs_flag;

//上楼梯所用距离
#define front_up 300000

#define back_up 305000
#define front_up2 (-20000)

#define CYLINDER_GPIO_PORT GPIOC
#define CYLINDER_PIN1 GPIO_PIN_5
#define CYLINDER_PIN2 GPIO_PIN_4
extern int climb_cnt;

//上下楼梯的函数
void ClimbStairs(void);

#endif //R2_CHASSIS_CLIMBSTAIRS_H