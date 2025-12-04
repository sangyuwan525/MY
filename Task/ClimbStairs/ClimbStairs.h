//
// Created by lcf on 2025/12/1.
//

#ifndef R2_CHASSIS_CLIMBSTAIRS_H
#define R2_CHASSIS_CLIMBSTAIRS_H

#include "gpio.h"
#include "dji_3508_2006_motor.h"

//气缸电磁阀端口定义
#define valve_port GPIOA
#define valve_pin_l GPIO_PIN_1//左气缸 sync
#define valve_pin_r GPIO_PIN_2//右气缸 sync

//上下台阶所用的5个电机
#define front_left_motor_id 0//3508 sync
#define front_right_motor_id 1//3508 sync
#define back_motor_id 2//3508
#define wheel_left_motor_id 3//2006 sync
#define wheel_right_motor_id 4//2006 sync

//信号量（标志位）
int upstairs_flag;

//上楼梯所用距离
#define front_

//上下楼梯的函数
void ClimbStairs(void);

#endif //R2_CHASSIS_CLIMBSTAIRS_H