//
// Created by 马皓然 on 2026/2/2.
//

#ifndef R1_SUPERSTRUCTURE_GLOBAL_MOTOR_CONF_H
#define R1_SUPERSTRUCTURE_GLOBAL_MOTOR_CONF_H
// --- 1. 定义所有电机的全局唯一索引 ---
typedef enum {
    // DJI 电机组
    DJI_YL_G = 0,
    DJI_YR_G,
    DJI_XL_G,
    DJI_XR_G,
    DJI_JOINT1_3508_G,
    DJI_JOINT2_2006_G,
    // 强制让达妙的索引接在 DJI 后面，不要重新从 0 开始
    DM_JOINT_G,
    // DM_JOINT2,
} Global_Motor_Index_e;
//当使用最顶层封装g_motor_list[].set_position(&g_motor_list[], angle,vel);时，注意使用全局索引_G
#endif //R1_SUPERSTRUCTURE_GLOBAL_MOTOR_CONF_H