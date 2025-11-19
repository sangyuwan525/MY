//
// Created by 马皓然 on 2025/11/5.
//

#ifndef R1_CHASSIS_LOCATOR_DRIVER_H
#define R1_CHASSIS_LOCATOR_DRIVER_H
/**
 * @brief 激光雷达定位结果结构体
 */
typedef struct LocatorResult
{
    float x, y, r;   ///< 位置和朝向 (世界坐标系)
    float vx, vy, vr; ///< 速度分量
    float pitch, roll,yaw; ///< 俯仰角、横滚角、偏航角
} Locator_Result_t;

extern Locator_Result_t lcResult;
#endif //R1_CHASSIS_LOCATOR_DRIVER_H