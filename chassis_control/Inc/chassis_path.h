//
// Created by 马皓然 on 2025/10/30.
//

#ifndef R1_CHASSIS_CHASSIS_PATH_H
#define R1_CHASSIS_CHASSIS_PATH_H

#include "main.h"
/* Definitions ---------------------------------------------------------------*/
// 轨迹信息数组 trace[4] 的索引宏定义
#define Line_A 0
#define Line_B 1
#define Line_C 2
#define center_x 0
#define center_y 1
#define circle_r 2
#define circle_angle 3
#define pi 3.1415926535
/**
 * @brief  计算二维向量的模长。
 * @note   这是一个宏定义，用于避免函数调用的开销，但应注意宏的副作用。
 */
#define vec_module(x,y) (sqrtf((x)*(x) + (y)*(y)))


/* Structs -------------------------------------------------------------------*/
/**
 * @brief 轨迹类型枚举
 */
typedef enum {
    line = 0,
    circle
}trace_type;
/**
 * @brief 轨迹结构体是否为轨迹最后一段枚举
 */
typedef enum {
    empty = 0,
    full
}Ifvoid;

/**
 * @brief 2D 坐标点结构体
 */
typedef struct {
    float x;
    float y;
}Point_struct;//点类型
/**
 * @brief 直线参数结构体 (Ax + By + C = 0)
 */
typedef struct {
    float a;
    float b;
    float c;
}Line_struct;//线类型
/**
 * @brief 2D 向量结构体
 */
typedef struct {
    float x;
    float y;
}vec2;
/**
 * @brief 核心轨迹结构体
 */
typedef struct {
    float trace[4];//包装轨迹信息
    //若轨迹类型为直线，则trace[0],trace[1]，trace[2]分别存储Ax + By + C = 0的ABC值,其余为0
    //若轨迹类型为圆弧，则trace[0],trace[1],trace[2],trace[3]分别存储圆心坐标x,y及半径r圆心角angle
    trace_type traceType;
    //用于区分轨迹类型
    Point_struct point_end;//记录轨迹终点坐标
    Point_struct point_start;//记录轨迹起点坐标
    float length;
    //记录轨迹长度
    Ifvoid ifvoid;
    //用于区分轨迹是否为最后一段
}Trajectory;
/**
 * @brief 完整路径结构体
 */
typedef struct {
    float start_angle;
    //记录轨迹起点处的朝向角
    float end_angle;
    //记录轨迹终点处的朝向角
    float length;
    //记录该段轨迹的长度
    uint8_t trajectory_num;
    //记录该路径包含的轨迹段数
    uint8_t trajectory_count;
    //记录当前正在跟踪的轨迹段索引
    Trajectory *trajectories;
    //指向该路径所包含的轨迹段数组
}Path_struct;

/**
 * @brief 路径速度规划参数结构体
 * @note 用于封装 S 型曲线速度分配所需的参数。
 */
typedef struct {
    float max_speed;    ///< 路径段中允许的最大速度
    float up_stage;     ///< 曲线加速阶段对应的路径长度
    float down_stage;   ///< 曲线减速阶段对应的路径长度
} path_spd_data_t;



#endif //R1_CHASSIS_CHASSIS_PATH_H