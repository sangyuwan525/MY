//
// Created by 马皓然 on 2025/10/30.
//
#include "path.h"

#include <math.h>

#include "FreeRTOS.h"



Path_struct path_test;
path_spd_data_t spd_test ={3000,500,500};

void path_init() {
    Trajectory trajectory_data[4] = {
        //trace                                     //traceType          //point_end                            //point_start                           //length             //ifvoid
        {{1.0f, 0.0f, 0.0f, 0.0f},      line,        {0, 10.0f},          {0.0f, 0.0f},        10.0f,       full},
        {{0.0f, 1.0f, -10.0f, 0.0f},    line,        {10.0f, 10.0f},      {0.0f, 10.0f},       10.0f,       full},
        {{1.0f, 0.0f, 10.0f, 0.0f},     line,        {10.0f, 0.0f},       {10.0f, 10.0f},      10.0f,       full},
        {{0.0f, 1.0f, 0.0f, 0.0f},      line,        {0.0f, 0.0f},        {10.0f, 0.0f},       10.0f,       empty}//此处为空表示该轨迹是这段路程的最后一段}
    };
    //1.计算轨迹段数
    const uint8_t TRAJECTORY_COUNT = sizeof(trajectory_data) / sizeof(Trajectory);
    path_test.trajectory_num = TRAJECTORY_COUNT;

    //2.为轨迹数组动态分配内存,计算总路程和轨迹赋值
    path_test.trajectories = (Trajectory *)pvPortMalloc(sizeof(Trajectory) * TRAJECTORY_COUNT);
    if (path_test.trajectories == NULL)
    {
        // 内存分配失败，应记录错误或执行故障处理（在竞赛中可能直接进入错误状态）
        // 这里仅作示例，实际应用中需有更健壮的错误处理机制
        // HAL_Delay(100);
        return;
    }
    float total_length = 0.0f;
    for (uint8_t i = 0; i < TRAJECTORY_COUNT; i++)
    {
        path_test.trajectories[i] = trajectory_data[i];
        total_length += trajectory_data[i].length;
    }
    path_test.length = total_length;
    //3.给定初始角度和终末角度
    path_test.start_angle = 0.0f;
    path_test.end_angle = -1.5708f;

}

void path_init_test() {
    Trajectory trajectory_data[2] = {
        //trace                                     //traceType          //point_end                            //point_start                           //length             //ifvoid
        {{0.0f, 1.0f, 0.0f, 0.0f},      line,        {-2000.0f, 0.0f},          {0.0f, 0.0f},        2000.0f,       full},
        {{0.0f, 1.0f, 0.0f, 0.0f},    line,        {0.0f, 0.0f},      {-2000.0f, 0.0f},       2000.0f,       empty}
    };
    //1.计算轨迹段数
    const uint8_t TRAJECTORY_COUNT = sizeof(trajectory_data) / sizeof(Trajectory);
    path_test.trajectory_num = TRAJECTORY_COUNT;

    //2.为轨迹数组动态分配内存,计算总路程和轨迹赋值
    path_test.trajectories = (Trajectory *)pvPortMalloc(sizeof(Trajectory) * TRAJECTORY_COUNT);
    if (path_test.trajectories == NULL)
    {
        // 内存分配失败，应记录错误或执行故障处理（在竞赛中可能直接进入错误状态）
        // 这里仅作示例，实际应用中需有更健壮的错误处理机制
        // HAL_Delay(100);
        return;
    }
    float total_length = 0.0f;
    for (uint8_t i = 0; i < TRAJECTORY_COUNT; i++)
    {
        path_test.trajectories[i] = trajectory_data[i];
        total_length += trajectory_data[i].length;
    }
    path_test.length = total_length;
    //3.给定初始角度和终末角度
    path_test.start_angle = 0.0f;
    path_test.end_angle = 0.0f;

}

/**
 * @brief  根据起点和终点生成直线轨迹参数
 * @param  start  路径起点
 * @param  end    路径终点
 * @param  is_end 是否为整条路径的最后一段 (empty 为最后一段, full 为中间段)
 * @return Trajectory 返回填充好的轨迹结构体
 */
Trajectory generate_line_trajectory(Point_struct start, Point_struct end, Ifvoid is_end) {
    Trajectory tra;

    // 1. 设置起终点和类型
    tra.point_start = start;
    tra.point_end = end;
    tra.traceType = line;
    tra.ifvoid = is_end;

    // 2. 计算直线方程参数 Ax + By + C = 0
    // 一般式方程推导: (y1 - y2)x + (x2 - x1)y + (x1y2 - x2y1) = 0
    float A = start.y - end.y;
    float B = end.x - start.x;
    float C = start.x * end.y - end.x * start.y;

    tra.trace[Line_A] = A;
    tra.trace[Line_B] = B;
    tra.trace[Line_C] = C;
    tra.trace[3] = 0.0f; // 占位

    // 3. 计算长度
    tra.length = vec_module(end.x - start.x, end.y - start.y);

    return tra;
}

/**
 * @brief  初始化一个简单的两点直线路径
 * @param  p_path 指向路径结构体的指针
 * @param  start  起点坐标
 * @param  end    终点坐标
 * @param  start_angle  起点角度
 * @param  end_angle    终点角度
 */
void init_single_line_path(Path_struct* p_path, Point_struct start, Point_struct end, float start_angle, float end_angle) {
    // --- 新增：安全释放旧内存 ---
    if (p_path->trajectories != NULL) {
        vPortFree(p_path->trajectories);
        p_path->trajectories = NULL; // 置空防止误操作
    }

    // 1. 设置路径基本信息
    p_path->trajectory_num = 1;
    p_path->trajectory_count = 0;
    p_path->start_angle = start_angle;
    p_path->end_angle = end_angle;

    // 2. 分配新内存
    p_path->trajectories = (Trajectory *)pvPortMalloc(sizeof(Trajectory) * p_path->trajectory_num);

    if (p_path->trajectories != NULL) {
        // 3. 生成轨迹段
        p_path->trajectories[0] = generate_line_trajectory(start, end, empty);
        // 4. 更新路径总长度
        p_path->length = p_path->trajectories[0].length;
    } else {
        // 异常处理：内存分配失败
        p_path->trajectory_num = 0;
    }
}