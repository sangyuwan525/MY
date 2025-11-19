//
// Created by 马皓然 on 2025/10/30.
//
#include "path.h"
#include "FreeRTOS.h"



Path_struct path_test;

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
