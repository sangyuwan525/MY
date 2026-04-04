//
// Created by lcf on 2025/10/30.
//

#ifndef R1_CHASSIS_PATH_H
#define R1_CHASSIS_PATH_H

#include "chassis_path.h"

extern Path_struct path_test;
extern path_spd_data_t spd_test;

void path_init_test();

// Trajectory generators.
Trajectory generate_line_trajectory(Point_struct start, Point_struct end, Ifvoid is_end);
Trajectory generate_circle_trajectory(Point_struct start, Point_struct end, Point_struct center, float central_angle, Ifvoid is_end);
Trajectory generate_bezier_trajectory_segment(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, float t0, float t1, Ifvoid is_end);

// Single-segment paths.
void init_single_line_path(Path_struct* p_path, Point_struct start, Point_struct end, float start_angle, float end_angle);
void init_single_circle_path(Path_struct* p_path, Point_struct start, Point_struct end, Point_struct center, float central_angle, float start_angle, float end_angle);
void init_single_bezier_path(Path_struct* p_path, Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, uint8_t segment_count, float start_angle, float end_angle);

// Multi-segment paths.
void init_line_circle_path(Path_struct* p_path, Point_struct line_start, Point_struct line_end_arc_start, Point_struct arc_end, Point_struct center, float central_angle, float start_angle, float end_angle);
void init_tangent_line_circle_path(Path_struct* p_path, Point_struct line_start, Point_struct arc_end, Point_struct center, uint8_t arc_ccw, float start_angle, float end_angle);

// 构建 R2 在“非树林区可达区域”内的路径。
// 当前支持的区域包括：一区(MC)、二区入口区、二区出口区、三区(CF)。
// 该接口只负责这些大区域之间的几何路径拼接，不负责树林内部的方块搜索路径。
int build_r2_accessible_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle);

typedef struct {
    // 端头架抓取位：R2 最终执行抓取动作时应到达的位置。
    Point_struct grab_point;
    // 等待位：从抓取位前方/侧方预留的稳定接近点，用来减小直接冲向端头架的风险。
    Point_struct wait_point;
    // 回撤位：抓取完成后先退回的安全点，避免端头架附近掉头或横移过大。
    Point_struct retreat_point;
    // 抓取位、等待位、回撤位分别期望的车体朝向。
    float grab_angle;
    float wait_angle;
    float retreat_angle;
    // 该组端头架路径默认使用的最小转弯半径。
    float turn_radius;
} R2_MC_HeadRouteConfig;

extern R2_MC_HeadRouteConfig g_r2_mc_head_route_cfg;

// 更新端头架路径的参数配置。
// 适合在比赛前根据实测场地坐标重新写入 grab / wait / retreat 三个点。
void set_r2_mc_head_route_config(const R2_MC_HeadRouteConfig* config);

// 一区端头架相关业务路径：
// 1. 从任意起点走到“等待位 -> 抓取位”
int build_r2_head_grab_path(Path_struct* p_path, Point_struct start);
// 2. 从任意起点稳定走到等待位
int build_r2_head_wait_path(Path_struct* p_path, Point_struct start);
// 3. 从抓取位离开时，先回撤再去终点
int build_r2_head_retreat_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle);

// 业务层统一入口：
// 上层只需要给“起点、终点、终点角度”，该函数内部会自动判断应该走：
// 1. 端头架抓取路径
// 2. 端头架等待路径
// 3. 端头架抓取后的回撤路径
// 4. 普通非树林区路径（直线 / 圆弧 / 经过坡道的多段拼接）
int build_r2_business_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle);

#endif //R1_CHASSIS_PATH_H
