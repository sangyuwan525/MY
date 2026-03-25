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

// Single-segment paths.
void init_single_line_path(Path_struct* p_path, Point_struct start, Point_struct end, float start_angle, float end_angle);
void init_single_circle_path(Path_struct* p_path, Point_struct start, Point_struct end, Point_struct center, float central_angle, float start_angle, float end_angle);

// Multi-segment paths.
void init_line_circle_path(Path_struct* p_path, Point_struct line_start, Point_struct line_end_arc_start, Point_struct arc_end, Point_struct center, float central_angle, float start_angle, float end_angle);
void init_tangent_line_circle_path(Path_struct* p_path, Point_struct line_start, Point_struct arc_end, Point_struct center, uint8_t arc_ccw, float start_angle, float end_angle);

// Build an R2 route in the accessible non-forest areas:
// MC, MF entry/exit zones, and CF.
int build_r2_accessible_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle);

typedef struct {
    Point_struct grab_point;
    Point_struct wait_point;
    Point_struct retreat_point;
    float grab_angle;
    float wait_angle;
    float retreat_angle;
    float turn_radius;
} R2_MC_HeadRouteConfig;

extern R2_MC_HeadRouteConfig g_r2_mc_head_route_cfg;

void set_r2_mc_head_route_config(const R2_MC_HeadRouteConfig* config);
int build_r2_head_grab_path(Path_struct* p_path, Point_struct start);
int build_r2_head_wait_path(Path_struct* p_path, Point_struct start);
int build_r2_head_retreat_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle);

// Business-layer route builder:
// input only start / end / end_angle, and internally choose line / arc / multi-segment path.
int build_r2_business_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle);

#endif //R1_CHASSIS_PATH_H
