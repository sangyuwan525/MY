//
// Created by 马皓然 on 2025/10/30.
//

#ifndef R1_CHASSIS_PATH_H
#define R1_CHASSIS_PATH_H

#include "chassis_path.h"
extern Path_struct path_test;
extern path_spd_data_t spd_test;

void path_init_test();
void init_single_line_path(Path_struct* p_path, Point_struct start, Point_struct end, float start_angle, float end_angle);

#endif //R1_CHASSIS_PATH_H