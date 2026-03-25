//
#include "path.h"

#include <math.h>

#include "FreeRTOS.h"
#include "locator_driver.h"
#include "ClimbStairs.h"

Path_struct path_test;
path_spd_data_t spd_test = {3000, 500, 500};

/**
 * @brief 准备/初始化路径存储空间
 * @param p_path 指向路径结构体的指针
 * @param trajectory_num 期望分配的轨迹段数量
 * @return int 0:成功; -1:参数错误或内存分配失败
 */
static int prepare_path_storage(Path_struct *p_path, uint8_t trajectory_num) {
    // 所有路径初始化统一走这里：避免重复代码，也保证先释放旧内存再分配新内存。
    if (p_path == NULL) {
        return -1;
    }

    // 1. 安全检查：如果之前已经分配过内存，先释放，防止内存泄漏
    if (p_path->trajectories != NULL) {
        vPortFree(p_path->trajectories);
        p_path->trajectories = NULL;
    }

    // 2. 重置路径元数据
    p_path->trajectory_num = trajectory_num;
    p_path->trajectory_count = 0;
    p_path->length = 0.0f;

    // 3. 如果请求数量为0，视为仅清空操作，直接返回
    if (trajectory_num == 0) {
        return 0;
    }

    // 4. 使用 RTOS 的内存分配函数分配轨迹段空间
    p_path->trajectories = (Trajectory *)pvPortMalloc(sizeof(Trajectory) * trajectory_num);

    // 5. 检查分配是否成功
    if (p_path->trajectories == NULL) {
        p_path->trajectory_num = 0;// 分配失败需同步更新数量
        return -1;
    }

    return 0;
}

/**
 * @brief 将弧度值归一化到 [0, 2pi) 区间,将任何角度（弧度制）转换到 [0, 2pi) 范围内，这是几何计算中处理旋转方向的前提。
 * @param angle 原始角度
 * @return float 归一化后的正角度
 */
static float normalize_angle_positive(float angle) {
    // 统一把角度归一到 [0, 2pi)，便于后续按顺/逆时针计算弧长。
    while (angle < 0.0f) {
        angle += 2.0f * pi;
    }
    while (angle >= 2.0f * pi) {
        angle -= 2.0f * pi;
    }
    return angle;
}

/**
 * @brief 根据旋转方向计算圆弧跨越的圆心角
 * @param start 起点坐标
 * @param end 终点坐标
 * @param center 圆心坐标
 * @param arc_ccw 是否为逆时针 (1:逆时针, 0:顺时针)
 * @return float 跨越的角度 (弧度)
 */
static float get_arc_angle_by_direction(Point_struct start, Point_struct end, Point_struct center, uint8_t arc_ccw) {
    // 1. 计算起点和终点相对于圆心的极角 (使用 atan2 处理全象限)
    const float start_angle = atan2f(start.y - center.y, start.x - center.x);
    const float end_angle = atan2f(end.y - center.y, end.x - center.x);
    // 2. 计算基础的逆时针增量角度
    const float ccw_delta = normalize_angle_positive(end_angle - start_angle);

    // 3. 如果要求逆时针，直接返回增量
    if (arc_ccw) {
        return ccw_delta;
    }
    
    // 4. 如果要求顺时针：用 2pi 减去逆时针增量。
    // 特殊处理：若增量接近0，说明是一整圈 (2pi) 而不是 0
    return (ccw_delta < 1e-6f) ? 2.0f * pi : (2.0f * pi - ccw_delta);
}

/**
 * @brief 在圆外一点向圆作切线，并根据旋转方向选择合适的切点这是最复杂的几何逻辑。当一条直线需要平滑地“滑入”一个圆弧时，
 * 从圆外一点出发会有两条几何切线。该函数根据轨迹的连续性（即直线方向必须与圆弧在切点处的切线方向一致）选出唯一的有效切点。
 * @param line_start 直线起点 (圆外)
 * @param arc_end 圆弧终点
 * @param center 圆心
 * @param arc_ccw 期望的圆弧方向 (1:CCW, 0:CW)
 * @param tangent_point [输出] 选定的切点坐标
 * @param central_angle [输出] 从切点到圆弧终点的圆心角
 * @return int 0:找到有效切点; -1:失败(点在圆内或无法满足连续性)
 */
static int select_tangent_point(Point_struct line_start, Point_struct arc_end, Point_struct center, uint8_t arc_ccw, Point_struct *tangent_point, float *central_angle) {
    // 1. 计算几何基本参数：半径、起点到圆心的距离
    const float radius = vec_module(arc_end.x - center.x, arc_end.y - center.y);
    const float vx = line_start.x - center.x;
    const float vy = line_start.y - center.y;
    const float d2 = vx * vx + vy * vy;
    const float r2 = radius * radius;
    float base;
    float factor;
    vec2 perp;
    Point_struct candidates[2];

    // d2<=r2 说明起点在圆上或圆内，无有效切线。
    if (radius < 1e-6f || d2 <= r2 + 1e-6f) {
        return -1;
    }

    base = r2 / d2;
    factor = radius * sqrtf(d2 - r2) / d2;
    perp.x = -vy;
    perp.y = vx;

    candidates[0].x = center.x + base * vx + factor * perp.x;
    candidates[0].y = center.y + base * vy + factor * perp.y;
    candidates[1].x = center.x + base * vx - factor * perp.x;
    candidates[1].y = center.y + base * vy - factor * perp.y;

    for (uint8_t i = 0; i < 2; i++) {
        vec2 line_dir;
        vec2 radius_dir;
        vec2 arc_tangent_dir;
        float line_norm;
        float tangent_norm;
        float dot;

        line_dir.x = candidates[i].x - line_start.x;
        line_dir.y = candidates[i].y - line_start.y;
        radius_dir.x = candidates[i].x - center.x;
        radius_dir.y = candidates[i].y - center.y;

        if (arc_ccw) {
            arc_tangent_dir.x = -radius_dir.y;
            arc_tangent_dir.y = radius_dir.x;
        } else {
            arc_tangent_dir.x = radius_dir.y;
            arc_tangent_dir.y = -radius_dir.x;
        }

        line_norm = vec_module(line_dir.x, line_dir.y);
        tangent_norm = vec_module(arc_tangent_dir.x, arc_tangent_dir.y);
        if (line_norm < 1e-6f || tangent_norm < 1e-6f) {
            continue;
        }

        line_dir.x /= line_norm;
        line_dir.y /= line_norm;
        arc_tangent_dir.x /= tangent_norm;
        arc_tangent_dir.y /= tangent_norm;
        dot = line_dir.x * arc_tangent_dir.x + line_dir.y * arc_tangent_dir.y;

        // dot 接近 1 表示直线方向与圆弧切向几乎同向，可视为满足相切且方向连续。
        if (dot > 0.99f) {
            *tangent_point = candidates[i];
            *central_angle = get_arc_angle_by_direction(candidates[i], arc_end, center, arc_ccw);
            return 0;
        }
    }

    return -1;
}

void path_init() {
    Trajectory trajectory_data[4] = {
        // trace                      traceType  point_end        point_start      length  ifvoid
        {{1.0f, 0.0f, 0.0f, 0.0f},    line,      {0, 10.0f},     {0.0f, 0.0f},   10.0f,  full},
        {{0.0f, 1.0f, -10.0f, 0.0f},  line,      {10.0f, 10.0f}, {0.0f, 10.0f},  10.0f,  full},
        {{1.0f, 0.0f, 10.0f, 0.0f},   line,      {10.0f, 0.0f},  {10.0f, 10.0f}, 10.0f,  full},
        {{0.0f, 1.0f, 0.0f, 0.0f},    line,      {0.0f, 0.0f},   {10.0f, 0.0f},  10.0f,  empty}
    };

    const uint8_t TRAJECTORY_COUNT = sizeof(trajectory_data) / sizeof(Trajectory);
    path_test.trajectory_num = TRAJECTORY_COUNT;

    if (prepare_path_storage(&path_test, TRAJECTORY_COUNT) != 0) {
        return;
    }

    float total_length = 0.0f;
    for (uint8_t i = 0; i < TRAJECTORY_COUNT; i++) {
        path_test.trajectories[i] = trajectory_data[i];
        total_length += trajectory_data[i].length;
    }

    path_test.length = total_length;
    path_test.start_angle = 0.0f;
    path_test.end_angle = -1.5708f;
}

void path_init_test() {
    Trajectory trajectory_data[2] = {
        // trace                    traceType  point_end         point_start       length   ifvoid
        {{0.0f, 1.0f, 0.0f, 0.0f},  line,      {-2000.0f, 0.0f}, {0.0f, 0.0f},    2000.0f, full},
        {{0.0f, 1.0f, 0.0f, 0.0f},  line,      {0.0f, 0.0f},     {-2000.0f, 0.0f}, 2000.0f, empty}
    };

    const uint8_t TRAJECTORY_COUNT = sizeof(trajectory_data) / sizeof(Trajectory);
    path_test.trajectory_num = TRAJECTORY_COUNT;

    if (prepare_path_storage(&path_test, TRAJECTORY_COUNT) != 0) {
        return;
    }

    float total_length = 0.0f;
    for (uint8_t i = 0; i < TRAJECTORY_COUNT; i++) {
        path_test.trajectories[i] = trajectory_data[i];
        total_length += trajectory_data[i].length;
    }

    path_test.length = total_length;
    path_test.start_angle = 0.0f;
    path_test.end_angle = 0.0f;
}

/**
 * @brief Generate line trajectory data from start and end points.
 */
Trajectory generate_line_trajectory(Point_struct start, Point_struct end, Ifvoid is_end) {
    Trajectory tra;
    float A = start.y - end.y;
    float B = end.x - start.x;
    float C = start.x * end.y - end.x * start.y;

    tra.point_start = start;
    tra.point_end = end;
    tra.traceType = line;
    tra.ifvoid = is_end;

    tra.trace[Line_A] = A;
    tra.trace[Line_B] = B;
    tra.trace[Line_C] = C;
    tra.trace[3] = 0.0f;
    tra.length = vec_module(end.x - start.x, end.y - start.y);

    return tra;
}

/**
 * @brief Generate arc trajectory data from start/end/center and central angle.
 * @note  central_angle is stored as a positive value in (0, 2*pi].
 */
Trajectory generate_circle_trajectory(Point_struct start, Point_struct end, Point_struct center, float central_angle, Ifvoid is_end) {
    Trajectory tra;
    float radius = vec_module(start.x - center.x, start.y - center.y);

    tra.point_start = start;
    tra.point_end = end;
    tra.traceType = circle;
    tra.ifvoid = is_end;

    // 起点异常时回退到终点半径，避免出现 0 半径导致控制端计算异常。
    if (radius < 1e-6f) {
        radius = vec_module(end.x - center.x, end.y - center.y);
    }

    if (central_angle < 0.0f) {
        central_angle = -central_angle;
    }
    if (central_angle > 2.0f * pi) {
        central_angle = fmodf(central_angle, 2.0f * pi);
        if (central_angle < 1e-6f) {
            central_angle = 2.0f * pi;
        }
    }

    tra.trace[center_x] = center.x;
    tra.trace[center_y] = center.y;
    tra.trace[circle_r] = radius;
    tra.trace[circle_angle] = central_angle;
    tra.length = radius * central_angle;

    return tra;
}

/**
 * @brief Initialize a single-segment line path.
 */
void init_single_line_path(Path_struct *p_path, Point_struct start, Point_struct end, float start_angle, float end_angle) {
    if (prepare_path_storage(p_path, 1) != 0) {
        return;
    }

    p_path->start_angle = start_angle;
    p_path->end_angle = end_angle;

    p_path->trajectories[0] = generate_line_trajectory(start, end, empty);
    p_path->length = p_path->trajectories[0].length;
}

/**
 * @brief 初始化单段圆弧路径。
 * @note  圆弧方向由 central_angle 的正值语义和跟踪器内部方向判定共同决定。
 */
void init_single_circle_path(Path_struct *p_path, Point_struct start, Point_struct end, Point_struct center, float central_angle, float start_angle, float end_angle) {
    if (prepare_path_storage(p_path, 1) != 0) {
        return;
    }

    p_path->start_angle = start_angle;
    p_path->end_angle = end_angle;

    p_path->trajectories[0] = generate_circle_trajectory(start, end, center, central_angle, empty);
    p_path->length = p_path->trajectories[0].length;
}

/**
 * @brief 初始化“直线 + 圆弧”两段路径（手工给拼接点）。
 * @note  此函数只保证几何连接，不自动保证切线连续。
 */
void init_line_circle_path(Path_struct *p_path, Point_struct line_start, Point_struct line_end_arc_start, Point_struct arc_end, Point_struct center, float central_angle, float start_angle, float end_angle) {
    if (prepare_path_storage(p_path, 2) != 0) {
        return;
    }

    p_path->start_angle = start_angle;
    p_path->end_angle = end_angle;

    p_path->trajectories[0] = generate_line_trajectory(line_start, line_end_arc_start, full);
    p_path->trajectories[1] = generate_circle_trajectory(line_end_arc_start, arc_end, center, central_angle, empty);
    p_path->length = p_path->trajectories[0].length + p_path->trajectories[1].length;
}

/**
 * @brief 初始化“直线 + 圆弧”自动相切路径。
 * @param arc_ccw 1=逆时针圆弧，0=顺时针圆弧。
 * @note  自动求切点失败时，会把路径置为空路径状态。
 */
void init_tangent_line_circle_path(Path_struct *p_path, Point_struct line_start, Point_struct arc_end, Point_struct center, uint8_t arc_ccw, float start_angle, float end_angle) {
    Point_struct tangent_point = {0.0f, 0.0f};
    float central_angle = 0.0f;

    if (prepare_path_storage(p_path, 2) != 0) {
        return;
    }

    if (select_tangent_point(line_start, arc_end, center, arc_ccw, &tangent_point, &central_angle) != 0) {
        // 自动求切点失败时恢复到“空路径”状态，避免保留半初始化结构造成后续误用。
        vPortFree(p_path->trajectories);
        p_path->trajectories = NULL;
        p_path->trajectory_num = 0;
        p_path->trajectory_count = 0;
        p_path->length = 0.0f;
        return;
    }

    p_path->start_angle = start_angle;
    p_path->end_angle = end_angle;
    p_path->trajectories[0] = generate_line_trajectory(line_start, tangent_point, full);
    p_path->trajectories[1] = generate_circle_trajectory(tangent_point, arc_end, center, central_angle, empty);
    p_path->length = p_path->trajectories[0].length + p_path->trajectories[1].length;
}

typedef enum {
    R2_REGION_INVALID = 0,
    R2_REGION_MC,
    R2_REGION_MF_ENTRY,
    R2_REGION_MF_EXIT,
    R2_REGION_CF
} R2_Accessible_Region;

#define R2_MIN_TURN_RADIUS_MM 650.0f
#define R2_START_ANGLE_NEAR_MM 150.0f
#define R2_ENTRY_ZONE_MAX_Y_MM 3000.0f
#define R2_EXIT_ZONE_MIN_Y_MM 7200.0f
#define R2_CF_ZONE_MIN_Y_MM 7800.0f
#define R2_HEAD_NEAR_THRESHOLD_MM 120.0f

R2_MC_HeadRouteConfig g_r2_mc_head_route_cfg = {
    // These are conservative defaults in MC only.
    // Replace them with measured head-rack grab/wait/retreat points in your field coordinates.
    .grab_point = {1490.0f, 900.0f},
    .wait_point = {1490.0f, 1250.0f},
    .retreat_point = {1490.0f, 1600.0f},
    .grab_angle = 0.0f,
    .wait_angle = 0.0f,
    .retreat_angle = 0.0f,
    .turn_radius = 650.0f
};

static float wrap_angle_pi_local(float angle) {
    while (angle > pi) {
        angle -= 2.0f * pi;
    }
    while (angle < -pi) {
        angle += 2.0f * pi;
    }
    return angle;
}

static vec2 yaw_to_dir(float yaw) {
    vec2 dir;
    dir.x = -sinf(yaw);
    dir.y = cosf(yaw);
    return dir;
}

static float yaw_from_points(Point_struct start, Point_struct end) {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    return wrap_angle_pi_local(atan2f(-dx, dy));
}

static int point_near(Point_struct a, Point_struct b, float threshold_mm) {
    return get_length(a, b) <= threshold_mm;
}

static int prepare_empty_path(Path_struct *p_path) {
    if (p_path == NULL) {
        return -1;
    }
    if (p_path->trajectories != NULL) {
        vPortFree(p_path->trajectories);
        p_path->trajectories = NULL;
    }
    p_path->trajectory_num = 0;
    p_path->trajectory_count = 0;
    p_path->length = 0.0f;
    p_path->start_angle = 0.0f;
    p_path->end_angle = 0.0f;
    return 0;
}

static int init_custom_path(Path_struct *p_path, const Trajectory *trajectories, uint8_t trajectory_num, float start_angle, float end_angle) {
    float total_length = 0.0f;

    if (prepare_path_storage(p_path, trajectory_num) != 0) {
        return -1;
    }

    p_path->start_angle = wrap_angle_pi_local(start_angle);
    p_path->end_angle = wrap_angle_pi_local(end_angle);

    for (uint8_t i = 0; i < trajectory_num; i++) {
        p_path->trajectories[i] = trajectories[i];
        total_length += trajectories[i].length;
    }
    p_path->length = total_length;
    return 0;
}

static void mark_last_segment(Trajectory *trajectories, uint8_t trajectory_num) {
    if (trajectory_num == 0) {
        return;
    }
    for (uint8_t i = 0; i + 1 < trajectory_num; i++) {
        trajectories[i].ifvoid = full;
    }
    trajectories[trajectory_num - 1].ifvoid = empty;
}

static int append_line(Trajectory *trajectories, uint8_t *trajectory_num, Point_struct start, Point_struct end) {
    trajectories[*trajectory_num] = generate_line_trajectory(start, end, full);
    (*trajectory_num)++;
    return 0;
}

static int append_best_line_arc_to_pose(Trajectory *trajectories, uint8_t *trajectory_num, Point_struct start, Point_struct end, float end_angle, float turn_radius) {
    const float direct_yaw = yaw_from_points(start, end);
    const float yaw_err = wrap_angle_pi_local(end_angle - direct_yaw);
    Point_struct best_tangent = {0.0f, 0.0f};
    Point_struct best_center = {0.0f, 0.0f};
    float best_central_angle = 0.0f;
    float best_total_length = 1e30f;
    uint8_t found_arc = 0;
    vec2 tangent_dir;
    vec2 left_normal;

    if (fabsf(yaw_err) < 0.18f || get_length(start, end) < turn_radius) {
        trajectories[*trajectory_num] = generate_line_trajectory(start, end, empty);
        (*trajectory_num)++;
        return 0;
    }

    tangent_dir = yaw_to_dir(end_angle);
    left_normal.x = -tangent_dir.y;
    left_normal.y = tangent_dir.x;

    for (uint8_t arc_ccw = 0; arc_ccw <= 1; arc_ccw++) {
        Point_struct center;
        Point_struct tangent_point;
        float central_angle = 0.0f;
        float total_length = 0.0f;

        if (arc_ccw) {
            center.x = end.x + left_normal.x * turn_radius;
            center.y = end.y + left_normal.y * turn_radius;
        } else {
            center.x = end.x - left_normal.x * turn_radius;
            center.y = end.y - left_normal.y * turn_radius;
        }

        if (select_tangent_point(start, end, center, arc_ccw, &tangent_point, &central_angle) != 0) {
            continue;
        }

        total_length = get_length(start, tangent_point) + turn_radius * central_angle;
        if (total_length < best_total_length) {
            best_total_length = total_length;
            best_tangent = tangent_point;
            best_center = center;
            best_central_angle = central_angle;
            found_arc = 1;
        }
    }

    if (!found_arc) {
        trajectories[*trajectory_num] = generate_line_trajectory(start, end, empty);
        (*trajectory_num)++;
        return 0;
    }

    trajectories[*trajectory_num] = generate_line_trajectory(start, best_tangent, full);
    (*trajectory_num)++;
    trajectories[*trajectory_num] = generate_circle_trajectory(best_tangent, end, best_center, best_central_angle, empty);
    (*trajectory_num)++;
    return 0;
}

static R2_Accessible_Region classify_r2_region(Point_struct point) {
    if (point.y <= R2_ENTRY_ZONE_MAX_Y_MM) {
        return R2_REGION_MC;
    }
    if (point.y < stairs_center[0].y) {
        return R2_REGION_MF_ENTRY;
    }
    if (point.y >= R2_CF_ZONE_MIN_Y_MM) {
        return R2_REGION_CF;
    }
    if (point.y >= R2_EXIT_ZONE_MIN_Y_MM) {
        return R2_REGION_MF_EXIT;
    }
    return R2_REGION_INVALID;
}

static Point_struct select_ramp_anchor(Point_struct reference) {
    const Point_struct left_anchor = {(float)stairs_center[14].x, (float)stairs_center[14].y};
    const Point_struct right_anchor = {(float)stairs_center[12].x, (float)stairs_center[12].y};
    const float dist_left = get_length(reference, left_anchor);
    const float dist_right = get_length(reference, right_anchor);
    return (dist_left < dist_right) ? left_anchor : right_anchor;
}

static int build_path_via_waypoints(Path_struct *p_path, Point_struct start, const Point_struct *waypoints, uint8_t waypoint_num, Point_struct end, float start_angle, float end_angle, float turn_radius) {
    Trajectory trajectories[6];
    uint8_t trajectory_num = 0;
    Point_struct current = start;

    for (uint8_t i = 0; i < waypoint_num; i++) {
        append_line(trajectories, &trajectory_num, current, waypoints[i]);
        current = waypoints[i];
    }

    append_best_line_arc_to_pose(trajectories, &trajectory_num, current, end, end_angle, turn_radius);
    mark_last_segment(trajectories, trajectory_num);
    return init_custom_path(p_path, trajectories, trajectory_num, start_angle, end_angle);
}

void set_r2_mc_head_route_config(const R2_MC_HeadRouteConfig* config) {
    if (config == NULL) {
        return;
    }
    g_r2_mc_head_route_cfg = *config;
    if (g_r2_mc_head_route_cfg.turn_radius < 300.0f) {
        g_r2_mc_head_route_cfg.turn_radius = 300.0f;
    }
}

int build_r2_head_grab_path(Path_struct* p_path, Point_struct start) {
    Point_struct waypoints[1];
    const Point_struct robot_now = {lcResult.x, lcResult.y};
    float start_angle = point_near(robot_now, start, R2_START_ANGLE_NEAR_MM) ? lcResult.r : yaw_from_points(start, g_r2_mc_head_route_cfg.wait_point);

    waypoints[0] = g_r2_mc_head_route_cfg.wait_point;
    return build_path_via_waypoints(
        p_path,
        start,
        waypoints,
        1,
        g_r2_mc_head_route_cfg.grab_point,
        start_angle,
        g_r2_mc_head_route_cfg.grab_angle,
        g_r2_mc_head_route_cfg.turn_radius
    );
}

int build_r2_head_wait_path(Path_struct* p_path, Point_struct start) {
    Trajectory trajectories[2];
    uint8_t trajectory_num = 0;
    const float start_angle = point_near((Point_struct){lcResult.x, lcResult.y}, start, R2_START_ANGLE_NEAR_MM) ? lcResult.r : yaw_from_points(start, g_r2_mc_head_route_cfg.wait_point);

    append_best_line_arc_to_pose(
        trajectories,
        &trajectory_num,
        start,
        g_r2_mc_head_route_cfg.wait_point,
        g_r2_mc_head_route_cfg.wait_angle,
        g_r2_mc_head_route_cfg.turn_radius
    );
    mark_last_segment(trajectories, trajectory_num);
    return init_custom_path(p_path, trajectories, trajectory_num, start_angle, g_r2_mc_head_route_cfg.wait_angle);
}

int build_r2_head_retreat_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle) {
    Point_struct waypoints[1];
    const float start_angle = point_near((Point_struct){lcResult.x, lcResult.y}, start, R2_START_ANGLE_NEAR_MM) ? lcResult.r : g_r2_mc_head_route_cfg.grab_angle;

    waypoints[0] = g_r2_mc_head_route_cfg.retreat_point;
    return build_path_via_waypoints(
        p_path,
        start,
        waypoints,
        1,
        end,
        start_angle,
        end_angle,
        g_r2_mc_head_route_cfg.turn_radius
    );
}

int build_r2_accessible_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle) {
    Trajectory trajectories[5];
    uint8_t trajectory_num = 0;
    const Point_struct robot_now = {lcResult.x, lcResult.y};
    R2_Accessible_Region start_region;
    R2_Accessible_Region end_region;
    float start_angle;

    if (p_path == NULL) {
        return -1;
    }

    start_region = classify_r2_region(start);
    end_region = classify_r2_region(end);
    if (start_region == R2_REGION_INVALID || end_region == R2_REGION_INVALID) {
        return -1;
    }

    start_angle = point_near(robot_now, start, R2_START_ANGLE_NEAR_MM) ? lcResult.r : yaw_from_points(start, end);

    if (start_region == end_region) {
        append_best_line_arc_to_pose(trajectories, &trajectory_num, start, end, end_angle, R2_MIN_TURN_RADIUS_MM);
        mark_last_segment(trajectories, trajectory_num);
        return init_custom_path(p_path, trajectories, trajectory_num, start_angle, end_angle);
    }

    if ((start_region == R2_REGION_MC && end_region == R2_REGION_MF_ENTRY) ||
        (start_region == R2_REGION_MF_ENTRY && end_region == R2_REGION_MC)) {
        append_best_line_arc_to_pose(trajectories, &trajectory_num, start, end, end_angle, R2_MIN_TURN_RADIUS_MM);
        mark_last_segment(trajectories, trajectory_num);
        return init_custom_path(p_path, trajectories, trajectory_num, start_angle, end_angle);
    }

    if ((start_region == R2_REGION_MF_EXIT && end_region == R2_REGION_CF) ||
        (start_region == R2_REGION_CF && end_region == R2_REGION_MF_EXIT)) {
        const Point_struct ramp_anchor = select_ramp_anchor(end_region == R2_REGION_CF ? end : start);
        append_line(trajectories, &trajectory_num, start, ramp_anchor);
        append_best_line_arc_to_pose(trajectories, &trajectory_num, ramp_anchor, end, end_angle, R2_MIN_TURN_RADIUS_MM);
        mark_last_segment(trajectories, trajectory_num);
        return init_custom_path(p_path, trajectories, trajectory_num, start_angle, end_angle);
    }

    return -1;
}

int build_r2_business_path(Path_struct* p_path, Point_struct start, Point_struct end, float end_angle) {
    if (point_near(end, g_r2_mc_head_route_cfg.grab_point, R2_HEAD_NEAR_THRESHOLD_MM)) {
        return build_r2_head_grab_path(p_path, start);
    }

    if (point_near(end, g_r2_mc_head_route_cfg.wait_point, R2_HEAD_NEAR_THRESHOLD_MM)) {
        return build_r2_head_wait_path(p_path, start);
    }

    if (point_near(start, g_r2_mc_head_route_cfg.grab_point, R2_HEAD_NEAR_THRESHOLD_MM) &&
        !point_near(end, g_r2_mc_head_route_cfg.grab_point, R2_HEAD_NEAR_THRESHOLD_MM)) {
        return build_r2_head_retreat_path(p_path, start, end, end_angle);
    }

    return build_r2_accessible_path(p_path, start, end, end_angle);
}
