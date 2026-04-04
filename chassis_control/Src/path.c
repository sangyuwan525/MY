//
#include "path.h"

#include <math.h>

#include "FreeRTOS.h"
#include "locator_driver.h"
#include "ClimbStairs.h"

Path_struct path_test;
path_spd_data_t spd_test = {2000, 500, 500};

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
    const float vx = line_start.x - center.x; // 向量 V = 圆心 -> 直线起点
    const float vy = line_start.y - center.y;
    const float d2 = vx * vx + vy * vy; // 距离平方
    const float r2 = radius * radius;   // 半径平方

    // 2. 检查合法性：若点在圆内 (d2 <= r2)，则无法作切线
    if (radius < 1e-6f || d2 <= r2 + 1e-6f) {
        return -1;
    }

    // 3. 使用几何法求解两个候选切点坐标
    float base = r2 / d2;
    float factor = radius * sqrtf(d2 - r2) / d2;
    vec2 perp = {-vy, vx}; // 垂直向量

    Point_struct candidates[2];
    candidates[0].x = center.x + base * vx + factor * perp.x;
    candidates[0].y = center.y + base * vy + factor * perp.y;
    candidates[1].x = center.x + base * vx - factor * perp.x;
    candidates[1].y = center.y + base * vy - factor * perp.y;

    // 4. 遍历两个候选切点，根据“方向连续性”进行筛选
    for (uint8_t i = 0; i < 2; i++) {
        vec2 line_dir;
        vec2 radius_dir;
        vec2 arc_tangent_dir;
        float line_norm;
        float tangent_norm;
        float dot;

        // 直线方向：从起点指向切点
        line_dir.x = candidates[i].x - line_start.x;
        line_dir.y = candidates[i].y - line_start.y;
        // 切点处的半径方向
        radius_dir.x = candidates[i].x - center.x;
        radius_dir.y = candidates[i].y - center.y;

        // 根据 CCW/CW 计算圆弧在该切点处的瞬时切向 (半径向量旋转90度)
        if (arc_ccw) {
            arc_tangent_dir.x = -radius_dir.y;
            arc_tangent_dir.y = radius_dir.x;
        } else {
            arc_tangent_dir.x = radius_dir.y;
            arc_tangent_dir.y = -radius_dir.x;
        }

        // 5. 归一化并计算点积 (Dot Product)
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
    tra.bezier_p1 = start;
    tra.bezier_p2 = end;
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
    tra.bezier_p1 = start;
    tra.bezier_p2 = end;
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
 * @brief 计算三次贝塞尔曲线在参数 t 处的坐标点。
 *
 * @param p0 起点控制点
 * @param p1 第一控制点
 * @param p2 第二控制点
 * @param p3 终点控制点
 * @param t  参数，范围 [0,1]
 * @return Point_struct 曲线上对应点
 *
 * @note
 * 采用标准三次贝塞尔公式：
 * B(t) = (1-t)^3*p0 + 3(1-t)^2*t*p1 + 3(1-t)*t^2*p2 + t^3*p3
 */
static Point_struct bezier_point(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, float t) {
    Point_struct p;
    const float u = 1.0f - t;
    const float uu = u * u;
    const float tt = t * t;
    const float uuu = uu * u;
    const float ttt = tt * t;

    p.x = uuu * p0.x + 3.0f * uu * t * p1.x + 3.0f * u * tt * p2.x + ttt * p3.x;
    p.y = uuu * p0.y + 3.0f * uu * t * p1.y + 3.0f * u * tt * p2.y + ttt * p3.y;
    return p;
}

/**
 * @brief 计算三次贝塞尔曲线在参数 t 处的一阶导数。
 */
static vec2 bezier_derivative(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, float t) {
    vec2 d;
    const float u = 1.0f - t;
    d.x = 3.0f * u * u * (p1.x - p0.x) + 6.0f * u * t * (p2.x - p1.x) + 3.0f * t * t * (p3.x - p2.x);
    d.y = 3.0f * u * u * (p1.y - p0.y) + 6.0f * u * t * (p2.y - p1.y) + 3.0f * t * t * (p3.y - p2.y);
    return d;
}

/**
 * @brief 线性插值两个点。
 */
static Point_struct point_lerp(Point_struct a, Point_struct b, float t) {
    Point_struct p;
    p.x = a.x + (b.x - a.x) * t;
    p.y = a.y + (b.y - a.y) * t;
    return p;
}

/**
 * @brief 将三次贝塞尔按参数 t 分割为左右两段。
 * @note left 为 [0,t]，right 为 [t,1]。
 */
static void bezier_split(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3,
                         float t,
                         Point_struct* l0, Point_struct* l1, Point_struct* l2, Point_struct* l3,
                         Point_struct* r0, Point_struct* r1, Point_struct* r2, Point_struct* r3) {
    const Point_struct q0 = point_lerp(p0, p1, t);
    const Point_struct q1 = point_lerp(p1, p2, t);
    const Point_struct q2 = point_lerp(p2, p3, t);
    const Point_struct r01 = point_lerp(q0, q1, t);
    const Point_struct r12 = point_lerp(q1, q2, t);
    const Point_struct s = point_lerp(r01, r12, t);

    *l0 = p0;  *l1 = q0;  *l2 = r01; *l3 = s;
    *r0 = s;   *r1 = r12; *r2 = q2;  *r3 = p3;
}

/**
 * @brief 提取原贝塞尔 [t0,t1] 子曲线，并输出等价三次贝塞尔控制点。
 */
static void bezier_subcurve(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3,
                            float t0, float t1,
                            Point_struct* q0, Point_struct* q1, Point_struct* q2, Point_struct* q3) {
    Point_struct l0, l1, l2, l3, r0, r1, r2, r3;
    Point_struct ll0, ll1, ll2, ll3, rr0, rr1, rr2, rr3;

    if (t0 <= 0.0f && t1 >= 1.0f) {
        *q0 = p0; *q1 = p1; *q2 = p2; *q3 = p3;
        return;
    }
    if (t1 <= 1e-6f) {
        *q0 = p0; *q1 = p0; *q2 = p0; *q3 = p0;
        return;
    }

    // 先分割出 [0,t1]
    bezier_split(p0, p1, p2, p3, t1, &l0, &l1, &l2, &l3, &r0, &r1, &r2, &r3);

    // 再在 [0,t1] 内按 u=t0/t1 分割，取右段即 [t0,t1]
    {
        const float u = (t0 <= 0.0f) ? 0.0f : (t0 / t1);
        bezier_split(l0, l1, l2, l3, u, &ll0, &ll1, &ll2, &ll3, &rr0, &rr1, &rr2, &rr3);
    }

    *q0 = rr0; *q1 = rr1; *q2 = rr2; *q3 = rr3;
}

/**
 * @brief 数值估计三次贝塞尔弧长。
 */
static float bezier_length(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, uint16_t samples) {
    float total = 0.0f;
    Point_struct prev = p0;
    if (samples < 8U) {
        samples = 8U;
    }
    for (uint16_t i = 1U; i <= samples; i++) {
        const float t = (float)i / (float)samples;
        const Point_struct cur = bezier_point(p0, p1, p2, p3, t);
        total += vec_module(cur.x - prev.x, cur.y - prev.y);
        prev = cur;
    }
    return total;
}

/**
 * @brief 生成贝塞尔轨迹段（traceType=bezier，不再离散为直线）。
 */
Trajectory generate_bezier_trajectory_segment(Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, float t0, float t1, Ifvoid is_end) {
    Trajectory tra;
    Point_struct q0, q1, q2, q3;
    vec2 d0, d1;

    if (t0 < 0.0f) t0 = 0.0f;
    if (t0 > 1.0f) t0 = 1.0f;
    if (t1 < 0.0f) t1 = 0.0f;
    if (t1 > 1.0f) t1 = 1.0f;
    if (t1 < t0) {
        const float temp = t0;
        t0 = t1;
        t1 = temp;
    }

    bezier_subcurve(p0, p1, p2, p3, t0, t1, &q0, &q1, &q2, &q3);
    d0 = bezier_derivative(q0, q1, q2, q3, 0.0f);
    d1 = bezier_derivative(q0, q1, q2, q3, 1.0f);

    tra.point_start = q0;
    tra.point_end = q3;
    tra.bezier_p1 = q1;
    tra.bezier_p2 = q2;
    tra.traceType = bezier;
    tra.ifvoid = is_end;

    // trace[] 用于给控制层留可读信息：起点/终点切向角
    tra.trace[0] = atan2f(d0.y, d0.x);
    tra.trace[1] = atan2f(d1.y, d1.x);
    tra.trace[2] = t0;
    tra.trace[3] = t1;

    // 用较高采样精度估算弧长，保证速度规划更接近真实路径长度
    tra.length = bezier_length(q0, q1, q2, q3, 120U);
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
 * @brief 初始化单条贝塞尔曲线路径（单段 bezier traceType）。
 *
 * @param segment_count 兼容旧接口保留；当前实现不离散，参数仅保留向后兼容。
 */
void init_single_bezier_path(Path_struct* p_path, Point_struct p0, Point_struct p1, Point_struct p2, Point_struct p3, uint8_t segment_count, float start_angle, float end_angle) {
    (void)segment_count;

    if (prepare_path_storage(p_path, 1) != 0) {
        return;
    }

    p_path->start_angle = start_angle;
    p_path->end_angle = end_angle;
    p_path->trajectories[0] = generate_bezier_trajectory_segment(p0, p1, p2, p3, 0.0f, 1.0f, empty);
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

/**
 * @brief 一区端头架路径的默认参数。
 *
 * @note
 * 1. 这组默认值只是“保守占位值”，保证新增接口在没有配置时也能工作。
 * 2. 真正比赛前应根据实车场地坐标，重新测量端头架抓取位 / 等待位 / 回撤位，
 *    然后通过 set_r2_mc_head_route_config() 写入。
 * 3. 这里把等待位和回撤位单独抽出来，是为了让 R2 在端头架附近不要用一条硬直线
 *    直接冲进冲出，从而降低抓取失败或姿态不稳的概率。
 */
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

/**
 * @brief 将角度包裹到 [-pi, pi]。
 *
 * @note
 * 你前面已经把整套角度定义统一成 [-pi, pi]，这里新增的业务路径辅助函数也必须
 * 遵守同一套约定，否则在终点角接近 +pi / -pi 交界时会出现跳变。
 */
static float wrap_angle_pi_local(float angle) {
    while (angle > pi) {
        angle -= 2.0f * pi;
    }
    while (angle < -pi) {
        angle += 2.0f * pi;
    }
    return angle;
}

/**
 * @brief 根据朝向角得到车头方向单位向量。
 *
 * @note
 * 结合你工程里“0 朝向沿 +Y 方向”的约定，这里不是常见的 (cos, sin)，而是：
 * x = -sin(yaw), y = cos(yaw)
 */
static vec2 yaw_to_dir(float yaw) {
    vec2 dir;
    dir.x = -sinf(yaw);
    dir.y = cosf(yaw);
    return dir;
}

/**
 * @brief 根据两点计算“从 start 指向 end”的期望朝向角。
 *
 * @note
 * 返回值同样按 [-pi, pi] 包角，用于路径起点缺少显式朝向时做一个几何近似。
 */
static float yaw_from_points(Point_struct start, Point_struct end) {
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    return wrap_angle_pi_local(atan2f(-dx, dy));
}

/**
 * @brief 判断两点是否足够接近。
 *
 * @note
 * 这个函数主要用于业务层“识别意图”，例如：
 * - 终点是否可以视为端头架抓取点
 * - 起点是否可以视为刚刚处于抓取位
 */
static int point_near(Point_struct a, Point_struct b, float threshold_mm) {
    return get_length(a, b) <= threshold_mm;
}

/**
 * @brief 把 Path_struct 置为空路径。
 *
 * @note
 * 这个函数和 prepare_path_storage() 的区别是：
 * - prepare_path_storage() 用于“准备一条新路径”
 * - prepare_empty_path() 用于“明确清空当前路径状态”
 * 目前主要作为新增业务接口的安全辅助函数保留。
 */
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

/**
 * @brief 用一组已经生成好的轨迹段，装配成完整 Path_struct。
 *
 * @param p_path           输出路径
 * @param trajectories     外部临时生成的轨迹段数组
 * @param trajectory_num   轨迹段数量
 * @param start_angle      整条路径起点朝向
 * @param end_angle        整条路径终点朝向
 *
 * @note
 * 1. 这个函数的作用是把“几何规划”和“路径结构体内存管理”解耦。
 * 2. 规划函数只关心该生成哪些段；真正写入 Path_struct、累加总长度都统一走这里。
 * 3. 起点/终点角都会在这里再次做包角，避免外部遗漏。
 */
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

/**
 * @brief 统一修正多段路径里每一段的 ifvoid 标记。
 *
 * @note
 * 在你现有控制器里，ifvoid == empty 表示“最后一段”，其余段应该是 full。
 * 所以凡是临时拼多段路径，最后都需要调用这个函数统一整理。
 */
static void mark_last_segment(Trajectory *trajectories, uint8_t trajectory_num) {
    if (trajectory_num == 0) {
        return;
    }
    for (uint8_t i = 0; i + 1 < trajectory_num; i++) {
        trajectories[i].ifvoid = full;
    }
    trajectories[trajectory_num - 1].ifvoid = empty;
}

/**
 * @brief 追加一段直线轨迹。
 *
 * @note
 * 这里只做数组尾插，不做终段标记；终段标记统一由 mark_last_segment() 处理。
 */
static int append_line(Trajectory *trajectories, uint8_t *trajectory_num, Point_struct start, Point_struct end) {
    trajectories[*trajectory_num] = generate_line_trajectory(start, end, full);
    (*trajectory_num)++;
    return 0;
}

/**
 * @brief 追加一段“尽量优先用 直线 + 圆弧 收敛到终点姿态”的路径。
 *
 * @param trajectories   轨迹段数组
 * @param trajectory_num 当前已写入段数
 * @param start          当前起点
 * @param end            终点
 * @param end_angle      终点期望朝向
 * @param turn_radius    允许使用的转弯半径
 *
 * @note
 * 这是新增业务规划里最关键的几何函数，逻辑分三步：
 * 1. 先看“直接连线的方向”和“终点期望朝向”差得大不大。
 *    如果差很小，直接用一段直线即可。
 * 2. 如果终点姿态要求明显不同，则以终点朝向为约束，在终点左右两侧各假设一个圆心，
 *    分别尝试构造“起点 -> 圆的切点 -> 终点圆弧”的路径。
 * 3. 如果两个候选都可行，选总长度更短的那一条。
 *
 * 这样做的目的，是让上层只关心“终点姿态要对”，而不需要手工指定圆心。
 */
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

/**
 * @brief 根据点的大致位置，把它归类到 R2 允许通过的几个大区域之一。
 *
 * @note
 * 当前只做“粗粒度区域分类”：
 * - 一区 MC
 * - 二区入口区
 * - 二区出口区
 * - 三区 CF
 *
 * 树林内部故意不在这里放开，因为树林内部应继续用你已有的方块规划逻辑处理。
 */
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

/**
 * @brief 在二区出口区到三区的连接处，选择一个坡道锚点。
 *
 * @note
 * 规则要求 R2 进入三区必须经坡道，所以这里不是直接连 start/end，
 * 而是强制路径先经过一个“坡道前锚点”。
 * 当前根据离目标更近原则，在左右两个候选锚点中选一个。
 */
static Point_struct select_ramp_anchor(Point_struct reference) {
    const Point_struct left_anchor = {(float)stairs_center[14].x, (float)stairs_center[14].y};
    const Point_struct right_anchor = {(float)stairs_center[12].x, (float)stairs_center[12].y};
    const float dist_left = get_length(reference, left_anchor);
    const float dist_right = get_length(reference, right_anchor);
    return (dist_left < dist_right) ? left_anchor : right_anchor;
}

/**
 * @brief 通过若干中间 waypoint 构建路径，末段自动收敛到终点姿态。
 *
 * @note
 * 用途是把业务层常见的“先到一个等待点/回撤点，再去目标点”统一起来。
 * 前面的 waypoint 段固定走直线，最后一段由 append_best_line_arc_to_pose()
 * 自动决定是直线还是“直线 + 圆弧”。
 */
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

/**
 * @brief 更新端头架抓取路径配置。
 *
 * @note
 * 这里额外对 turn_radius 做了下限保护，避免上层误传太小的半径，导致路径几何不可行
 * 或者虽然可行但对 800x800 尺寸的 R2 不够友好。
 */
void set_r2_mc_head_route_config(const R2_MC_HeadRouteConfig* config) {
    if (config == NULL) {
        return;
    }
    g_r2_mc_head_route_cfg = *config;
    if (g_r2_mc_head_route_cfg.turn_radius < 300.0f) {
        g_r2_mc_head_route_cfg.turn_radius = 300.0f;
    }
}

/**
 * @brief 生成“去端头架抓取”的业务路径。
 *
 * @note
 * 路线不是直接 start -> grab_point，而是：
 * start -> wait_point -> grab_point
 *
 * 这么做的好处是：
 * - 车先到稳定等待位，再以较平滑的姿态靠近端头架
 * - 后续如果你要微调抓取动作，只需要改 wait/grab 两个点，不需要改状态机
 */
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

/**
 * @brief 生成“去端头架等待位”的业务路径。
 *
 * @note
 * 适合用于：
 * - 抓取前先占位等待
 * - 跟 R1 配合时先进入稳定姿态
 */
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

/**
 * @brief 生成“从端头架抓取位离开”的业务路径。
 *
 * @note
 * 离开时不建议直接从抓取位大转向去远处目标，而是：
 * grab_point -> retreat_point -> end
 *
 * 这样做是为了把端头架附近最容易卡顿/碰撞/姿态发散的这一小段单独处理掉。
 */
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

/**
 * @brief 构建 R2 在“非树林区可达区域”内的普通业务路径。
 *
 * @note
 * 该函数负责大区域之间的规则约束：
 * 1. 同一区域内：直接规划到目标姿态
 * 2. 一区 <-> 二区入口区：允许直接规划
 * 3. 二区出口区 <-> 三区：强制经过坡道锚点
 * 4. 其他组合：返回失败，让上层决定是否走树林规划或别的业务逻辑
 */
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

/**
 * @brief R2 路径规划的业务总入口。
 *
 * @note
 * 上层状态机只传 start / end / end_angle，不关心底层应该走哪一种路径。
 * 这里按“业务意图”自动分流：
 * 1. 如果终点接近端头架抓取位 -> 走抓取路径
 * 2. 如果终点接近端头架等待位 -> 走等待路径
 * 3. 如果起点接近抓取位且终点已不是抓取位 -> 走回撤路径
 * 4. 否则 -> 走普通非树林区路径
 *
 * 这样状态机层就不需要自己判断“我现在该拼直线还是圆弧还是先回撤”。
 */
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
