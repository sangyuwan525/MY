//
// Created by 马皓然 on 2025/10/30.
//
#include "chassis_path.h"
#include <math.h>
#include "chassis_pid.h"
#include "locator_driver.h"
#include "chassis_driver.h"

#define ANGLE_TOLERANCE 0.01f

/**
 * @brief  计算两个二维向量的和 (a + b)。
 *
 * @param  a 向量 a
 * @param  b 向量 b
 *
 * @return vec2 结果向量，x 分量为 a.x + b.x，y 分量为 a.y + b.y。
 *
 * @note   用于路径规划或运动学中的向量叠加。
 * @author stm32小高手
 * @date   2025/11/01
 */
vec2 sum_vec2(vec2 a, vec2 b) {
    vec2 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}
/**
 * @brief  计算两个二维向量的和 (a + b)。
 *
 * @param  a 向量 a
 * @param  b 向量 b
 *
 * @return vec2 结果向量，x 分量为 a.x + b.x，y 分量为 a.y + b.y。
 *
 * @note   用于路径规划或运动学中的向量叠加。
 * @author stm32小高手
 * @date   2025/11/01
 */
float get_length(Point_struct a, Point_struct b) {
    return vec_module(a.x - b.x , a.y - b.y);
}
/**
 * @brief  计算两个二维向量间的夹角（弧度制）。
 *
 * @param  a 向量 a
 * @param  b 向量 b
 *
 * @return float 两个向量之间的夹角，取值范围 $[0, \pi]$ 弧度。
 *
 * @note   使用点积公式 $\theta = \arccos\left(\frac{a \cdot b}{|a| |b|}\right)$。
 * 此函数计算的是非有向夹角。
 * @author stm32小高手
 * @date   2025/11/01
 */
float get_angle(vec2 a, vec2 b) {
    const float vec_dot = a.x * b.x + a.y * b.y;
    const float len_a = vec_module(a.x, a.y);
    const float len_b = vec_module(b.x, b.y);
    //安全检查防止除零
    const float len_product = len_a * len_b;
    if (len_product <1e-6) {
        return 0.0f;
    }
    //防止浮点数计算越界
    float cos_angle = vec_dot / len_product;
    if (cos_angle > 1.0f) cos_angle = 1.0f;
    else if (cos_angle < -1.0f) cos_angle = -1.0f;
    return acosf(cos_angle);
}

/**
 * @brief 计算三次贝塞尔在参数 t 的点。
 */
static Point_struct bezier_eval_tra(const Trajectory *tra, float t) {
    const Point_struct p0 = tra->point_start;
    const Point_struct p1 = tra->bezier_p1;
    const Point_struct p2 = tra->bezier_p2;
    const Point_struct p3 = tra->point_end;
    const float u = 1.0f - t;
    const float uu = u * u;
    const float tt = t * t;
    const float uuu = uu * u;
    const float ttt = tt * t;
    Point_struct p;
    p.x = uuu * p0.x + 3.0f * uu * t * p1.x + 3.0f * u * tt * p2.x + ttt * p3.x;
    p.y = uuu * p0.y + 3.0f * uu * t * p1.y + 3.0f * u * tt * p2.y + ttt * p3.y;
    return p;
}

/**
 * @brief 计算三次贝塞尔在参数 t 的一阶导。
 */
static vec2 bezier_d1_tra(const Trajectory *tra, float t) {
    const Point_struct p0 = tra->point_start;
    const Point_struct p1 = tra->bezier_p1;
    const Point_struct p2 = tra->bezier_p2;
    const Point_struct p3 = tra->point_end;
    const float u = 1.0f - t;
    vec2 d;
    d.x = 3.0f * u * u * (p1.x - p0.x) + 6.0f * u * t * (p2.x - p1.x) + 3.0f * t * t * (p3.x - p2.x);
    d.y = 3.0f * u * u * (p1.y - p0.y) + 6.0f * u * t * (p2.y - p1.y) + 3.0f * t * t * (p3.y - p2.y);
    return d;
}

/**
 * @brief 计算三次贝塞尔在参数 t 的二阶导。
 */
static vec2 bezier_d2_tra(const Trajectory *tra, float t) {
    const Point_struct p0 = tra->point_start;
    const Point_struct p1 = tra->bezier_p1;
    const Point_struct p2 = tra->bezier_p2;
    const Point_struct p3 = tra->point_end;
    const float u = 1.0f - t;
    vec2 d2;
    d2.x = 6.0f * u * (p2.x - 2.0f * p1.x + p0.x) + 6.0f * t * (p3.x - 2.0f * p2.x + p1.x);
    d2.y = 6.0f * u * (p2.y - 2.0f * p1.y + p0.y) + 6.0f * t * (p3.y - 2.0f * p2.y + p1.y);
    return d2;
}

/**
 * @brief 近似求“当前点到贝塞尔曲线”的最近参数 t。
 */
static float bezier_project_t(Point_struct now_point, const Trajectory *tra) {
    // 先粗采样，再在最优区间做三分迭代，兼顾稳定性和实时性。
    const int coarse_n = 40;
    int best_i = 0;
    float best_d2 = 1e30f;

    for (int i = 0; i <= coarse_n; i++) {
        const float t = (float)i / (float)coarse_n;
        const Point_struct p = bezier_eval_tra(tra, t);
        const float dx = p.x - now_point.x;
        const float dy = p.y - now_point.y;
        const float d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            best_i = i;
        }
    }

    float lo = ((float)best_i - 1.0f) / (float)coarse_n;
    float hi = ((float)best_i + 1.0f) / (float)coarse_n;
    if (lo < 0.0f) lo = 0.0f;
    if (hi > 1.0f) hi = 1.0f;

    for (int k = 0; k < 14; k++) {
        const float m1 = lo + (hi - lo) / 3.0f;
        const float m2 = hi - (hi - lo) / 3.0f;
        const Point_struct p1 = bezier_eval_tra(tra, m1);
        const Point_struct p2 = bezier_eval_tra(tra, m2);
        const float d1x = p1.x - now_point.x;
        const float d1y = p1.y - now_point.y;
        const float d2x = p2.x - now_point.x;
        const float d2y = p2.y - now_point.y;
        const float f1 = d1x * d1x + d1y * d1y;
        const float f2 = d2x * d2x + d2y * d2y;
        if (f1 < f2) {
            hi = m2;
        } else {
            lo = m1;
        }
    }

    return 0.5f * (lo + hi);
}

/**
 * @brief 近似计算贝塞尔从 0 到 t 的弧长。
 */
static float bezier_length_0_to_t(const Trajectory *tra, float t) {
    float total = 0.0f;
    const int n = 40;
    if (t <= 0.0f) return 0.0f;
    if (t > 1.0f) t = 1.0f;

    Point_struct prev = tra->point_start;
    for (int i = 1; i <= n; i++) {
        const float ti = t * ((float)i / (float)n);
        const Point_struct cur = bezier_eval_tra(tra, ti);
        total += vec_module(cur.x - prev.x, cur.y - prev.y);
        prev = cur;
    }
    return total;
}
/**
 * @brief  计算当前点到给定轨迹的垂足 (Foot Point)。
 *
 * @param  now_point 机器人当前的坐标点 P(x, y)。
 * @param  tra 当前跟踪的轨迹段结构体。
 *
 * @return Point_struct 垂足的坐标。若轨迹为直线，则是点到直线的垂足；若轨迹为圆弧，则是点到圆心的连线与圆弧的交点。
 *
 * @note   1. 直线垂足公式来自：点 P(x0, y0) 到直线 Ax+By+C=0 的垂足 F(xf, yf)。
 * 2. 圆弧垂足（投影点）通过计算点到圆心向量的单位化，再乘半径得到。
 * 3. 必须进行分母安全检查，防止轨迹参数异常导致除零。
 * @see    Trajectory
 * @author stm32小高手
 * @date   2025/11/01
 */
Point_struct get_foot_point(Point_struct now_point,Trajectory tra) {
    Point_struct foot_point = {0};
    if (tra.traceType == line) {
        const float A = tra.trace[Line_A];
        const float B = tra.trace[Line_B];
        const float C = tra.trace[Line_C];
        const float x0 = now_point.x;
        const float y0 = now_point.y;
        const float denominator = A * A + B * B;
        if (fabsf(denominator) < 1e-6f) {
            // 如果分母接近零，说明 A=0 且 B=0，轨迹无效或不是直线，返回起点作为默认值
            return tra.point_start;
        }
        foot_point.x = (B * B * x0 - A * B * y0 - A * C) / denominator;
        foot_point.y = (A * A * y0 - A * B * x0 - B * C) / denominator;
    }
    else if (tra.traceType == circle) {
        const float Cx = tra.trace[center_x]; // 圆心 X
        const float Cy = tra.trace[center_y]; // 圆心 Y
        const float R = tra.trace[circle_r];   // 半径 R
        vec2 O_to_p;//圆心指向当前点的向量
        O_to_p.x = now_point.x - Cx;
        O_to_p.y = now_point.y - Cy;
        const float dist_to_center = vec_module(O_to_p.x, O_to_p.y);
        if (dist_to_center < 1e-6f) {
            // 如果当前点就是圆心，无法计算方向，返回圆心作为默认值
            foot_point.x = Cx;
            foot_point.y = Cy;
            return foot_point;
        }
        const float k = R / dist_to_center;
        foot_point.x = O_to_p.x * k + Cx;
        foot_point.y = O_to_p.y * k + Cy;
    }
    else if (tra.traceType == bezier) {
        const float t = bezier_project_t(now_point, &tra);
        foot_point = bezier_eval_tra(&tra, t);
    }
    return foot_point;
}
/**
 * @brief 计算圆形轨迹上当前点与起点之间的有符号圆心角。
 * @param foot_point     当前点坐标。
 * @param tra_cir       圆形轨迹结构体，包含圆心、终点起点和总弧度。
 * @return float      当前点与起点之间的圆心角（带符号），正值表示顺方向，负值表示逆方向。
 */
float get_central_angle(Point_struct foot_point,Trajectory tra_cir) {

    const float Cx = tra_cir.trace[center_x]; // 圆心 X
    const float Cy = tra_cir.trace[center_y]; // 圆心 Y
    const float total_angel = tra_cir.trace[circle_angle];//圆心角总量
    vec2 vec_os;//圆心指向起点的向量
    vec2 vec_on;//圆心指向当前点的向量
    vec2 vec_oe;//圆心指向终点的向量

    vec_os.x = tra_cir.point_start.x - Cx;
    vec_os.y = tra_cir.point_start.y - Cy;
    vec_on.x = foot_point.x - Cx;
    vec_on.y = foot_point.y - Cy;
    vec_oe.x = tra_cir.point_end.x - Cx;
    vec_oe.y = tra_cir.point_end.y - Cy;

    float angle_now_s = get_angle(vec_os,vec_on);//起点到当前点的圆心角
    if (angle_now_s < ANGLE_TOLERANCE) angle_now_s = 0;
    float angle_now_e = get_angle(vec_oe,vec_on);//终点到当前点的圆心角
    if (angle_now_e < ANGLE_TOLERANCE) angle_now_e = 0;
    //此时angle_now_s和angle_now_e均为正值
    if (total_angel <= pi) {// 圆心角为劣弧
        const float sum_ns_ne = angle_now_s + angle_now_e;
        if (sum_ns_ne - total_angel > 0.2)
            angle_now_s = -angle_now_s;
        return angle_now_s;
    }//此处的逻辑必须详细注释
    //当圆心角劣弧时，若起点到当前点和终点到当前点的圆心角和大于总圆心角，则说明当前点在逆方向上或超过终点，应取负值
    //此处角度取负的含义仅影响速度方向的判断
    //正负极度与顺逆时针无关
    else if (total_angel > pi) {
        const float length_n_s = vec_module((foot_point.x - tra_cir.point_start.x),(foot_point.y - tra_cir.point_start.y));
        const float length_n_e = vec_module((foot_point.x - tra_cir.point_end.x),(foot_point.y - tra_cir.point_end.y));
        if (length_n_s < length_n_e) {//当前点更靠近起点
            float sum_ns_ne = angle_now_s + angle_now_e;
            if (fabs(sum_ns_ne + total_angel - 2 *pi) < 0.2)
                angle_now_s = -angle_now_s;
            return angle_now_s;
        }
        else if (length_n_s >= length_n_e) {
            angle_now_s = total_angel - angle_now_e;
            return angle_now_s;
        }
    }
    return angle_now_s;
}
/**
 * @brief  计算在当前垂足点处的轨迹切向速度方向向量。
 *
 * @param  foot_point 机器人当前位置在轨迹上的垂足（投影点）。
 * @param  tra 当前跟踪的轨迹段结构体。
 *
 * @return vec2 归一化后的速度方向向量。
 *
 * @note   1. 直线：方向向量为 (终点 - 起点) 的单位向量。
 * 2. 圆弧：方向向量为当前点到圆心向量的法向量，方向根据圆心角 (angle) 确定。
 * 3. 使用静态变量 last_spddir 在圆弧方向判断失败时提供稳定回退。
 * @see    get_central_angle, vec_module
 * @date   2025/11/02
 */
vec2 get_spd_dir(Point_struct foot_point, Trajectory tra) {
    static vec2 last_dir;
    vec2 dir = {0};
    if (tra.traceType == line) {
        dir.x = tra.point_end.x - tra.point_start.x;
        dir.y = tra.point_end.y - tra.point_start.y;
    }
    else if (tra.traceType == circle) {
        const float Cx = tra.trace[center_x];
        const float Cy = tra.trace[center_y];
        vec2 O_to_p;
        O_to_p.x = foot_point.x - Cx;
        O_to_p.y = foot_point.y - Cy;
        vec2 tan_ccw = {-O_to_p.y, O_to_p.x};
        vec2 tan_cw  = { O_to_p.y,-O_to_p.x};
        // 严格几何切向：CCW=(-ry,rx), CW=(ry,-rx)
        {
            const float angle_geom = get_central_angle(foot_point, tra);
            if (angle_geom > 1e-4f) {
                dir = tan_ccw;
            }
            else if (angle_geom < -1e-4f) {
                dir = tan_cw;
            }
            else {
                // 起点附近角度符号不稳定时，用“更指向弧终点”的切向作为回退。
                vec2 to_end = {tra.point_end.x - foot_point.x, tra.point_end.y - foot_point.y};
                const float d_ccw = tan_ccw.x * to_end.x + tan_ccw.y * to_end.y;
                const float d_cw = tan_cw.x * to_end.x + tan_cw.y * to_end.y;
                dir = (d_ccw >= d_cw) ? tan_ccw : tan_cw;
            }
            goto circle_dir_ready;
        }
        vec2 O_to_s;
        O_to_s.x = tra.point_start.x - tra.trace[center_x];
        O_to_s.y = tra.point_start.y - tra.trace[center_y];

        float angle = get_central_angle(foot_point, tra);
        float dot = O_to_s.x * dir.x + O_to_s.y * dir.y;

        if (fabsf(dot) < 1e-4f) {//垂直时无法根据点积正负分辨速度方向，采用上次的方向
            return last_dir;
        }

        // --- 核心逻辑：根据圆弧运动方向 (angle) 和点积 (dot) 修正速度方向 ---
        if (angle < 0.0f) {
            if (dot < 0.0f) {
                dir.x = -dir.x;
                dir.y = -dir.y;
            }
        }
        else if (angle > 0.0f) {
            if (angle < pi){
                if (dot > 0.0f)
                {
                    dir.x = -dir.x;
                    dir.y = -dir.y;
                }
            }
            else
            {
                if (dot < 0)
                {
                    dir.x = -dir.x;
                    dir.y = -dir.y;
                }
            }
        }
circle_dir_ready:;
    }
    float moul = vec_module(dir.x,dir.y);
    if (moul < 1e-6f) {
        return last_dir;
    }
    dir.x /= moul;
    dir.y /= moul;
    last_dir = dir;
    return dir;
}
/**
 * @brief  计算当前垂足点在整个路径中的累积长度。
 *
 * @param  foot_point 当前轨迹段上的垂足坐标。
 * @param  tra_array 路径所包含的所有轨迹段数组。
 * @param  current_tra_index 当前正在跟踪的轨迹段索引。
 *
 * @return float 机器人已经行进的累积路径长度。
 *
 * @note   1. 累加之前所有已完成轨迹段的 length。
 * 2. 计算当前轨迹段上，起点到垂足点的距离（直线）或弧长（圆弧）。
 * @see    get_length, get_central_angle
 * @date   2025/11/02
 */
float get_length_in_path(Point_struct foot_point,Trajectory* tra_array,uint8_t current_tra_index) {
    float total_length = 0.0f;
    for (uint8_t i = 0; i < current_tra_index; i++) {
        total_length += tra_array[i].length;
    }
    if (tra_array[current_tra_index].traceType == line) {
        total_length += get_length(tra_array[current_tra_index].point_start, foot_point);
    }
    else if (tra_array[current_tra_index].traceType == circle) {
        const float angle = get_central_angle(foot_point, tra_array[current_tra_index]);
        const float r = tra_array[current_tra_index].trace[circle_r];
        total_length += angle * r;
    }
    else if (tra_array[current_tra_index].traceType == bezier) {
        const float t = bezier_project_t(foot_point, &tra_array[current_tra_index]);
        total_length += bezier_length_0_to_t(&tra_array[current_tra_index], t);
    }
    return total_length;
}
/**
 * @brief  计算路径上某位置的期望朝向角 (Yaw Angle)。
 *
 * @param  path_pos 机器人当前在路径上行进的累积距离。
 * @param  length   当前路径段（或总路径）的总长度。
 * @param  start_angle 路径（或路径段）起点的期望朝向角（弧度）。
 * @param  end_angle   路径（或路径段）终点的期望朝向角（弧度）。
 *
 * @return float  当前位置 path_pos 对应的期望朝向角（弧度）。
 *
 * @note   1. 该函数在整条路径上做朝向角过渡，进度参数为 t=clamp(path_pos/length, 0, 1)。
 * 2. 过渡曲线使用 quintic smoothstep：t^3(10-15t+6t^2)，可降低起止段角速度突变。
 * 3. 角度差始终按 [-pi, pi] 最短路径取值，避免跨边界时大角度跳变。
 * 4. 当 length 很小（接近 0）时直接返回 end_angle（并包角到 [-pi, pi]）。
 * @date   2025/11/02
 */
static float wrap_angle_to_pi(float angle) {
    while (angle > pi) angle -= 2.0f * pi;
    while (angle < -pi) angle += 2.0f * pi;
    return angle;
}

/**
 * @brief 将数值限制到 [0,1] 区间。
 */
static float clamp01f(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

/**
 * @brief Quintic smoothstep，输入输出都在 [0,1]。
 */
static float smoothstep5(float t) {
    t = clamp01f(t);
    return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
}

/**
 * @brief 角度按最短路径插值，并包角到 [-pi, pi]。
 */
static float lerp_angle_shortest(float from, float to, float t) {
    const float delta = wrap_angle_to_pi(to - from);
    return wrap_angle_to_pi(from + delta * clamp01f(t));
}

/**
 * @brief 计算末段平滑混合系数 alpha（0=纯路径跟踪，1=纯靠近终点）。
 */
static float calc_end_blend_alpha(float distance,
                                  float path_remain,
                                  float near_distance_threshold,
                                  float near_remain_threshold,
                                  float done_remain_threshold) {
    const float dis_denom = fmaxf(near_distance_threshold - 50.0f, 1e-3f);
    const float alpha_dis = clamp01f((near_distance_threshold - distance) / dis_denom);

    const float rem_denom = fmaxf(near_remain_threshold - done_remain_threshold, 1e-3f);
    const float alpha_rem = clamp01f((near_remain_threshold - path_remain) / rem_denom);

    return smoothstep5(fmaxf(alpha_dis, alpha_rem));
}

/**
 * @brief 自适应版路径速度分配（按总路径长度自动缩放加减速段）。
 * @note  保留原函数以兼容旧逻辑，新的 go_path_control 使用此函数。
 */
static vec2 get_spd_on_path_calculate_adaptive(path_spd_data_t path_spd, float path_pos, float length, vec2 spd_dir)
{
    vec2 spd = {0.0f, 0.0f};
    const float L = fmaxf(length, 1e-3f);
    const float s = fminf(fmaxf(path_pos, 0.0f), L);

    // 起步/末段保底速度，防止低速区粘滞
    // 超激进起步底速：进一步提升起步响应
    const float v_start = fmaxf(700.0f, 0.30f * path_spd.max_speed);
    const float v_end = fmaxf(220.0f, 0.12f * path_spd.max_speed);

    // 根据路径总长自动约束加减速段
    float up_eff = fminf(fmaxf(path_spd.up_stage, 60.0f), 0.45f * L);
    float down_eff = fminf(fmaxf(path_spd.down_stage, 60.0f), 0.45f * L);

    // 短路径自动压缩阶段长度，避免“阶段和 > 路径长”
    {
        const float stage_sum = up_eff + down_eff;
        const float stage_limit = 0.90f * L;
        if (stage_sum > stage_limit) {
            const float scale = stage_limit / fmaxf(stage_sum, 1e-3f);
            up_eff *= scale;
            down_eff *= scale;
        }
    }

    const float cruise_start = up_eff;
    const float cruise_end = L - down_eff;
    float abs_spd;

    if (s < cruise_start) {
        const float t = s / fmaxf(up_eff, 1e-3f);
        abs_spd = v_start + (path_spd.max_speed - v_start) * smoothstep5(t);
    } else if (s > cruise_end) {
        const float t = (L - s) / fmaxf(down_eff, 1e-3f);
        abs_spd = v_end + (path_spd.max_speed - v_end) * smoothstep5(t);
    } else {
        abs_spd = path_spd.max_speed;
    }

    if (abs_spd < 0.0f) abs_spd = 0.0f;
    if (abs_spd > path_spd.max_speed) abs_spd = path_spd.max_speed;

    spd.x = abs_spd * spd_dir.x;
    spd.y = abs_spd * spd_dir.y;
    return spd;
}

float get_angle_in_path(float path_pos,float length,float start_angle,float end_angle) {
    float t;
    float smooth_t;
    float delta;

    if (length <= 1e-6f) {
        return wrap_angle_to_pi(end_angle);
    }

    // Full-path progress: yaw transition is distributed over the whole path.
    t = path_pos / length;
    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;

    // Quintic smoothstep for smoother yaw-rate at path start/end.
    smooth_t = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));

    // Always use shortest angular difference in [-pi, pi].
    delta = wrap_angle_to_pi(end_angle - start_angle);
    return wrap_angle_to_pi(start_angle + delta * smooth_t);
}

/**
 * @brief 轨迹切向方向（稳定版）：
 * 直线：start->end
 * 圆弧：先在“轨迹段级别”固定 CCW/CW，再用严格几何切向，避免切点附近翻转抖动。
 */
static vec2 get_spd_dir_stable(Point_struct foot_point, Trajectory tra) {
    static vec2 last_dir = {1.0f, 0.0f};
    vec2 dir = {0.0f, 0.0f};

    if (tra.traceType == line) {
        dir.x = tra.point_end.x - tra.point_start.x;
        dir.y = tra.point_end.y - tra.point_start.y;
    } else if (tra.traceType == circle) {
        const float Cx = tra.trace[center_x];
        const float Cy = tra.trace[center_y];
        const float total_angle = tra.trace[circle_angle];

        const float start_ang = atan2f(tra.point_start.y - Cy, tra.point_start.x - Cx);
        const float end_ang = atan2f(tra.point_end.y - Cy, tra.point_end.x - Cx);

        float ccw_delta = end_ang - start_ang;
        while (ccw_delta < 0.0f) ccw_delta += 2.0f * pi;
        while (ccw_delta >= 2.0f * pi) ccw_delta -= 2.0f * pi;
        float cw_delta = 2.0f * pi - ccw_delta;
        if (ccw_delta < 1e-6f) cw_delta = 2.0f * pi;

        // 匹配轨迹存储的 total_angle 来确定该段固定方向。
        const int seg_ccw = (fabsf(total_angle - ccw_delta) <= fabsf(total_angle - cw_delta)) ? 1 : 0;

        const float rx = foot_point.x - Cx;
        const float ry = foot_point.y - Cy;
        if (seg_ccw) {
            dir.x = -ry;
            dir.y = rx;
        } else {
            dir.x = ry;
            dir.y = -rx;
        }
    } else if (tra.traceType == bezier) {
        // 用投影点对应参数处的一阶导作为切向方向，保证贝塞尔轨迹切向连续。
        const float t = bezier_project_t(foot_point, &tra);
        dir = bezier_d1_tra(&tra, t);
    }

    const float n = vec_module(dir.x, dir.y);
    if (n < 1e-6f) {
        return last_dir;
    }
    dir.x /= n;
    dir.y /= n;
    last_dir = dir;
    return dir;
}
/**
 * @brief  根据 S 型曲线算法计算路径上某位置的期望速度向量。
 *
 * @param  path_spd 路径速度信息结构体，包含最大速度和阶段长度。
 * @param  path_pos 机器人当前在路径上行进的累积距离。
 * @param  length   当前路径段的总长度。
 * @param  spd_dir  期望速度的方向单位向量。
 *
 * @return vec2 归一化后的期望速度向量。
 *
 * @note   该函数使用 Sigmoid (S型) 曲线实现加速和减速的平滑过渡。
 * @date   2025/11/02
 */
vec2 get_spd_on_path_calculate(path_spd_data_t path_spd, float path_pos, float length, vec2 spd_dir)
{
    // 参数K：是控制曲线陡峭程度的参数。这里使用您原始代码中的结构
    // 注意：1200 这个魔数可能与您的控制频率或单位有关，使用 float 类型
    const float k = path_spd.max_speed / path_spd.up_stage / 1200.0f;

    const float v_min = 500.0f; // 最小速度或减速终点速度
    float abs_spd;              // 最终计算的速度大小
    vec2 spd = {0.0f, 0.0f};

    // 路径剩余长度
    const float path_remain = length - path_pos;

    // --- 速度分配逻辑 (S型曲线分段) ---

    if (path_pos < path_spd.up_stage) // 1. 加速阶段
    {
        // 简化加速阶段 S 曲线：以 path_pos 为自变量，从 0 爬升到 max_speed
        // 将 [0, up_stage] 映射到 Sigmoid 曲线的陡峭上升段
        abs_spd = path_spd.max_speed / (1.0f + expf(-k * (path_pos - path_spd.up_stage / 2.0f)));

        // 边界处理：确保速度不会在起始阶段过大
        if (abs_spd > path_spd.max_speed) abs_spd = path_spd.max_speed;
    }
    else if (path_remain < path_spd.down_stage) // 3. 减速阶段
    {
        // 减速阶段 S 曲线：以 path_remain 为自变量，从 max_speed 降到 v_min
        // 将 [0, down_stage] 映射到 Sigmoid 曲线的下降段

        // 减速曲线的起始点应该在 path_spd.max_speed，终点在 v_min。
        // 速度差为 (path_spd.max_speed - v_min)
        abs_spd = v_min + (path_spd.max_speed - v_min) / (1.0f + expf(-k * (path_remain - path_spd.down_stage / 2.0f)));

        // 边界处理：确保速度不低于 v_min
        if (abs_spd < v_min) abs_spd = v_min;
    }
    else // 2. 匀速巡航阶段
    {
        abs_spd = path_spd.max_speed;
    }
    // 最终速度向量 = 速度大小 * 方向单位向量
    spd.x = abs_spd * spd_dir.x;
    spd.y = abs_spd * spd_dir.y;

    return spd;
}
/**
 * @brief  将向量从世界坐标系 (World Frame) 转换到机器人局部坐标系 (Body Frame)。
 *
 * @param  src   世界坐标系下的原始向量 (vec2)。
 * @param  angle 机器人当前在世界坐标系下的朝向角（Yaw，弧度）。
 *
 * @return vec2  机器人局部坐标系下的目标向量。
 *
 * @note   该转换通常用于将期望的速度或位置误差从世界坐标系投影到底盘电机可执行的局部坐标系。
 * 旋转公式：X_local = X_world * cos(angle) + Y_world * sin(angle)
 * Y_local = Y_world * cos(angle) - X_world * sin(angle)
 * 使用 float 类型和单精度数学函数。
 * @date   2025/11/02
 */
vec2 change_world_to_local(vec2 src, float angle)
{
    vec2 dst;

    // 统一使用 float 类型的单精度数学函数
    const float sin_a = sinf(angle);
    const float cos_a = cosf(angle);

    // 确保使用 float 类型的输入和输出
    const float src_x = src.x;
    const float src_y = src.y;

    // 转换公式：将世界坐标系下的向量绕 Z 轴逆时针旋转 angle 为正
    // 这里的旋转公式是正确的（假设 angle 是世界系下的 yaw 角）
    dst.x = src_x * cos_a + src_y * sin_a;
    dst.y = src_y * cos_a - src_x * sin_a;

    return dst;
}

//测试target_angle
/**
 * @brief 平滑版路径控制：
 * 1) 速度按路径总长自适应分配；
 * 2) 最后一段使用“路径跟踪 + 终点靠近”平滑混合，避免速度突变。
 */
int go_path_control_smooth(Path_struct* path, path_spd_data_t path_spd)
{
    const Point_struct now_point = {lcResult.x, lcResult.y};
    const float now_pos = wrap_angle_to_pi(lcResult.r);

    const float near_distance_threshold = fmaxf(180.0f, fminf(550.0f, path_spd.down_stage + 120.0f));
    const float switch_distance_threshold = fmaxf(80.0f, fminf(200.0f, path_spd.down_stage * 0.5f + 60.0f));
    const float near_remain_threshold = fmaxf(120.0f, fminf(450.0f, path_spd.down_stage * 0.8f + 100.0f));
    const float switch_remain_threshold = fmaxf(50.0f, fminf(180.0f, path_spd.down_stage * 0.35f + 30.0f));
    const float done_remain_threshold = fmaxf(20.0f, fminf(80.0f, path_spd.down_stage * 0.2f + 20.0f));

    if ((*path).trajectory_count >= (*path).trajectory_num) {
        cha_remote(0.0f, 0.0f, 0.0f);
        return 1;
    }

    Point_struct current_end_point = (*path).trajectories[(*path).trajectory_count].point_end;
    float distance = get_length(now_point, current_end_point);

    Point_struct foot_point = get_foot_point(now_point, (*path).trajectories[(*path).trajectory_count]);
    float path_pos = get_length_in_path(foot_point, (*path).trajectories, (*path).trajectory_count);
    float path_remain = (*path).length - path_pos;
    if (path_remain < 0.0f) path_remain = 0.0f;

    // 非末段切段；末段不硬切模式。
    //if ((*path).trajectories[(*path).trajectory_count].ifvoid != empty &&
    if ((*path).trajectories[(*path).trajectory_count].ifvoid != empty &&
        (distance < near_distance_threshold || path_remain < near_remain_threshold) &&
        (distance <= switch_distance_threshold || path_remain <= switch_remain_threshold)) {
        (*path).trajectory_count++;
        return 0;
    }

    float target_angle = get_angle_in_path(path_pos, (*path).length, (*path).start_angle, (*path).end_angle);
    vec2 spd_dir = get_spd_dir_stable(foot_point, (*path).trajectories[(*path).trajectory_count]);
    vec2 target_spd_world = get_spd_on_path_calculate_adaptive(path_spd, path_pos, (*path).length, spd_dir);

    // 曲率相关自动降速：圆弧按半径，贝塞尔按局部曲率半径。
    {
        const Trajectory *cur_tra = &(*path).trajectories[(*path).trajectory_count];
        float curve_ratio = 1.0f;

        if (cur_tra->traceType == circle) {
            const float r = cur_tra->trace[circle_r];
            curve_ratio = fmaxf(0.60f, fminf(0.95f, r / (r + 700.0f)));
        } else if (cur_tra->traceType == bezier) {
            const float t = bezier_project_t(foot_point, cur_tra);
            const vec2 d1 = bezier_d1_tra(cur_tra, t);
            const vec2 d2 = bezier_d2_tra(cur_tra, t);
            const float cross = fabsf(d1.x * d2.y - d1.y * d2.x);
            const float n = vec_module(d1.x, d1.y);
            float radius = 1e6f;
            if (n > 1e-4f && cross > 1e-6f) {
                const float kappa = cross / (n * n * n);
                if (kappa > 1e-6f) {
                    radius = 1.0f / kappa;
                }
            }
            curve_ratio = fmaxf(0.58f, fminf(0.98f, radius / (radius + 700.0f)));
        }

        target_spd_world.x *= curve_ratio;
        target_spd_world.y *= curve_ratio;
    }

    vec2 correct_spd_world = PID_Correct_Calculate(&chassis_correct_pid, now_point, foot_point);
    vec2 cmd_spd_world = sum_vec2(target_spd_world, correct_spd_world);

    // 末段平滑混合
    if ((*path).trajectories[(*path).trajectory_count].ifvoid == empty) {
        vec2 approach_spd_world = PID_Approaching_Calculate(&chassis_kaojin_pid, now_point, current_end_point);
        const float approach_limit = fmaxf(450.0f, fminf(900.0f, path_spd.max_speed * 0.30f));
        const float approach_norm = vec_module(approach_spd_world.x, approach_spd_world.y);
        if (approach_norm > approach_limit && approach_norm > 1e-3f) {
            const float k = approach_limit / approach_norm;
            approach_spd_world.x *= k;
            approach_spd_world.y *= k;
        }

        const float alpha = calc_end_blend_alpha(distance,
                                                 path_remain,
                                                 near_distance_threshold,
                                                 near_remain_threshold,
                                                 done_remain_threshold);
        cmd_spd_world.x = (1.0f - alpha) * cmd_spd_world.x + alpha * approach_spd_world.x;
        cmd_spd_world.y = (1.0f - alpha) * cmd_spd_world.y + alpha * approach_spd_world.y;
        target_angle = lerp_angle_shortest(target_angle, (*path).end_angle, alpha);
    }

    const float rotation_spd = PID_Angle_Calculate(&chassis_yaw_pid, target_angle, now_pos);
    vec2 spd_local_final = change_world_to_local(cmd_spd_world, now_pos);
    cha_remote(spd_local_final.x, spd_local_final.y, rotation_spd);

    // 到位判定
    if ((*path).trajectories[(*path).trajectory_count].ifvoid == empty) {
        const float yaw_err = wrap_angle_to_pi((*path).end_angle - now_pos);
        if (distance < 50.0f && path_remain < done_remain_threshold && fabsf(yaw_err) < 0.1f &&
            fabsf(lcResult.vx) < 50.0f && fabsf(lcResult.vy) < 50.0f && fabsf(lcResult.vr) < 50.0f) {
            cha_remote(0.0f, 0.0f, 0.0f);
            return 1;
        }
    }

    return 0;
}

float test_angle;

#if 0
#undef go_path_control
#define go_path_control go_path_control_legacy
int go_path_control(Path_struct* path, path_spd_data_t path_spd)
{
    const Point_struct now_point = {lcResult.x, lcResult.y}; // 机器人当前坐标点
    const float now_pos = wrap_angle_to_pi(lcResult.r);      // 机器人当前朝向角（约定 [-pi, pi]）

    // 动态阈值：结合减速段长度，避免短路径切换过早或过晚。
    const float near_distance_threshold = fmaxf(180.0f, fminf(550.0f, path_spd.down_stage + 120.0f));
    const float switch_distance_threshold = fmaxf(80.0f, fminf(200.0f, path_spd.down_stage * 0.5f + 60.0f));
    const float near_remain_threshold = fmaxf(120.0f, fminf(450.0f, path_spd.down_stage * 0.8f + 100.0f));
    const float switch_remain_threshold = fmaxf(50.0f, fminf(180.0f, path_spd.down_stage * 0.35f + 30.0f));
    const float done_remain_threshold = fmaxf(20.0f, fminf(80.0f, path_spd.down_stage * 0.2f + 20.0f));

    // 检查当前轨迹段是否有效，防止越界。
    if ((*path).trajectory_count >= (*path).trajectory_num) {
        cha_remote(0.0f, 0.0f, 0.0f);
        return 1;
    }

    // 当前轨迹段的终点
    Point_struct current_end_point = (*path).trajectories[(*path).trajectory_count].point_end;
    float distance = get_length(now_point, current_end_point); // 当前点到轨迹段终点的直线距离

    // 先计算足点与累计路径位置，用于“按剩余长度”做切段/完成判定。
    Point_struct foot_point = get_foot_point(now_point, (*path).trajectories[(*path).trajectory_count]);
    float path_pos = get_length_in_path(foot_point, (*path).trajectories, (*path).trajectory_count);
    float path_remain = (*path).length - path_pos;
    if (path_remain < 0.0f) path_remain = 0.0f;

    // --- 1. 轨迹切换/末端靠近阶段 ---
    // 条件由“终点欧式距离 + 剩余路径长度”联合决定。
    if ((*path).trajectories[(*path).trajectory_count].ifvoid != empty &&
        (distance < near_distance_threshold || path_remain < near_remain_threshold))
    {
        // 1.1. 非终点轨迹段的切换逻辑
        if ((*path).trajectories[(*path).trajectory_count].ifvoid != empty &&
            (distance <= switch_distance_threshold || path_remain <= switch_remain_threshold))
        {
            (*path).trajectory_count++; // 进入下一段路径
            return 0;
        }

        // 1.2. 终点轨迹段的精确对位/路径完成判断
        else if ((*path).trajectories[(*path).trajectory_count].ifvoid == empty) // 是终点
        {
            // 计算靠近速度 (世界坐标系)，使用全局靠近PID实例
            vec2 adjust_spd_world = PID_Approaching_Calculate(&chassis_kaojin_pid, now_point, current_end_point);

            // 速度转换到车身局部坐标系
            vec2 spd_local_temp = change_world_to_local(adjust_spd_world, now_pos);
            // 靠近阶段限速
            const float close_limit = 500.0f;
            if (spd_local_temp.x > close_limit) spd_local_temp.x = close_limit;
            else if (spd_local_temp.x < -close_limit) spd_local_temp.x = -close_limit;
            if (spd_local_temp.y > close_limit) spd_local_temp.y = close_limit;
            else if (spd_local_temp.y < -close_limit) spd_local_temp.y = -close_limit;

            // 终点角度同样按 [-pi, pi] 使用，避免跨边界跳变。
            float tar_ang_kaojin = wrap_angle_to_pi((*path).end_angle);
            // 旋转速度计算，使用全局角度PID实例
            float vr = PID_Angle_Calculate(&chassis_yaw_pid, tar_ang_kaojin, now_pos);

            // 路径完成判断：位置 + 剩余长度 + 角度 + 速度
            float yaw_err = wrap_angle_to_pi((*path).end_angle - now_pos);
            if (distance < 50.0f && path_remain < done_remain_threshold && fabsf(yaw_err) < 0.1f &&
                fabsf(lcResult.vx) < 50.0f && fabsf(lcResult.vy) < 50.0f && fabsf(lcResult.vr) < 50.0f) {
                cha_remote(0.0f, 0.0f, 0.0f);
                return 1; // 路径完成
            } else {
                cha_remote(spd_local_temp.x, spd_local_temp.y, vr); // 输出末端调整速度
            }
            return 0;
        }
    }

    // --- 2. Pure Pursuit/航迹跟踪主循环逻辑 ---

    // 2.2. 角度控制
    float target_angle = get_angle_in_path(path_pos, (*path).length, (*path).start_angle, (*path).end_angle);
    //test_angle=target_angle;
    float rotation_spd = PID_Angle_Calculate(&chassis_yaw_pid, target_angle, now_pos);// 使用全局角度PID

    vec2 spd_dir = get_spd_dir(foot_point, (*path).trajectories[(*path).trajectory_count]);

    // 期望沿轨迹速度
    // 使用“按路径长度自适应”的速度分配，长路径可跑满，短路径不突兀。
    vec2 target_spd_world = get_spd_on_path_calculate_adaptive(path_spd, path_pos, (*path).length, spd_dir);

    // 横向纠正速度，使用全局修正PID
    // 使用按路径总长自适应的速度分配（长路径可跑满，短路径更平滑）
    vec2 target_spd_world = get_spd_on_path_calculate_adaptive(path_spd, path_pos, (*path).length, spd_dir);
    vec2 target_spd_world = get_spd_on_path_calculate_adaptive(path_spd, path_pos, (*path).length, spd_dir);
    vec2 correct_spd_world = PID_Correct_Calculate(&chassis_correct_pid, now_point, foot_point);

    // 末段平滑混合：避免“进入靠近阶段”时速度突变
    vec2 cmd_spd_world;
    cmd_spd_world.x = 0.0f;
    cmd_spd_world.y = 0.0f;

    // 由路径跟踪分量初始化
    cmd_spd_world = sum_vec2(target_spd_world, correct_spd_world);

    if ((*path).trajectories[(*path).trajectory_count].ifvoid == empty) {
        vec2 approach_spd_world = PID_Approaching_Calculate(&chassis_kaojin_pid, now_point, current_end_point);
        const float approach_limit = fmaxf(450.0f, fminf(900.0f, path_spd.max_speed * 0.30f));
        const float approach_norm = vec_module(approach_spd_world.x, approach_spd_world.y);
        if (approach_norm > approach_limit && approach_norm > 1e-3f) {
            const float k = approach_limit / approach_norm;
            approach_spd_world.x *= k;
            approach_spd_world.y *= k;
        }

        const float alpha = calc_end_blend_alpha(distance,
                                                 path_remain,
                                                 near_distance_threshold,
                                                 near_remain_threshold,
                                                 done_remain_threshold);

        cmd_spd_world.x = (1.0f - alpha) * cmd_spd_world.x + alpha * approach_spd_world.x;
        cmd_spd_world.y = (1.0f - alpha) * cmd_spd_world.y + alpha * approach_spd_world.y;
        target_angle = lerp_angle_shortest(target_angle, (*path).end_angle, alpha);
    }

    // 混合后再计算角速度，保证角度目标连续
    rotation_spd = PID_Angle_Calculate(&chassis_yaw_pid, target_angle, now_pos);

    vec2 sum_spd_world = sum_vec2(target_spd_world, correct_spd_world);// 总速度

    // 2.4. 局部速度输出
    vec2 spd_local_final = change_world_to_local(sum_spd_world, now_pos);// 速度转换到车身坐标系

    cha_remote(spd_local_final.x, spd_local_final.y, rotation_spd);// 为电机速度赋值

    return 0; // 路径正在进行中
}
#endif


//----------------------------------------分割线----------------------------//

