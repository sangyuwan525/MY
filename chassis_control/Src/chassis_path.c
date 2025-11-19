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
        if (fabsf(O_to_p.y)<1e-4f) {
            dir.x = 0.0f;
            dir.y = 1.0f;
        }
        else {
            dir.x = 1.0f;
            dir.y = -(O_to_p.x / O_to_p.y);
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
 * @note   1. 该函数采用带平滑起始区的线性插值方法来过渡朝向角。
 * 2. 在路径开始的 10% 距离内 (path_pos < length * 0.1)，朝向角保持为 start_angle (tpro=0)。
 * 3. 在路径的 10% 到 55% 距离内 (tpro 从 0 增加到 1.0)，朝向角从 start_angle 线性插值到 end_angle。
 * 4. 在路径的 55% 之后 (tpro > 1.0f)，朝向角保持为 end_angle。
 * 5. 插值公式为: angle = start_angle + (end_angle - start_angle) * tpro。
 *
 * @par 内部 tpro 逻辑分析:
 * - 当 path_pos < 0.1*length 时，tpro = 0。
 * - 临界点 path_pos = 0.55*length 时，tpro = 2 * 0.55*length / length = 1.1，被钳位到 1.0f。
 * - 临界点 path_pos = 0.5*length 时，tpro = 2 * 0.5*length / length = 1.0f。
 * - 实际上，该函数在路径的 **0% 到 50%** 距离内完成朝向角的过渡。
 * @date   2025/11/02
 */
float get_angle_in_path(float path_pos,float length,float start_angle,float end_angle) {
    float tpro;
    if (path_pos < length * 0.1) {
        tpro = 0;
    }
    else {
        tpro = 2 * path_pos /length;
    }
    if (tpro > 1.0f) tpro = 1.0f;
    return start_angle + (end_angle - start_angle) * tpro;
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
        abs_spd = v_min + (path_spd.max_speed - v_min) * (1.0f - 1.0f / (1.0f + expf(-k * (path_remain - path_spd.down_stage / 2.0f))));

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

int go_path_control(Path_struct* path, path_spd_data_t path_spd)
{
    const Point_struct now_point = {lcResult.x, lcResult.y}; // 机器人当前坐标点
    const float now_pos = lcResult.yaw;                        // 机器人当前朝向角
    // 检查当前轨迹段是否有效，防止越界
    if ((*path).trajectory_count >= (*path).trajectory_num) {
        cha_remote(0.0f, 0.0f, 0.0f);
        return 1;
    }

    // 当前轨迹段的终点
    Point_struct current_end_point = (*path).trajectories[(*path).trajectory_count].point_end;
	float distance = get_length(now_point, current_end_point); // 当前点到轨迹段终点的直线距离

	// --- 1. 轨迹切换/末端靠近阶段 ---
	if (distance < 550.0f)
	{
		// 1.1. 非终点轨迹段的切换逻辑
		if((*path).trajectories[(*path).trajectory_count].ifvoid != empty&& distance <= 200.0f)
		{
				(*path).trajectory_count++; // 进入下一段路径
				//printf("count %d\n", (*path).count);
                // ⚠️ 建议在这里重置 PID 状态
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

            // 角度规划
            float tar_ang_kaojin = (*path).start_angle + ((*path).end_angle - (*path).start_angle) * (
                                       1.0f - (distance / (*path).length));

            // 旋转速度计算，使用全局角度PID实例
            float vr = PID_Angle_Calculate(&chassis_yaw_pid, tar_ang_kaojin, now_pos);

            // 路径完成判断
            if (distance < 10.0f && fabsf((*path).end_angle - now_pos) < 0.1f &&
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

	// 2.1. 几何信息计算
	Point_struct foot_point = get_foot_point(now_point, (*path).trajectories[(*path).trajectory_count]);
	float path_pos = get_length_in_path(foot_point, (*path).trajectories, (*path).trajectory_count);

    // 2.2. 角度控制
	float target_angle = get_angle_in_path(path_pos, (*path).length, (*path).start_angle, (*path).end_angle);
	float rotation_spd = PID_Angle_Calculate(&chassis_yaw_pid, target_angle, now_pos);// 使用全局角度PID

	vec2 spd_dir = get_spd_dir(foot_point, (*path).trajectories[(*path).trajectory_count]);

	// 期望沿轨迹速度
	vec2 target_spd_world = get_spd_on_path_calculate(path_spd, path_pos, (*path).length, spd_dir);

	// 横向纠正速度，使用全局修正PID
	vec2 correct_spd_world = PID_Correct_Calculate(&chassis_correct_pid, now_point, foot_point);

	vec2 sum_spd_world = sum_vec2(target_spd_world, correct_spd_world);// 总速度

	// 2.4. 局部速度输出
	vec2 spd_local_final = change_world_to_local(sum_spd_world, now_pos);// 速度转换到车身坐标系

	cha_remote(spd_local_final.x, spd_local_final.y, rotation_spd);// 为电机速度赋值

	return 0; // 路径正在进行中
}



//----------------------------------------分割线----------------------------//

