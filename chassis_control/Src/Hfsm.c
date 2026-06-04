#include "Hfsm.h"

#include <math.h>

#include "chassis_driver.h"
#include "chassis_pid.h"
#include "locator_driver.h"

#define TOTAL_STICK  1  // 一区总共拿取的杆数量

#define PICK_HEAD_TARGET_X_MM          1000.0f
#define PICK_HEAD_TARGET_R_RAD         0.0f
#define PICK_HEAD_TARGET_LASER_MM      300.0f

/* 0: use 0x100 laser, 1: use 0x101 laser. Change this according to the half field. */
#define PICK_HEAD_USE_LASER_2          0U
/* If the car moves away from the target when laser error is positive, change this to -1.0f. */
#define PICK_HEAD_LASER_CMD_SIGN       1.0f

Point_struct slope_entry = {3890,8390};
Point_struct slope_end = {3890,10500};
Point_struct MF_wait_point = {-710,10100};

R2_Context_t g_robot_ctx = {
    .current_top_state = STATE_MC_AREA,
    .sub_state.mc = MC_INIT,
    .stick_count = 0,
    .already_taken = -1,
    .current_r2_taken_idx = 0,
    .r2_taken_mask = 0,
    .kfs_count = 0,
    .path_inited = false
};
volatile int MF_flag = 0;
volatile int MC_flag = 0;
volatile int CF_flag = 0;

//向上层发送信息
uint8_t send_flag_to_up(uint8_t id, uint8_t data_byte)
{
    uint8_t data[1] = {data_byte};
    return fdcanx_send_ex_data(&hfdcan3, 0x300+id, data, 1, CAN_ID_STD);
}

// 接收信号
int receive_flag(){
    return MF_flag;
}

// 判断在该节点是否要拿取目标KFS
bool is_target_kfs(int8_t current_id, int8_t r2_taken_id) {
    return is_adjacent(current_id,r2_taken_id);
    //return (target_id == res.r2_taken[0] || target_id == res.r2_taken[1]);
}

// 判断是否是障碍KFS
static bool is_obstacle_kfs(int target_id, const PlanResult *res) {
    for (int i = 0; i < res->r2r_cnt && i < MAX_R2_REMOVE; i++) {
        if (target_id == res->r2_removed[i]) {
            return true;
        }
    }
    return false;
}

static bool is_r2_taken_done(R2_Context_t *r2, int idx) {
    return (r2->r2_taken_mask & (1U << idx)) != 0;
}

static void mark_r2_taken_done(R2_Context_t *r2, int idx) {
    r2->r2_taken_mask |= (uint8_t)(1U << idx);
    r2->already_taken = idx;
}

static int find_next_r2_taken_idx(R2_Context_t *r2) {
    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        if (!is_r2_taken_done(r2, i)) {
            return i;
        }
    }
    return -1;
}

static int find_reachable_r2_taken_idx(R2_Context_t *r2, int8_t current_id) {
    int idx = find_next_r2_taken_idx(r2);
    if (idx >= 0 && is_target_kfs(current_id, r2->plan.r2_taken[idx])) {
        return idx;
    }
    return -1;
}

static bool is_entry_side_r2(int8_t target_id) {
    return target_id == 0 || target_id == 2;
}

static void set_top_state(R2_Context_t *r2, TopState_t next) {
    if (r2->current_top_state != next) {
        r2->current_top_state = next;
        r2->path_inited = false;
    }
}

static void set_mc_state(R2_Context_t *r2, MCSubState_t next) {
    if (r2->sub_state.mc != next) {
        r2->sub_state.mc = next;
        r2->path_inited = false;
    }
}

static void set_mf_state(R2_Context_t *r2, MFSubState_t next) {
    if (r2->sub_state.mf != next) {
        r2->sub_state.mf = next;
        r2->path_inited = false;
    }
}

static void set_cf_state(R2_Context_t *r2, CFSubState_t next) {
    if (r2->sub_state.cf != next) {
        r2->sub_state.cf = next;
        r2->path_inited = false;
    }
}


//  一区逻辑
static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

static float angle_error_rad(float target, float current)
{
    float err = target - current;

    while (err > pi) {
        err -= 2.0f * pi;
    }
    while (err < -pi) {
        err += 2.0f * pi;
    }
    return err;
}

static float pick_head_get_laser_mm(void)
{
    return (PICK_HEAD_USE_LASER_2 != 0U) ? lcResult.laser_current_2 : lcResult.laser_current_1;
}

static int pick_head_laser_control_loop(uint8_t reset)
{
    static uint8_t stable_cnt = 0U;
    static bool aligned = false;
    const float laser_min_mm = 30.0f;
    const float laser_max_mm = 1500.0f;
    const float x_ok_mm = 15.0f;
    const float r_ok_rad = 0.03f;
    const float laser_ok_mm = 2.0f;
    const float x_kp = 1.0f;
    const float laser_kp = 1.5f;
    const float x_max_speed_mm_s = 250.0f;
    const float laser_max_speed_mm_s = 120.0f;
    const uint8_t stable_target = 5U;

    if (reset != 0U) {
        stable_cnt = 0U;
        aligned = false;
        cha_remote(0.0f, 0.0f, 0.0f);
        return 0;
    }

    if (aligned) {
        cha_remote(0.0f, 0.0f, 0.0f);
        return 1;
    }

    const float laser = pick_head_get_laser_mm();
    const bool laser_valid = laser >= laser_min_mm && laser <= laser_max_mm;
    const float x_err = PICK_HEAD_TARGET_X_MM - lcResult.x;
    const float r_err = angle_error_rad(PICK_HEAD_TARGET_R_RAD, lcResult.r);
    float laser_err = 0.0f;

    const float x_cmd_world = clamp_float(x_kp * x_err,
                                          -x_max_speed_mm_s,
                                          x_max_speed_mm_s);
    vec2 cmd_local = change_world_to_local((vec2){x_cmd_world, 0.0f}, lcResult.r);
    const float vr = PID_Angle_Calculate(&chassis_yaw_pid, PICK_HEAD_TARGET_R_RAD, lcResult.r);

    if (laser_valid) {
        const float laser_cmd = clamp_float(PICK_HEAD_LASER_CMD_SIGN *
                                            laser_kp *
                                            (laser - PICK_HEAD_TARGET_LASER_MM),
                                            -laser_max_speed_mm_s,
                                            laser_max_speed_mm_s);

        laser_err = laser - PICK_HEAD_TARGET_LASER_MM;
        cmd_local.y = laser_cmd;
    }

    if (fabsf(x_err) < x_ok_mm &&
        fabsf(r_err) < r_ok_rad &&
        laser_valid &&
        fabsf(laser_err) < laser_ok_mm) {
        stable_cnt++;
    } else {
        stable_cnt = 0U;
    }

    if (stable_cnt >= stable_target) {
        stable_cnt = 0U;
        aligned = true;
        cha_remote(0.0f, 0.0f, 0.0f);
        return 1;
    }

    cha_remote(cmd_local.x, cmd_local.y, vr);
    return 0;
}

void Handle_MC_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.mc) {
        case MC_INIT:  //  初始状态
            // 初始化传感器，定位
            set_mc_state(r2, MC_PICK_HEAD);
            break;

        case MC_PICK_HEAD:  //  出发取端头
            // 规则4.3.3: R2从端头架取下一个端头 [cite: 98]
            if (!r2->path_inited) {
                pick_head_laser_control_loop(1U);
                r2->path_inited = true;
            }
            if (pick_head_laser_control_loop(0U) == 1) {
               // printf("MC_PICK_HEAD\n");
                if (MC_flag==1) {   // 收到上层信息
                    r2->stick_count++;
                    set_mc_state(r2, MC_ASSEMBLE_WAIT);
                }
            }
            break;

        case MC_ASSEMBLE_WAIT:  // 移动到组装位置并等待组装
            // 移动到预定组装位置，视觉对准长杆
            if (go_path_control(&path_test,spd_test) == 1) {
               // printf("MC_ASSEMBLE_WAIT\n");
                if (MC_flag==2) set_mc_state(r2, MC_ASSEMBLE_ACT);
            }
            break;

        case MC_ASSEMBLE_ACT:  // 执行组装动作
            // 规则4.3.6: 组装过程中R1与R2不得直接肢体接触
            // R2保持端头稳定，等待R1插入
            send_flag_to_up(FLAG_ASSEMBLE,0);  // 向上层发送组装信号
            if (MC_flag==3) {  // 收到组装完成的信号
                if (r2->stick_count < TOTAL_STICK) {
                    set_mc_state(r2, MC_PICK_HEAD);
                }else {
                    r2->weapon_ready = true;
                    set_mc_state(r2, MC_WAIT_R1_EXIT);
                }
            }
            break;

        case MC_WAIT_R1_EXIT:
            // 规则4.3.10: 只有在R1完全离开武馆后，R2才能离开
            if (MC_flag==4 || r2->r1_left_mc) {
                // 切换到顶层状态：进入梅林
                r2->plan = plan_route(initial_map); // 规划路径
                r2->already_taken = -1;
                r2->current_r2_taken_idx = 0;
                r2->r2_taken_mask = 0;
                set_top_state(r2, STATE_MF_AREA);
                set_mf_state(r2, MF_ENTRY_CHECK);
            }
            break;
    }
}

//  二区逻辑
void Handle_MF_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.mf) {
        case MF_ENTRY_CHECK:
            r2->current_step = 1;
            r2->target_stair_id = r2->plan.path[r2->current_step];
            r2->current_stair_id = r2->plan.path[r2->current_step-1];
            int idx = find_next_r2_taken_idx(r2);
            if (idx < 0 || r2->plan.r2_taken[idx]==1) {
                set_mf_state(r2, MF_ENTRY);
            }else if (is_entry_side_r2(r2->plan.r2_taken[idx])) {
                r2->current_r2_taken_idx = idx;
                if (!r2->path_inited) {
                    Point_struct cur_point = {lcResult.x,lcResult.y};
                    init_single_line_path(&path_test,cur_point,entry_point[r2->plan.r2_taken[idx]],lcResult.r,0);
                    r2->path_inited = true;
                }
                if (go_path_control(&path_test, spd_test) == 1) {
                    set_mf_state(r2, MF_PICK_ADJACENT);
                }
            }else {
                set_mf_state(r2, MF_ENTRY);
            }

            break;
        case MF_ENTRY: // 进入树林入口
            // 规则：从入口方块(1,2,3)进入，假设此处调用路径控制前往入口
            if (!r2->path_inited) {
                Point_struct cur_point = {lcResult.x,lcResult.y};
                init_single_line_path(&path_test,cur_point,entry_point[1],lcResult.r,0);
                r2->path_inited = true;
            }
            if (go_path_control(&path_test, spd_test) == 1) {
                // 进入成功后，调用 path_plan.c 中的算法进行全局规划
                // 假设输入地图数据 map，获取最优路径
                if (MF_flag==1) {
                    r2->current_step = 1;  // 第一步为走到入口处
                    set_mf_state(r2, MF_ACTION_JUDGE);
                }
            }
            break;

        case MF_ACTION_JUDGE: // 决策下一步动作
            // 获取当前路径点的目标
            if (MF_flag==2) {
                r2->target_stair_id = r2->plan.path[r2->current_step];
                r2->current_stair_id = r2->plan.path[r2->current_step-1];
                if (r2->current_step >= r2->plan.path_len-1) {
                    // 如果当前走到了倒数第二步，即目标格子是出口，路径走完，准备退出
                    set_mf_state(r2, MF_EXIT_NAV);
                } else {
                    int idx = find_reachable_r2_taken_idx(r2, r2->current_stair_id);
                    // 根据 path_plan.h 中的规划结果判断
                    if (idx >= 0) {
                        r2->current_r2_taken_idx = idx;
                        set_mf_state(r2, MF_PICK_ADJACENT);
                    } else if (is_obstacle_kfs(r2->target_stair_id,&r2->plan)) {
                        // 如果目标节点是要移出的 R2 KFS
                        set_mf_state(r2, MF_REMOVE_KFS);
                    } else {
                        // 如果目标节点是 KFS_NONE 或 R1_KFS
                        set_mf_state(r2, MF_MOVE_TO_BLOCK);
                    }
                }
            }
            break;

        case MF_MOVE_TO_BLOCK: // 爬楼梯移动
            // // 获取路径中下一个要去的节点
            // int8_t next_node = r2->plan.path[r2->current_step];
            //
            // // --- 局部修正检查 ---
            // // 假设通过视觉或通信获取 R1 当前位置
            // int8_t r1_current_pos = Get_R1_Position_Via_Comm();
            // if (next_node == r1_current_pos) {
            //     // 如果 R1 挡住了路，调用新的修正函数重规划，绕开 r1_current_pos
            //     State new_res;
            //     if (Path_Replan_With_Obstacles(r2->current_node, r2->target_node, r1_current_pos, &new_res, parents)) {
            //         r2->path_count = reconstruct_path(r2->current_node, r2->target_node, parents, r2->planned_path);
            //         r2->current_step_idx = 1; // 重新开始新路径
            //         break; // 退出当前 switch，等待下一帧处理新路径
            //     }
            // }
            // 调用您修改后的 int 返回值类型的 ClimbStairs
            //printf("%d   %d\n",r2->current_stair_id,r2->target_stair_id);
            if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[r2->target_stair_id]<0) {
                //ClimbStairs(r2->current_stair_id, r2->target_stair_id);
                 if (ClimbStairs(r2->current_stair_id, r2->target_stair_id)) {
                     // 上楼梯完成，step++，返回判断阶段
                     r2->current_step++;
                     set_mf_state(r2, MF_ACTION_JUDGE);
                 }
            }else {
                if (DownStairs(r2->current_stair_id,r2->target_stair_id)) {
                    // 上楼梯完成，step++，返回判断阶段
                    r2->current_step++;
                    set_mf_state(r2, MF_ACTION_JUDGE);
                }
            }
            break;

        case MF_PICK_ADJACENT: // 抓取相邻 KFS r2_taken[current_r2_taken_idx]
            // Pick current planned R2 KFS.
            {
                int idx = r2->current_r2_taken_idx;
                if (idx < 0 || idx >= R2_TAKEN_COUNT) {
                    set_mf_state(r2, MF_ACTION_JUDGE);
                    break;
                }
                int8_t target = r2->plan.r2_taken[idx];
                if (Move_to_Edge(r2->current_stair_id,target)) {
                    if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[target]<0 && target-r2->current_stair_id==3)
                    {
                        send_flag_to_up(FLAG_GRAB_KFS_FRONT_HIGH_KEEP,0);
                    }else if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[target]>0 && target-r2->current_stair_id==3)
                    {
                        send_flag_to_up(FLAG_GRAB_KFS_FRONT_LOW_KEEP,0);
                    }else if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[target]<0 && target-r2->current_stair_id==1)
                    {
                        send_flag_to_up(FLAG_GRAB_KFS_LEFT_HIGH_KEEP,0);
                    }else if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[target]>0 && target-r2->current_stair_id==1)
                    {
                        send_flag_to_up(FLAG_GRAB_KFS_LEFT_LOW_KEEP,0);
                    }else if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[target]<0 && target-r2->current_stair_id==-1)
                    {
                        send_flag_to_up(FLAG_GRAB_KFS_RIGHT_HIGH_KEEP,0);
                    }else if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[target]>0 && target-r2->current_stair_id==-1)
                    {
                        send_flag_to_up(FLAG_GRAB_KFS_RIGHT_LOW_KEEP,0);
                    }
                    if (MF_flag==3) {  // 抓取成功
                        r2->kfs_count++;
                        mark_r2_taken_done(r2, idx);
                        if (is_entry_side_r2(target) && r2->current_stair_id==ENTRY_NODE) {
                            r2->plan.entry_grab = 0;
                            set_mf_state(r2, MF_ENTRY);
                        }else if (target==r2->target_stair_id) {
                            set_mf_state(r2, MF_MOVE_TO_BLOCK);
                        }else{
                            set_mf_state(r2, MF_BACK_TO_CENTER);
                        }
                    }
                }
            }
            break;

        case MF_BACK_TO_CENTER:
            if (Move_back_to_Center(r2->current_stair_id)) {    // 如果kfs所在方块不是要移动的目标方块，返回中心进行判断
                set_mf_state(r2, MF_ACTION_JUDGE);
            }
            break;

        case MF_REMOVE_KFS: // 移除障碍 KFS
            // 规则 4.4.4: R2 可以移除阻碍路径的非目标 KFS（不能放入储藏区）
            if (Move_to_Edge(r2->current_stair_id,r2->target_stair_id)) {
                if (HEIGHT_MAP[r2->current_stair_id]-HEIGHT_MAP[r2->target_stair_id]<0)
                {
                    send_flag_to_up(FLAG_GRAB_KFS_FRONT_HIGH_REMOVE,0);
                }else
                {
                    send_flag_to_up(FLAG_GRAB_KFS_FRONT_LOW_REMOVE,0);
                }

                if (MF_flag==4) {
                    set_mf_state(r2, MF_MOVE_TO_BLOCK);
                }
            }
            break;

        case MF_EXIT_NAV: // 导航至出口
            if (DownStairs(r2->current_stair_id,r2->current_stair_id+3)) {
                // 切换到顶级状态：三区对抗区
                if (MF_flag==5) {
                    set_top_state(r2, STATE_CF_AREA);
                    set_cf_state(r2, CF_CLIMB_RAMP);
                }
            }
            break;
    }
}

//  三区逻辑
void Handle_CF_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.cf) {
        case CF_CLIMB_RAMP:
            // 爬坡进入对抗区
            if (!r2->path_inited) {
                Trajectory tra_slope[3];
                Point_struct cur_point = {lcResult.x,lcResult.y};
                tra_slope[0] = generate_line_trajectory(cur_point,slope_entry,full);
                tra_slope[1] = generate_line_trajectory(slope_entry,slope_end,full);
                tra_slope[2] = generate_line_trajectory(slope_end,MF_wait_point,empty);
                init_custom_path(&path_test,tra_slope,3,lcResult.r,-pi/2.0f);
                r2->path_inited = true;
            }
            if (go_path_control(&path_test, spd_test) == 1) {   // 移动到决策位置
                set_cf_state(r2, CF_DECISION);
            }
            break;

        case CF_DECISION:
            // 根据场上局势决定策略
            if (go_path_control(&path_test, spd_test) == 1) {   // 写一条从当前位置移动到决策位置的路径，然后移动到决策位置
                if (CF_flag==1) {   // 收到放顶层的决策
                    set_cf_state(r2, CF_WAIT_LIFT);
                } else {
                    set_cf_state(r2, CF_PLACE_MID);
                }
                if (r2->kfs_count == 0) {
                    set_top_state(r2, STATE_FINISHED);     //暂时不考虑回到梅林区
                }
            }
            break;

        case CF_PLACE_MID:
            // 规则4.5.13: R2把KFS放到九宫格中层
            if (go_path_control(&path_test, spd_test) == 1) {
                send_flag_to_up(FLAG_PUT_KFS_MID,0);
                if (CF_flag==2){
                    r2->kfs_count--;
                    set_cf_state(r2, CF_DECISION); // 循环决策，直到放完
                }
            }
            break;

        case CF_WAIT_LIFT:
            // 规则3.8(3): R1举起R2 [cite: 121]
            // R2检测自身IMU或高度传感器确认被举起
            if (go_path_control(&path_test, spd_test) == 1) {   // 移动到被抬起的位置
                send_flag_to_up(FLAG_LIFT,0);
                if (CF_flag==3){
                    set_cf_state(r2, CF_PLACE_TOP);
                }
            }
            break;

        case CF_PLACE_TOP:
            // 规则4.5.16: 被R1举起后放置顶层
            if (CF_flag==4) {   // 收到r1移动到位指令
                send_flag_to_up(FLAG_PUT_KFS_TOP,0);     //向上层发送放置KFS到顶层的指令
                // 放置完成后等待R1放下
                if (CF_flag==5) {
                    r2->kfs_count--;
                    set_cf_state(r2, CF_DECISION);
                }
            }
            break;
    }

    // 规则3.9: 如果获得“武术大师”，立即获胜 [cite: 121]
    if (CF_flag==6) {   // 收到大胜指令
        set_top_state(r2, STATE_FINISHED);
    }
}

int chassis_auto_control(R2_Context_t *robot_ctx) {
    // R2_Context_t robot_ctx = {0};
    // robot_ctx.current_top_state = STATE_MC_AREA;
    // robot_ctx.sub_state.mc = MC_INIT;
    // 全局安全检测
    // if (Sensors_EmergencyStopPressed()) {
    //     robot_ctx.current_top_state = STATE_EMERGENCY;
    // }

    // 分层状态机调度
    switch (robot_ctx->current_top_state) {
        case STATE_MC_AREA:
            Handle_MC_Logic(robot_ctx);
            break;

        case STATE_MF_AREA:
            Handle_MF_Logic(robot_ctx);
            break;

        case STATE_CF_AREA:
            Handle_CF_Logic(robot_ctx);
            break;

        case STATE_FINISHED:
            // 庆祝动作
            return 1;
            break;

        case STATE_EMERGENCY:
            //执行重试路径
            //Hardware_StopAllMotors();
            // 等待复位
            break;
    }
    return 0;
}
