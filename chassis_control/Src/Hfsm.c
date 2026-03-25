#include "Hfsm.h"

#include "locator_driver.h"

#define TOTAL_STICK  2  // 一区总共拿取的杆数量

R2_Context_t g_robot_ctx = {
    .current_top_state = STATE_MC_AREA,
    .sub_state.mc = MC_INIT,
    .stick_count = 0,
    .already_taken = -1,
    .kfs_count = 0
};
int MF_flag = 0;
int MC_flag = 0;
int CF_flag = 0;

//向上层发送信息
void send_flag_to_up(int flag){}

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
bool is_obstacle_kfs(int target_id, PlanResult res) {
    return (target_id == res.r2_removed[0] || target_id == res.r2_removed[1]);
}


//  一区逻辑
void Handle_MC_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.mc) {
        case MC_INIT:  //  初始状态
            // 初始化传感器，定位
            r2->sub_state.mc = MC_PICK_HEAD;
            break;

        case MC_PICK_HEAD:  //  出发取端头
            // 规则4.3.3: R2从端头架取下一个端头 [cite: 98]
            Point_struct now_point = {lcResult.x,lcResult.y};
            init_single_line_path(&path_test,now_point,entry_point,lcResult.r,0);
            if (go_path_control(&path_test,spd_test) == 1) {
               // printf("MC_PICK_HEAD\n");
                if (MC_flag==1) {   // 收到上层信息
                    r2->stick_count++;
                    r2->sub_state.mc = MC_ASSEMBLE_WAIT;
                }
            }
            break;

        case MC_ASSEMBLE_WAIT:  // 移动到组装位置并等待组装
            // 移动到预定组装位置，视觉对准长杆
            if (go_path_control(&path_test,spd_test) == 1) {
               // printf("MC_ASSEMBLE_WAIT\n");
                if (MC_flag==2) r2->sub_state.mc = MC_ASSEMBLE_ACT;
            }
            break;

        case MC_ASSEMBLE_ACT:  // 执行组装动作
            // 规则4.3.6: 组装过程中R1与R2不得直接肢体接触
            // R2保持端头稳定，等待R1插入
            send_flag_to_up(FLAG_ASSEMBLE);  // 向上层发送组装信号
            if (MC_flag==3) {  // 收到组装完成的信号
                if (r2->stick_count < TOTAL_STICK) {
                    r2->sub_state.mc = MC_PICK_HEAD;
                }else {
                    r2->weapon_ready = true;
                    r2->sub_state.mc = MC_WAIT_R1_EXIT;
                }
            }
            break;

        case MC_WAIT_R1_EXIT:
            // 规则4.3.10: 只有在R1完全离开武馆后，R2才能离开
            if (MC_flag==4 || r2->r1_left_mc) {
                // 切换到顶层状态：进入梅林
                r2->plan = plan_route(initial_map); // 规划路径
                r2->current_top_state = STATE_MF_AREA;
                r2->sub_state.mf = MF_ENTRY_CHECK;
            }
            break;
    }
}

//  二区逻辑
void Handle_MF_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.mf) {
        case MF_ENTRY_CHECK:
            if (r2->plan.r2_taken[0]==1 || r2->plan.r2_taken[1]==1) {
                r2->sub_state.mf = MF_ENTRY;
            }else if (r2->plan.r2_taken[0]==0) {
                Point_struct now_point = {lcResult.x,lcResult.y};
                init_single_line_path(&path_test,now_point,entry_point[0],lcResult.r,0);
                if (go_path_control(&path_test, spd_test) == 1) {
                    // 进入成功后，调用 path_plan.c 中的算法进行全局规划
                    // 假设输入地图数据 map，获取最优路径
                    if (Move_to_Edge(r2->current_stair_id,r2->plan.r2_taken[0])) {
                        send_flag_to_up(FLAG_GRAB_KFS);
                        if (MF_flag==3) {
                            // 抓取成功
                            r2->kfs_count++;
                            if (r2->already_taken==1) {
                                r2->already_taken = 2;  // 两个都已抓取
                            }else {
                                r2->already_taken = 0;  // 已抓取r2_taken[0]
                            }
                        }
                        r2->sub_state.mf = MF_ENTRY;
                    }
                }
            }else if (r2->plan.r2_taken[0]==2) {
                Point_struct now_point = {lcResult.x,lcResult.y};
                init_single_line_path(&path_test,now_point,entry_point[2],lcResult.r,0);
                if (go_path_control(&path_test, spd_test) == 1) {
                    // 进入成功后，调用 path_plan.c 中的算法进行全局规划
                    // 假设输入地图数据 map，获取最优路径
                    if (Move_to_Edge(r2->current_stair_id,r2->plan.r2_taken[0])) {
                        send_flag_to_up(FLAG_GRAB_KFS);
                        if (MF_flag==3) {
                            // 抓取成功
                            r2->kfs_count++;
                            if (r2->already_taken==1) {
                                r2->already_taken = 2;  // 两个都已抓取
                            }else {
                                r2->already_taken = 0;  // 已抓取r2_taken[0]
                            }
                        }
                        r2->sub_state.mf = MF_ENTRY;
                    }
                }
            }
            break;
        case MF_ENTRY: // 进入树林入口
            // 规则：从入口方块(1,2,3)进入，假设此处调用路径控制前往入口
            Point_struct cur_point = {lcResult.x,lcResult.y};
            init_single_line_path(&path_test,cur_point,entry_point[1],lcResult.r,0);
            if (go_path_control(&path_test, spd_test) == 1) {
                // 进入成功后，调用 path_plan.c 中的算法进行全局规划
                // 假设输入地图数据 map，获取最优路径
                if (MF_flag==1) {
                    r2->current_step = 1;  // 第一步为走到入口处
                    r2->sub_state.mf = MF_ACTION_JUDGE;
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
                    r2->sub_state.mf = MF_EXIT_NAV;
                } else {
                    // 根据 path_plan.h 中的规划结果判断
                    if (is_target_kfs(r2->current_stair_id,r2->plan.r2_taken[0])&&r2->already_taken!=0&&r2->already_taken!=2) {     // r2_taken[0]没被拿
                        // 如果当前节点是要执行拿取的 R2 KFS 0 的动作
                        r2->sub_state.mf = MF_PICK_ADJACENT_0;
                    } else if (is_target_kfs(r2->current_stair_id,r2->plan.r2_taken[1])&&r2->already_taken!=1&&r2->already_taken!=2) {  //r2_taken[1]没被拿
                        // 如果当前节点是要执行拿取的 R2 KFS 1 的动作
                        r2->sub_state.mf = MF_PICK_ADJACENT_1;
                    } else if (is_obstacle_kfs(r2->target_stair_id,r2->plan)) {
                        // 如果目标节点是要移出的 R2 KFS
                        r2->sub_state.mf = MF_REMOVE_KFS;
                    } else {
                        // 如果目标节点是 KFS_NONE 或 R1_KFS
                        r2->sub_state.mf = MF_MOVE_TO_BLOCK;
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
                     r2->sub_state.mf = MF_ACTION_JUDGE;
                 }
            }else {
                if (DownStairs(r2->current_stair_id,r2->target_stair_id)) {
                    // 上楼梯完成，step++，返回判断阶段
                    r2->current_step++;
                    r2->sub_state.mf = MF_ACTION_JUDGE;
                }
            }
            break;

        case MF_PICK_ADJACENT_0: // 抓取相邻 KFS r2_taken[0]
            // 执行机械臂抓取动作
            if (Move_to_Edge(r2->current_stair_id,r2->plan.r2_taken[0])) {
                send_flag_to_up(FLAG_GRAB_KFS);
                if (MF_flag==3) {  // 抓取成功
                    r2->kfs_count++;
                    if (r2->already_taken==1) {
                        r2->already_taken = 2;  // 两个都已抓取
                    }else {
                        r2->already_taken = 0;  // 已抓取r2_taken[0]
                    }
                    if (r2->plan.r2_taken[0]==r2->target_stair_id) {    // 如果kfs所在方块是要移动的目标方块，直接移动
                        r2->sub_state.mf = MF_MOVE_TO_BLOCK;
                    }else{
                        r2->sub_state.mf = MF_BACK_TO_CENTER;
                    }
                }
            }
            break;
        case MF_PICK_ADJACENT_1: // 抓取相邻 KFS r2_taken[1]
            // 执行机械臂抓取动作
            if (Move_to_Edge(r2->current_stair_id,r2->plan.r2_taken[1])) {
                send_flag_to_up(FLAG_GRAB_KFS);
                if (MF_flag==3) {  // 抓取成功
                    r2->kfs_count++;
                    if (r2->already_taken==0) {
                        r2->already_taken = 2;  // 两个都已抓取
                    }else {
                        r2->already_taken = 1;  // 已抓取r2_taken[1]
                    }
                    if (r2->plan.r2_taken[1]==r2->target_stair_id) {    // 如果kfs所在方块是要移动的目标方块，直接移动
                        r2->sub_state.mf = MF_MOVE_TO_BLOCK;
                    }else{
                        r2->sub_state.mf = MF_BACK_TO_CENTER;
                    }
                }
            }
            break;

        case MF_BACK_TO_CENTER:
            if (Move_back_to_Center(r2->current_stair_id)) {    // 如果kfs所在方块不是要移动的目标方块，返回中心进行判断
                r2->sub_state.mf = MF_ACTION_JUDGE;
            }
            break;

        case MF_REMOVE_KFS: // 移除障碍 KFS
            // 规则 4.4.4: R2 可以移除阻碍路径的非目标 KFS（不能放入储藏区）
            if (Move_to_Edge(r2->current_stair_id,r2->target_stair_id)) {
                send_flag_to_up(FLAG_REMOVE_KFS);
                if (MF_flag==4) {
                    r2->sub_state.mf = MF_MOVE_TO_BLOCK;
                }
            }
            break;

        case MF_EXIT_NAV: // 导航至出口
            if (DownStairs(r2->current_stair_id,r2->current_stair_id+3)) {
                // 切换到顶级状态：三区对抗区
                if (MF_flag==5) {
                    r2->current_top_state = STATE_CF_AREA;
                    r2->sub_state.cf = CF_CLIMB_RAMP;
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
            if (go_path_control(&path_test, spd_test) == 1) {   // 移动到决策位置
                r2->sub_state.cf = CF_DECISION;
            }
            break;

        case CF_DECISION:
            // 根据场上局势决定策略
            if (go_path_control(&path_test, spd_test) == 1) {   // 写一条从当前位置移动到决策位置的路径，然后移动到决策位置
                if (CF_flag==1) {   // 收到放顶层的决策
                    r2->sub_state.cf = CF_WAIT_LIFT;
                } else {
                    r2->sub_state.cf = CF_PLACE_MID;
                }
                if (r2->kfs_count == 0) {
                    r2->current_top_state = STATE_FINISHED;     //暂时不考虑回到梅林区
                }
            }
            break;

        case CF_PLACE_MID:
            // 规则4.5.13: R2把KFS放到九宫格中层
            if (go_path_control(&path_test, spd_test) == 1) {
                send_flag_to_up(FLAG_PUT_KFS_MID);
                if (CF_flag==2){
                    r2->kfs_count--;
                    r2->sub_state.cf = CF_DECISION; // 循环决策，直到放完
                }
            }
            break;

        case CF_WAIT_LIFT:
            // 规则3.8(3): R1举起R2 [cite: 121]
            // R2检测自身IMU或高度传感器确认被举起
            if (go_path_control(&path_test, spd_test) == 1) {   // 移动到被抬起的位置
                send_flag_to_up(FLAG_LIFT);
                if (CF_flag==3){
                    r2->sub_state.cf = CF_PLACE_TOP;
                }
            }
            break;

        case CF_PLACE_TOP:
            // 规则4.5.16: 被R1举起后放置顶层
            if (CF_flag==4) {   // 收到r1移动到位指令
                send_flag_to_up(FLAG_PUT_KFS_TOP);     //向上层发送放置KFS到顶层的指令
                // 放置完成后等待R1放下
                if (CF_flag==5) {
                    r2->kfs_count--;
                    r2->sub_state.cf = CF_DECISION;
                }
            }
            break;
    }

    // 规则3.9: 如果获得“武术大师”，立即获胜 [cite: 121]
    if (CF_flag==6) {   // 收到大胜指令
        r2->current_top_state = STATE_FINISHED;
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