#include "Hfsm.h"

#define TOTAL_STICK  1  // 一区总共拿取的杆数量

//向上层发送信息
void send_flag_to_up(int flag){}
int receive_flag(){
    return 1;
}

//  一区逻辑
void Handle_MC_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.mc) {
        case MC_INIT:  //  初始状态
            // 初始化传感器，定位
            r2->sub_state.mc = MC_PICK_HEAD;
            break;

        case MC_PICK_HEAD:  //  出发取杆
            // 规则4.3.3: R2从端头架取下一个端头 [cite: 98]
            if (go_path_control(&path_test,spd_test) == 1) {
                r2->stick_count++;
                r2->sub_state.mc = MC_ASSEMBLE_WAIT;
            }
            break;

        case MC_ASSEMBLE_WAIT:  // 移动到组装位置并等待组装
            // 移动到预定组装位置，视觉对准长杆
            if (go_path_control(&path_test,spd_test) == 1) {
                r2->sub_state.mc = MC_ASSEMBLE_ACT;
            }
            break;

        case MC_ASSEMBLE_ACT:  // 执行组装动作
            // 规则4.3.6: 组装过程中R1与R2不得直接肢体接触
            // R2保持端头稳定，等待R1插入
            send_flag_to_up(FLAG_ASSEMBLE);  // 向上层发送组装信号
            if (receive_flag() == 1) {  // 收到组装完成的信号
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
            if (receive_flag()==1 || r2->r1_left_mc) {
                // 切换到顶层状态：进入梅林
                r2->current_top_state = STATE_MF_AREA;
                r2->sub_state.mf = MF_ENTRY;
            }
            break;
    }
}

//  二区逻辑
void Handle_MF_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.mf) {
        case MF_ENTRY: // 进入树林入口
            // 规则：从入口方块(1,2,3)进入，假设此处调用路径控制前往入口
            if (go_path_control(&path_test, spd_test) == 1) {
                // 进入成功后，调用 path_plan.c 中的算法进行全局规划
                // 假设输入地图数据 map，获取最优路径
                r2->plan = plan_route();
                r2->current_step = 1;
                r2->sub_state.mf = MF_ACTION_JUDGE;
            }
            break;

        case MF_ACTION_JUDGE: // 决策下一步动作
            if (r2->current_step >= r2->plan.path_len) {
                // 如果目标格子是出口，路径走完，准备退出
                r2->sub_state.mf = MF_EXIT_NAV;
            } else {
                // 获取当前路径点的目标
                r2->target_stair_id = r2->plan.path[r2->current_step];
                r2->approach_face = 0;  //calculate_face(r2->target_stair_id); // 根据位置计算朝向
                // 根据 path_plan.h 中的规划结果判断
                if (is_target_kfs(r2->target_stair_id)) {
                    r2->sub_state.mf = MF_PICK_ADJACENT;
                } else if (is_obstacle_kfs(r2->target_stair_id)) {
                    r2->sub_state.mf = MF_REMOVE_KFS;
                } else {
                    r2->current_step++;
                    r2->sub_state.mf = MF_MOVE_TO_BLOCK;
                }
            }
            break;

        case MF_MOVE_TO_BLOCK: // 爬楼梯移动
            // 调用您修改后的 int 返回值类型的 ClimbStairs
            int climb_status = ClimbStairs(r2->target_stair_id, r2->approach_face);
            if (climb_status == 1) {
                // 爬坡并定位完成后，判断该位置是要拿取 KFS 还是移除障碍
                // 根据 path_plan.h 中的规划结果判断
                if (is_target_kfs(r2->target_stair_id)) {
                    r2->sub_state.mf = MF_PICK_ADJACENT;
                } else if (is_obstacle_kfs(r2->target_stair_id)) {
                    r2->sub_state.mf = MF_REMOVE_KFS;
                } else {
                    r2->current_step++;
                    r2->sub_state.mf = MF_ACTION_JUDGE;
                }
            }
            break;

        case MF_PICK_ADJACENT: // 抓取相邻 KFS
            // 执行机械臂抓取动作
            if (Hardware_PickKFSAction()) {
                r2->kfs_count++; // R2 秘籍计数
                r2->current_step++;
                r2->sub_state.mf = MF_ACTION_JUDGE;
            }
            break;

        case MF_REMOVE_KFS: // 移除障碍 KFS
            // 规则 4.4.4: R2 可以移除阻碍路径的非目标 KFS（不能放入储藏区）
            if (Hardware_RemoveObstacleAction()) {
                r2->current_step++;
                r2->sub_state.mf = MF_ACTION_JUDGE;
            }
            break;

        case MF_EXIT_NAV: // 导航至出口
            // 规则：前往 10/11/12 号方块准备进入三区
            if (go_path_control(&exit_path, spd_test) == 1) {
                // 切换到顶级状态：三区对抗区
                r2->current_top_state = STATE_CF_AREA;
                r2->sub_state.cf = CF_INIT;
            }
            break;
    }
}

//  三区逻辑
void Handle_CF_Logic(R2_Context_t *r2) {
    switch (r2->sub_state.cf) {
        case CF_CLIMB_RAMP:
            // 爬坡进入对抗区
            if (Hardware_ClimbRamp()) {
                r2->sub_state.cf = CF_DECISION;
            }
            break;

        case CF_DECISION:
            // 根据场上局势决定策略
            if (Algorithm_NeedTopLayer()) {
                r2->sub_state.cf = CF_WAIT_LIFT;
            } else {
                r2->sub_state.cf = CF_PLACE_MID;
            }
            break;

        case CF_PLACE_MID:
            // 规则4.5.13: R2把KFS放到九宫格中层
            if (Hardware_PlaceKFS_Middle()) {
                r2->kfs_count--;
                r2->sub_state.cf = CF_DECISION; // 循环决策，直到放完
            }
            break;

        case CF_WAIT_LIFT:
            // 规则3.8(3): R1举起R2 [cite: 121]
            // R2检测自身IMU或高度传感器确认被举起
            if (Sensors_IsLifted()) {
                r2->sub_state.cf = CF_PLACE_TOP;
            }
            break;

        case CF_PLACE_TOP:
            // 规则4.5.16: 被R1举起后放置顶层
            if (Hardware_PlaceKFS_Top()) {
                r2->kfs_count--;
                // 放置完成后等待R1放下
                if (Sensors_IsOnGround()) {
                    r2->sub_state.cf = CF_DECISION;
                }
            }
            break;
    }

    // 规则3.9: 如果获得“武术大师”，立即获胜 [cite: 121]
    if (Referee_IsMartialArtsMaster()) {
        r2->current_top_state = STATE_FINISHED;
    }
}

int main_loop() {
    R2_Context_t robot_ctx = {0};
    robot_ctx.current_top_state = STATE_MC_AREA;
    robot_ctx.sub_state.mc = MC_INIT;
    // 全局安全检测
    if (Sensors_EmergencyStopPressed()) {
        robot_ctx.current_top_state = STATE_EMERGENCY;
    }

    // 分层状态机调度
    switch (robot_ctx.current_top_state) {
        case STATE_MC_AREA:
            Handle_MC_Logic(&robot_ctx);
            break;

        case STATE_MF_AREA:
            Handle_MF_Logic(&robot_ctx);
            break;

        case STATE_CF_AREA:
            Handle_CF_Logic(&robot_ctx);
            break;

        case STATE_FINISHED:
            Hardware_StopAllMotors();
            // 庆祝动作
            break;

        case STATE_EMERGENCY:
            Hardware_StopAllMotors();
            // 等待复位
            break;
    return 0;
}