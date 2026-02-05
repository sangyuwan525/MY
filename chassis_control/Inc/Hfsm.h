
#ifndef HFSM_H
#define HFSM_H
#include <stdbool.h>
#include <stdio.h>
#include "ClimbStairs.h"
#include "chassis_path.h"
#include "path.h"
#include "path_plan.h"

// --- 向上层发送的动作指令 ---
typedef enum {
    FLAG_ASSEMBLE,      // 一区组装武器指令
    FLAG_GRAB_KFS,      // 二区抓取KFS指令
    FLAG_REMOVE_KFS,    // 二区移出KFS指令
    FLAG_PUT_KFS_MID,   // 三区放置KFS到中层指令
    FLAG_PUT_KFS_TOP,   // 三区放置KFS到顶层指令
    FLAG_LIFT,          // 三区爬上R1指令
} FLAG_TO_UP;

// --- 顶级状态：区域逻辑 (Top-Level States) ---
typedef enum {
    STATE_MC_AREA,    // 一区：武馆 (Martial Arts Hall)
    STATE_MF_AREA,    // 二区：梅林 (Plum Blossom Forest)
    STATE_CF_AREA,    // 三区：对抗区 (Confrontation Area)
    STATE_EMERGENCY,  // 紧急停止
    STATE_FINISHED    // 完赛
} TopState_t;

// --- 子状态：一区武馆 (Sub-states for MC) ---
typedef enum {
    MC_INIT,            // 初始状态
    MC_PICK_HEAD,       // 抓取端头
    MC_ASSEMBLE_WAIT,   // 前往组装位等待R1
    MC_ASSEMBLE_ACT,    // 配合组装动作
    MC_WAIT_R1_EXIT     // 等待R1离开武馆
} MCSubState_t;

// --- 子状态：二区梅林 (Sub-states for MF) ---
typedef enum {
    MF_ENTRY,           // 进入树林入口
    MF_ACTION_JUDGE,    // 下一步动作判断，移动还是拿取还是移出
    MF_MOVE_TO_BLOCK,   // 移动到目标方块
    MF_PICK_ADJACENT_0,   // 抓取相邻方块的KFS r2_taken[0]
    MF_PICK_ADJACENT_1,   // 抓取相邻方块的KFS r2_taken[1]
    MF_REMOVE_KFS,      // 移除相邻方块上的KFS
   // MF_RECOVER_STUCK,   // 【新增】跌落或堵塞恢复
    MF_EXIT_NAV         // 导航至出口 (10/11/12号方块) [cite: 133]
} MFSubState_t;

// --- 子状态：三区对抗区 (Sub-states for CF) ---
typedef enum {
    CF_CLIMB_RAMP,      // 爬坡
    CF_DECISION,        // 策略判定 (放中层还是配合R1放顶层)
    CF_PLACE_MID,       // 放中层 [cite: 121]
    CF_WAIT_LIFT,       // 等待被R1举起 [cite: 123]
    CF_PLACE_TOP,       // 放顶层 [cite: 121]
   // CF_CELEBRATE        // 庆祝/待机
} CFSubState_t;

// --- 机器人数据上下文 ---
typedef struct {
    TopState_t current_top_state;
    union {
        MCSubState_t mc;
        MFSubState_t mf;
        CFSubState_t cf;
    } sub_state;

    int stick_count;      // 已取杆的数量
    bool weapon_ready;    // 兵器是否组装完成
    bool r1_left_mc;      // R1是否已离开武馆信号
    bool is_lifted;       // 是否被R1举起
    PlanResult plan;       // 存储 path_plan.c 生成的全局规划结果
    int current_step;      // 当前执行到规划路径的第几步
    int target_stair_id;   // 当前目标方块ID
    int current_stair_id;     // 当前方块ID
    int already_taken;    // 已经取得kfs方块id，-1表示未取，0表示取r2_taken[0],1表示取r2_taken[1],2表示都已取得，初始为-1
    int kfs_count;        // 持有的kfs数量，初始为0
} R2_Context_t;

extern int MF_flag;
extern int MC_flag;
extern int CF_flag;
extern R2_Context_t g_robot_ctx;
int chassis_auto_control(R2_Context_t *robot_ctx);

#endif
