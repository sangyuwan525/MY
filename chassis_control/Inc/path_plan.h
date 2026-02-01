
#ifndef PATH_PLAN_H
#define PATH_PLAN_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


// ================= 配置与宏 =================
// 网格行列与节点编号约定：
// - 网格为 4 行 × 3 列，总共有 12 个实际格点（编号 0..11）
// - 入口和出口使用虚拟节点编号，入口为 12，出口为 13
#define ROWS 4
#define COLS 3
#define GRID_NODES 12
#define ENTRY_NODE 12
#define EXIT_NODE 13
#define TOTAL_NODES 14

// 路径长度上限、R1 使用上限、允许移除的 R2 数量等常量
#define MAX_PATH_LEN 64
#define MAX_R1_LIMIT 2
#define MAX_R2_REMOVE 2
// INF 用作不可达或初始大值
#define INF 32767

// KFS 类型说明：
// - KFS_NONE: 无特殊设施
// - KFS_R1: 第一类 KFS（会增加 r1_used 计数）
// - KFS_R2: 第二类 KFS（为必须取用或可被移除的目标）
// - KFS_FAKE: 假节点（不可通行）
typedef enum { KFS_NONE=0, KFS_R1, KFS_R2, KFS_FAKE } KFS_Type;

// ================= 搜索状态 =================
// State 表示 Dijkstra / 状态图中的一个节点状态（不仅仅是位置，还包含资源/已移除信息）
typedef struct {
    int8_t node;        // 当前所在节点索引（0..GRID_NODES-1 或入口/出口虚拟节点）
    int8_t r1_used;     // 到当前状态为止，已通过 / 使用的 R1 节点数量（用于约束）
    int8_t r2_removed;  // 到当前状态为止，被额外（非目标）移除的 R2 数量
    uint8_t r2_mask;    // R2 采集遮罩（位掩码）：bit0 表示已取走 t1，bit1 表示已取走 t2
    int16_t cost;       // 到达该状态的累计代价（用于 Dijkstra 优先级比较）
} State;

// ================= 规划结果 =================
// PlanResult 用于保存回溯得到的最终规划结果，包括路径、移除信息和代价
typedef struct {
    int8_t path[MAX_PATH_LEN]; // 保存路径节点序列（从入口到出口）
    int8_t path_len;          // 路径长度（节点数）

    int8_t r1_removed[MAX_R1_LIMIT]; // 记录被移除的 R1 节点（按发现顺序）
    int8_t r1_cnt;                    // 被移除的 R1 数量

    int8_t r2_removed[MAX_R2_REMOVE]; // 记录被非目标移除的 R2 节点
    int8_t r2r_cnt;                   // 被移除的 R2 数量

    int8_t r2_taken[2]; // 实际被作为任务目标取走的两个 R2 节点索引
    int16_t cost;       // 最终代价
} PlanResult;

extern KFS_Type map[TOTAL_NODES];
PlanResult plan_route();

#endif
