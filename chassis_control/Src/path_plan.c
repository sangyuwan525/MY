
#include "path_plan.h"
#include "string.h"

// ================= 高度 =================
// HEIGHT_MAP 表示每个节点的绝对高度（单位任意，例如毫米）。
// 在移动条件中，只允许高度差为 200 的相邻节点间移动（即坡度限制）
const uint16_t HEIGHT_MAP[TOTAL_NODES] = {
    400,200,400,
    200,400,600,
    400,600,400,
    200,400,200,
    0,0
};

// 地图布局（示例）：数组下标对应节点编号（0..11 为网格，12=入口，13=出口）
KFS_Type initial_map[TOTAL_NODES]={
    KFS_R2, KFS_R2, KFS_R1,
    KFS_R2, KFS_R2, KFS_R1,
    KFS_FAKE, KFS_NONE, KFS_R1,
    KFS_NONE,KFS_NONE, KFS_R1,
    KFS_NONE, KFS_NONE
};

// ================= 邻接判断辅助函数 =================

static inline bool is_adjacent_grid(int8_t a, int8_t b){
    if(a < 0 || b < 0 || a >= GRID_NODES || b >= GRID_NODES) return false;
    int r1 = a / COLS, c1 = a % COLS;
    int r2 = b / COLS, c2 = b % COLS;
    return abs(r1 - r2) + abs(c1 - c2) == 1;
}

/**
 * @brief 判断两个节点是否在物理上相邻
 * @param a 节点A编号 (0-13)
 * @param b 节点B编号 (0-13)
 * @return bool 如果相邻返回 true，否则返回 false
 */
bool is_adjacent(int8_t a, int8_t b) {
    // 1. 同一节点不互为相邻
    if (a == b) return false;

    // 2. 处理入口 (ENTRY_NODE = 12) 的特殊连接
    // 入口连接网格的第一行：0, 1, 2
    if (a == ENTRY_NODE) return (b >= 0 && b <= 2);
    if (b == ENTRY_NODE) return (a >= 0 && a <= 2);

    // 3. 处理出口 (EXIT_NODE = 13) 的特殊连接
    // 出口连接网格的最后一行：9, 10, 11
    if (a == EXIT_NODE) return (b >= 9 && b <= 11);
    if (b == EXIT_NODE) return (a >= 9 && a <= 11);

    // 4. 处理标准 4x3 网格节点 (0-11)
    if (a < GRID_NODES && b < GRID_NODES) {
        int8_t row_a = a / COLS; // 行号 = 编号 / 3
        int8_t col_a = a % COLS; // 列号 = 编号 % 3
        int8_t row_b = b / COLS;
        int8_t col_b = b % COLS;

        // 计算曼哈顿距离：行差 + 列差
        // 如果距离为 1，说明是上下或左右相邻（不包含斜对角）
        int diff = abs(row_a - row_b) + abs(col_a - col_b);
        return (diff == 1);
    }

    return false;
}

typedef State ParentTable[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][R2_MASK_STATES];

static uint8_t target_mask_for_node(int8_t node, const int8_t targets[R2_TAKEN_COUNT]) {
    uint8_t mask = 0;
    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        if (targets[i] == node) {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static int popcount_u8(uint8_t value) {
    int count = 0;
    while (value != 0) {
        count += value & 1U;
        value >>= 1U;
    }
    return count;
}

static bool node_in_list(int8_t node, const int8_t list[R2_TAKEN_COUNT], int count) {
    for (int i = 0; i < count; i++) {
        if (list[i] == node) {
            return true;
        }
    }
    return false;
}

// ================= Dijkstra (带状态扩展) =================
// 使用扩展的 Dijkstra 搜索，在状态空间中同时跟踪：
// - 位置 node
// - 已使用的 R1 数量 r1_used（受限）
// - 被移除的非目标 R2 数量 r2_removed（受限）
// - 已采集目标 R2 的掩码 r2_mask（R2_TAKEN_COUNT 个 bit 表示各目标是否已采集）
// state 的代价保存在 State.cost 中，搜索目标是到达 target 且 r2_mask==R2_TARGET_MASK

bool run_dijkstra(
    int8_t start,
    int8_t target,
    const int8_t targets[R2_TAKEN_COUNT],
    KFS_Type *map,
    State *end,
    ParentTable parent
){
    static int16_t dist[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][R2_MASK_STATES];
    static bool used[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][R2_MASK_STATES];

    // 初始化阵列
    for(int i=0;i<TOTAL_NODES;i++)
        for(int j=0;j<=MAX_R1_LIMIT;j++)
            for(int k=0;k<=MAX_R2_REMOVE;k++)
                for(int m=0;m<R2_MASK_STATES;m++){
                    dist[i][j][k][m]=INF;
                    used[i][j][k][m]=false;
                }

    bool entry_has_r2 = (map[0] == KFS_R2 || map[1] == KFS_R2 || map[2] == KFS_R2);
    uint8_t entry_target_mask = (uint8_t)(target_mask_for_node(0, targets) |
                                         target_mask_for_node(1, targets) |
                                         target_mask_for_node(2, targets));
    if (entry_has_r2 && entry_target_mask == 0) {
        return false;
    }

    // =========================================================
    // 修复：只要门口有 R2，就不允许 0 代价直接走进去！
    // =========================================================
    if (map[1] == KFS_R2) {
        // 1 号有，强制优先拿 1 号
        uint8_t target_mask = target_mask_for_node(1, targets);
        if (target_mask == 0) return false;
        dist[start][0][0][target_mask] = 0;
    } else if (entry_has_r2) {
        // 1 号没有，但 0 或 2 有 R2。此时【必须】交出探身代价去拿目标
        uint8_t left_mask = target_mask_for_node(0, targets);
        uint8_t right_mask = target_mask_for_node(2, targets);
        if (left_mask != 0) dist[start][0][0][left_mask] = ENTRY_SIDE_GRAB_PENALTY;
        if (right_mask != 0) dist[start][0][0][right_mask] = ENTRY_SIDE_GRAB_PENALTY;
    } else {
        // 只有在入口处完全干干净净没有任何 R2 时，才能 0 代价直接进入
        dist[start][0][0][0] = 0;
    }

    // 主循环
    while(1){
        State cur={-1,0,0,0,0};
        int best=INF;

        for(int i=0;i<TOTAL_NODES;i++)
            for(int j=0;j<=MAX_R1_LIMIT;j++)
                for(int k=0;k<=MAX_R2_REMOVE;k++)
                    for(int m=0;m<R2_MASK_STATES;m++)
                        if(!used[i][j][k][m] && dist[i][j][k][m]<best){
                            best=dist[i][j][k][m];
                            cur=(State){i,j,k,(uint8_t)m,dist[i][j][k][m]};
                        }

        if(cur.node==-1) break;
        used[cur.node][cur.r1_used][cur.r2_removed][cur.r2_mask]=true;

        if(cur.node==target && cur.r2_mask==R2_TARGET_MASK){
            *end=cur;
            return true;
        }

        // 进出约束
        int8_t neigh[5]; int n=0;
        if(cur.node==ENTRY_NODE){
            neigh[n++]=1;
        }else if (cur.node!=EXIT_NODE){
            int r=cur.node/COLS, c=cur.node%COLS;
            int dr[]={-1,1,0,0}, dc[]={0,0,-1,1};
            for(int i=0;i<4;i++){
                int nr=r+dr[i], nc=c+dc[i];
                if(nr>=0 && nr<ROWS && nc>=0 && nc<COLS)
                    neigh[n++]=nr*COLS+nc;
            }
            if(cur.node==9 || cur.node==11) neigh[n++]=EXIT_NODE;
        }

        // 遍历可达邻居
        for(int i=0;i<n;i++){
            int8_t nx=neigh[i];

            if(abs((int)HEIGHT_MAP[nx]-(int)HEIGHT_MAP[cur.node])!=200) continue;

            int nr1 = cur.r1_used;
            int nr2 = cur.r2_removed;
            uint8_t base_mask = cur.r2_mask;
            int base_cost = cur.cost + MOVE_COST;

            if(nx<GRID_NODES){
                if(map[nx]==KFS_FAKE) continue;

                if(map[nx]==KFS_R1){
                    if(++nr1>MAX_R1_LIMIT) continue;
                }

                if(map[nx]==KFS_R2){
                    uint8_t target_mask = target_mask_for_node(nx, targets);
                    if(target_mask != 0) base_mask |= target_mask;
                    else {
                        if(++nr2>MAX_R2_REMOVE) continue;
                        base_cost+=REMOVE_PENALTY;
                    }
                }

                if(base_cost < dist[nx][nr1][nr2][base_mask]){
                    dist[nx][nr1][nr2][base_mask] = base_cost;
                    parent[nx][nr1][nr2][base_mask] = cur;
                }

                uint8_t can_sg_mask = 0;
                for (int target_idx = 0; target_idx < R2_TAKEN_COUNT; target_idx++) {
                    uint8_t bit = (uint8_t)(1U << target_idx);
                    if ((base_mask & bit) == 0 && is_adjacent_grid(nx, targets[target_idx])) {
                        can_sg_mask |= bit;
                    }
                }

                for (uint8_t add_mask = can_sg_mask; add_mask != 0; add_mask = (uint8_t)((add_mask - 1U) & can_sg_mask)) {
                    int cost_sg = base_cost + SIDE_GRAB_PENALTY * popcount_u8(add_mask);
                    uint8_t mask_sg = base_mask | add_mask;
                    if(cost_sg < dist[nx][nr1][nr2][mask_sg]){
                        dist[nx][nr1][nr2][mask_sg] = cost_sg;
                        parent[nx][nr1][nr2][mask_sg] = cur;
                    }
                }
            }
            else {
                if(base_cost < dist[nx][nr1][nr2][base_mask]){
                    dist[nx][nr1][nr2][base_mask] = base_cost;
                    parent[nx][nr1][nr2][base_mask] = cur;
                }
            }
        }
    }
    return false;
}

// ================= 回溯路径 + KFS 记录 =================
// 将 Dijkstra 得到的终态沿 parent 表回溯，重建路径并记录在路上遇到的 KFS 变化
void reconstruct(
    State end,
    ParentTable parent,
    KFS_Type *map,
    const int8_t targets[R2_TAKEN_COUNT],
    PlanResult *res
){
    State cur=end;
    int len=0;

    // 初始化结果结构
    res->r1_cnt=0;
    res->r2r_cnt=0;
    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        res->r2_taken[i]=targets[i];
    }
    res->cost=end.cost;

    // 从终点回溯到入口，沿途记录被移除的 R1 / 非目标 R2
    while(cur.node!=ENTRY_NODE){
        // 记录路径（暂时为逆序）
        res->path[len++]=cur.node;

        // 取得父状态以继续回溯
        State prev = parent[cur.node][cur.r1_used][cur.r2_removed][cur.r2_mask];

        if(cur.node<GRID_NODES){
            // 如果当前格子原本是 R1，则记录为被移除的 R1
            if(map[cur.node]==KFS_R1){
                res->r1_removed[res->r1_cnt++]=cur.node;
            }

            // 如果遇到 R2 且不是目标集合中的节点，则记为被移除的 R2
            if(map[cur.node]==KFS_R2 && target_mask_for_node(cur.node, targets)==0){
                res->r2_removed[res->r2r_cnt++]=cur.node;
            }
        }

        // 继续回溯
        cur = prev;
    }

    // 把入口节点加入路径末尾（因为回溯时是逆序的）
    res->path[len++]=ENTRY_NODE;

    // 将路径反转为从入口到终点的顺序
    for(int i=0;i<len/2;i++){
        int8_t t=res->path[i];
        res->path[i]=res->path[len-1-i];
        res->path[len-1-i]=t;
    }
    res->path_len=len;
}

static void evaluate_target_combination(
    int depth,
    int start_idx,
    const int8_t r2s[],
    int cnt,
    int8_t targets[R2_TAKEN_COUNT],
    KFS_Type *map,
    ParentTable parent,
    int *best,
    PlanResult *best_res
){
    if (depth == R2_TAKEN_COUNT) {
        State end;
        if(run_dijkstra(ENTRY_NODE, EXIT_NODE, targets, map, &end, parent)){
            if(end.cost < *best){
                *best = end.cost;
                reconstruct(end, parent, map, targets, best_res);
            }
        }
        return;
    }

    int remaining = R2_TAKEN_COUNT - depth;
    for(int i = start_idx; i <= cnt - remaining; i++){
        targets[depth] = r2s[i];
        evaluate_target_combination(depth + 1, i + 1, r2s, cnt, targets, map, parent, best, best_res);
    }
}

static void order_r2_taken_for_pick_sequence(PlanResult *res) {
    int8_t ordered[R2_TAKEN_COUNT];
    int ordered_count = 0;

    for (int path_idx = 0; path_idx < res->path_len - 1; path_idx++) {
        int8_t current = res->path[path_idx];
        int8_t next = res->path[path_idx + 1];
        bool entry_direct_move = (current == ENTRY_NODE && next == 1);

        // 同一位置既能侧抓、又要抓路径下一格的 KFS 时，先安排侧抓。
        for (int target_idx = 0; target_idx < R2_TAKEN_COUNT; target_idx++) {
            int8_t target = res->r2_taken[target_idx];
            if (target != next &&
                is_adjacent(current, target) &&
                !entry_direct_move &&
                !node_in_list(target, ordered, ordered_count)) {
                ordered[ordered_count++] = target;
            }
        }

        for (int target_idx = 0; target_idx < R2_TAKEN_COUNT; target_idx++) {
            int8_t target = res->r2_taken[target_idx];
            if (target == next && !node_in_list(target, ordered, ordered_count)) {
                ordered[ordered_count++] = target;
            }
        }
    }

    for (int target_idx = 0; target_idx < R2_TAKEN_COUNT; target_idx++) {
        int8_t target = res->r2_taken[target_idx];
        if (!node_in_list(target, ordered, ordered_count)) {
            ordered[ordered_count++] = target;
        }
    }

    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        res->r2_taken[i] = ordered[i];
    }
}

// ================= 主函数 =================
// 构造一个示例地图，寻找最优的 R2_TAKEN_COUNT 个目标 R2 的取货顺序与路径
PlanResult plan_route(KFS_Type map[]){

    // 收集所有 R2 节点，用于枚举 R2_TAKEN_COUNT 个目标组合
    int8_t r2s[GRID_NODES],cnt=0;
    for(int i=0;i<GRID_NODES;i++) if(map[i]==KFS_R2) r2s[cnt++]=i;

    int best=INF;
    static ParentTable parent;
    PlanResult best_res;
    memset(&best_res, 0, sizeof(best_res));
    best_res.entry_grab = -1;
    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        best_res.r2_taken[i] = -1;
    }

    // 枚举所有不重复目标组合
    if (cnt >= R2_TAKEN_COUNT) {
        int8_t targets[R2_TAKEN_COUNT];
        evaluate_target_combination(0, 0, r2s, cnt, targets, map, parent, &best, &best_res);
    }
    if (best < INF) {
        order_r2_taken_for_pick_sequence(&best_res);
    }

    if (best < INF) {
        for (int i = 0; i < R2_TAKEN_COUNT; i++) {
            if (best_res.r2_taken[i]==0 || best_res.r2_taken[i]==2) {
                best_res.entry_grab = 1;
            }
        }
    }

    // 输出结果
    if(best<INF){
        printf("=== Plan Success ===\n");
        printf("Cost: %d\n",best_res.cost);

        printf("R2 Taken: ");
        for(int i=0;i<R2_TAKEN_COUNT;i++)
            printf("%d ",best_res.r2_taken[i]);
        printf("\n");

        printf("R2 Removed: ");
        for(int i=0;i<best_res.r2r_cnt;i++)
            printf("%d ",best_res.r2_removed[i]);

        printf("\nR1 Removed: ");
        for(int i=0;i<best_res.r1_cnt;i++)
            printf("%d ",best_res.r1_removed[i]);

        printf("\nPath: ");
        for(int i=0;i<best_res.path_len;i++){
            if(best_res.path[i]==ENTRY_NODE) printf("Entry ");
            else if(best_res.path[i]==EXIT_NODE) printf("Exit ");
            else printf("%d ",best_res.path[i]);
        }
        printf("\n");
    }else{
        printf("No valid path\n");
    }
    return best_res;
}
// ================= 【新增】局部修正逻辑函数 =================

/**
 * @brief 局部路径修正包装器
 * @param start 当前位置
 * @param target 最终目标
 * @param t1, t2 目标KFS
 * @param blocked_node 实时探测到的障碍物节点 (如R1所在位置)，若无则传-1
 * @param final_res 存储规划出的新路径
 */
bool Path_Replan_Local_Update(int8_t start, int8_t target, int8_t t1, int8_t t2, int8_t blocked_node, PlanResult *final_res) {
    // 1. 静态分配巨大的 ParentInfo 数组（避免栈溢出）
    static ParentTable temp_parent;

    // 2. 创建临时地图副本，用于局部修正
    KFS_Type temp_map[TOTAL_NODES];
    memcpy(temp_map, initial_map, sizeof(temp_map));

    // 3. 将 R1 当前占据的点临时设为不可通行 (FAKE)
    if (blocked_node >= 0 && blocked_node < TOTAL_NODES) {
        temp_map[blocked_node] = KFS_FAKE;
    }

    // 4. 调用原有的核心规划算法
    State end_state;
    if (final_res == NULL) {
        return false;
    }

    int8_t targets[R2_TAKEN_COUNT];
    for (int i = 0; i < R2_TAKEN_COUNT; i++) {
        targets[i] = (i == 0) ? t1 : t2;
        if (final_res != NULL && final_res->r2_taken[i] >= 0) {
            targets[i] = final_res->r2_taken[i];
        }
    }
    targets[0] = t1;
#if R2_TAKEN_COUNT > 1
    targets[1] = t2;
#endif

    if (run_dijkstra(start, target, targets, temp_map, &end_state, temp_parent)) {
        // 5. 如果规划成功，回溯路径
        reconstruct(end_state, temp_parent, temp_map, targets, final_res);
        return true;
    }

    return false; // 无法绕过障碍物找到路径
}

// PlanResult plan_route(int8_t start, int8_t target, KFS_Type map[]){
//     // 收集所有 R2 节点，用于枚举两两组合作为任务目标
//     int8_t r2s[4],cnt=0;
//     for(int i=0;i<GRID_NODES;i++) if(map[i]==KFS_R2) r2s[cnt++]=i;
//
//     int best=INF;
//     State best_end;
//     static State parent[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];
//     PlanResult best_res;
//
//     // 枚举所有两两组合（不重复的顺序对）作为 t1,t2
//     for(int i=0;i<cnt;i++)for(int j=i+1;j<cnt;j++){
//         State end;
//         // 运行 Dijkstra（在状态空间中搜索）
//         if(run_dijkstra(start, target,r2s[i],r2s[j],map,&end,parent)){
//             if(end.cost<best){
//                 best=end.cost;
//                 best_end=end;
//                 // 回溯得到具体路径与被移除的 KFS 列表
//                 reconstruct(end,parent,map,r2s[i],r2s[j],&best_res);
//             }
//         }
//     }
//
//     // 输出结果
//     if(best<INF){
//         printf("=== Plan Success ===\n");
//         printf("Cost: %d\n",best_res.cost);
//
//         printf("R2 Taken: %d %d\n",
//                best_res.r2_taken[0],
//                best_res.r2_taken[1]);
//
//         printf("R2 Removed: ");
//         for(int i=0;i<best_res.r2r_cnt;i++)
//             printf("%d ",best_res.r2_removed[i]);
//
//         printf("\nR1 Removed: ");
//         for(int i=0;i<best_res.r1_cnt;i++)
//             printf("%d ",best_res.r1_removed[i]);
//
//         printf("\nPath: ");
//         for(int i=0;i<best_res.path_len;i++){
//             if(best_res.path[i]==ENTRY_NODE) printf("Entry ");
//             else if(best_res.path[i]==EXIT_NODE) printf("Exit ");
//             else printf("%d ",best_res.path[i]);
//         }
//         printf("\n");
//     }else{
//         printf("No valid path\n");
//     }
//     return best_res;
// }
