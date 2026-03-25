
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
    if(a >= GRID_NODES || b >= GRID_NODES) return false;
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

// ================= Dijkstra (带状态扩展) =================
// 使用扩展的 Dijkstra 搜索，在状态空间中同时跟踪：
// - 位置 node
// - 已使用的 R1 数量 r1_used（受限）
// - 被移除的非目标 R2 数量 r2_removed（受限）
// - 已采集目标 R2 的掩码 r2_mask（两个 bit 表示 t1 和 t2 是否已采集）
// state 的代价保存在 State.cost 中，搜索目标是到达 target 且 r2_mask==3（即两个目标均已采集）

bool run_dijkstra(
    int8_t start,
    int8_t target,
    int8_t t1,
    int8_t t2,
    KFS_Type *map,
    State *end,
    State parent[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4]
){
    static int16_t dist[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];
    static bool used[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];

    // 初始化阵列
    for(int i=0;i<TOTAL_NODES;i++)
        for(int j=0;j<=MAX_R1_LIMIT;j++)
            for(int k=0;k<=MAX_R2_REMOVE;k++)
                for(int m=0;m<4;m++){
                    dist[i][j][k][m]=INF;
                    used[i][j][k][m]=false;
                }

    bool entry_has_r2 = (map[0] == KFS_R2 || map[1] == KFS_R2 || map[2] == KFS_R2);
    if (entry_has_r2 && (t1!=0 && t1!=1 && t1!=2) && (t2!=0 && t2!=1 && t2!=2)) {
        return false;
    }

    // =========================================================
    // 修复：只要门口有 R2，就不允许 0 代价直接走进去！
    // =========================================================
    if (map[1] == KFS_R2) {
        // 1 号有，强制优先拿 1 号
        if (t1 == 1) dist[start][0][0][1] = SIDE_GRAB_PENALTY;
        else if (t2 == 1) dist[start][0][0][2] = SIDE_GRAB_PENALTY;
        else return false;
    } else if (entry_has_r2) {
        // 1 号没有，但 0 或 2 有 R2。此时【必须】交出探身代价去拿目标
        if (t1 == 0 || t1 == 2) dist[start][0][0][1] = SIDE_GRAB_PENALTY;
        if (t2 == 0 || t2 == 2) dist[start][0][0][2] = SIDE_GRAB_PENALTY;
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
                    for(int m=0;m<4;m++)
                        if(!used[i][j][k][m] && dist[i][j][k][m]<best){
                            best=dist[i][j][k][m];
                            cur=(State){i,j,k,(uint8_t)m,dist[i][j][k][m]};
                        }

        if(cur.node==-1) break;
        used[cur.node][cur.r1_used][cur.r2_removed][cur.r2_mask]=true;

        if(cur.node==target && cur.r2_mask==3){
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
                    if(nx==t1) base_mask|=1;
                    else if(nx==t2) base_mask|=2;
                    else {
                        if(++nr2>MAX_R2_REMOVE) continue;
                        base_cost+=REMOVE_PENALTY;
                    }
                }

                bool can_sg_t1 = ((base_mask & 1) == 0) && is_adjacent_grid(nx, t1);
                bool can_sg_t2 = ((base_mask & 2) == 0) && is_adjacent_grid(nx, t2);

                if(base_cost < dist[nx][nr1][nr2][base_mask]){
                    dist[nx][nr1][nr2][base_mask] = base_cost;
                    parent[nx][nr1][nr2][base_mask] = cur;
                }

                if(can_sg_t1){
                    int cost_sg = base_cost + SIDE_GRAB_PENALTY;
                    uint8_t mask_sg = base_mask | 1;
                    if(cost_sg < dist[nx][nr1][nr2][mask_sg]){
                        dist[nx][nr1][nr2][mask_sg] = cost_sg;
                        parent[nx][nr1][nr2][mask_sg] = cur;
                    }
                }

                if(can_sg_t2){
                    int cost_sg = base_cost + SIDE_GRAB_PENALTY;
                    uint8_t mask_sg = base_mask | 2;
                    if(cost_sg < dist[nx][nr1][nr2][mask_sg]){
                        dist[nx][nr1][nr2][mask_sg] = cost_sg;
                        parent[nx][nr1][nr2][mask_sg] = cur;
                    }
                }

                if(can_sg_t1 && can_sg_t2){
                    int cost_sg = base_cost + (SIDE_GRAB_PENALTY * 2);
                    uint8_t mask_sg = base_mask | 3;
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
    State parent[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4],
    KFS_Type *map,
    int8_t t1,
    int8_t t2,
    PlanResult *res
){
    State cur=end;
    int len=0;

    // 初始化结果结构
    res->r1_cnt=0;
    res->r2r_cnt=0;
    res->r2_taken[0]=t1;
    res->r2_taken[1]=t2;
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

            // 如果遇到 R2 且不是两个目标（t1/t2），则记为被移除的 R2
            if(map[cur.node]==KFS_R2 && cur.node!=t1 && cur.node!=t2){
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

// ================= 主函数 =================
// 构造一个示例地图，寻找最优的两个目标 R2 的取货顺序与路径
PlanResult plan_route(KFS_Type map[]){

    // 收集所有 R2 节点，用于枚举两两组合作为任务目标
    int8_t r2s[4],cnt=0;
    for(int i=0;i<GRID_NODES;i++) if(map[i]==KFS_R2) r2s[cnt++]=i;

    int best=INF;
    State best_end;
    static State parent[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];
    PlanResult best_res;

    // 枚举所有两两组合（不重复的顺序对）作为 t1,t2
    for(int i=0;i<cnt;i++)for(int j=i+1;j<cnt;j++){
        State end;
        // 运行 Dijkstra（在状态空间中搜索）
        if(run_dijkstra(ENTRY_NODE,EXIT_NODE,r2s[i],r2s[j],map,&end,parent)){
            if(end.cost<best){
                best=end.cost;
                best_end=end;
                // 回溯得到具体路径与被移除的 KFS 列表
                reconstruct(end,parent,map,r2s[i],r2s[j],&best_res);
            }
        }
    }
    for (int i=0;i<best_res.path_len;i++)
    {
        if (best_res.r2_taken[0]==best_res.path[i]&&best_res.r2_taken[1]!=best_res.path[i])
        {
            int n = best_res.r2_taken[0];
            best_res.r2_taken[0]=best_res.r2_taken[1];
            best_res.r2_taken[1]=n;
        }
    }

    // 输出结果
    if(best<INF){
        printf("=== Plan Success ===\n");
        printf("Cost: %d\n",best_res.cost);

        printf("R2 Taken: %d %d\n",
               best_res.r2_taken[0],
               best_res.r2_taken[1]);

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
    static State temp_parent[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];

    // 2. 创建临时地图副本，用于局部修正
    KFS_Type temp_map[TOTAL_NODES];
    memcpy(temp_map, initial_map, sizeof(temp_map));

    // 3. 将 R1 当前占据的点临时设为不可通行 (FAKE)
    if (blocked_node >= 0 && blocked_node < TOTAL_NODES) {
        temp_map[blocked_node] = KFS_FAKE;
    }

    // 4. 调用原有的核心规划算法
    State end_state;
    if (run_dijkstra(start, target, t1, t2, temp_map, &end_state, temp_parent)) {
        // 5. 如果规划成功，回溯路径
        reconstruct(end_state, temp_parent, temp_map, t1, t2, final_res);
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
