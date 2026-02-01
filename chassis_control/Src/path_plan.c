
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
    KFS_R1, KFS_NONE, KFS_R2,
    KFS_R1, KFS_NONE, KFS_FAKE,
    KFS_R1, KFS_R2, KFS_R2,
    KFS_NONE, KFS_NONE, KFS_R2,
    KFS_NONE, KFS_NONE
};

// ================= 邻接判断辅助函数 =================
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
    // 四维数组 dist/used：索引维度分别为 node、r1_used、r2_removed、r2_mask
    static int16_t dist[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];
    static bool used[TOTAL_NODES][MAX_R1_LIMIT+1][MAX_R2_REMOVE+1][4];

    // 初始化距离为 INF，未被访问
    for(int i=0;i<TOTAL_NODES;i++)
        for(int j=0;j<=MAX_R1_LIMIT;j++)
            for(int k=0;k<=MAX_R2_REMOVE;k++)
                for(int m=0;m<4;m++){
                    dist[i][j][k][m]=INF;
                    used[i][j][k][m]=false;
                }

    // 起始状态：在 start，r1_used=0, r2_removed=0, mask=0
    dist[start][0][0][0]=0;

    // 主循环：每次挑选未访问的最小 dist 状态
    while(1){
        State cur={-1,0,0,0,0};
        int best=INF;

        // O(状态数) 扫描选择最小距离的未访问状态（可用优先队列优化）
        for(int i=0;i<TOTAL_NODES;i++)
            for(int j=0;j<=MAX_R1_LIMIT;j++)
                for(int k=0;k<=MAX_R2_REMOVE;k++)
                    for(int m=0;m<4;m++)
                        if(!used[i][j][k][m] && dist[i][j][k][m]<best){
                            best=dist[i][j][k][m];
                            cur=(State){i,j,k,(uint8_t)m,dist[i][j][k][m]};
                        }

        // 没有可选状态，搜索失败
        if(cur.node==-1) break;
        used[cur.node][cur.r1_used][cur.r2_removed][cur.r2_mask]=true;

        // 如果到达目标节点且两个目标 R2 都已采集（mask==3），则成功
        if(cur.node==target && cur.r2_mask==3){
            *end=cur;
            return true;
        }

        // 构建邻居列表（考虑入口/出口特殊连接）
        int8_t neigh[5]; int n=0;
        if(cur.node==ENTRY_NODE){
            neigh[n++]=0; neigh[n++]=1; neigh[n++]=2;
        }else{
            int r=cur.node/COLS,c=cur.node%COLS;
            int dr[]={-1,1,0,0},dc[]={0,0,-1,1};
            for(int i=0;i<4;i++){
                int nr=r+dr[i],nc=c+dc[i];
                if(nr>=0&&nr<ROWS&&nc>=0&&nc<COLS)
                    neigh[n++]=nr*COLS+nc;
            }
            if(cur.node>=9&&cur.node<=11) neigh[n++]=EXIT_NODE;
        }

        // 遍历每个可达邻居，计算状态转移和代价
        for(int i=0;i<n;i++){
            int8_t nx=neigh[i];
            // 高度差必须等于 200 才允许移动（坡度限制）
            if(abs((int)HEIGHT_MAP[nx]-(int)HEIGHT_MAP[cur.node])!=200) continue;

            // 继承当前状态的计数和掩码
            int nr1=cur.r1_used;
            int nr2=cur.r2_removed;
            uint8_t mask=cur.r2_mask;
            int cost=cur.cost+10; // 默认移动代价为 10

            if(nx<GRID_NODES){
                // 不可通行的假节点直接跳过
                if(map[nx]==KFS_FAKE) continue;

                // 碰到 R1，则增加 r1 计数；超过上限则无法通行
                if(map[nx]==KFS_R1){
                    if(++nr1>MAX_R1_LIMIT) continue;
                }

                // 碰到 R2：如果是 t1 或 t2 则标记已采集；否则视为被移除的 R2（增加成本/计数）
                if(map[nx]==KFS_R2){
                    if(nx==t1) mask|=1;             // t1 被取走
                    else if(nx==t2) mask|=2;        // t2 被取走
                    else{
                        // 非目标的 R2 被“移除/绕开”，有更高代价和数量限制
                        if(++nr2>MAX_R2_REMOVE) continue;
                        cost+=5; // 额外代价（示意性增加）
                    }
                }
            }

            // 松弛操作：若新的代价更小则更新 dist 与 parent
            if(cost<dist[nx][nr1][nr2][mask]){
                dist[nx][nr1][nr2][mask]=cost;
                parent[nx][nr1][nr2][mask]=cur;
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
