// #include "Hfsm.h"
//
//
// //  一区逻辑
// void Handle_MC_Logic(R2_Context_t *r2) {
//     switch (r2->sub_state.mc) {
//         case MC_INIT:
//             // 初始化传感器，定位
//             r2->sub_state.mc = MC_PICK_HEAD;
//             break;
//
//         case MC_PICK_HEAD:
//             // 规则4.3.3: R2从端头架取下一个端头 [cite: 98]
//             if (Hardware_PickHeadAction()) {
//                 r2->sub_state.mc = MC_ASSEMBLE_WAIT;
//             }
//             break;
//
//         case MC_ASSEMBLE_WAIT:
//             // 移动到预定组装位置，视觉对准长杆
//             if (Hardware_MoveToAssemblePoint()) {
//                 r2->sub_state.mc = MC_ASSEMBLE_ACT;
//             }
//             break;
//
//         case MC_ASSEMBLE_ACT:
//             // 规则4.3.6: 组装过程中R1与R2不得直接肢体接触
//             // R2保持端头稳定，等待R1插入
//             if (Sensors_DetectAssemblyComplete()) {
//                 r2->weapon_ready = true;
//                 r2->sub_state.mc = MC_WAIT_R1_EXIT;
//             }
//             break;
//
//         case MC_WAIT_R1_EXIT:
//             // 规则4.3.10: 只有在R1完全离开武馆后，R2才能离开
//             if (Sensors_IsR1LeftMC() || r2->r1_left_mc) {
//                 // 切换到顶层状态：进入梅林
//                 r2->current_top_state = STATE_MF_AREA;
//                 r2->sub_state.mf = MF_ENTRY;
//             }
//             break;
//     }
// }
//
// //  二区逻辑
// void Handle_MF_Logic(R2_Context_t *r2) {
//     static int target_block_id = 0;
//
//     switch (r2->sub_state.mf) {
//         case MF_ENTRY:
//             // 规则4.4.13: 必须通过R2入口进入
//             if (Hardware_EnterForest()) {
//                 r2->sub_state.mf = MF_SCAN_PATH;
//             }
//             break;
//
//         case MF_SCAN_PATH:
//             // 视觉扫描KFS，排除假KFS (规则3.6.3)
//             // 规划路径：必须基于“相邻”关系移动 [cite: 85]
//             target_block_id = Algorithm_FindNearestKFS();
//             if (target_block_id != 0) {
//                 r2->sub_state.mf = MF_MOVE_TO_BLOCK;
//             } else {
//                 // 如果没有KFS可捡或者已满，准备离开
//                 r2->sub_state.mf = MF_EXIT_NAV;
//             }
//             break;
//
//         case MF_MOVE_TO_BLOCK:
//             // 移动到底盘控制算法计算出的相邻方块
//             if (Hardware_MoveToBlock(target_block_id)) {
//                 r2->sub_state.mf = MF_PICK_ADJACENT;
//             }
//             break;
//
//         case MF_PICK_ADJACENT:
//             // 规则4.4.14: R2只能拿取相邻方块上的R2 KFS
//             if (Hardware_PickKFS()) {
//                 r2->kfs_count++;
//                 r2->sub_state.mf = MF_SCAN_PATH; // 继续寻找下一个
//             }
//             break;
//
//         case MF_EXIT_NAV:
//             // 规则4.4.19: 必须经由10, 11或12号方块之一离开 [cite: 133]
//             // 规则4.4.16: 离开前必须携带至少一个R2 KFS [cite: 133]
//             if (r2->kfs_count > 0 && Hardware_MoveToExitBlocks()) {
//                 r2->current_top_state = STATE_CF_AREA;
//                 r2->sub_state.cf = CF_CLIMB_RAMP;
//             }
//             break;
//     }
// }
//
// //  三区逻辑
// void Handle_CF_Logic(R2_Context_t *r2) {
//     switch (r2->sub_state.cf) {
//         case CF_CLIMB_RAMP:
//             // 爬坡进入对抗区
//             if (Hardware_ClimbRamp()) {
//                 r2->sub_state.cf = CF_DECISION;
//             }
//             break;
//
//         case CF_DECISION:
//             // 根据场上局势决定策略
//             if (Algorithm_NeedTopLayer()) {
//                 r2->sub_state.cf = CF_WAIT_LIFT;
//             } else {
//                 r2->sub_state.cf = CF_PLACE_MID;
//             }
//             break;
//
//         case CF_PLACE_MID:
//             // 规则4.5.13: R2把KFS放到九宫格中层
//             if (Hardware_PlaceKFS_Middle()) {
//                 r2->kfs_count--;
//                 r2->sub_state.cf = CF_DECISION; // 循环决策，直到放完
//             }
//             break;
//
//         case CF_WAIT_LIFT:
//             // 规则3.8(3): R1举起R2 [cite: 121]
//             // R2检测自身IMU或高度传感器确认被举起
//             if (Sensors_IsLifted()) {
//                 r2->sub_state.cf = CF_PLACE_TOP;
//             }
//             break;
//
//         case CF_PLACE_TOP:
//             // 规则4.5.16: 被R1举起后放置顶层
//             if (Hardware_PlaceKFS_Top()) {
//                 r2->kfs_count--;
//                 // 放置完成后等待R1放下
//                 if (Sensors_IsOnGround()) {
//                     r2->sub_state.cf = CF_DECISION;
//                 }
//             }
//             break;
//     }
//
//     // 规则3.9: 如果获得“武术大师”，立即获胜 [cite: 121]
//     if (Referee_IsMartialArtsMaster()) {
//         r2->current_top_state = STATE_FINISHED;
//     }
// }
//
// int main_loop() {
//     R2_Context_t robot_ctx = {0};
//     robot_ctx.current_top_state = STATE_MC_AREA;
//     robot_ctx.sub_state.mc = MC_INIT;
//
//     while (1) {
//         // 全局安全检测
//         if (Sensors_EmergencyStopPressed()) {
//             robot_ctx.current_top_state = STATE_EMERGENCY;
//         }
//
//         // 分层状态机调度
//         switch (robot_ctx.current_top_state) {
//             case STATE_MC_AREA:
//                 Handle_MC_Logic(&robot_ctx);
//                 break;
//
//             case STATE_MF_AREA:
//                 Handle_MF_Logic(&robot_ctx);
//                 break;
//
//             case STATE_CF_AREA:
//                 Handle_CF_Logic(&robot_ctx);
//                 break;
//
//             case STATE_FINISHED:
//                 Hardware_StopAllMotors();
//                 // 庆祝动作
//                 break;
//
//             case STATE_EMERGENCY:
//                 Hardware_StopAllMotors();
//                 // 等待复位
//                 break;
//         }
//
//         // 维持控制频率 (e.g., 100Hz)
//         Delay_ms(10);
//     }
//     return 0;
// }