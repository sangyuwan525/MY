//
// Created by 马皓然 on 2026/2/4.
//

#ifndef R1_CHASSIS_DEBUG_H
#define R1_CHASSIS_DEBUG_H

// --- 基础配置 ---
#define RTT_LOG_CHANNEL 0
// --- 类 printf 的万能 LOG 宏 ---
// 使用 ##__VA_ARGS__ 处理变长参数
// 注意：RTT 原生 printf 不支持 %f，打印浮点数请使用下方的 RTT_LOG_FLOAT
#define RTT_LOG(fmt, ...) \
SEGGER_RTT_printf(RTT_LOG_CHANNEL, "[%d][LOG]: " fmt "\r\n", HAL_GetTick(), ##__VA_ARGS__)

// --- 专门处理浮点数的打印宏 (弥补 RTT 无法使用 %f 的缺陷) ---
#define RTT_LOG_FLOAT(label, val) \
do { \
SEGGER_RTT_printf(RTT_LOG_CHANNEL, "[%d][FLOAT] %s: ", HAL_GetTick(), label); \
_RTT_Internal_PrintFloat(val); \
SEGGER_RTT_printf(RTT_LOG_CHANNEL, "\r\n"); \
} while(0)

void RTT_Poll_And_Process(void);
void Process_Command_Buffer(uint8_t *buffer, uint16_t size);
void RTT_Printf(const char *format, ...);
#endif //R1_CHASSIS_DEBUG_H