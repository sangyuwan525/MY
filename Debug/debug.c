//
// Created by 马皓然 on 2026/2/4.
//
#include <stdio.h>
#include <stdint.h>

#include "debug.h"
#include "SEGGER_RTT.h"
//#include "map_server.h"
#include "SEGGER_RTT_Conf.h"
#include "Task_DJI_Control.h"
#include "Task_chassis.h"

/**
 * @brief 内部函数：手动拆分浮点数并打印
 */
void _RTT_Internal_PrintFloat(float val) {
    float abs_val = (val < 0) ? -val : val;
    int int_part = (int)abs_val;
    int frac_part = (int)((abs_val - int_part) * 1000);
    SEGGER_RTT_printf(RTT_LOG_CHANNEL, "%s%d.%03d", (val < 0 ? "-" : ""), int_part, frac_part);
}

/**
 * @brief 万能 RTT 打印函数，用法完全等同于 printf
 * @note  如果无法打印浮点数，请看下方的“重要提示”
 */
void RTT_Printf(const char *format, ...)
{
    static char buffer[128]; // 中转缓冲区，如果打印的内容很长，可以调大
    va_list args;

    va_start(args, format);
    // 使用标准库的 vsnprintf 将格式化字符串转换到 buffer
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0)
    {
        // 加上时间戳（可选，方便调试）
        // SEGGER_RTT_printf(0, "[%d] ", HAL_GetTick());

        // 直接将格式化好的字符串通过 RTT 发出
        SEGGER_RTT_Write(0, buffer, len);
    }
}