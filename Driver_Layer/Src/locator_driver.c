//
// Created by 马皓然 on 2025/11/5.
//
#include "locator_driver.h"

#include <stdio.h>

Locator_Result_t lcResult={0};

// 解析定位器X/Y坐标（大端序，8字节数据拆分为2个4字节float）
void analysis_locator_X_Y(Locator_Result_t* lcResult, const Locator_Rx_Queue_t* rx_msg_tmp) {
    uint32_t database_x;
    uint32_t database_y;

    // 先校验指针非空，避免空指针崩溃
    if (lcResult == NULL || rx_msg_tmp == NULL) {
        return; // 指针无效直接返回，不处理
    }
    //printf("%x\n",rx_msg_tmp->msg_identifier);
    //  匹配目标报头，解析大端序float数据
    if (rx_msg_tmp->msg_identifier == 0x12) {
        //printf("ok222222\n");
        // 前4字节（rx_data[0-3]）拼接为X的32位整数（大端序）
        database_x = ((uint32_t)rx_msg_tmp->rx_data[3] << 24) |
                     ((uint32_t)rx_msg_tmp->rx_data[2] << 16) |
                     ((uint32_t)rx_msg_tmp->rx_data[1] << 8)  |
                     ((uint32_t)rx_msg_tmp->rx_data[0]);

        // 内存强转：将32位整数的二进制解析为float
        lcResult->x = *(float*)(&database_x);

        // 后4字节（rx_data[4-7]）拼接为Y的32位整数（大端序）
        database_y = ((uint32_t)rx_msg_tmp->rx_data[7] << 24) |
                     ((uint32_t)rx_msg_tmp->rx_data[6] << 16) |
                     ((uint32_t)rx_msg_tmp->rx_data[5] << 8)  |
                     ((uint32_t)rx_msg_tmp->rx_data[4]);

        // 内存强转：解析Y坐标
        lcResult->y = *(float*)(&database_y);
    }
}

void analysis_locator_Z_R(Locator_Result_t* lcResult, const Locator_Rx_Queue_t* rx_msg_tmp) {
    uint32_t database_r;

    if (lcResult == NULL || rx_msg_tmp == NULL) {
        return;
    }

    //  匹配目标报头，解析大端序float数据
    if (rx_msg_tmp->msg_identifier == 0x13) {

        // 后4字节（rx_data[4-7]）拼接为R的32位整数（大端序）
        database_r = ((uint32_t)rx_msg_tmp->rx_data[7] << 24) |
                     ((uint32_t)rx_msg_tmp->rx_data[6] << 16) |
                     ((uint32_t)rx_msg_tmp->rx_data[5] << 8)  |
                     ((uint32_t)rx_msg_tmp->rx_data[4]);

        // 内存强转：解析R
        lcResult->r = *(float*)(&database_r);
    }
}