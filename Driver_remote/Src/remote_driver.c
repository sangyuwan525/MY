//
// Created by 马皓然 on 2025/11/6.
//
#include "remote_driver.h"
rc_info_t rc;
void code_unzipread(uint8_t *code){

    rc.ch1 = (code[0] << 3) | (code[1] >> 5);
    rc.ch1 = 660 -rc.ch1;	// 解压x_temp
    rc.ch2 = ((code[1] & 0x1F) << 6) | (code[2] >> 2);
    rc.ch2-=660;	// 解压y_temp
    rc.cir = ((code[2] & 0x03) << 9) | (code[3] << 1) | (code[4] >> 7);
    rc.cir =  660 - rc.cir;
    // 解压c_temp
    // 开关解压（每个开关2位）
    rc.sw1 = (code[4] >> 5) & 0x03;
    rc.sw2 = (code[4] >> 3) & 0x03;
    rc.sw3 = (code[4] >> 1) & 0x03;
    rc.sw4 = ((code[4] & 0x01) << 1) | ((code[5] >> 7) & 0x01);
    rc.sw5 = (code[5] >> 5) & 0x03;

    // 按钮解压（每个按钮1位）
    rc.button1 = (code[5] >> 4) & 0x01;
    rc.button2 = (code[5] >> 3) & 0x01;
    rc.button3 = (code[5] >> 2) & 0x01;
    rc.button4 = (code[5] >> 1) & 0x01;
    rc.button5 = (code[5] ) & 0x01;
    rc.button6 = (code[6] >> 7) & 0x01;

}