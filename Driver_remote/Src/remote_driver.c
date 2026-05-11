//
// Created by 马皓然 on 2025/11/6.
//
#include "remote_driver.h"
#define RC_CHANNEL_MAX      660.0f
#define RC_CHANNEL_MID      0.0f
//目前底下这两个值还未经过标定，只是个模糊值
#define MAX_CHASSIS_SPEED   3000.0f // max chassis translation speed (mm/s)
#define MAX_CIRCLE_SPEED   5.0f // max chassis yaw speed (rpm)

rc_info_t rc;
remote_engineer_t remote_engineer;

//上次速度
int16_t vx_last=0;
int16_t vy_last=0;
int16_t vw_last=0;

int16_t vel_threshold =500;

/**
 * @brief  对遥控器数据进行解算并**保护性**更新全局结构体
 *
 * @param  code 包含遥控器原始数据的数据帧指针
 *
 * @return 无
 *
 * @note   该函数使用互斥量保护对全局变量 rc 的写入操作。
 */
void code_unzipread(uint8_t *code){
        rc.ch1 = (code[1] << 3) | (code[2] >> 5);
        rc.ch1 = 660 - rc.ch1; // 解压x_temp
        rc.ch2 = ((code[2] & 0x1F) << 6) | (code[3] >> 2);
        rc.ch2 -= 660; // 解压y_temp
        rc.cir = ((code[3] & 0x03) << 9) | (code[4] << 1) | (code[5] >> 7);
        rc.cir = 660 - rc.cir;
        // 解压c_temp
        // 开关解压（每个开关2位）
        rc.sw1 = (code[5] >> 5) & 0x03;
        rc.sw2 = (code[5] >> 3) & 0x03;
        rc.sw3 = (code[5] >> 1) & 0x03;
        rc.sw4 = ((code[5] & 0x01) << 1) | ((code[6] >> 7) & 0x01);
        rc.sw5 = (code[6] >> 5) & 0x03;

        // 按钮解压（每个按钮1位）
        rc.button1 = (code[6] >> 4) & 0x01;
        rc.button2 = (code[6] >> 3) & 0x01;
        rc.button3 = (code[6] >> 2) & 0x01;
        rc.button4 = (code[6] >> 1) & 0x01;
        rc.button5 = (code[6]) & 0x01;
        rc.button6 = (code[7] >> 7) & 0x01;
}
/**
 * @brief  将遥控器原始数据转换为底盘控制所需的工程量
 *
 * @param  rc_data 遥控器原始数据指针 (rc_info_t 类型)
 * @param  engineer_data 用于存放转换后工程量的结构体指针
 *
 * @return 无
 *
 * @note   该函数负责数据的归一化、速度标定和模式切换逻辑。
 */
void Remote_Data_Convert(const rc_info_t *rc_data, remote_engineer_t *engineer_data) {

    // 归一化：将通道值映射到 [-1.0, 1.0]
    float ch1_norm = (float)rc_data->ch1 / RC_CHANNEL_MAX; // ch1 范围: [-660, 660]
    float ch2_norm = (float)rc_data->ch2 / RC_CHANNEL_MAX; // ch2 范围: [-660, 660]
    float cir_norm = (float)rc_data->cir / RC_CHANNEL_MAX; // cir 范围: [-660, 660]

    int16_t vx_tmp=ch1_norm * MAX_CHASSIS_SPEED;
    int16_t vy_tmp=ch2_norm * MAX_CHASSIS_SPEED;
    int16_t vw_tmp=cir_norm * MAX_CIRCLE_SPEED;

    if(vx_tmp-vx_last>vel_threshold) vx_tmp=vx_last+vel_threshold;
    else if (vx_tmp-vx_last<-vel_threshold) vx_tmp=vx_last-vel_threshold;
    if(vy_tmp-vy_last>vel_threshold) vy_tmp=vy_last+vel_threshold;
    else if (vy_tmp-vy_last<-vel_threshold) vy_tmp=vy_last-vel_threshold;
    if(vw_tmp-vw_last>vel_threshold) vw_tmp=vw_last+vel_threshold;
    else if (vw_tmp-vw_last<-vel_threshold) vw_tmp=vw_last-vel_threshold;

    vx_last=vx_tmp;
    vy_last=vy_tmp;
    vw_last=vw_tmp;

    // 转换为实际工程量速度
    // engineer_data->vx = (int16_t)(ch1_norm * MAX_CHASSIS_SPEED);
    // engineer_data->vy = (int16_t)(ch2_norm * MAX_CHASSIS_SPEED);
    // engineer_data->vw = (int16_t)(cir_norm * MAX_CHASSIS_W_RAD);
    engineer_data->vx = vx_tmp;
    engineer_data->vy =vy_tmp;
    engineer_data->vw =vw_tmp;

    // 模式和比例因子判断（由开关控制）
    if (rc_data->sw1 == 1 && rc_data->sw2 == 1)  {
        engineer_data->mode = CHASSIS_MODE_MANUAL;// 手动模式
    } else if (rc_data->sw1 == 2 && rc_data->sw2 == 2) {
        engineer_data->mode = CHASSIS_MODE_AUTO; // 自动模式
    } else if (rc_data->sw1 == 1 && rc_data->sw2 == 2) {
        engineer_data->mode = CHASSIS_MODE_TEST; // 测试模式
    } else {
        engineer_data->mode = CHASSIS_MODE_STANDBY; // 待机模式
    }
    //testmode
    if (rc_data->sw4==1) {
        engineer_data->test_mode=CLIMB_MODE;
    } else if (rc_data->sw4==2) {
        engineer_data->test_mode=DOWN_MODE;
    }else if (rc_data->sw4==3)
    {
        engineer_data->test_mode=UP_MODE;
    }
    engineer_data->button1 = rc_data->button1;
    engineer_data->button2 = rc_data->button2;
    engineer_data->button3 = rc_data->button3;
    engineer_data->button4 = rc_data->button4;
    engineer_data->button5 = rc_data->button5;
    engineer_data->button6 = rc_data->button6;
}
/**
 * @brief  获取受保护的遥控器工程量数据
 *
 * @param  engineer_data 用于存储拷贝出的工程量数据的结构体指针
 *
 * @return pdPASS (1) - 数据获取成功
 * @return pdFAIL (0) - 互斥量获取失败
 *
 * @note   控制任务应调用此函数来安全地读取工程量数据。
 */
BaseType_t Remote_GetEngineerData(remote_engineer_t *engineer_data) {
    // 使用与 rc 结构体相同的互斥量保护 engineer 结构体
    if (osMutexAcquire(rc_mutexHandle, osWaitForever) == osOK) {
        *engineer_data = remote_engineer;
        osMutexRelease(rc_mutexHandle);
        return pdPASS;
    }
    return pdFAIL;
}
