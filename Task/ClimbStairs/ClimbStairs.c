//
// Created by lcf on 2025/12/1.
//

#include "ClimbStairs.h"

//距离转换为编码数
int DisToEncoder(float dis,int motor_id)
{
    float reduction_ratio[5]={19};//电机减速比
    float gear_ratio[5]={1};//电机传动比
    float fix_k[5]={1};//修正系数

    return (int)(dis*reduction_ratio[motor_id]*gear_ratio[motor_id]*fix_k[motor_id]);
}

void motor_move(int encoder_counts,int motor_id)
{
    int   lower_lim[5]={};
    int upper_lim[5]={};
    //int now_loc=Get_dji_information(motor_id).angle;
    if (encoder_counts<lower_lim[motor_id]) encoder_counts=lower_lim[motor_id];
    if (encoder_counts>upper_lim[motor_id]) encoder_counts=upper_lim[motor_id];
    int send_loc=encoder_counts;
    Change_dji_loc(motor_id,send_loc);
}

//电机零点在哪，往哪为上?
void ClimbStairs(void)
{
    upstairs_flag=1;

    if (upstairs_flag)//可换成信号量
    {
        //step1 前两个3508 抬升，后3508回到地面，气缸伸长

        //step2 底盘前移，前3508下降（？）
        //step3 气缸收回
        //step4 （3电机同步）后3508为负，前3508也跟着往上
        //step5  2006带动车身往前走
        //step6 后3508收回，完成上台阶
    }
}

