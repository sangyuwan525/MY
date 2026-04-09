//
// Created by 马皓然 on 2025/11/27.
//
#include "remote_driver.h"
#include "chassis_driver.h"
#include "Task_chassis.h"

#include "chassis_path.h"
#include "dji_3508_2006_motor.h"
#include "ClimbStairs.h"
#include "SEGGER_RTT.h"
#include "stdio.h"
#include "stm32g4xx_hal.h"  // 根据你的MCU型号选择对应的头文件
#include "usart.h"
#include "path.h"
#include "Task_chassis.h"
#include "chassis_pid.h"
#include "locator_driver.h"
#include "Hfsm.h"

int turning_flag=1;//判断车子左右运动状态
int chassis_control_cnt;
int button3_flag=0;
int button4_flag=0;
bool valve_state=0;
int climb_test_cnt=0;



void StartTask_chassis(void *argument)
{
    /* USER CODE BEGIN StartTask_chassis */
    remote_engineer_t rc_engineer_data;
    chassis_control_cnt=0;

    //go_path_test
    PID_Init();
    path_init_test();

    /* Infinite loop */
    for(;;)
    {
        if (chassis_control_cnt>=20)//失联超过200ms,速度衰减
        {
            rc_engineer_data.vx*=0.95;
            rc_engineer_data.vy*=0.95;
            rc_engineer_data.vw*=0.95;
            cha_remote(rc_engineer_data.vx,
                       rc_engineer_data.vy,
                       rc_engineer_data.vw);
            //cha_remote(0,0,0);
        }else
        {
            chassis_control_cnt++;
            // SEGGER_RTT_SetTerminal(0);
            // 使用 Remote_GetEngineerData 确保在互斥量保护下安全读取
            if (Remote_GetEngineerData(&rc_engineer_data) == pdPASS)
            {
                // 模式 2 为手动模式
                if (rc_engineer_data.mode == CHASSIS_MODE_MANUAL)
                {
                    vec2 v_world,remote;
                    remote.x=rc_engineer_data.vx;
                    remote.y=rc_engineer_data.vy;
                    v_world= change_world_to_local(remote,lcResult.r);
                    // 将遥控器工程量速度 (vx, vy, vw) 传入底盘驱动
                    cha_remote(v_world.x,
                               v_world.y,
                               rc_engineer_data.vw);
                    if (rc_engineer_data.test_mode==CLIMB_MODE) {
                        if (climb_test_cnt==0)
                        {
                            Point_struct now_point = {lcResult.x,lcResult.y};
                            init_single_line_path(&path_test,now_point,entry_point[1],lcResult.r,0);
                            if (go_path_control(&path_test, spd_test) == 1){
                                climb_test_cnt++;
                            }
                        }
                        else if (climb_test_cnt==1)
                        {
                            if (ClimbStairs(12,1)) {
                                climb_test_cnt++;
                            };
                        }
                        else if (climb_test_cnt==2)
                        {
                            if (ClimbStairs(1,0)) {
                                climb_test_cnt++;
                            };
                        }
                        else if (climb_test_cnt==3)
                        {
                            if (DownStairs(0,3)) {
                                climb_test_cnt=-1;
                            };
                        }
                    }else if (rc_engineer_data.test_mode==DOWN_MODE) {
                        if (climb_test_cnt==0)
                        {
                            if (DownStairs(0,3)) {
                                climb_test_cnt=-1;
                            };
                        }
                        //DownStairs();
                    }else if (rc_engineer_data.test_mode==UP_MODE) {
                        Change_dji_loc(DJI_M_CLIMB_LF,-100000);
                        Change_dji_loc(DJI_M_CLIMB_RF,100000);
                        Change_dji_loc(DJI_M_CLIMB_RB,-100000);
                        Change_dji_loc(DJI_M_CLIMB_LB,100000);

                    }
                }
                else if (rc_engineer_data.mode == CHASSIS_MODE_AUTO)
                {
                    chassis_auto_control(&g_robot_ctx);
                }
                else if (rc_engineer_data.mode == CHASSIS_MODE_TEST)
                {

                }
                else // 其他模式 (待机/自动)，底盘速度清零
                {
                    // 停止底盘，发送 (0, 0, 0) 指令
                    cha_remote(0.0f, 0.0f, 0.0f);
                    Change_dji_speed(DJI_2006_L, 0);
                    Change_dji_speed(DJI_2006_R, 0);
                    if (climb_test_cnt==0){
                        Change_dji_loc(DJI_M_CLIMB_LF,0);
                        Change_dji_loc(DJI_M_CLIMB_RF,0);
                        Change_dji_loc(DJI_M_CLIMB_RB,0);
                        Change_dji_loc(DJI_M_CLIMB_LB,0);
                    }
                    climb_cnt=0;
                    down_cnt=0;
                }
                //前3508抬升
                // if (rc_engineer_data.button1 == 1)
                // {
                //     // 按钮1被按下，后轮2006往前走
                //
                //     //气缸测试 收
                //     if (rc_engineer_data.test_mode==UP_MODE)
                //     {
                //         Change_dji_speed(DJI_2006_L,-2500);
                //         Change_dji_speed(DJI_2006_R,2500);
                //     }
                //
                // }
                // else
                // {
                //     if (rc_engineer_data.test_mode==UP_MODE)
                //     {
                //         Change_dji_speed(DJI_2006_L,0);
                //         Change_dji_speed(DJI_2006_R,0);
                //     }
                //
                // }
                //

                //全自动上楼梯 按键1 用于让R2停止
                if (rc_engineer_data.button1 == 1)
                {
                    Change_dji_loc(DJI_M_CLIMB_LF,-back_up);
                    Change_dji_loc(DJI_M_CLIMB_RF,back_up);
                    Change_dji_loc(DJI_M_CLIMB_RB,-25000);
                    Change_dji_loc(DJI_M_CLIMB_LB,25000);
                }else {
                    //MF_flag = 0;
                }
                if (rc_engineer_data.button2 == 1)
                {
                    g_robot_ctx.current_top_state=1;
                    // g_robot_ctx.sub_state.mf=
                    // // 按钮2,前侧和后侧将机身顶起
                    // Change_dji_loc(DJI_M_CLIMB_LF,10000);
                    // Change_dji_loc(DJI_M_CLIMB_RF,-10000);
                    // Change_dji_loc(DJI_M_CLIMB_LB,590000);
                    // Change_dji_loc(DJI_M_CLIMB_RB,-590000);
                    // Change_dji_speed(DJI_2006_L, -2000);
                    // Change_dji_speed(DJI_2006_R, 2000);

                }else
                {
                    //Change_dji_speed(DJI_2006_L, 0);
                    //Change_dji_speed(DJI_2006_R, 0);
                }
                if (rc_engineer_data.button3 == 1)
                {
                    // 按钮3被按下，2006推动底盘向前运动
                    // Change_dji_speed(DJI_2006_L,2500);
                    // Change_dji_speed(DJI_2006_R,-2500);
                    //气缸测试 放
                    // HAL_GPIO_WritePin(valve_port,valve_pin_l,1);
                    // HAL_GPIO_WritePin(valve_port,valve_pin_r,1);
                    //按钮3，在不同模式下状态机状态变换
                    if (button3_flag==0)
                    {
                        if (rc_engineer_data.test_mode==CLIMB_MODE)
                        {
                            climb_cnt++;
                        }
                        else if (rc_engineer_data.test_mode==DOWN_MODE)
                        {
                            down_cnt++;
                        }
                        climb_test_cnt=1;
                        PID_Init();
                        MF_flag++;
                        MC_flag++;
                        CF_flag++;
                        button3_flag=1;
                    }
                }
                else
                {
                    button3_flag=0;
                }
                if (rc_engineer_data.button4 == 1)
                {
                    if (button4_flag==0)
                    {
                        //if (rc_engineer_data.)down_cnt++;
                        // valve_state=!valve_state;
                        // HAL_GPIO_WritePin(valve_port,valve_pin_l,valve_state);
                        // HAL_GPIO_WritePin(valve_port,valve_pin_r,valve_state);
                        PID_Init();
                        climb_cnt=0;
                        climb_test_cnt=0;//climb_test_cnt=1-climb_test_cnt;
                        current_climb_state=0;
                        current_down_state=0;
                        MF_flag--;
                        MC_flag--;
                        CF_flag--;
                        button4_flag=1;
                    }
                }else
                {
                    button4_flag=0;
                }
            //    else
            //     {
            //         Change_dji_speed(DJI_2006_L,0);
            //         Change_dji_speed(DJI_2006_R,0);
            //     }
                if (rc_engineer_data.button5 == 1)
                {
                    Change_dji_loc(DJI_M_CLIMB_LF, 0);
                    Change_dji_loc(DJI_M_CLIMB_RF, 0);
                    Change_dji_loc(DJI_M_CLIMB_LB, climb_front_up);
                    Change_dji_loc(DJI_M_CLIMB_RB, -climb_front_up);
                    // 按钮5被按下，四个3508一起抬升底盘
                    //Change_dji_loc(6,back_up);
                    //Change_dji_loc(4,-front_up2);
                    //Change_dji_loc(5,front_up2);
                    // Change_dji_loc(DJI_M_CLIMB_LF,-front_up);
                    // Change_dji_loc(DJI_M_CLIMB_RF,front_up);
                    //
                    // Change_dji_loc(DJI_M_CLIMB_LB,back_up);
                    // Change_dji_loc(DJI_M_CLIMB_RB,-back_up);
                }
                if (rc_engineer_data.button6 == 1)
                {
                    // 按钮3被按下，一起抬升
                    //Change_dji_loc(6,back_up);
                    //Change_dji_loc(4,-front_up2);
                    //Change_dji_loc(5,front_up2);
                    // Change_dji_loc(DJI_M_CLIMB_LF,-front_up);
                    // Change_dji_loc(DJI_M_CLIMB_RF,front_up);
                    // Change_dji_loc(DJI_M_CLIMB_LB,0);
                    // Change_dji_loc(DJI_M_CLIMB_RB,0);
                }
            }
            else
            {
                // 如果获取数据失败（互斥量争夺失败），可以考虑错误处理或跳过本次循环
                // rc_engineer_data.vx*=0.9;
                // rc_engineer_data.vy*=0.9;
                // rc_engineer_data.vw*=0.9;
                // cha_remote(rc_engineer_data.vx,
                //            rc_engineer_data.vy,
                //            rc_engineer_data.vw);
                cha_remote(0,0,0);
            }
            //Change_dji_speed(0,1000);
            // 任务延时，保证任务周期性运行，例如每 10ms 运行一次
        }
        osDelay(CHASSIS_TASK_PERIOD);
    }
    /* USER CODE END StartTask_chassis */
}