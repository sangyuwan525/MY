//
// Created by 马皓然 on 2025/11/6.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os2.h"
#include "Task_Printf.h"
#include "chassis_path.h"
#include "ClimbStairs.h"
#include "debug.h"
#include "cmsis_gcc.h"
#include "locator_driver.h"
#include "Hfsm.h"
#include "Task_chassis.h"
#include "remote_driver.h"
#include "SEGGER_RTT.h"
#include "global_motor_conf.h"
#include "motor_registry.h"
#include "stm32g4xx_hal.h"  // 根据你的MCU型号选择对应的头文件

#define RTT_CMD_BUFFER_SIZE 64U

static const char *RTT_CmdSkipSpace(const char *s)
{
    while (*s == ' ' || *s == '\t') {
        ++s;
    }
    return s;
}

static const char *RTT_CmdReadToken(const char *s, char *token, uint32_t token_size)
{
    uint32_t i = 0;

    s = RTT_CmdSkipSpace(s);
    while (*s != '\0' && *s != ' ' && *s != '\t') {
        if (i + 1U < token_size) {
            token[i++] = *s;
        }
        ++s;
    }
    token[i] = '\0';
    return s;
}

static int RTT_CmdParseMotor(const char *token)
{
    if (token == NULL || token[0] == '\0') {
        return -1;
    }

    if (strcmp(token, "xiaomi1") == 0 || strcmp(token, "xm1") == 0) {
        return XIAOMI_MOTOR1_G;
    }
    if (strcmp(token, "xiaomi2") == 0 || strcmp(token, "xm2") == 0) {
        return XIAOMI_MOTOR2_G;
    }
    if (strcmp(token, "dm") == 0 || strcmp(token, "dm1") == 0) {
        return DM_FRONT_LEFT_G;
    }
    if (strcmp(token, "dm2") == 0) {
        return DM_FRONT_RIGHT_G;
    }
    if (strcmp(token, "unitree") == 0 || strcmp(token, "unitree1") == 0 || strcmp(token, "u1") == 0) {
        return UNITREE_GO_M8010_6_MOTOR1_G;
    }
    if (strcmp(token, "unitree2") == 0 || strcmp(token, "u2") == 0) {
        return UNITREE_GO_M8010_6_MOTOR2_G;
    }
    if (strcmp(token, "blazer") == 0 || strcmp(token, "blazer1") == 0 || strcmp(token, "bfoc1") == 0) {
        return BLAZER_FOC_MOTOR1_G;
    }
    if (strcmp(token, "blazer2") == 0 || strcmp(token, "bfoc2") == 0) {
        return BLAZER_FOC_MOTOR2_G;
    }
    if (strcmp(token, "blazer3") == 0 || strcmp(token, "bfoc3") == 0) {
        return BLAZER_FOC_MOTOR3_G;
    }
    if (strcmp(token, "blazer4") == 0 || strcmp(token, "bfoc4") == 0) {
        return BLAZER_FOC_MOTOR4_G;
    }

    if (token[0] >= '0' && token[0] <= '9') {
        int index = atoi(token);
        if (index >= 0 && index < MOTOR_TOTAL_NUM) {
            return index;
        }
    }

    return -1;
}

static void RTT_CmdPrintHelp(void)
{
    RTT_Printf("cmd: stop <motor> [clear]\r\n");
    RTT_Printf("cmd: zero <motor>\r\n");
    RTT_Printf("cmd: enable <motor>\r\n");
    RTT_Printf("motor: xiaomi1/xiaomi2/dm1/dm2/unitree1/unitree2/blazer1..blazer4 or global index\r\n");
}

static void RTT_CmdExecute(char *line)
{
    char cmd[16];
    char motor_name[20];
    char arg[16];
    const char *p;
    int motor_index;

    p = RTT_CmdReadToken(line, cmd, sizeof(cmd));
    p = RTT_CmdReadToken(p, motor_name, sizeof(motor_name));
    (void)RTT_CmdReadToken(p, arg, sizeof(arg));

    if (cmd[0] == '\0') {
        return;
    }

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        RTT_CmdPrintHelp();
        return;
    }

    motor_index = RTT_CmdParseMotor(motor_name);
    if (motor_index < 0) {
        RTT_Printf("bad motor: %s\r\n", motor_name);
        RTT_CmdPrintHelp();
        return;
    }

    if (strcmp(cmd, "stop") == 0) {
        uint8_t clear_error = (strcmp(arg, "clear") == 0 || atoi(arg) != 0) ? 1U : 0U;
        Motor_Stop(motor_index, clear_error);
        RTT_Printf("stop motor %d clear=%d\r\n", motor_index, clear_error);
    } else if (strcmp(cmd, "zero") == 0 || strcmp(cmd, "setzero") == 0) {
        Motor_SetZero(motor_index);
        RTT_Printf("zero motor %d\r\n", motor_index);
    } else if (strcmp(cmd, "enable") == 0) {
        Motor_Enable(motor_index);
        RTT_Printf("enable motor %d\r\n", motor_index);
    } else {
        RTT_Printf("bad cmd: %s\r\n", cmd);
        RTT_CmdPrintHelp();
    }
}

static void RTT_CmdPoll(void)
{
    static char line[RTT_CMD_BUFFER_SIZE];
    static uint32_t line_len = 0;
    char rx[16];
    unsigned rx_len;

    rx_len = SEGGER_RTT_Read(0, rx, sizeof(rx));
    for (unsigned i = 0; i < rx_len; ++i) {
        char ch = rx[i];

        if (ch == '\r' || ch == '\n') {
            if (line_len > 0U) {
                line[line_len] = '\0';
                RTT_CmdExecute(line);
                line_len = 0U;
            }
        } else if (ch == '\b' || ch == 0x7F) {
            if (line_len > 0U) {
                --line_len;
            }
        } else if (line_len + 1U < RTT_CMD_BUFFER_SIZE) {
            line[line_len++] = ch;
        } else {
            line_len = 0U;
            RTT_Printf("cmd too long\r\n");
        }
    }
}

void StartTask_Printf(void *argument)
{
    /* USER CODE BEGIN StartTask_Printf */
    remote_engineer_t chassis_cmd; // 用于接收工程量数据的局部变量
    /* Infinite loop */
    for(;;)
    {
        RTT_CmdPoll();

        if (Remote_GetEngineerData(&chassis_cmd) == pdPASS) {
            // sprintf(message,"desired_vx: %.2f, desired_vy: %.2f, desired_vw: %.2f\r\n",desired_vx, desired_vy, desired_vw);
            // printf("%s",message);
            // printf("desired_vx = %f\n", 1000*desired_vx);
            (void)chassis_cmd;
        }
        //printf("x66 y55 z66\n");
        // printf("x=%.1f,",lcResult.x);
        // printf("y=%.1f,",lcResult.y);
        // printf("r=%.3f\n",lcResult.r);
        // RTT_Printf("vx=%f  vy=%f  vr=%f\n",remote_engineer.vx,remote_engineer.vy,remote_engineer.vw);
        // RTT_Printf("dis=%f\n",lcResult.laser_current);
        //RTT_Printf("vx=%f  vy=%f  vr=%f\n",remote_engineer.mode,remote_engineer.vy,remote_engineer.vw);
        //RTT_Printf("test_cnt%d,state%d,cnt%d\n",climb_test_cnt,current_climb_state,climb_cnt);
        //printf("开关:%d\n",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11));
        //printf("top:%d, mc:%d, mf:%d, cf:%d\n",g_robot_ctx.current_top_state,g_robot_ctx.sub_state.mc,g_robot_ctx.sub_state.mf,g_robot_ctx.sub_state.cf);
        //printf("MC_flag:%d,MF_flag:%d,CF_flag:%d\n",MC_flag,MF_flag,CF_flag);
        //printf("cid=%d,tid=%d,r2_taken=%d,c_step=%d,path_len=%d\n",g_robot_ctx.current_stair_id,g_robot_ctx.target_stair_id,g_robot_ctx.already_taken,g_robot_ctx.current_step,g_robot_ctx.plan.path_len);
        // RTT_Printf("pb10 %d,pb11 %d\n",HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10),HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11));
        // RTT_Printf("pc0 %d,pc1 %d\n",HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0),HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1));
        //RTT_Printf("face %d\n",get_face(1,0));
        // RTT_Printf("xiaomi1_online=%d  xiaomi1_angle=%f\n",
        //            g_xiaomi_motor_registry[XIAOMI_Motor1].feedback.online,
        //            g_xiaomi_motor_registry[XIAOMI_Motor1].feedback.angle);
        // RTT_Printf("xiaomi2_online=%d  xiaomi2_angle=%f\n",
        //            g_xiaomi_motor_registry[XIAOMI_Motor2].feedback.online,
        //            g_xiaomi_motor_registry[XIAOMI_Motor2].feedback.angle);
        RTT_Printf("current_climb_state:%d climb_cnt:%d\r\n",current_climb_state,climb_cnt);
        if (g_motor_list[DM_FRONT_LEFT_G].get_state != NULL &&
            g_motor_list[DM_FRONT_RIGHT_G].get_state != NULL &&
            g_motor_list[BLAZER_FOC_MOTOR2_G].get_state != NULL &&
            g_motor_list[BLAZER_FOC_MOTOR4_G].get_state != NULL) {
                    Motor_State_t dm_l = g_motor_list[DM_FRONT_LEFT_G].get_state(&g_motor_list[DM_FRONT_LEFT_G]);
                    Motor_State_t dm_r = g_motor_list[DM_FRONT_RIGHT_G].get_state(&g_motor_list[DM_FRONT_RIGHT_G]);
                    Motor_State_t foc_l = g_motor_list[BLAZER_FOC_MOTOR2_G].get_state(&g_motor_list[BLAZER_FOC_MOTOR2_G]);
                    Motor_State_t foc_r = g_motor_list[BLAZER_FOC_MOTOR4_G].get_state(&g_motor_list[BLAZER_FOC_MOTOR4_G]);

                    RTT_Printf("dm(rad/s): L=%f R=%f | foc(rpm): L=%f R=%f\r\n",
                               dm_l.speed,
                               dm_r.speed,
                               foc_l.speed,
                               foc_r.speed);
            RTT_Printf("dm_dbg: L set=%f fb=%f vmax=%f\r\n",
                               g_dm_motor_registry[DM_Motor1].ctrl.vel_set,
                               g_dm_motor_registry[DM_Motor1].para.vel,
                               g_dm_motor_registry[DM_Motor1].tmp.VMAX
                               );
        }


        osDelay(1000);
    }
    /* USER CODE END StartTask_Printf */
}
