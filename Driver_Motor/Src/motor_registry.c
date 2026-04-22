//
// Created by 马皓然 on 2026/1/31.
//
#include "motor_registry.h"

#include "motor_registry.h"
#include <string.h>

#include "SEGGER_RTT.h"

/* 外部变量引用 */
extern Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT]; //dji电机系列注册表
extern Damiao_Motor_t g_dm_motor_registry[DM_MOTOR_COUNT]; // 达妙电机系列注册表
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

Motor_Class_t g_motor_list[MOTOR_TOTAL_NUM];

// ======================================================
// 1. DJI 电机适配器
// ======================================================

static void DJI_Adapter_Init(Motor_Class_t* self) {
    (void)self; // DJI 初始化在底层 Dji_Motor_Registry_Init 已完成
}

static void DJI_Adapter_SetSpeed(Motor_Class_t* self, float speed) {
    if (self == NULL || self->instance == NULL) return;
    Dji_Motor_t* dji = (Dji_Motor_t*)self->instance;
    int index = dji - g_dji_motor_registry;
    // 调用底层函数，它会自动将 dji->control_mode 切为 SPEED_MODE
    Dji_Motor_SetSpeed((Dji_MotorID_e)index, speed);
}

static void DJI_Adapter_SetPosition(Motor_Class_t* self, float position, float vel_limit) {
    if (self == NULL || self->instance == NULL) return;
    Dji_Motor_t* dji = (Dji_Motor_t*)self->instance;
    int index = dji - g_dji_motor_registry;
    // DJI 的双环 PID 中，外环输出限制通常是写死的 PID 参数
    // 如果需要动态改变速度限制，可以修改 dji->pid_params.loc.out_limit_up
    // 这里暂时只设置位置
    if (vel_limit > 0) {
        // 可选：动态修改最大速度限制
        dji->pid_params.loc.out_limit_up = vel_limit;
        dji->pid_params.loc.out_limit_down = -vel_limit;
    }

    // 判断是否属于同步组
    if (dji->sync_group_id != -1) {
        Dji_Motor_SetGroupLoc(dji->sync_group_id, position);
    } else {
        Dji_Motor_SetLoc((Dji_MotorID_e)index, position);
    }
}

static void DJI_Adapter_SetMIT(Motor_Class_t* self, float position, float speed, float kp, float kd, float torque) {
    (void)self;
    (void)position;
    (void)speed;
    (void)kp;
    (void)kd;
    (void)torque;
}

static void DJI_Adapter_SetPSI(Motor_Class_t* self, float position, float speed, float current) {
    (void)self;
    (void)position;
    (void)speed;
    (void)current;
}

static void DJI_Adapter_Update(Motor_Class_t* self, uint8_t* rx_data) {
    Dji_Motor_Update((Dji_Motor_t*)self->instance, rx_data);
}

static Motor_State_t DJI_Adapter_GetState(Motor_Class_t* self) {
    Motor_State_t state = {0}; // 初始化为0，防止有未赋值的字段

    if (self && self->instance) {
        Dji_Motor_t* dji = (Dji_Motor_t*)self->instance;
        state.angle  = (float)dji->feedback.total_angle;
        state.speed  = (float)dji->feedback.speed_rpm;
        state.torque = (float)dji->feedback.given_current;
        state.temp   = (float)dji->feedback.temperate;
    }

    return state; // 直接返回结构体
}
// ======================================================
// 2. 达妙 (Damiao) 电机适配器
// ======================================================

static void DM_Adapter_Init(Motor_Class_t* self) {
    Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

    // dm->hcan = &hfdcan2; // 绑定 CAN 句柄
    // dm->id = DM_MOTOR1_CAN_ID; // 设置 ID
    // dm->ctrl.mode = pos_mode;  // 默认模式
    // dm_motor_clear_para(dm);   // 清除参数
    dm_motor_clear_para(dm);

    if (dm->hcan != NULL) {
        // --- 暴力唤醒序列 ---
        // A. 先狂发几次清除错误 (防止电机上电有报错)
        for(int i=0; i<3; i++) {
            dm_motor_clear_err(dm);
            osDelay(10); // 注意：初始化阶段还没进 RTOS，必须用 HAL_Delay
        }

        // B. 狂发几次使能 (确保电机收到)
        // 达妙电机如果收到模式指令，会由红灯变绿灯/闪烁
        for(int i=0; i<5; i++) {
            dm_motor_enable(dm);
            osDelay(100);
        }
    }
    //dm_motor_enable(dm);
}

static void DM_Adapter_SetSpeed(Motor_Class_t* self, float speed) {
    Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

    // 切换模式标记，Loop 函数会根据这个标记调用 spd_ctrl
    dm->ctrl.mode = spd_mode;

    dm->ctrl.vel_set = speed;
    dm->ctrl.pos_set = 0; // 速度模式下位置通常无意义，或者是持续累加，清零较安全
}

static void DM_Adapter_SetPosition(Motor_Class_t* self, float position, float vel_limit) {
    Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

    // 切换模式标记，Loop 函数会根据这个标记调用 pos_ctrl
    dm->ctrl.mode = pos_mode;

    dm->ctrl.pos_set = position;
    dm->ctrl.vel_set = vel_limit; // 在位置模式下，这通常代表前馈速度或速度限制
}

static void DM_Adapter_SetMIT(Motor_Class_t* self, float position, float speed, float kp, float kd, float torque) {
    Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

    dm->ctrl.mode = mit_mode;
    dm->ctrl.pos_set = position;
    dm->ctrl.vel_set = speed;
    dm->ctrl.kp_set = kp;
    dm->ctrl.kd_set = kd;
    dm->ctrl.tor_set = torque;
}

static void DM_Adapter_SetPSI(Motor_Class_t* self, float position, float speed, float current) {
    Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

    dm->ctrl.mode = psi_mode;
    dm->ctrl.pos_set = position;
    dm->ctrl.vel_set = speed;
    dm->ctrl.cur_set = current;
}

static void DM_Adapter_Update(Motor_Class_t* self, uint8_t* rx_data) {
    Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

    dm_motor_fbdata(dm, rx_data);
    receive_motor_data(dm, rx_data);
}

static Motor_State_t DM_Adapter_GetState(Motor_Class_t* self) {
    Motor_State_t state = {0};

    if (self && self->instance) {
        Damiao_Motor_t* dm = (Damiao_Motor_t*)self->instance;

        state.angle  = dm->para.pos;
        state.speed  = dm->para.vel;
        state.torque = dm->para.tor;
        state.temp   = dm->para.Tmos;
    }

    return state;
}
// ======================================================
// 3. 注册表初始化
// ======================================================

void Motor_Registry_Init(void) {
    // 1. 初始化底层
    Dji_Motor_Registry_Init();
    SEGGER_RTT_printf(0,"finish dji init\r\n");
    dm_motor_init();
    SEGGER_RTT_printf(0,"finish dm init\r\n");


    // 2. 注册 DJI 电机
    for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
        g_motor_list[i].type = MOTOR_TYPE_DJI;
        g_motor_list[i].instance = &g_dji_motor_registry[i];

        g_motor_list[i].init = DJI_Adapter_Init;
        g_motor_list[i].set_speed = DJI_Adapter_SetSpeed;      // 绑定 SetSpeed
        g_motor_list[i].set_position = DJI_Adapter_SetPosition; // 绑定 SetPosition
        g_motor_list[i].set_mit = DJI_Adapter_SetMIT;
        g_motor_list[i].set_psi = DJI_Adapter_SetPSI;
        g_motor_list[i].update_feedback = DJI_Adapter_Update;
        g_motor_list[i].get_state = DJI_Adapter_GetState;
    }

    // 3. 注册 达妙 电机
    int dm_start_idx = DJI_MOTOR_COUNT;

    for (int i = 0; i < DM_MOTOR_COUNT; i++) {
        int global_idx = dm_start_idx + i;
        if (global_idx >= MOTOR_TOTAL_NUM) break; // 防止越界

        g_motor_list[global_idx].type = MOTOR_TYPE_DAMIAO;

        // 【关键】直接指向底层定义好的实例
        g_motor_list[global_idx].instance = &g_dm_motor_registry[i];

        // 绑定函数
        g_motor_list[global_idx].init = DM_Adapter_Init;
        g_motor_list[global_idx].set_speed = DM_Adapter_SetSpeed;
        g_motor_list[global_idx].set_position = DM_Adapter_SetPosition;
        g_motor_list[global_idx].set_mit = DM_Adapter_SetMIT;
        g_motor_list[global_idx].set_psi = DM_Adapter_SetPSI;
        g_motor_list[global_idx].update_feedback = DM_Adapter_Update;
        g_motor_list[global_idx].get_state = DM_Adapter_GetState;
    }

    // 4. 执行初始化
    for (int i = 0; i < MOTOR_TOTAL_NUM; i++) {
        if (g_motor_list[i].init != NULL) {
            g_motor_list[i].init(&g_motor_list[i]);
        }
    }
}
// ======================================================
// 4. 反馈分发与控制循环 (保持之前修复的逻辑)
// ======================================================

/**
 * @brief 统一反馈分发函数
 * @param hfdcan 接收到数据的 CAN 句柄
 * @param identifier 接收到的 CAN ID
 * @param data 接收到的 8 字节数据指针
 */
void Motor_Feedback_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data) {

    // 遍历整个电机列表 (DJI + DM)
    for (int i = 0; i < MOTOR_TOTAL_NUM; i++) {
        Motor_Class_t *motor_obj = &g_motor_list[i];

        // --- 匹配逻辑 ---
        bool is_match = false;

        // 1. DJI 电机匹配
        if (motor_obj->type == MOTOR_TYPE_DJI) {
            Dji_Motor_t *dji = (Dji_Motor_t *)motor_obj->instance;
            // 检查：CAN 句柄一致 且 ID 一致
            if (dji->hcan_tx == hfdcan && dji->can_rx_id == identifier) {
                is_match = true;
            }
        }
        // 2. 达妙 电机匹配
        else if (motor_obj->type == MOTOR_TYPE_DAMIAO) {
            Damiao_Motor_t *dm = (Damiao_Motor_t *)motor_obj->instance;
            // 检查：CAN 句柄一致 (存于 wrapper)
            // 且 ID 匹配 (通常反馈ID == 设定ID，或 设定ID | 0x10)
            if (dm->hcan == hfdcan && dm->mst_id == identifier) {
                is_match = true;
            }
        }
        // --- 执行更新 ---
        if (is_match) {
            if (motor_obj->update_feedback != NULL) {
                // 调用多态更新函数
                motor_obj->update_feedback(motor_obj, data);
            }
            // 找到匹配对象后，直接退出循环（假设 ID 不冲突）
            return;
        }
    }
}

void Motor_All_Control_Loop(void) {
    // 1. DJI 计算与发送
    Dji_3508_all_motor_control();

    // 2. 达妙 发送
    for (int i = DJI_MOTOR_COUNT; i < MOTOR_TOTAL_NUM; i++) {
        Motor_Class_t *cls = &g_motor_list[i];
        if (cls->type == MOTOR_TYPE_DAMIAO && cls->instance != NULL ) {
            // dm_motor_ctrl_send 会检查 internal mode (pos_mode/spd_mode) 并发送对应指令
            dm_motor_ctrl_send((Damiao_Motor_t*)cls->instance);
        }
    }
}

void Motor_SetMIT(int motor_index, float position, float speed, float kp, float kd, float torque) {
    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    Motor_Class_t *motor = &g_motor_list[motor_index];
    if (motor->set_mit != NULL) {
        motor->set_mit(motor, position, speed, kp, kd, torque);
    }
}

void Motor_SetPSI(int motor_index, float position, float speed, float current) {
    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    Motor_Class_t *motor = &g_motor_list[motor_index];
    if (motor->set_psi != NULL) {
        motor->set_psi(motor, position, speed, current);
    }
}
