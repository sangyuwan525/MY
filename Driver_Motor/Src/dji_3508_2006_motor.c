#include "dji_3508_2006_motor.h"
#include "main.h"
#include "fdcan.h"
#include "pid.h"
//#include "basic.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>

//3508电流范围   -16384-16384
//2006电流范围   -10000-10000

static int first_time = 0;

/**************内部变量与函数begin**************/
// static Dji_Send_Buffer_t g_send_buffers[MAX_CAN_HANDLES];
static motor_pid_parameter motor_3508_pid_g[9];/*电机pid参数*/
static int control_flag=1;
// --- 内部静态函数声明 ---
static void Get_total_angle(motor_measure_t *p);
static void Dji_pid_parameter_init(void);
static void Can_dji_3508_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
/**************内部变量与函数end**************/

const uint8_t SYNC_GROUP_IDS[TOTAL_SYNC_GROUPS][MAX_MOTORS_PER_GROUP] ={
    {DJI_YL,DJI_YR},
    {DJI_XL,DJI_XR}
};

/**************外部接口begin**************/
//Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT];//全局注册表
/**************外部接口end**************/

/**
 * @brief  各套 PID 参数初始化参数
 * @note   必须先调用
 */
static void Dji_pid_parameter_init(void){
    Pid_increment_struct_init(&motor_3508_pid_g[0].loc,   0.15f, 	0.001f,	0.035f,  400,    -400);
    Pid_increment_struct_init(&motor_3508_pid_g[0].spd,    16.7f,	0.98f,	0.024314f,  10000,  -10000);


    Pid_increment_struct_init(&motor_3508_pid_g[1].loc,   0.15f, 	0.001f,	0.035f,   400,    -400);
    Pid_increment_struct_init(&motor_3508_pid_g[1].spd,  16.7f,	0.98f,	0.024314f,  10000,  -10000);

    Pid_increment_struct_init(&(motor_3508_pid_g[2].loc), 0.15f, 	0.001f,	0.035f,  400,    -400);
    Pid_increment_struct_init(&(motor_3508_pid_g[2].spd),   16.7f,	0.98f,	0.024314f,  10000,  -10000);

    Pid_increment_struct_init(&motor_3508_pid_g[3].loc,  0.15f, 	0.001f,	0.035f,  400,    -400);
    Pid_increment_struct_init(&motor_3508_pid_g[3].spd,  16.7f,	0.98f,	0.024314f,  10000,  -10000);

    Pid_increment_struct_init(&motor_3508_pid_g[4].loc,  0.15f, 	0.001f,	0.035f,   600,    -600);
    Pid_increment_struct_init(&motor_3508_pid_g[4].spd,  16.7f,	0.98f,	0.024314f,  10000,  -10000);// 10.7f,	0.98f,	0.024314f,  10000,  -10000

    Pid_increment_struct_init(&motor_3508_pid_g[5].loc, 0.15f,	0.001f,	0.035f,   600,    -600);//3508-J1
    Pid_increment_struct_init(&motor_3508_pid_g[5].spd,   16.7f,	0.98f,	0.024314f,  10000,  -10000);

    Pid_increment_struct_init(&motor_3508_pid_g[6].loc,  0.3f, 	0.001f,	0.03f,  600,    -600);//2006-R1
    Pid_increment_struct_init(&motor_3508_pid_g[6].spd,  12.7f, 	0.98f,	  0.035f,   10000,  -10000);

    Pid_increment_struct_init(&motor_3508_pid_g[7].loc,  0.15f, 	0.001f,	0.03f,  1000,    -1000);//2006
    Pid_increment_struct_init(&motor_3508_pid_g[7].spd,  15.0f, 	0.98f,	  0.035f,   10000,  -10000);

    Pid_increment_struct_init(&motor_3508_pid_g[8].loc,  0.15f, 	0.001f,	0.03f,  4000,    -4000);
    Pid_increment_struct_init(&motor_3508_pid_g[8].spd,  24.0f, 	1.2f,	  0.035f,   9500,  -9500);

    //同步
    //参数要调试
    // Y轴组 (ID 0, 1)
    Pid_increment_struct_init(&motor_3508_pid_g[0].diff, 0.015f, 0.0f, 0.0f, 600, -600);
    Pid_increment_struct_init(&motor_3508_pid_g[1].diff, 0.015f, 0.0f, 0.0f, 600, -600);

    // X轴组 (ID 2, 3)
    Pid_increment_struct_init(&motor_3508_pid_g[2].diff, 0.015f, 0.0f, 0.0f, 600, -600);
    Pid_increment_struct_init(&motor_3508_pid_g[3].diff, 0.015f, 0.0f, 0.0f, 600, -600);
}
/**
  * @brief  统一的DJI电机注册表
  * @note   电机数量为 DJI_MOTOR_COUNT。配置必须在这里明确定义，包括嵌入的 PID 参数。
  */
Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT] =
{
    // ------------------------------------------------------------------------
    // 索引 0: DJI_YL - 上下(Y轴)左边电机 - CAN1 - ID 0x201
    // ------------------------------------------------------------------------
    [DJI_YL] = {
        .hcan_tx          = &hfdcan1,
        .can_rx_id        = CAN_3508_M1_ID,
        .can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 0,
        .target_spd       = 0,
        .target_loc       = 0,
        .control_mode     = GROUP_MODE,
        .is_enabled       = true,
    },
    // ------------------------------------------------------------------------
    // 索引 1: DJI_YR - 上下(Y轴)右边电机 - CAN1 - ID 0x202
    // ------------------------------------------------------------------------
    [DJI_YR] = {
        .hcan_tx          = &hfdcan1,
        .can_rx_id        = CAN_3508_M2_ID,
        .can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 1,
        .target_spd       = 0,
        .target_loc       = 0,
        .control_mode     = GROUP_MODE,
        .is_enabled       = true,
    },
    // ------------------------------------------------------------------------
    // 索引 2: DJI_XL - 前后(X轴)左边电机 - CAN1 - ID 0x203
    // ------------------------------------------------------------------------
    [DJI_XL] = {
        .hcan_tx          = &hfdcan1,
        .can_rx_id        = CAN_3508_M3_ID,
        .can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 2,
        .target_spd       = 0,
        .target_loc       = 0,
        .control_mode     = GROUP_MODE,
        .is_enabled       = true,
    },
    // ------------------------------------------------------------------------
    // 索引 3: DJI_XR - 前后(X轴)右边电机 - CAN1 - ID 0x204
    // ------------------------------------------------------------------------
    [DJI_XR] = {
        .hcan_tx          = &hfdcan1,
        .can_rx_id        = CAN_3508_M4_ID,
        .can_tx_header_id = CAN_FIRST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 3,
        .target_spd       = 0,
        .target_loc       = 0,
        .control_mode     = GROUP_MODE,
        .is_enabled       = true,
    },

    // ------------------------------------------------------------------------
    // 索引 4: DJI_JOINT1_3508 - 机械臂关节1 - CAN2 - ID 0x205
    // ------------------------------------------------------------------------
    [DJI_JOINT1_3508] = {
        .hcan_tx          = &hfdcan2,
        .can_rx_id        = CAN_3508_M5_ID,
        .can_tx_header_id = CAN_LAST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 0,
        .target_spd       = 0,
        .target_loc       = 0,
        .control_mode     = LOC_MODE,
        .is_enabled       = true,
    },
    // ------------------------------------------------------------------------
    // 索引 5: DJI_JOINT2_2006 - 机械臂关节2 - CAN2 - ID 0x206
    // ------------------------------------------------------------------------
    [DJI_JOINT2_2006] = {
        .hcan_tx          = &hfdcan2,
        .can_rx_id        = CAN_2006_M6_ID,
        .can_tx_header_id = CAN_LAST_FOUR_MOTOR_ALL_ID,
        .tx_index         = 1,
        .target_spd       = 0,
        .target_loc       = 0,
        .control_mode     = LOC_MODE,
        .is_enabled       = true,
    },
};

Can_Tx_Buffer_t g_can_tx_buffers[MAX_CAN_HANDLES] = {
    {.hcan = &hfdcan1, .need_to_send = false}, // Index 0: FDCAN1
    {.hcan = &hfdcan2, .need_to_send = false}, // Index 1: FDCAN2
    {.hcan = &hfdcan3, .need_to_send = false}  // Index 2: FDCAN3
};

/**
 * @brief  初始化DJI电机注册表中的所有配置。
 *
 * @param  None
 *
 * @return None
 *
 * @note   1. 调用 Dji_pid_parameter_init() 初始化 motor_3508_pid_g 数组。
 * @note   2. 将初始化好的 PID 参数拷贝到注册表中的相应电机实例。
 * @see    Dji_pid_parameter_init
 */
void Dji_Motor_Registry_Init(void)
{
    Dji_pid_parameter_init();
    g_can_tx_buffers[0].hcan = &hfdcan1;
    g_can_tx_buffers[1].hcan = &hfdcan2;
    g_can_tx_buffers[2].hcan = &hfdcan3;
    for (int i = 0; i < DJI_MOTOR_COUNT; i++)
    {
        Dji_Motor_t *motor = &g_dji_motor_registry[i];

        // 初始化通用状态
        motor->feedback.first = 0;
        motor->current_set = 0;
        motor->is_online = false;

        motor->sync_group_id = -1;
        motor->average_loc = 0;

        // 遍历查找该电机属于哪个组
        for(int g = 0; g < TOTAL_SYNC_GROUPS; g++) {
            for(int m = 0; m < MAX_MOTORS_PER_GROUP; m++) {
                if(SYNC_GROUP_IDS[g][m] == i) {
                    motor->sync_group_id = g;
                    motor->pid_params.diff.err = 0;
                    motor->pid_params.diff.err_last = 0;
                    break;
                }
            }
        }
        // 假设 motor_3508_pid_g 的索引 i 对应注册表中的电机实例 i
        if (i < DJI_MOTOR_COUNT) // 确保不越界访问 motor_3508_pid_g 数组 (大小为 9)
        {
            memcpy(&motor->pid_params,
                   &motor_3508_pid_g[i],
                   sizeof(motor_pid_parameter));
        }
    }
}

/**
 * @brief 计算某一组的平均位置
 * @param group_id
 */
static float Calculate_Group_Average_Angle(int group_id)
{
    if (group_id < 0 || group_id >= TOTAL_SYNC_GROUPS) return 0.0f;

    int32_t total_sum = 0;
    int count = 0;

    for (int i = 0; i < MAX_MOTORS_PER_GROUP; i++)
    {
        uint8_t motor_id = SYNC_GROUP_IDS[group_id][i];
        Dji_Motor_t *motor = &g_dji_motor_registry[motor_id];

        // 只有当电机启用了 GROUP_MODE 且已在线时才参与计算
        if (motor->control_mode == GROUP_MODE && motor->feedback.first == 1)
        {
            total_sum += motor->feedback.total_angle;
            count++;
        }
    }

    if (count > 0) return (float)total_sum / count;
    return 0.0f;
}

/**
  * @brief  获取指定电机的反馈信息
  * @param  motor_id: 电机索引 (0 - DJI_MOTOR_COUNT-1)
  * @return motor_measure_t 结构体，包含电机的反馈数据
  */
motor_measure_t Get_dji_information(int motor_id){
    motor_measure_t temp_info = {0};
    if (motor_id < DJI_MOTOR_COUNT)
    {
        taskENTER_CRITICAL();
        {
            temp_info = g_dji_motor_registry[motor_id].feedback;
        }
        taskEXIT_CRITICAL();
    }
    return temp_info;
}

void Change_dji_loc(int motor_id, float location)
{
    if (motor_id < 0 || motor_id >= DJI_MOTOR_COUNT) {
        return;
    }
    Dji_Motor_SetLoc((Dji_MotorID_e)motor_id, location);
}

void Change_dji_speed(int motor_id, float speed)
{
    if (motor_id < 0 || motor_id >= DJI_MOTOR_COUNT) {
        return;
    }
    Dji_Motor_SetSpeed((Dji_MotorID_e)motor_id, speed);
}

void Dji_Motor_Update_Status(FDCAN_HandleTypeDef *hcan, uint32_t identifier, uint8_t *rx_data)
{
    for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
        Dji_Motor_t *motor = &g_dji_motor_registry[i];
        if (motor->hcan_tx == hcan && motor->can_rx_id == identifier) {
            Dji_Motor_Update(motor, rx_data);
            return;
        }
    }
}

/**
  * @brief  设置电机位置控制目标
  * @param  id:       电机索引 (0 - DJI_MOTOR_COUNT-1)
  * @param  location: 目标位置 (单位：编码器计数)
  */
void Dji_Motor_SetLoc(Dji_MotorID_e id, float location)
{
    Dji_Motor_t *motor = &g_dji_motor_registry[id];

    // 2. 设置目标值
    motor->target_loc = location;

    // 3. 核心：自动切换控制模式！
    // 这样你就不用担心忘记切模式导致电机乱跑了
    if (motor->control_mode != LOC_MODE) {
        motor->control_mode = LOC_MODE;

        // 可选：切换模式时清空 PID 积分项，防止瞬间突变
        // motor->pid_params.loc.err = 0;
        // motor->pid_params.loc.now_out = 0;
    }
}

/**
  * @brief  设置电机速度控制目标
  * @param  id:    电机索引 (0 - DJI_MOTOR_COUNT-1)
  * @param  speed: 目标速度 (单位：RPM)
  */
void Dji_Motor_SetSpeed(Dji_MotorID_e id, float speed)
{

    Dji_Motor_t *motor = &g_dji_motor_registry[id];

    motor->target_spd = speed;

    // 核心：自动切换为速度模式
    if (motor->control_mode != SPEED_MODE) {
        motor->control_mode = SPEED_MODE;

        // 切换模式时的平滑处理
        motor->pid_params.spd.err = 0;
    }
}

/**
 * @brief 设置群组电机位置
 * @param group_id
 * @param target_loc
 */
void Dji_Motor_SetGroupLoc(int group_id, float target_loc) {
    if (group_id < 0 || group_id >= TOTAL_SYNC_GROUPS) return;

    for (int i = 0; i < MAX_MOTORS_PER_GROUP; i++) {
        uint8_t motor_id = SYNC_GROUP_IDS[group_id][i];
        Dji_Motor_t *motor = &g_dji_motor_registry[motor_id];

        motor->target_loc = target_loc;
        if (motor->control_mode != GROUP_MODE)
        {
            motor->control_mode = GROUP_MODE; // 自动切入同步模式
        }
    }
}

/**
 * @brief 是否控制电机
 */
void Discontrol_dji_motor(void){
    control_flag=0;
}
void Recontrol_dji_motor(void){
    control_flag=1;
}

// 检查电机 ID 是否需要反向
// 假设 右侧电机 (XR, YR) 是镜像安装的
static bool Is_Motor_Inverted(int motor_id) {
    if (motor_id == DJI_XL || motor_id == DJI_YL) {
        return true;
    }
    return false;
}

/**
  * @brief  更新电机反馈数据 (在 CAN 接收任务中调用)
  * @param  motor:   电机结构体指针
  * @param  rx_data: CAN 接收到的 8 字节原始数据
  */
void Dji_Motor_Update(Dji_Motor_t *motor, uint8_t *rx_data) {

    // 1. 先解析出原始数据
    uint16_t raw_angle   = (uint16_t)((rx_data[0] << 8) | rx_data[1]);
    int16_t  raw_speed   = (int16_t)((rx_data[2] << 8) | rx_data[3]);
    int16_t  raw_current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
    uint8_t  temp        = rx_data[6];

    // 保存上一时刻的角度（注意：这里保存的是修正前的还是修正后的？
    // 为了多圈解算正确，我们应该保存逻辑角度。
    // 所以这一行放到修正逻辑之后比较好，或者直接用结构体里的旧值）
    motor->feedback.last_angle = motor->feedback.angle;

    // 2. 【核心修改】方向修正
    // 获取当前电机在数组中的索引 (通过指针倒推或者结构体里存个ID)
    // 你的结构体里没有直接存 ID，我们通过指针判断
    int motor_id = motor - g_dji_motor_registry;

    if (Is_Motor_Inverted(motor_id) && g_dji_motor_registry[motor_id].control_mode == GROUP_MODE) {
        // --- 反向电机处理 ---

        // 角度取反 (0~8191)
        // 比如物理是 100，逻辑上认为是 -100 (对应编码器就是 8192-100)
        if (raw_angle == 0) {
            motor->feedback.angle = 0;
        } else {
            motor->feedback.angle = 8192 - raw_angle;
        }

        // 速度和电流直接取负
        motor->feedback.speed_rpm = -raw_speed;
        motor->feedback.given_current = -raw_current;
    }
    else {
        // --- 正向电机：原样赋值 ---
        motor->feedback.angle = raw_angle;
        motor->feedback.speed_rpm = raw_speed;
        motor->feedback.given_current = raw_current;
    }

    motor->feedback.temperate = temp;

    // 3. 初始化处理
    if (motor->feedback.first == 0) {
        motor->feedback.first = 1;
        motor->feedback.last_angle = motor->feedback.angle;
        motor->feedback.total_angle = 0;
        return;
    }

    // 4. 多圈解算
    Get_total_angle(&motor->feedback);

    motor->is_online = true;

}

/**
  * @brief  计算 PID 输出 (在控制任务中定时调用)
  * @param  motor: 电机结构体指针
  */
void Dji_Motor_Calc(Dji_Motor_t *motor)
{
    // 如果还没收到过反馈数据，不要计算，防止误差累积导致飞车
    if (motor->feedback.first == 0) {
        motor->current_set = 0;
        return;
    }

    // ----------------------
    // 模式 1: 位置模式 (串级 PID)
    // ----------------------
    if (motor->control_mode == LOC_MODE)
    {
        // 1. 外环 (位置环)
        // 输入: 当前总角度 (total_angle)
        // 目标: 目标角度 (target_loc)
        // 输出: 存入 pid.loc.now_out，作为内环的目标速度
        Pid_incremental_cal(&motor->pid_params.loc, (ElemType)motor->feedback.total_angle, (ElemType)motor->target_loc);

        // 2. 内环 (速度环)
        // 输入: 当前转速 (speed_rpm)
        // 目标: 外环输出 (pid.loc.now_out)
        Pid_incremental_cal(&motor->pid_params.spd, (ElemType)motor->feedback.total_angle, (ElemType)motor->pid_params.loc.now_out);
    }
    // ----------------------
    // 模式 2: 速度模式 (单环 PID)
    // ----------------------
    else if (motor->control_mode == SPEED_MODE)
    {
        // 1. 速度环
        // 输入: 当前转速
        // 目标: 目标速度 (target_speed)
        Pid_incremental_cal(&motor->pid_params.spd, (float)motor->feedback.speed_rpm, motor->target_spd);
    }

    // ----------------------
    // 输出赋值
    // ----------------------
    // 最终电流值取自速度环的输出
    motor->current_set = (int16_t)motor->pid_params.spd.now_out;
}

/**
  * @brief  发送四个DJI电机的电流指令
  * @param  hcan:            FDCAN句柄指针
  * @param  all_response_id: 四电机统一发送ID
  * @param  motor1:         电机1电流指令
  * @param  motor2:         电机2电流指令
  * @param  motor3:         电机3电流指令
  * @param  motor4:         电机4电流指令
  */
static void Can_dji_3508_motor_send(FDCAN_HandleTypeDef* hcan , uint32_t all_response_id , int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4){
    static FDCAN_TxHeaderTypeDef  	first_four_motor_tx_message;
    static uint8_t              	first_four_motor_can_send_data[8];

    first_four_motor_tx_message.Identifier 	=	all_response_id;
    first_four_motor_tx_message.IdType		=	FDCAN_STANDARD_ID;
    first_four_motor_tx_message.TxFrameType = 	FDCAN_DATA_FRAME;
    first_four_motor_tx_message.DataLength 	= 	0x08;
    first_four_motor_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    first_four_motor_tx_message.BitRateSwitch = FDCAN_BRS_OFF;
    first_four_motor_tx_message.FDFormat = FDCAN_FD_CAN;
    first_four_motor_tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    first_four_motor_tx_message.MessageMarker = 0;

    first_four_motor_can_send_data[0] = motor1 >> 8;
    first_four_motor_can_send_data[1] = motor1;
    first_four_motor_can_send_data[2] = motor2 >> 8;
    first_four_motor_can_send_data[3] = motor2;
    first_four_motor_can_send_data[4] = motor3 >> 8;
    first_four_motor_can_send_data[5] = motor3;
    first_four_motor_can_send_data[6] = motor4 >> 8;
    first_four_motor_can_send_data[7] = motor4;

    if(hcan==&hfdcan3){
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &first_four_motor_tx_message, first_four_motor_can_send_data);
    }
    else if(hcan==&hfdcan1){
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &first_four_motor_tx_message, first_four_motor_can_send_data);
    }
    else if(hcan==&hfdcan2){
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &first_four_motor_tx_message, first_four_motor_can_send_data);
    }
}
/**
  * @brief  计算电机的多圈总角度
  * @param  p: 指向 motor_measure_t 结构体的指针
  */
static void Get_total_angle(motor_measure_t *p)
{
    int res1, res2, delta;

    // 计算跨越 0 点的两种可能情况
    if (p->angle < p->last_angle) {
        // 比如从 8100 变到 100，可能是正转跨过0点
        res1 = p->angle + 8192 - p->last_angle;
        // 也可能是反转了一大圈（极少见，除非转速极快）
        res2 = p->angle - p->last_angle;
    } else {
        // 比如从 100 变到 8100
        res1 = p->angle - 8192 - p->last_angle;
        res2 = p->angle - p->last_angle;
    }

    // 取绝对值较小的那个作为真实变化量（假设采样周期内电机转不过半圈）
    if (abs(res1) < abs(res2))
        delta = res1;
    else
        delta = res2;

    p->total_angle += delta;
    p->last_angle = p->angle;
    // p->last_angle 已经在 Update 函数开头更新过了，这里不需要再更新
}

/**
 * @brief  集成 PID 计算和发送所有电机电流
 *
 * @param  无
 *
 * @return 无
 *
 * @note   该函数在 FreeRTOS 任务中定期调用，负责执行所有控制算法和CAN发送。
 * @see    Calculate_Group_Average_Angle
 */
void Dji_3508_all_motor_control(void) {
    // 1. 定义局部变量作为“快照”
    // 为什么要用局部变量？因为局部变量是私有的，别的任务改不了，不用加锁，算起来最安全。
    motor_measure_t feedback_snapshot[DJI_MOTOR_COUNT];

    // ================= 阶段1：读取数据 (临界区保护) =================
    taskENTER_CRITICAL();

    // A. 顺便把发送缓冲区清了（因为这个很快）
    for (int i = 0; i < MAX_CAN_HANDLES; i++) {
       memset(g_can_tx_buffers[i].currents_0x200, 0, sizeof(g_can_tx_buffers[i].currents_0x200));
       memset(g_can_tx_buffers[i].currents_0x1FF, 0, sizeof(g_can_tx_buffers[i].currents_0x1FF));
       g_can_tx_buffers[i].need_to_send = false;
    }

    // B. 【核心】拍摄快照
    // 瞬间把所有电机的当前状态复制到局部变量里
    for(int i=0; i<DJI_MOTOR_COUNT; i++) {
        feedback_snapshot[i] = g_dji_motor_registry[i].feedback;
    }

    taskEXIT_CRITICAL();

    // ================= 阶段2：复杂计算 (无锁，放心算) =================
    // 注意：下面的代码全部使用 feedback_snapshot[i]，不要直接读 g_dji_motor_registry

    float group_avgs[TOTAL_SYNC_GROUPS] = {0};

    // 1. 计算平均值 (这里简化了逻辑，直接在这里算，不用调外部函数了，方便用snapshot)
    for(int g=0; g<TOTAL_SYNC_GROUPS; g++) {
        float sum = 0; int cnt = 0;
        for(int m=0; m<MAX_MOTORS_PER_GROUP; m++) {
            int mid = SYNC_GROUP_IDS[g][m];
            // 模式判断读全局没事，因为模式不会频繁变
            if(g_dji_motor_registry[mid].control_mode == GROUP_MODE && feedback_snapshot[mid].first == 1) {
                sum += feedback_snapshot[mid].total_angle; // 读快照！
                cnt++;
            }
        }
        if(cnt > 0) group_avgs[g] = sum / cnt;
    }

    // 2. PID 计算
    for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
        Dji_Motor_t *motor = &g_dji_motor_registry[i]; // 指针指向全局对象，用来存结果和读参数
        motor_measure_t *fb = &feedback_snapshot[i];   // 指针指向快照，用来读反馈

        int can_index = -1;
        if (motor->hcan_tx == &hfdcan1) can_index = FDCNA1;
        else if (motor->hcan_tx == &hfdcan2) can_index = FDCNA2;
        else if (motor->hcan_tx == &hfdcan3) can_index = FDCNA3;

        // 检查快照里的状态
        if (motor->is_enabled && fb->first == 1  && can_index != -1) {

            // 将计算好的平均值填入电机结构体 (更新全局变量，没事)
            if(motor->sync_group_id != -1) {
                motor->average_loc = (int32_t)group_avgs[motor->sync_group_id];
            }

            // --- PID 计算 (耗时操作) ---
            if (motor->control_mode == LOC_MODE)
            {
                Pid_incremental_cal(&motor->pid_params.loc,
                                    (ElemType)fb->total_angle, // 读快照
                                    (ElemType)motor->target_loc);

                Pid_incremental_cal(&motor->pid_params.spd,
                                    (ElemType)fb->speed_rpm,   // 读快照
                                    motor->pid_params.loc.now_out);
            }
            else if (motor->control_mode == SPEED_MODE)
            {
                Pid_incremental_cal(&motor->pid_params.spd,
                                    (ElemType)fb->speed_rpm,   // 读快照
                                    (ElemType)motor->target_spd);
            }
            else if (motor->control_mode == GROUP_MODE)
            {
                Pid_incremental_cal(&motor->pid_params.loc,
                                    (float)fb->total_angle,    // 读快照
                                    (float)motor->target_loc);

                Pid_incremental_cal(&motor->pid_params.diff,
                                    (float)fb->total_angle,    // 读快照
                                    (float)motor->average_loc);

                float spd_ref = motor->pid_params.loc.now_out + motor->pid_params.diff.now_out;

                Pid_incremental_cal(&motor->pid_params.spd,
                                    (float)fb->speed_rpm,      // 读快照
                                    spd_ref);
            }

            // 存入暂存变量
            motor->current_set = (int16_t)motor->pid_params.spd.now_out;

            if (Is_Motor_Inverted(i)) {
                motor->current_set = -motor->current_set;
            }
        }
        else
        {
            motor->current_set = 0;
        }

        if (control_flag == 0) {
            motor->current_set = 0;
        }
    }

    // ================= 阶段3：填充缓冲区 (临界区保护) =================
    // 因为这涉及到写 g_can_tx_buffers，要防止冲突（虽然这里单任务写，但加上更规范）
    taskENTER_CRITICAL();

    for (int i = 0; i < DJI_MOTOR_COUNT; i++) {
        Dji_Motor_t *motor = &g_dji_motor_registry[i];

        // 重新判断一下索引
        int can_index = -1;
        if (motor->hcan_tx == &hfdcan1) can_index = FDCNA1;
        else if (motor->hcan_tx == &hfdcan2) can_index = FDCNA2;
        else if (motor->hcan_tx == &hfdcan3) can_index = FDCNA3;

        if (can_index != -1 && motor->current_set != 0) {
           if (motor->can_tx_header_id == CAN_FIRST_FOUR_MOTOR_ALL_ID && motor->tx_index < 4) {
              g_can_tx_buffers[can_index].currents_0x200[motor->tx_index] = motor->current_set;
              g_can_tx_buffers[can_index].need_to_send = true;
           } else if (motor->can_tx_header_id == CAN_LAST_FOUR_MOTOR_ALL_ID && motor->tx_index < 4) {
              g_can_tx_buffers[can_index].currents_0x1FF[motor->tx_index] = motor->current_set;
              g_can_tx_buffers[can_index].need_to_send = true;
           }
        }
    }

    taskEXIT_CRITICAL(); // <--- 再次开中断

    // ================= 阶段4：发送 (外部IO操作，无需保护) =================
    for (int i = 0; i < MAX_CAN_HANDLES; i++) {
       Can_Tx_Buffer_t *buffer = &g_can_tx_buffers[i];
       if (buffer->need_to_send) {
          // 发送代码不变...
          Can_dji_3508_motor_send(buffer->hcan,
              CAN_FIRST_FOUR_MOTOR_ALL_ID,
              buffer->currents_0x200[0],
              buffer->currents_0x200[1],
              buffer->currents_0x200[2],
              buffer->currents_0x200[3]);
          Can_dji_3508_motor_send(buffer->hcan,
              CAN_LAST_FOUR_MOTOR_ALL_ID,
              buffer->currents_0x1FF[0],
              buffer->currents_0x1FF[1],
              buffer->currents_0x1FF[2],
              buffer->currents_0x1FF[3]);
       }
    }
}
