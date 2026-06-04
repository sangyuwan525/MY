# R1 底盘板与上层板 CAN3 通信协议

本文档根据当前 `r2_chassis_fixed` 工程整理。两块板通过 CAN3 通信。

## 1. 总线与帧格式

| 项目 | 当前约定 |
| --- | --- |
| 通道 | CAN3 / FDCAN3 |
| ID 类型 | Standard ID |
| 底盘 FDCAN3 模式 | `FDCAN_FRAME_FD_BRS` |
| 定位报文 `0x12` | CAN FD，DLC = 16 |
| 短控制报文 | DLC <= 8 时底盘发送为 Classic CAN |
| 字节序 | 小端序 |
| 浮点格式 | IEEE754 float32 |

底盘 FDCAN3 初始化在：

```c
// Core/Src/fdcan.c
hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
```

底盘发送函数在 `BSP/Src/bsp_can.c` 中会根据长度选择 Classic CAN 或 CAN FD：

```c
tx_header.FDFormat = (len <= 8U) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
```

## 2. 底盘板发送给上层板

底盘通过 `send_flag_to_up()` 给上层板发送动作指令。

代码位置：

```c
// chassis_control/Src/Hfsm.c
uint8_t send_flag_to_up(uint8_t id)
{
    uint8_t data[1] = {8};
    return fdcanx_send_ex_data(&hfdcan3, 0x300 + id, data, 1, CAN_ID_STD);
}
```

### 2.1 报文格式

| 字段 | 内容 |
| --- | --- |
| CAN ID | `0x300 + flag_id` |
| ID 类型 | 标准帧 |
| DLC | 1 |
| Data[0] | 当前代码固定发送 `0x08` |
| 发送接口 | `send_flag_to_up(flag_id)` |

注意：当前动作编号编码在 CAN ID 中，不在 Data[0] 中。

### 2.2 底盘发送的 flag_id

代码位置：

```c
// chassis_control/Inc/Hfsm.h
typedef enum {
    FLAG_GRAB_KFS_FRONT_HIGH_KEEP = 1,
    FLAG_GRAB_KFS_FRONT_LOW_KEEP,
    FLAG_GRAB_KFS_LEFT_HIGH_KEEP,
    FLAG_GRAB_KFS_LEFT_LOW_KEEP,
    FLAG_GRAB_KFS_RIGHT_HIGH_KEEP,
    FLAG_GRAB_KFS_RIGHT_LOW_KEEP,
    FLAG_GRAB_KFS_FRONT_HIGH_REMOVE,
    FLAG_GRAB_KFS_FRONT_LOW_REMOVE,
    FLAG_ASSEMBLE,
    FLAG_PUT_KFS_MID,
    FLAG_PUT_KFS_TOP,
    FLAG_LIFT,
} FLAG_TO_UP;
```

| flag_id | CAN ID | 含义 |
| --- | --- | --- |
| 1 | `0x301` | 二号区抓前方高块并保留 |
| 2 | `0x302` | 二号区抓前方低块并保留 |
| 3 | `0x303` | 二号区抓左侧高块并保留 |
| 4 | `0x304` | 二号区抓左侧低块并保留 |
| 5 | `0x305` | 二号区抓右侧高块并保留 |
| 6 | `0x306` | 二号区抓右侧低块并保留 |
| 7 | `0x307` | 二号区抓前方高块并移走 |
| 8 | `0x308` | 二号区抓前方低块并移走 |
| 9 | `0x309` | 一区组装武器，`FLAG_ASSEMBLE` |
| 10 | `0x30A` | 三区放置 KFS 到中层 |
| 11 | `0x30B` | 三区放置 KFS 到顶层 |
| 12 | `0x30C` | 三区请求 R1 抬升 |

组装武器时，底盘当前调用：

```c
send_flag_to_up(FLAG_ASSEMBLE);
```

对应上层收到：

```text
CAN ID = 0x309
DLC    = 1
Data   = 08
```

### 2.3 上层板接收底盘 flag 示例代码

上层板只需要判断标准帧 ID 是否在 `0x301..0x30C` 范围内。

```c
#define CHASSIS_FLAG_ID_BASE 0x300U
#define CHASSIS_FLAG_ID_MIN  0x301U
#define CHASSIS_FLAG_ID_MAX  0x30CU

typedef enum {
    UP_CMD_NONE = 0,
    UP_CMD_GRAB_KFS_FRONT_HIGH_KEEP = 1,
    UP_CMD_GRAB_KFS_FRONT_LOW_KEEP,
    UP_CMD_GRAB_KFS_LEFT_HIGH_KEEP,
    UP_CMD_GRAB_KFS_LEFT_LOW_KEEP,
    UP_CMD_GRAB_KFS_RIGHT_HIGH_KEEP,
    UP_CMD_GRAB_KFS_RIGHT_LOW_KEEP,
    UP_CMD_GRAB_KFS_FRONT_HIGH_REMOVE,
    UP_CMD_GRAB_KFS_FRONT_LOW_REMOVE,
    UP_CMD_ASSEMBLE,
    UP_CMD_PUT_KFS_MID,
    UP_CMD_PUT_KFS_TOP,
    UP_CMD_LIFT,
} ChassisFlagToUp_t;

static volatile uint8_t g_chassis_flag = 0;

static void Upper_ProcessChassisFlag(const FDCAN_RxHeaderTypeDef *rx_header,
                                     const uint8_t *rx_data)
{
    if (rx_header == NULL || rx_data == NULL) {
        return;
    }

    if (rx_header->IdType != FDCAN_STANDARD_ID) {
        return;
    }

    if (rx_header->Identifier < CHASSIS_FLAG_ID_MIN ||
        rx_header->Identifier > CHASSIS_FLAG_ID_MAX) {
        return;
    }

    if (rx_header->DataLength != FDCAN_DLC_BYTES_1) {
        return;
    }

    /*
     * 当前底盘 Data[0] 固定为 0x08，动作编号由 CAN ID 表示。
     * 例如 0x309 - 0x300 = 9，即 UP_CMD_ASSEMBLE。
     */
    g_chassis_flag = (uint8_t)(rx_header->Identifier - CHASSIS_FLAG_ID_BASE);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        Upper_ProcessChassisFlag(&rx_header, rx_data);
    }
}
```

## 3. 上层板发送给底盘板

当前底盘工程已经实现接收两类上层报文：

| CAN ID | 来源 | 用途 | 队列 | 解析函数 |
| --- | --- | --- | --- | --- |
| `0x12` | 上层/雷达 | `x/y/z/yaw` 位姿 | `locatorQueue_x_yHandle` | `analysis_locator_X_Y()` |
| `0x100` | 上层/laser1 | laser float 数据 | `locatorQueue_z_rHandle` | `analysis_locator_laser()` |
| `0x101` | 上层/laser2 | laser float 数据 | `locatorQueue_z_rHandle` | `analysis_locator_laser()` |

另外状态机里已有三类上层信号变量：

```c
extern int MC_flag;
extern int MF_flag;
extern int CF_flag;
```

当前工程已经支持通过不同 CAN ID 给这三个变量赋值。底盘在 CAN3 接收中断处理中识别上层信号 ID，并直接更新 `MC_flag/MF_flag/CF_flag`。

### 3.1 上层发送定位报文：`0x12`

| 字段 | 内容 |
| --- | --- |
| CAN ID | `0x12` |
| ID 类型 | 标准帧 |
| DLC | 16 |
| 帧格式 | CAN FD |
| 字节序 | 小端 |

Payload：

| 字节 | 类型 | 含义 | 底盘保存 |
| --- | --- | --- | --- |
| 0..3 | `float` | x | `lcResult.x = x * 1000` |
| 4..7 | `float` | y | `lcResult.y = y * 1000` |
| 8..11 | `float` | z | 当前未保存 |
| 12..15 | `float` | yaw | `lcResult.r = yaw` |

底盘解析代码：

```c
// Driver_Layer/Src/locator_driver.c
static float Locator_ReadFloatLE(const uint8_t *data)
{
    uint32_t raw;
    float value;

    raw = ((uint32_t)data[0]) |
          ((uint32_t)data[1] << 8) |
          ((uint32_t)data[2] << 16) |
          ((uint32_t)data[3] << 24);
    memcpy(&value, &raw, sizeof(value));
    return value;
}

void analysis_locator_X_Y(Locator_Result_t *lcResult,
                          const Locator_Rx_Queue_t *rx_msg_tmp)
{
    if (lcResult == NULL || rx_msg_tmp == NULL) {
        return;
    }

    if (rx_msg_tmp->msg_identifier != 0x12U || rx_msg_tmp->data_len < 16U) {
        return;
    }

    lcResult->x = 1000.0f * Locator_ReadFloatLE(&rx_msg_tmp->rx_data[0]);
    lcResult->y = 1000.0f * Locator_ReadFloatLE(&rx_msg_tmp->rx_data[4]);
    lcResult->r = Locator_ReadFloatLE(&rx_msg_tmp->rx_data[12]);
}
```

上层发送示例：

```c
static void WriteFloatLE(uint8_t *dst, float value)
{
    memcpy(dst, &value, sizeof(value));
}

void Upper_SendPoseToChassis(FDCAN_HandleTypeDef *hfdcan,
                             float x, float y, float z, float yaw)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t data[16] = {0};

    WriteFloatLE(&data[0], x);
    WriteFloatLE(&data[4], y);
    WriteFloatLE(&data[8], z);
    WriteFloatLE(&data[12], yaw);

    tx_header.Identifier = 0x12U;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_16;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}
```

### 3.2 上层发送 laser 报文：`0x100` / `0x101`

| 字段 | 内容 |
| --- | --- |
| CAN ID | `0x100` 或 `0x101` |
| ID 类型 | 标准帧 |
| DLC | 至少 4 |
| 字节序 | 小端 |

Payload：

| 字节 | 类型 | 含义 | 底盘保存 |
| --- | --- | --- | --- |
| 0..3 | `float` | laser current | `0x100 -> lcResult.laser_current_1`，`0x101 -> lcResult.laser_current_2` |

底盘解析代码：

```c
// Driver_Layer/Src/locator_driver.c
void analysis_locator_laser(Locator_Result_t *lcResult,
                            const Locator_Rx_Queue_t *rx_msg_tmp)
{
    float laser_current;

    if (lcResult == NULL || rx_msg_tmp == NULL) {
        return;
    }

    if (rx_msg_tmp->data_len < 4U) {
        return;
    }

    laser_current = Locator_ReadFloatLE(&rx_msg_tmp->rx_data[0]);

    if (rx_msg_tmp->msg_identifier == 0x100U) {
        lcResult->laser_current_1 = laser_current;
    } else if (rx_msg_tmp->msg_identifier == 0x101U) {
        lcResult->laser_current_2 = laser_current;
    }
}
```

### 3.3 上层发送给底盘的状态信号

当前状态机使用：

```c
volatile int MF_flag = 0;
volatile int MC_flag = 0;
volatile int CF_flag = 0;
```

上层使用不同标准帧 ID 表示不同事件。底盘只根据 CAN ID 区分信号，当前不解析 Data。

| CAN ID | 写入变量 | 写入值 | 含义 |
| --- | --- | --- | --- |
| `0x311` | `MC_flag` | 1 | 取端头完成 |
| `0x312` | `MC_flag` | 2 | 可以进入组装动作 |
| `0x313` | `MC_flag` | 3 | 组装完成 |
| `0x314` | `MC_flag` | 4 | R1 已离开一区 |
| `0x321` | `MF_flag` | 1 | 进入梅花林入口完成/允许进入二区流程 |
| `0x322` | `MF_flag` | 2 | 允许进行二区动作判断 |
| `0x323` | `MF_flag` | 3 | KFS 抓取成功 |
| `0x324` | `MF_flag` | 4 | 障碍 KFS 移除完成 |
| `0x325` | `MF_flag` | 5 | 二区出口流程完成 |
| `0x331` | `CF_flag` | 1 | 决策为放顶层 |
| `0x332` | `CF_flag` | 2 | 放中层完成 |
| `0x333` | `CF_flag` | 3 | R1 抬升完成 |
| `0x334` | `CF_flag` | 4 | R1 移动到顶层放置位置 |
| `0x335` | `CF_flag` | 5 | 放顶层完成 |
| `0x336` | `CF_flag` | 6 | 大胜/比赛完成 |

推荐报文格式：

| 字段 | 内容 |
| --- | --- |
| ID 类型 | 标准帧 |
| DLC | 1 |
| Data[0] | 当前底盘不使用，可固定为 `0x00` |

底盘侧当前接收代码：

```c
typedef enum {
    UPPER_CAN_ID_MC_PICK_HEAD_DONE = 0x311U,
    UPPER_CAN_ID_MC_ASSEMBLE_READY = 0x312U,
    UPPER_CAN_ID_MC_ASSEMBLE_DONE = 0x313U,
    UPPER_CAN_ID_MC_R1_LEFT = 0x314U,

    UPPER_CAN_ID_MF_ENTRY_DONE = 0x321U,
    UPPER_CAN_ID_MF_ACTION_READY = 0x322U,
    UPPER_CAN_ID_MF_GRAB_DONE = 0x323U,
    UPPER_CAN_ID_MF_REMOVE_DONE = 0x324U,
    UPPER_CAN_ID_MF_EXIT_DONE = 0x325U,

    UPPER_CAN_ID_CF_PLACE_TOP_DECISION = 0x331U,
    UPPER_CAN_ID_CF_PUT_MID_DONE = 0x332U,
    UPPER_CAN_ID_CF_LIFT_DONE = 0x333U,
    UPPER_CAN_ID_CF_R1_IN_POSITION = 0x334U,
    UPPER_CAN_ID_CF_PUT_TOP_DONE = 0x335U,
    UPPER_CAN_ID_CF_WIN = 0x336U,
} Upper_To_Chassis_CanId_e;

static bool Is_Upper_Signal_Message(FDCAN_HandleTypeDef *hfdcan,
                                    const FDCAN_RxHeaderTypeDef *rx_header)
{
    if (hfdcan != &hfdcan3 || rx_header == NULL || rx_header->IdType != FDCAN_STANDARD_ID) {
        return false;
    }

    return ((rx_header->Identifier >= UPPER_CAN_ID_MC_PICK_HEAD_DONE &&
             rx_header->Identifier <= UPPER_CAN_ID_MC_R1_LEFT) ||
            (rx_header->Identifier >= UPPER_CAN_ID_MF_ENTRY_DONE &&
             rx_header->Identifier <= UPPER_CAN_ID_MF_EXIT_DONE) ||
            (rx_header->Identifier >= UPPER_CAN_ID_CF_PLACE_TOP_DECISION &&
             rx_header->Identifier <= UPPER_CAN_ID_CF_WIN));
}

static void Process_Upper_Signal_Message(uint32_t id)
{
    switch (id) {
        case UPPER_CAN_ID_MC_PICK_HEAD_DONE:
        case UPPER_CAN_ID_MC_ASSEMBLE_READY:
        case UPPER_CAN_ID_MC_ASSEMBLE_DONE:
        case UPPER_CAN_ID_MC_R1_LEFT:
            MC_flag = (int)(id - 0x310U);
            break;

        case UPPER_CAN_ID_MF_ENTRY_DONE:
        case UPPER_CAN_ID_MF_ACTION_READY:
        case UPPER_CAN_ID_MF_GRAB_DONE:
        case UPPER_CAN_ID_MF_REMOVE_DONE:
        case UPPER_CAN_ID_MF_EXIT_DONE:
            MF_flag = (int)(id - 0x320U);
            break;

        case UPPER_CAN_ID_CF_PLACE_TOP_DECISION:
        case UPPER_CAN_ID_CF_PUT_MID_DONE:
        case UPPER_CAN_ID_CF_LIFT_DONE:
        case UPPER_CAN_ID_CF_R1_IN_POSITION:
        case UPPER_CAN_ID_CF_PUT_TOP_DONE:
        case UPPER_CAN_ID_CF_WIN:
            CF_flag = (int)(id - 0x330U);
            break;

        default:
            break;
    }
}
```

该逻辑位于 `BSP/Src/bsp_can.c` 的 `Process_Rx_Message()` 中，位置在定位报文判断之后、普通 motor/chassis 队列分流之前：

```c
while (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &rx_header, rx_data) == HAL_OK) {
    if (Is_Locator_Rx_Message(hfdcan, &rx_header)) {
        /* 当前工程已有：0x12 和 0x100/0x101 入定位/laser队列 */
        ...
        continue;
    }

    if (Is_Upper_Signal_Message(hfdcan, &rx_header)) {
        Process_Upper_Signal_Message(rx_header.Identifier);
        continue;
    }

    /* 当前工程已有：其他报文进入 motor queue 或 chassis queue */
    ...
}
```

上层发送状态信号示例：

```c
static void Upper_SendChassisSignal(FDCAN_HandleTypeDef *hfdcan, uint32_t id)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t data[1] = {0};

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_1;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}

void Upper_SendPickHeadDone(FDCAN_HandleTypeDef *hfdcan)
{
    Upper_SendChassisSignal(hfdcan, UPPER_CAN_ID_MC_PICK_HEAD_DONE);
}

void Upper_SendAssembleDone(FDCAN_HandleTypeDef *hfdcan)
{
    Upper_SendChassisSignal(hfdcan, UPPER_CAN_ID_MC_ASSEMBLE_DONE);
}

void Upper_SendKfsGrabDone(FDCAN_HandleTypeDef *hfdcan)
{
    Upper_SendChassisSignal(hfdcan, UPPER_CAN_ID_MF_GRAB_DONE);
}
```

## 4. 当前底盘工程接收流程

底盘 CAN3 中断收到报文后，进入：

```c
// BSP/Src/bsp_can.c
static void Process_Rx_Message(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
```

定位和 laser 报文判断：

```c
static bool Is_Locator_Rx_Message(FDCAN_HandleTypeDef *hfdcan,
                                  const FDCAN_RxHeaderTypeDef *rx_header)
{
    if (hfdcan != &hfdcan3 || rx_header == NULL || rx_header->IdType != FDCAN_STANDARD_ID) {
        return false;
    }

    return (rx_header->Identifier == 0x12U ||
            rx_header->Identifier == 0x100U ||
            rx_header->Identifier == 0x101U);
}
```

中断里只保存原始数据并入队，不直接解析 float：

```c
Locator_Rx_Queue_t locator_msg;

locator_msg.msg_identifier = rx_header.Identifier;
locator_msg.data_len = FDCAN_DlcToBytes(rx_header.DataLength);
memcpy(locator_msg.rx_data, rx_data, locator_msg.data_len);

if (rx_header.Identifier == 0x12U) {
    xQueueSendFromISR(locatorQueue_x_yHandle, &locator_msg, &xHigherPriorityTaskWoken);
} else if (rx_header.Identifier == 0x100U ||
           rx_header.Identifier == 0x101U) {
    xQueueSendFromISR(locatorQueue_z_rHandle, &locator_msg, &xHigherPriorityTaskWoken);
}
```

任务中解析：

```c
// Task/Src/Task_locator_recv.c
for (;;) {
    while (xQueueReceive((QueueHandle_t)locatorQueue_x_yHandle, &rx_msg_tmp, 0) == pdPASS) {
        analysis_locator_X_Y(&lcResult, &rx_msg_tmp);
    }

    while (xQueueReceive((QueueHandle_t)locatorQueue_z_rHandle, &rx_msg_tmp, 0) == pdPASS) {
        analysis_locator_laser(&lcResult, &rx_msg_tmp);
    }

    osDelay(20);
}
```

## 5. 调试对照

若上层发送：

```text
ID   = 0x12
Data = 74 AF FB 3F EF 50 64 3E AC 7B 84 BF 31 64 B1 BA
```

小端 float32 解析为：

```text
x   = 1.966291904
y   = 0.222964987
z   = -1.035024166
yaw = -0.001353389
```

底盘当前保存：

```text
lcResult.x = 1966.292
lcResult.y = 222.965
lcResult.r = -0.001353
```

因为当前底盘代码会对 `x/y` 乘以 `1000`。
