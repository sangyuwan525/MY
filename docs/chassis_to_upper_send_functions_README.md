# 底盘代码发送给上层板相关函数整理

本文档只整理当前底盘工程中“底盘板发送给上层板”的相关函数、调用位置和报文格式。

## 1. 总体发送链路

当前业务层给上层发送动作指令时，主链路是：

```text
状态机业务代码
    -> send_flag_to_up(flag_id)
        -> fdcanx_send_ex_data(&hfdcan3, 0x300 + flag_id, data, 1, CAN_ID_STD)
            -> fdcanx_send_impl(...)
                -> HAL_FDCAN_AddMessageToTxFifoQ(...)
                    -> FDCAN3 发出 CAN 报文
```

也就是说，上层动作指令统一从 `send_flag_to_up()` 发出，底层最终通过 FDCAN3 发送。

## 2. 上层板 CAN/FDCAN 配置建议

底盘板使用 CAN3/FDCAN3 与上层板通信。虽然底盘发给上层的动作 flag 是 1 字节 Classic CAN 报文，但同一条 CAN3 总线上还会有上层发给底盘的 `0x12` 定位 CAN FD 报文，所以建议上层板也按 FDCAN 来配置。

### 2.1 推荐基础配置

| 配置项 | 建议值 | 说明 |
| --- | --- | --- |
| CAN 外设 | 上层板接到底盘 CAN3 的那个 CAN/FDCAN 口 | 物理接线与底盘 CAN3 对应 |
| 工作模式 | Normal mode | 正常通信 |
| FrameFormat | `FDCAN_FRAME_FD_BRS` | 同时兼容 Classic CAN 与 CAN FD |
| ID 类型 | Standard ID | 底盘发出的 flag 使用标准 ID |
| 接收 FIFO | FIFO0 或 FIFO1 均可 | 上层代码保持一致即可 |
| AutoRetransmission | 建议 ENABLE，若需与底盘完全一致可 DISABLE | 底盘当前为 DISABLE |
| ProtocolException | 建议 DISABLE | 与底盘当前配置一致 |
| 过滤器 | 接收 `0x301..0x30C` | 底盘给上层的动作 flag 范围 |

底盘 FDCAN3 当前配置在 `Core/Src/fdcan.c`：

```c
hfdcan3.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
hfdcan3.Init.AutoRetransmission = DISABLE;
hfdcan3.Init.ProtocolException = DISABLE;
hfdcan3.Init.NominalPrescaler = 10;
hfdcan3.Init.NominalSyncJumpWidth = 1;
hfdcan3.Init.NominalTimeSeg1 = 12;
hfdcan3.Init.NominalTimeSeg2 = 4;
hfdcan3.Init.DataPrescaler = 5;
hfdcan3.Init.DataSyncJumpWidth = 1;
hfdcan3.Init.DataTimeSeg1 = 12;
hfdcan3.Init.DataTimeSeg2 = 4;
```

如果上层板也是 STM32G4，并且 FDCAN kernel clock 与底盘一致为 170 MHz，可以直接使用同一组时序：

| 段 | Prescaler | SyncJumpWidth | TimeSeg1 | TimeSeg2 | 约等效速率 |
| --- | --- | --- | --- | --- | --- |
| Nominal | 10 | 1 | 12 | 4 | 1 Mbps |
| Data | 5 | 1 | 12 | 4 | 2 Mbps |

计算方式：

```text
bitrate = FDCAN kernel clock / Prescaler / (1 + TimeSeg1 + TimeSeg2)
```

若上层板 FDCAN 时钟不是 170 MHz，不要照抄 Prescaler；应保持目标速率一致：

```text
Nominal bitrate: 1 Mbps
Data bitrate:    2 Mbps
Sample point:    约 76.5%
```

### 2.2 接收底盘 flag 的过滤器建议

底盘发给上层的动作指令 ID 范围是：

```text
0x301 ~ 0x30C
```

上层可以配置一个标准 ID mask/range filter，只接收这段 ID。若调试阶段想省事，也可以先全接收标准帧，再在软件中判断 ID。

推荐软件判断条件：

```c
if (rx_header.IdType == FDCAN_STANDARD_ID &&
    rx_header.Identifier >= 0x301U &&
    rx_header.Identifier <= 0x30CU) {
    flag_id = rx_header.Identifier - 0x300U;
}
```

### 2.3 上层接收底盘 flag 时需要注意

底盘发给上层的 flag 报文当前是：

```text
Classic CAN
Standard ID
DLC = 1
BRS = OFF
Data[0] = 0x08
```

因此上层解析动作编号时应看 CAN ID，不要看 `Data[0]`：

```text
flag_id = CAN_ID - 0x300
```

例如：

```text
CAN ID = 0x309 -> flag_id = 9 -> FLAG_ASSEMBLE
```

### 2.4 硬件与调试建议

| 项目 | 建议 |
| --- | --- |
| 终端电阻 | CAN 总线两端各 120 Ohm |
| 共地 | 上层板与底盘板需要共地 |
| 收发器 | 确认 CANH/CANL 没有接反 |
| 波特率 | 先确认 nominal 1 Mbps 能收到 `0x309` 等 Classic 报文 |
| CAN FD | 再确认能发送/接收 `0x12` DLC 16 的 FD 报文 |
| 调试软件 | 用 Cangaroo/PCAN 等抓包时，确认显示的通道和 CAN3 物理总线一致 |

若上层只测试接收底盘 `send_flag_to_up()`，可以先不用发送 CAN FD，只要能收到 `0x301..0x30C` 的 Classic CAN 报文即可。

## 3. 业务层发送入口：`send_flag_to_up`

代码位置：

```c
// chassis_control/Src/Hfsm.c
uint8_t send_flag_to_up(uint8_t id)
{
    uint8_t data[1] = {8};
    return fdcanx_send_ex_data(&hfdcan3, 0x300 + id, data, 1, CAN_ID_STD);
}
```

函数声明：

```c
// chassis_control/Inc/Hfsm.h
uint8_t send_flag_to_up(uint8_t id);
```

### 2.1 函数含义

| 参数/返回值 | 含义 |
| --- | --- |
| `id` | 动作编号，也就是 `FLAG_TO_UP` 枚举值 |
| 返回 `0` | 发送成功 |
| 返回 `1` | 发送失败 |

### 2.2 实际 CAN 报文格式

| 字段 | 当前代码值 |
| --- | --- |
| CAN 外设 | `hfdcan3` |
| CAN ID | `0x300 + id` |
| ID 类型 | 标准帧，`CAN_ID_STD` |
| DLC | 1 |
| Data[0] | 固定 `0x08` |
| FD/Classic | 因为长度为 1，底层发送为 Classic CAN |

注意：动作编号不放在 `Data[0]`，而是编码在 CAN ID 里。  
例如：

```c
send_flag_to_up(FLAG_ASSEMBLE);
```

实际发出的报文是：

```text
CAN ID = 0x300 + FLAG_ASSEMBLE = 0x309
DLC    = 1
Data   = 08
```

## 4. 底盘发送给上层的动作编号

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

| 枚举名 | id | CAN ID | 发送含义 |
| --- | --- | --- | --- |
| `FLAG_GRAB_KFS_FRONT_HIGH_KEEP` | 1 | `0x301` | 二号区抓前方高块并保留 |
| `FLAG_GRAB_KFS_FRONT_LOW_KEEP` | 2 | `0x302` | 二号区抓前方低块并保留 |
| `FLAG_GRAB_KFS_LEFT_HIGH_KEEP` | 3 | `0x303` | 二号区抓左侧高块并保留 |
| `FLAG_GRAB_KFS_LEFT_LOW_KEEP` | 4 | `0x304` | 二号区抓左侧低块并保留 |
| `FLAG_GRAB_KFS_RIGHT_HIGH_KEEP` | 5 | `0x305` | 二号区抓右侧高块并保留 |
| `FLAG_GRAB_KFS_RIGHT_LOW_KEEP` | 6 | `0x306` | 二号区抓右侧低块并保留 |
| `FLAG_GRAB_KFS_FRONT_HIGH_REMOVE` | 7 | `0x307` | 二号区抓前方高块并移走 |
| `FLAG_GRAB_KFS_FRONT_LOW_REMOVE` | 8 | `0x308` | 二号区抓前方低块并移走 |
| `FLAG_ASSEMBLE` | 9 | `0x309` | 一区组装武器 |
| `FLAG_PUT_KFS_MID` | 10 | `0x30A` | 三区放置 KFS 到中层 |
| `FLAG_PUT_KFS_TOP` | 11 | `0x30B` | 三区放置 KFS 到顶层 |
| `FLAG_LIFT` | 12 | `0x30C` | 三区请求 R1 抬升 |

## 5. 当前所有 `send_flag_to_up()` 调用位置

### 4.1 一区组装武器

代码位置：

```c
// chassis_control/Src/Hfsm.c
case MC_ASSEMBLE_ACT:
    send_flag_to_up(FLAG_ASSEMBLE);
```

发送报文：

```text
CAN ID = 0x309
Data   = 08
```

用途：底盘进入组装动作阶段，通知上层执行组装武器相关动作。

### 4.2 二号区抓取并保留 KFS

代码位置：

```c
// chassis_control/Src/Hfsm.c
case MF_PICK_ADJACENT:
    send_flag_to_up(FLAG_GRAB_KFS_FRONT_HIGH_KEEP);
    send_flag_to_up(FLAG_GRAB_KFS_FRONT_LOW_KEEP);
    send_flag_to_up(FLAG_GRAB_KFS_LEFT_HIGH_KEEP);
    send_flag_to_up(FLAG_GRAB_KFS_LEFT_LOW_KEEP);
    send_flag_to_up(FLAG_GRAB_KFS_RIGHT_HIGH_KEEP);
    send_flag_to_up(FLAG_GRAB_KFS_RIGHT_LOW_KEEP);
```

实际发送哪个 flag，由当前方块和目标方块的相对方向、高度差决定。

| 条件 | 发送 flag | CAN ID |
| --- | --- | --- |
| 目标在前方，目标更高 | `FLAG_GRAB_KFS_FRONT_HIGH_KEEP` | `0x301` |
| 目标在前方，目标更低 | `FLAG_GRAB_KFS_FRONT_LOW_KEEP` | `0x302` |
| 目标在左侧，目标更高 | `FLAG_GRAB_KFS_LEFT_HIGH_KEEP` | `0x303` |
| 目标在左侧，目标更低 | `FLAG_GRAB_KFS_LEFT_LOW_KEEP` | `0x304` |
| 目标在右侧，目标更高 | `FLAG_GRAB_KFS_RIGHT_HIGH_KEEP` | `0x305` |
| 目标在右侧，目标更低 | `FLAG_GRAB_KFS_RIGHT_LOW_KEEP` | `0x306` |

### 4.3 二号区移除障碍 KFS

代码位置：

```c
// chassis_control/Src/Hfsm.c
case MF_REMOVE_KFS:
    send_flag_to_up(FLAG_GRAB_KFS_FRONT_HIGH_REMOVE);
    send_flag_to_up(FLAG_GRAB_KFS_FRONT_LOW_REMOVE);
```

| 条件 | 发送 flag | CAN ID |
| --- | --- | --- |
| 目标方块更高 | `FLAG_GRAB_KFS_FRONT_HIGH_REMOVE` | `0x307` |
| 目标方块更低或不更高 | `FLAG_GRAB_KFS_FRONT_LOW_REMOVE` | `0x308` |

### 4.4 三区放置 KFS 到中层

代码位置：

```c
// chassis_control/Src/Hfsm.c
case CF_PLACE_MID:
    send_flag_to_up(FLAG_PUT_KFS_MID);
```

发送报文：

```text
CAN ID = 0x30A
Data   = 08
```

### 4.5 三区请求 R1 抬升

代码位置：

```c
// chassis_control/Src/Hfsm.c
case CF_WAIT_LIFT:
    send_flag_to_up(FLAG_LIFT);
```

发送报文：

```text
CAN ID = 0x30C
Data   = 08
```

### 4.6 三区放置 KFS 到顶层

代码位置：

```c
// chassis_control/Src/Hfsm.c
case CF_PLACE_TOP:
    send_flag_to_up(FLAG_PUT_KFS_TOP);
```

发送报文：

```text
CAN ID = 0x30B
Data   = 08
```

## 6. 底层发送函数：`fdcanx_send_ex_data`

代码位置：

```c
// BSP/Src/bsp_can.c
uint8_t fdcanx_send_ex_data(hcan_t *hfdcan,
                            uint32_t id,
                            uint8_t *data,
                            uint32_t len,
                            CAN_Id_Type_e id_type)
{
    return fdcanx_send_impl(hfdcan, id, data, len, id_type);
}
```

函数声明：

```c
// BSP/Inc/bsp_can.h
uint8_t fdcanx_send_ex_data(hcan_t *hfdcan,
                            uint32_t id,
                            uint8_t *data,
                            uint32_t len,
                            CAN_Id_Type_e id_type);
```

### 5.1 函数作用

这是当前 `send_flag_to_up()` 直接调用的底层 CAN 发送接口。

| 参数 | 含义 |
| --- | --- |
| `hfdcan` | 要使用的 FDCAN 外设，例如 `&hfdcan3` |
| `id` | CAN ID |
| `data` | 数据指针 |
| `len` | 数据长度，单位 byte |
| `id_type` | `CAN_ID_STD` 或 `CAN_ID_EXT` |

## 7. 核心发送实现：`fdcanx_send_impl`

代码位置：

```c
// BSP/Src/bsp_can.c
static uint8_t fdcanx_send_impl(hcan_t *hfdcan,
                                uint32_t id,
                                uint8_t *data,
                                uint32_t len,
                                CAN_Id_Type_e id_type)
```

这是实际组包和调用 HAL 发送的函数。

### 6.1 关键逻辑

```c
uint32_t dlc = FDCAN_BytesToDlc(len);
```

把 byte 长度转换成 FDCAN DLC。

```c
tx_header.Identifier = id;
tx_header.IdType = (id_type == CAN_ID_EXT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
tx_header.TxFrameType = FDCAN_DATA_FRAME;
tx_header.DataLength = dlc;
tx_header.BitRateSwitch = FDCAN_BRS_OFF;
tx_header.FDFormat = (len <= 8U) ? FDCAN_CLASSIC_CAN : FDCAN_FD_CAN;
```

当前给上层发 flag 时 `len = 1`，所以：

```text
FDFormat = FDCAN_CLASSIC_CAN
BitRateSwitch = FDCAN_BRS_OFF
```

```c
if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data) == HAL_OK) {
    ret = 0U;
}
```

最终调用 HAL 把报文放进发送 FIFO。

### 6.2 发送保护

`fdcanx_send_impl()` 发送前会：

1. 检查参数是否为空。
2. 检查长度能否转换成合法 DLC。
3. 调用 `FDCAN_LockTx()` 获取对应 CAN 口发送互斥锁。
4. 调用 `FDCAN_WaitTxFifoFree()` 等待硬件 TX FIFO 有空位。
5. 调用 `HAL_FDCAN_AddMessageToTxFifoQ()`。
6. 调用 `FDCAN_UnlockTx()` 释放互斥锁。

## 8. 发送相关辅助函数

这些函数也在 `BSP/Src/bsp_can.c` 中，属于发送链路辅助函数。

### 7.1 `FDCAN_BytesToDlc`

```c
static uint32_t FDCAN_BytesToDlc(uint32_t len)
```

作用：把数据长度 byte 转换为 HAL FDCAN DLC 宏。

当前支持：

```text
0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
```

如果长度不合法，返回 `0xFFFFFFFFU`，发送函数会直接失败。

### 7.2 `FDCAN_GetTxMutex`

```c
static osMutexId_t FDCAN_GetTxMutex(FDCAN_HandleTypeDef *hfdcan)
```

作用：根据 `hfdcan1 / hfdcan2 / hfdcan3` 返回对应发送互斥锁。

### 7.3 `FDCAN_LockTx`

```c
static uint8_t FDCAN_LockTx(FDCAN_HandleTypeDef *hfdcan)
```

作用：如果 FreeRTOS 已经运行，则加锁对应 CAN 口，避免多个任务同时写同一个 TX FIFO。

### 7.4 `FDCAN_UnlockTx`

```c
static void FDCAN_UnlockTx(FDCAN_HandleTypeDef *hfdcan)
```

作用：发送结束后释放对应 CAN 口互斥锁。

### 7.5 `FDCAN_WaitTxFifoFree`

```c
static uint8_t FDCAN_WaitTxFifoFree(FDCAN_HandleTypeDef *hfdcan)
```

作用：等待硬件 TX FIFO 有空位。超时时间：

```c
#define FDCAN_TX_FIFO_WAIT_TIMEOUT_MS 5U
```

## 9. CAN 初始化中和发送相关的部分

代码位置：

```c
// BSP/Src/bsp_can.c
void bsp_can_init(osMessageQueueId_t motor_q, osMessageQueueId_t chassis_q)
{
    g_motor_queue = motor_q;
    g_chassis_queue = chassis_q;
    g_fdcan1_tx_mutex = osMutexNew(NULL);
    g_fdcan2_tx_mutex = osMutexNew(NULL);
    g_fdcan3_tx_mutex = osMutexNew(NULL);

    bsp_can_start(&hfdcan1);
    bsp_can_start(&hfdcan2);
    bsp_can_start(&hfdcan3);
}
```

和发送有关的是这三个互斥锁：

```c
g_fdcan1_tx_mutex
g_fdcan2_tx_mutex
g_fdcan3_tx_mutex
```

`send_flag_to_up()` 使用 `hfdcan3`，所以最终会使用 `g_fdcan3_tx_mutex`。

## 10. 其他 CAN 发送函数说明

当前工程里还有几个 CAN 发送函数，但它们不是 `send_flag_to_up()` 这条上层动作指令链路的主入口。

### 9.1 `fdcanx_send_data`

```c
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    return fdcanx_send_impl(hfdcan, id, data, len, CAN_ID_STD);
}
```

作用：发送标准帧，内部同样走 `fdcanx_send_impl()`。

当前主要被电机驱动使用，例如达妙电机发送。

### 9.2 `bsp_can_send_std_msg`

```c
uint8_t bsp_can_send_std_msg(FDCAN_HandleTypeDef *hfdcan,
                             uint32_t id,
                             uint8_t *Txdata,
                             uint8_t len,
                             CAN_Id_Type_e id_type)
```

作用：直接组 Classic CAN 报文并调用 `HAL_FDCAN_AddMessageToTxFifoQ()`。

注意：这个函数当前没有走 `FDCAN_LockTx()` 互斥锁，也没有用 `FDCAN_BytesToDlc()` 转换 DLC。当前 `send_flag_to_up()` 不使用它。

### 9.3 `bsp_can_rev_fd_msg`

```c
uint8_t bsp_can_rev_fd_msg(FDCAN_HandleTypeDef *hfdcan,
                           uint32_t id,
                           uint8_t *data,
                           uint8_t len,
                           CAN_Id_Type_e id_type)
{
    return bsp_can_send_std_msg(hfdcan, id, data, len, id_type);
}
```

作用：名字里有 `rev_fd`，但当前实现只是调用 `bsp_can_send_std_msg()` 发送 Classic CAN。

当前 `send_flag_to_up()` 不使用它。

## 11. 上层板接收底盘动作指令的最小逻辑

上层板可以这样解析底盘发来的动作指令：

```c
#define CHASSIS_TO_UP_BASE_ID 0x300U
#define CHASSIS_TO_UP_MIN_ID  0x301U
#define CHASSIS_TO_UP_MAX_ID  0x30CU

static uint8_t ParseChassisFlag(const FDCAN_RxHeaderTypeDef *rx_header,
                                const uint8_t *rx_data)
{
    if (rx_header == NULL || rx_data == NULL) {
        return 0U;
    }

    if (rx_header->IdType != FDCAN_STANDARD_ID) {
        return 0U;
    }

    if (rx_header->Identifier < CHASSIS_TO_UP_MIN_ID ||
        rx_header->Identifier > CHASSIS_TO_UP_MAX_ID) {
        return 0U;
    }

    if (rx_header->DataLength != FDCAN_DLC_BYTES_1) {
        return 0U;
    }

    /*
     * 当前底盘 Data[0] 固定为 0x08。
     * 动作编号 = CAN ID - 0x300。
     */
    (void)rx_data;
    return (uint8_t)(rx_header->Identifier - CHASSIS_TO_UP_BASE_ID);
}
```

例如上层收到：

```text
ID   = 0x309
DLC  = 1
Data = 08
```

则：

```text
flag_id = 0x309 - 0x300 = 9
含义    = FLAG_ASSEMBLE，底盘请求组装武器
```
