# 爬升楼梯流程说明

本文档对应当前代码中的上楼梯流程，主要涉及：

- `chassis_control/Src/ClimbStairs.c`
- `chassis_control/Src/lift_walk_controller.c`
- `chassis_control/Inc/lift_walk_controller.h`

当前新版上楼动作的核心入口是：

```c
int ClimbStairs(int curr_id, int stair_id)
```

其中 `ClimbStairs()` 是上楼梯状态机，`ClimbLift_*()` 是对底层 `LiftWalk` 控制器的封装，真正计算和下发抬升机构、前后轮速度的是 `LiftWalk_Update()` / `LiftWalk_RunLiftAction()`。

## 1. 关键参数

这些参数定义在 `ClimbStairs.c` 顶部：

```c
#define CLIMB_LIFT_TARGET_HEIGHT_MM 198.0f
#define CLIMB_LIFT_APPROACH_HEIGHT_MM 0.0f
#define CLIMB_LIFT_FORWARD_MM_S 200.0f
#define CLIMB_LIFT_START_HEIGHT_MM 0.0f
#define CLIMB_LIFT_DONE_TOLERANCE_MM 2.0f
#define CLIMB_LIFT_DT_S 0.01f
```

含义如下：

| 参数 | 含义 |
| --- | --- |
| `CLIMB_LIFT_TARGET_HEIGHT_MM` | 正式爬升时的目标抬升高度，目前是 `198mm`。 |
| `CLIMB_LIFT_APPROACH_HEIGHT_MM` | 接近台阶阶段的目标高度，目前是 `0mm`，表示低位靠近。 |
| `CLIMB_LIFT_FORWARD_MM_S` | 爬楼过程中的前进速度，目前是 `200mm/s`。 |
| `CLIMB_LIFT_START_HEIGHT_MM` | 抬升动作开始时认为的初始高度，目前是 `0mm`。 |
| `CLIMB_LIFT_DONE_TOLERANCE_MM` | 抬升完成判定容差，目前是 `2mm`。 |
| `CLIMB_LIFT_DT_S` | 控制周期，目前是 `0.01s`，也就是 `10ms`。 |

底层 `LiftWalk_DefaultConfig()` 里还有抬升轨迹限制：

```c
cfg->min_height_mm = 0.0f;
cfg->max_height_mm = 198.0f;
cfg->lift_vmax_mm_s = 10.0f;
cfg->lift_amax_mm_s2 = 20.0f;
```

也就是说，高度目标会被限制在 `0 ~ 198mm`，并且高度参考值会按最大速度 `10mm/s`、最大加速度 `20mm/s^2` 平滑变化，不会瞬间跳到目标高度。

## 2. 总体调用关系

当前上楼动作大致是这条链路：

```text
上层任务循环
  -> ClimbStairs(curr_id, stair_id)
      -> ClimbLift_HoldApproachHeight()
      -> ClimbLift_ApproachForward()
      -> ClimbLift_RunStep()
      -> ClimbLift_HoldHeight()
      -> ClimbLift_DriveForward()
      -> ClimbLift_Reset()

ClimbLift_*()
  -> LiftWalk_Update()
  -> LiftWalk_RunLiftAction()

LiftWalk_Update()
  -> 生成平滑高度轨迹
  -> 计算四个支撑点高度
  -> 换算小米滑轨目标位置
  -> 换算宇树小臂目标角度
  -> 计算前后轮速度
  -> 下发电机命令
```

`ClimbStairs()` 每次被周期调用时只推进状态机一步，返回值含义是：

```c
0: 当前上楼流程还没结束
1: 当前上楼流程完成
```

## 3. ClimbStairs() 状态机流程

### 3.1 CLIMB_IDLE

首次进入 `ClimbStairs()` 时，如果当前状态是 `CLIMB_IDLE`：

```c
ClimbLift_Reset();
current_climb_state = CLIMB_STEP1_FRONT_UP;
return 0;
```

作用：

- 停止并清空 `LiftWalk` 控制器状态。
- 把上楼状态机切到 `CLIMB_STEP1_FRONT_UP`。
- 本周期直接返回，下一周期继续执行。

### 3.2 CLIMB_STEP1_FRONT_UP

代码：

```c
ClimbLift_HoldApproachHeight();
current_climb_state = CLIMB_STEP2_BASE_FORWARD;
```

作用：

- 初始化 `LiftWalk`。
- 让机构保持在接近高度 `CLIMB_LIFT_APPROACH_HEIGHT_MM`，当前为 `0mm`。
- 不前进。
- 直接进入下一阶段。

注意：枚举名里有 `FRONT_UP`，但当前新版代码并不是旧 3508 单独前侧抬升逻辑，而是用 `LiftWalk` 保持接近高度。

### 3.3 CLIMB_STEP2_BASE_FORWARD

代码逻辑：

```c
if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
    ClimbLift_HoldApproachHeight();
    current_climb_state = CLIMB_STEP3_LIFT_UP;
} else {
    ClimbLift_ApproachForward();
}
```

作用：

- 在低位 `0mm` 状态下向前靠近台阶。
- `GPIOB PIN11` 是碰撞开关。
- 如果碰撞开关触发，说明已经接触台阶，进入正式抬升阶段。
- 如果未触发，就继续低位前进。

这一阶段调用：

```c
ClimbLift_ApproachForward()
```

它等价于：

```c
target_height_mm = 0.0f;
vy_mm_s = 200.0f;
```

也就是“低位向前走”。

### 3.4 CLIMB_STEP3_LIFT_UP

代码：

```c
if (ClimbLift_RunStep() != 0U) {
    current_climb_state = CLIMB_STEP4_REAR_FORWARD;
}
```

作用：

- 执行正式抬升动作。
- 目标高度是 `198mm`。
- 同时给前进速度 `200mm/s`。
- 当 `ClimbLift_RunStep()` 返回 `1`，说明高度已经到达目标并且速度基本降到 0，然后进入下一阶段。

这是当前爬楼流程里最核心的抬升阶段。

### 3.5 CLIMB_STEP4_REAR_FORWARD

代码逻辑：

```c
if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)) {
    ClimbLift_HoldHeight();
    current_climb_state = CLIMB_STEP5_RESET_ALL;
} else {
    ClimbLift_DriveForward();
}
```

作用：

- 保持 `198mm` 高度继续向前走。
- `GPIOB PIN10` 是后光电开关。
- 如果后光电检测到位，就停止前进，只保持高度，进入下一阶段。
- 如果还没到位，就继续高位前进。

这一阶段调用：

```c
ClimbLift_DriveForward()
```

它等价于：

```c
target_height_mm = 198.0f;
vy_mm_s = 200.0f;
```

也就是“高位向前走”。

### 3.6 CLIMB_STEP5_RESET_ALL

代码逻辑：

```c
if (is_on_stair_center(stair_id)) {
    ClimbLift_HoldHeight();
    current_climb_state = CLIMB_COMPLETE;
} else {
    ClimbLift_DriveForward();
}
```

作用：

- 继续判断机器人是否到达目标台阶中心。
- 如果到达台阶中心，保持 `198mm`，进入完成状态。
- 如果还没到台阶中心，继续保持高位向前走。

`is_on_stair_center(stair_id)` 的判断条件是：

```c
fabsf(lcResult.x - stairs_center[stair_id].x) < 20
fabsf(lcResult.y - stairs_center[stair_id].y) < 20
```

即当前位置和目标台阶中心点的 `x/y` 偏差都小于 `20mm`。

### 3.7 CLIMB_COMPLETE

代码：

```c
ClimbLift_Reset();
current_climb_state = CLIMB_IDLE;
climb_cnt = 0;
return 1;
```

作用：

- 停止 `LiftWalk` 控制器。
- 状态机回到空闲。
- 返回 `1`，通知上层本次上楼动作完成。

## 4. ClimbLift_* 函数作用

这些函数都在 `ClimbStairs.c` 中，是对底层 `LiftWalk` 的简单封装。

### 4.1 ClimbLift_InitOnce()

作用：

- 只初始化一次 `LiftWalk` 控制器。
- 调用 `LiftWalk_DefaultConfig()` 加载默认配置。
- 读取当前小米滑轨和宇树小臂的电机角度，作为零点：

```c
cfg.slider_zero_rad[LEFT/RIGHT]
cfg.arm_motor_zero_rad[LEFT/RIGHT]
```

- 当前代码里把姿态闭环 PID 全部清零：

```c
cfg.roll_kp = 0.0f;
cfg.pitch_kp = 0.0f;
cfg.yaw_kp = 0.0f;
```

所以当前抬升时默认是“四角等高抬升”，不会根据 IMU 自动修正 roll/pitch/yaw。

### 4.2 ClimbLift_Reset()

作用：

- 如果已经初始化过，就调用：

```c
LiftWalk_Stop(&climb_lift_ctrl);
```

- 停止前后轮速度输出。
- 清除 `climb_lift_inited` 标志。
- 下次再调用 `ClimbLift_InitOnce()` 时会重新取当前位置作为零点。

调试时要注意：如果机构已经不在机械零位，调用 `ClimbLift_Reset()` 后再初始化，就会把当前位置当成新的零点。

### 4.3 ClimbLift_UpdateTarget(target_height_mm, vy_mm_s)

作用：

- 设置本周期的目标高度和前进速度。
- 设置姿态输入为 0。
- 设置控制周期 `dt_s = 0.01s`。
- 开启电机输出：

```c
climb_lift_in.enable_motor_output = 1U;
```

- 最后调用：

```c
LiftWalk_Update(&climb_lift_ctrl, &climb_lift_in);
```

这个函数不会判断动作是否完成，只是持续按照目标高度和速度更新控制输出。

### 4.4 ClimbLift_HoldHeight()

代码：

```c
ClimbLift_UpdateTarget(CLIMB_LIFT_TARGET_HEIGHT_MM, 0.0f);
```

作用：

- 目标高度 `198mm`。
- 前进速度 `0mm/s`。
- 用于“保持高位不走”。

### 4.5 ClimbLift_DriveForward()

代码：

```c
ClimbLift_UpdateTarget(CLIMB_LIFT_TARGET_HEIGHT_MM, CLIMB_LIFT_FORWARD_MM_S);
```

作用：

- 目标高度 `198mm`。
- 前进速度 `200mm/s`。
- 用于“保持高位继续向前走”。

### 4.6 ClimbLift_ApproachForward()

代码：

```c
ClimbLift_InitOnce();
ClimbLift_UpdateTarget(CLIMB_LIFT_APPROACH_HEIGHT_MM, CLIMB_LIFT_FORWARD_MM_S);
```

作用：

- 目标高度 `0mm`。
- 前进速度 `200mm/s`。
- 用于“低位向前靠近台阶”。

### 4.7 ClimbLift_HoldApproachHeight()

代码：

```c
ClimbLift_InitOnce();
ClimbLift_UpdateTarget(CLIMB_LIFT_APPROACH_HEIGHT_MM, 0.0f);
```

作用：

- 目标高度 `0mm`。
- 前进速度 `0mm/s`。
- 用于“低位保持不动”。

### 4.8 ClimbLift_RunStep()

代码核心：

```c
climb_lift_in.vx_mm_s = 0.0f;
climb_lift_in.vy_mm_s = CLIMB_LIFT_FORWARD_MM_S;
climb_lift_in.target_height_mm = CLIMB_LIFT_TARGET_HEIGHT_MM;
climb_lift_in.dt_s = CLIMB_LIFT_DT_S;
climb_lift_in.enable_motor_output = 1U;

return LiftWalk_RunLiftAction(&climb_lift_ctrl,
                              &climb_lift_in,
                              CLIMB_LIFT_FORWARD_MM_S,
                              CLIMB_LIFT_START_HEIGHT_MM,
                              CLIMB_LIFT_DONE_TOLERANCE_MM);
```

作用：

- 初始化 `LiftWalk`。
- 设置目标高度为 `198mm`。
- 设置前进速度为 `200mm/s`。
- 调用 `LiftWalk_RunLiftAction()` 执行一次“边走边抬升”动作。
- 返回 `0` 表示还在抬升。
- 返回 `1` 表示已经到达目标高度并完成。

注意：当前 `ClimbLift_RunStep()` 不是单纯原地抬升，它会带 `200mm/s` 的前进速度。如果只想单独试底盘抬升，应把传给 `LiftWalk_RunLiftAction()` 的 `vy_mm_s` 改成 `0.0f`，或者单独写一个测试函数。

## 5. LiftWalk_RunLiftAction() 做了什么

`LiftWalk_RunLiftAction()` 是带完成判定的抬升动作函数。

它的参数：

```c
uint8_t LiftWalk_RunLiftAction(LiftWalk_Controller_t *ctrl,
                               const LiftWalk_Input_t *in,
                               float vy_mm_s,
                               float start_height_mm,
                               float done_tolerance_mm);
```

含义：

| 参数 | 含义 |
| --- | --- |
| `ctrl` | `LiftWalk` 控制器对象。 |
| `in` | 本周期输入，包括目标高度、姿态角、控制周期、是否下发电机。 |
| `vy_mm_s` | 本动作强制使用的前进速度。函数内部会覆盖 `in->vy_mm_s`。 |
| `start_height_mm` | 动作第一次启动时的高度参考起点。 |
| `done_tolerance_mm` | 完成判定高度容差。 |

内部流程：

1. 复制输入 `in` 到局部变量 `action_in`。
2. 强制设置：

```c
action_in.vx_mm_s = 0.0f;
action_in.vy_mm_s = vy_mm_s;
```

3. 把目标高度限制在配置允许范围内。
4. 如果目标高度变化超过容差，重新开始一次动作。
5. 如果动作第一次启动，调用：

```c
LiftWalk_Reset(ctrl, start_height_mm);
```

6. 每周期调用：

```c
LiftWalk_Update(ctrl, &action_in);
```

7. 判断完成条件：

```c
abs(height_ref_mm - target) <= tolerance
abs(height_dot_ref_mm_s) <= lift_amax_mm_s2 * dt_s
```

满足后返回 `1`。

## 6. LiftWalk_Update() 做了什么

`LiftWalk_Update()` 是底层每周期控制计算函数。

当前流程：

1. 检查输入和初始化状态。
2. 检查 roll/pitch 是否超过安全限制。
3. 生成平滑高度轨迹。
4. 根据姿态 PID 计算 roll/pitch 修正量。
5. 把姿态修正量分配到四个支撑点高度。
6. 把后侧支撑点高度换算成小米滑轨位置。
7. 把前侧支撑点高度换算成宇树小臂角度和角速度。
8. 根据 `vy_mm_s` 计算前后轮速度。
9. 如果 `enable_motor_output != 0`，下发电机命令。

当前电机输出包括：

| 执行器 | 输出方式 |
| --- | --- |
| 小米滑轨左右 | `set_position(position, vel_limit)` |
| 宇树小臂左右 | `set_mit(position, speed, kp, kd, torque)` |
| 前轮达妙左右 | `set_speed(rad/s 换算后的电机速度)` |
| 后轮 Blazer FOC 左右 | `set_speed(rpm)` |

## 7. 单独调试底盘抬升建议

如果你现在只想尝试“底盘抬升”这一个动作，建议不要直接用当前的 `ClimbLift_RunStep()` 原样测试，因为它会边走边抬升。

可以写一个临时测试函数，把前进速度改成 `0.0f`：

```c
static uint8_t ClimbLift_TestLiftOnly(void)
{
    ClimbLift_InitOnce();

    climb_lift_in.vx_mm_s = 0.0f;
    climb_lift_in.vy_mm_s = 0.0f;
    climb_lift_in.target_height_mm = CLIMB_LIFT_TARGET_HEIGHT_MM;

    climb_lift_in.roll_rad = 0.0f;
    climb_lift_in.pitch_rad = 0.0f;
    climb_lift_in.yaw_rad = 0.0f;
    climb_lift_in.yaw_ref_rad = 0.0f;
    climb_lift_in.roll_rate_rad_s = 0.0f;
    climb_lift_in.pitch_rate_rad_s = 0.0f;
    climb_lift_in.yaw_rate_rad_s = 0.0f;

    climb_lift_in.dt_s = CLIMB_LIFT_DT_S;
    climb_lift_in.enable_motor_output = 1U;

    return LiftWalk_RunLiftAction(&climb_lift_ctrl,
                                  &climb_lift_in,
                                  0.0f,
                                  CLIMB_LIFT_START_HEIGHT_MM,
                                  CLIMB_LIFT_DONE_TOLERANCE_MM);
}
```

调用方式：

- 周期调用这个函数。
- 返回 `0` 表示还在抬升。
- 返回 `1` 表示抬升到目标高度。

如果想先不下发电机，只看计算输出，可以把：

```c
climb_lift_in.enable_motor_output = 0U;
```

然后通过 `LiftWalk_GetOutput(&climb_lift_ctrl)` 观察：

- `height_ref_mm`
- `height_dot_ref_mm_s`
- `support_z_mm[]`
- `slider_motor_rad[]`
- `arm_theta_rad[]`
- `front_wheel_rad_s[]`
- `rear_wheel_rpm[]`
- `status`

## 8. 调试时重点关注

1. `ClimbLift_InitOnce()` 会把当前电机角度作为零点，测试前要确认机构在期望的机械初始位置。
2. 当前 roll/pitch/yaw PID 都被置零，所以不会自动姿态纠偏。
3. `ClimbLift_RunStep()` 当前是边走边抬，不是原地抬。
4. `CLIMB_LIFT_DT_S` 应和实际调用周期一致，否则高度轨迹和完成判定会不准。
5. `LiftWalk_RunLiftAction()` 返回 `1` 的条件不仅是高度接近目标，还要求高度速度已经足够小。
6. 如果 `LiftWalk_Update()` 返回保护状态，`LiftWalk_RunLiftAction()` 会返回 `0` 并清除动作状态，需要检查 `ctrl->out.status`。

## 9. 上楼流程简图

```text
CLIMB_IDLE
  |
  | ClimbLift_Reset()
  v
CLIMB_STEP1_FRONT_UP
  |
  | HoldApproachHeight: 高度 0mm，速度 0
  v
CLIMB_STEP2_BASE_FORWARD
  |
  | 未触发 PB11: ApproachForward，高度 0mm，速度 200mm/s
  | 触发 PB11: 进入抬升
  v
CLIMB_STEP3_LIFT_UP
  |
  | ClimbLift_RunStep: 高度 198mm，速度 200mm/s
  | 返回 1 后进入下一步
  v
CLIMB_STEP4_REAR_FORWARD
  |
  | 未触发 PB10: DriveForward，高度 198mm，速度 200mm/s
  | 触发 PB10: HoldHeight，高度 198mm，速度 0
  v
CLIMB_STEP5_RESET_ALL
  |
  | 未到台阶中心: DriveForward，高度 198mm，速度 200mm/s
  | 到台阶中心: HoldHeight，高度 198mm，速度 0
  v
CLIMB_COMPLETE
  |
  | ClimbLift_Reset()
  v
CLIMB_IDLE
```
