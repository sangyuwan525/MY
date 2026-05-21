# 爬升楼梯流程说明

本文档对应当前新版上楼梯流程，主要涉及：

- `chassis_control/Src/ClimbStairs.c`
- `chassis_control/Inc/ClimbStairs.h`
- `chassis_control/Src/lift_walk_controller.c`
- `chassis_control/Inc/lift_walk_controller.h`

当前上楼动作入口是：

```c
int ClimbStairs(int curr_id, int stair_id)
```

`ClimbStairs()` 是上楼状态机。现在的流程已经改成：

```text
初始时达妙前轮不挨地
  -> 宇树小臂先向前转，让达妙前轮挨地，同时可以向前走
  -> 以“小臂已放地、车身未抬升”的姿态作为 LiftWalk 抬升零点
  -> 边抬升车身边向前走
  -> 到台阶边缘后，宇树小臂向后收，同时继续向前走
  -> 后侧小米滑轨收回
  -> 继续走到台阶中心
```

最关键的一点是：`ClimbLift_InitOnce()` 不在前臂落地前调用。它会在 `ClimbLift_RunStep()` 第一次执行时初始化，因此记录的是“达妙前轮已经挨地”的宇树小臂角度和小米滑轨角度。

## 1. 状态机

`Climb_State_e` 当前定义为：

```c
typedef enum
{
    CLIMB_IDLE = 0,
    CLIMB_STEP1_FRONT_ARM_DEPLOY,
    CLIMB_STEP2_LIFT_AND_FORWARD,
    CLIMB_STEP3_FRONT_ARM_RETRACT,
    CLIMB_STEP4_REAR_SLIDER_RETRACT,
    CLIMB_STEP5_CENTER_FORWARD,
    CLIMB_COMPLETE
} Climb_State_e;
```

各状态含义：

| 状态 | 作用 |
| --- | --- |
| `CLIMB_IDLE` | 空闲/准备开始。清掉上一次 `LiftWalk` 状态，停止轮子，清掉前臂动作记录。 |
| `CLIMB_STEP1_FRONT_ARM_DEPLOY` | 宇树小臂向前转，让达妙前轮落地；底盘用 `cha_remote()` 前进，达妙前轮按同等线速度提前空转。此阶段不初始化 `LiftWalk`。 |
| `CLIMB_STEP2_LIFT_AND_FORWARD` | 调用 `ClimbLift_RunStep()`，此时才初始化 `LiftWalk`，并开始边抬升边前进。 |
| `CLIMB_STEP3_FRONT_ARM_RETRACT` | 到台阶边缘后，保持后侧小米滑轨当前位置，宇树小臂向后收，同时继续前进。 |
| `CLIMB_STEP4_REAR_SLIDER_RETRACT` | 宇树小臂保持收回位置，后侧小米滑轨回到抬升零点。 |
| `CLIMB_STEP5_CENTER_FORWARD` | 小臂已收、后滑轨已收，继续前进到目标台阶中心。 |
| `CLIMB_COMPLETE` | 停止并复位状态机，返回上楼完成。 |

## 2. 关键参数

### 2.1 LiftWalk 抬升参数

```c
#define CLIMB_LIFT_TARGET_HEIGHT_MM 198.0f
#define CLIMB_LIFT_APPROACH_HEIGHT_MM 0.0f
#define CLIMB_LIFT_FORWARD_MM_S 200.0f
#define CLIMB_LIFT_START_HEIGHT_MM 0.0f
#define CLIMB_LIFT_DONE_TOLERANCE_MM 2.0f
#define CLIMB_LIFT_DT_S 0.01f
#define CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S CLIMB_LIFT_FORWARD_MM_S
```

| 参数 | 含义 |
| --- | --- |
| `CLIMB_LIFT_TARGET_HEIGHT_MM` | 抬升目标高度，当前 `198mm`。 |
| `CLIMB_LIFT_APPROACH_HEIGHT_MM` | 旧低位靠近参数，当前新流程基本不再使用。 |
| `CLIMB_LIFT_FORWARD_MM_S` | 上楼流程中的前进速度，当前 `200mm/s`。 |
| `CLIMB_LIFT_START_HEIGHT_MM` | `LiftWalk_RunLiftAction()` 的起始高度，当前 `0mm`。 |
| `CLIMB_LIFT_DONE_TOLERANCE_MM` | 抬升完成高度容差，当前 `2mm`。 |
| `CLIMB_LIFT_DT_S` | 控制周期，当前 `0.01s`。 |
| `CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S` | 前臂放地阶段的前进速度，当前等于 `CLIMB_LIFT_FORWARD_MM_S`。 |

底层 `LiftWalk_DefaultConfig()` 里还限制了高度轨迹：

```c
cfg->min_height_mm = 0.0f;
cfg->max_height_mm = 198.0f;
cfg->lift_vmax_mm_s = 10.0f;
cfg->lift_amax_mm_s2 = 20.0f;
```

所以 `target_height_mm` 会限制在 `0 ~ 198mm`，高度参考值会按最大速度 `10mm/s` 和最大加速度 `20mm/s^2` 平滑逼近目标。

### 2.2 前臂放地/收回参数

```c
#define CLIMB_FRONT_ARM_DEPLOY_LEFT_OFFSET_RAD 0.80f
#define CLIMB_FRONT_ARM_DEPLOY_RIGHT_OFFSET_RAD 0.80f
#define CLIMB_FRONT_ARM_MOVE_MAX_RAD_S 0.60f
#define CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD 0.03f
#define CLIMB_FRONT_ARM_DONE_SPEED_RAD_S 0.08f
#define CLIMB_FRONT_ARM_KP 0.08f
#define CLIMB_FRONT_ARM_KD 0.01f
#define CLIMB_FRONT_ARM_TORQUE_FF 0.0f
```

| 参数 | 含义 |
| --- | --- |
| `CLIMB_FRONT_ARM_DEPLOY_LEFT_OFFSET_RAD` | 左宇树小臂从初始收回姿态向前放地的相对角度。 |
| `CLIMB_FRONT_ARM_DEPLOY_RIGHT_OFFSET_RAD` | 右宇树小臂从初始收回姿态向前放地的相对角度。 |
| `CLIMB_FRONT_ARM_MOVE_MAX_RAD_S` | 前臂平滑转动的最大角速度。 |
| `CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD` | 前臂到位角度容差。 |
| `CLIMB_FRONT_ARM_DONE_SPEED_RAD_S` | 前臂到位速度阈值。 |
| `CLIMB_FRONT_ARM_KP/KD/TORQUE_FF` | 宇树小臂 MIT 控制参数。 |

上车调试时最先调这两个：

```c
CLIMB_FRONT_ARM_DEPLOY_LEFT_OFFSET_RAD
CLIMB_FRONT_ARM_DEPLOY_RIGHT_OFFSET_RAD
```

如果小臂方向反了，就把对应 offset 改成负数；如果前轮没落地，就增大绝对值；如果压得太多，就减小绝对值。

### 2.3 后滑轨收回和轮速参数

```c
#define CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD 0.05f
#define CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S 0.0f
#define CLIMB_FRONT_WHEEL_RADIUS_MM 41.5f
#define CLIMB_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD (74.0f / 24.0f)
#define CLIMB_REAR_WHEEL_RADIUS_MM 60.0f
```

| 参数 | 含义 |
| --- | --- |
| `CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD` | 小米滑轨回零到位容差，单位是电机角度 rad。 |
| `CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S` | 后滑轨收回时的前进速度，当前为 `0`，表示停住收后滑轨。 |
| `CLIMB_FRONT_WHEEL_RADIUS_MM` | 达妙前轮半径。 |
| `CLIMB_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD` | 达妙前轮轮子角速度到电机角速度的传动比。 |
| `CLIMB_REAR_WHEEL_RADIUS_MM` | 后轮半径。 |

## 3. 当前 ClimbStairs() 总流程

### 3.1 CLIMB_IDLE

首次进入 `ClimbStairs()` 时：

```c
ClimbLift_Reset();
ClimbDrive_Stop();
climb_front_arm_pose_captured = 0U;
climb_front_arm_move_started = 0U;
current_climb_state = CLIMB_STEP1_FRONT_ARM_DEPLOY;
return 0;
```

作用：

- 清掉上一次 `LiftWalk` 抬升控制状态。
- 停止达妙前轮和后轮。
- 清掉前臂“收回姿态”和“动作开始”记录。
- 切到前臂放地阶段。

### 3.2 CLIMB_STEP1_FRONT_ARM_DEPLOY

代码逻辑：

```c
float vr = PID_Angle_Calculate(&chassis_yaw_pid, face_angle(face), lcResult.r);

cha_remote(0.0f, CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S, vr);
ClimbFrontWheel_SetLinearSpeed(CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S);

if (climb_front_arm_pose_captured == 0U) {
    // 记录当前宇树角度作为收回姿态
}

if (climb_front_arm_move_started == 0U) {
    // 启动宇树小臂平滑转到放地目标
}

if (左右宇树小臂都到位) {
    climb_front_arm_move_started = 0U;
    ClimbLift_Reset();
    cha_remote(0.0f, 0.0f, 0.0f);
    (void)ClimbLift_RunStep();
    current_climb_state = CLIMB_STEP2_LIFT_AND_FORWARD;
}
```

作用：

- 用 `cha_remote()` 控制底盘前进，并用 yaw PID 保持朝向。
- 达妙前轮还没接地时不作为底盘驱动，只按 `CLIMB_FRONT_ARM_DEPLOY_FORWARD_MM_S` 对应的线速度提前空转。
- 记录当前宇树小臂角度作为“收回姿态”：

```c
climb_front_arm_retract_rad[LEFT]
climb_front_arm_retract_rad[RIGHT]
```

- 目标放地角度为：

```c
retract_angle + CLIMB_FRONT_ARM_DEPLOY_*_OFFSET_RAD
```

- 通过 `Motor_StartSmoothGotoMIT()` 让左右宇树小臂平滑转到放地角度。
- 到位后切到 `CLIMB_STEP2_LIFT_AND_FORWARD`。

此阶段可以前进，但不能在前轮落地前调用 `ClimbLift_InitOnce()`，否则会把“前轮还没挨地”的姿态当成抬升零点。前臂放地完成后会清掉普通底盘 `cha_remote()` 命令，并在同一周期立即调用 `ClimbLift_RunStep()`，把控制源切到 `LiftWalk` 抬升行走，因此不需要真实停一下。

### 3.3 CLIMB_STEP2_LIFT_AND_FORWARD

代码逻辑：

```c
if (ClimbLift_RunStep() != 0U) {
    ClimbLift_DriveForward();
}

if (is_on_stair_edge(stair_id, face)) {
    climb_front_arm_move_started = 0U;
    current_climb_state = CLIMB_STEP3_FRONT_ARM_RETRACT;
}
```

作用：

- 执行正式抬升动作。
- `ClimbLift_RunStep()` 第一次执行时会调用 `ClimbLift_InitOnce()`。
- 因为前一个阶段已经让达妙前轮落地，所以 `ClimbLift_InitOnce()` 记录的是：

```text
宇树小臂已放地的位置 = 抬升控制的小臂零点
小米滑轨当前位置 = 抬升控制的滑轨零点
```

- `ClimbLift_RunStep()` 设置：

```c
target_height_mm = 198mm
vy_mm_s = 200mm/s
start_height_mm = 0mm
done_tolerance_mm = 2mm
```

- 它会调用 `LiftWalk_RunLiftAction()`，由 `LiftWalk` 同时控制：

```text
前侧宇树小臂
后侧小米滑轨
达妙前轮速度
后轮速度
```

- 当前阶段切到下一步的条件是 `is_on_stair_edge(stair_id, face)`，也就是走到台阶边缘。

注意：`ClimbLift_RunStep()` 返回 `1` 只表示高度轨迹到达 `198mm`，但本阶段真正切状态是看台阶边缘。也就是说代码允许“抬升完成后继续高位前进”，直到到达边缘。

### 3.4 CLIMB_STEP3_FRONT_ARM_RETRACT

代码逻辑：

```c
ClimbLift_HoldRearSliderAndDrive(CLIMB_LIFT_FORWARD_MM_S);

if (climb_front_arm_move_started == 0U) {
    // 启动宇树小臂平滑回到收回姿态
}

if (左右宇树小臂都到位) {
    climb_front_arm_move_started = 0U;
    current_climb_state = CLIMB_STEP4_REAR_SLIDER_RETRACT;
}
```

作用：

- 后侧小米滑轨保持上一阶段 `LiftWalk` 算出来的位置。
- 轮子继续以 `CLIMB_LIFT_FORWARD_MM_S` 向前走。
- 宇树小臂不再由 `LiftWalk` 高度解算控制，而是单独收回到最开始记录的 `climb_front_arm_retract_rad[]`。
- 收回到位后进入后滑轨收回阶段。

这个阶段的目的：让车的前半部分上台阶，同时把前侧宇树小臂收回来，避免前轮继续作为前侧支撑点。

### 3.5 CLIMB_STEP4_REAR_SLIDER_RETRACT

代码逻辑：

```c
// 宇树小臂保持收回角度
if (ClimbLift_RetractRearSlider(CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S) != 0U) {
    current_climb_state = CLIMB_STEP5_CENTER_FORWARD;
}
```

作用：

- 宇树小臂保持收回位置。
- 小米滑轨回到 `ClimbLift_InitOnce()` 时记录的零点：

```c
climb_lift_ctrl.cfg.slider_zero_rad[LEFT]
climb_lift_ctrl.cfg.slider_zero_rad[RIGHT]
```

- 当前 `CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S = 0`，所以后滑轨收回时车不前进。
- 左右滑轨误差都小于 `CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD` 后，进入下一步。

### 3.6 CLIMB_STEP5_CENTER_FORWARD

代码逻辑：

```c
// 宇树小臂保持收回角度
if (is_on_stair_center(stair_id)) {
    ClimbDrive_Stop();
    current_climb_state = CLIMB_COMPLETE;
} else {
    ClimbDrive_SetForwardSpeed(CLIMB_LIFT_FORWARD_MM_S);
}
```

作用：

- 宇树小臂保持收回。
- 如果还没到台阶中心，就继续向前走。
- 到台阶中心后停止轮子并进入完成状态。

`is_on_stair_center(stair_id)` 的判断条件是：

```c
fabsf(lcResult.x - stairs_center[stair_id].x) < 20
fabsf(lcResult.y - stairs_center[stair_id].y) < 20
```

即当前位置和目标台阶中心点的 `x/y` 偏差都小于 `20mm`。

### 3.7 CLIMB_COMPLETE

代码逻辑：

```c
ClimbLift_Reset();
ClimbDrive_Stop();
climb_front_arm_pose_captured = 0U;
climb_front_arm_move_started = 0U;
current_climb_state = CLIMB_IDLE;
climb_cnt = 0;
return 1;
```

作用：

- 停止 `LiftWalk`。
- 停止轮子。
- 清掉前臂记录。
- 状态机回到空闲。
- 返回 `1`，通知上层本次上楼完成。

## 4. 前臂动作逻辑

为了让 `ClimbStairs()` 更容易顺着读，前臂放下/收回逻辑现在直接写在对应状态里，没有再拆成 `ClimbFrontArm_Deploy()`、`ClimbFrontArm_Retract()` 这一类小函数。

### 4.1 记录收回姿态

在 `CLIMB_STEP1_FRONT_ARM_DEPLOY` 中，第一次进入时记录当前左右宇树小臂角度，作为后续收回目标：

```c
climb_front_arm_retract_rad[LIFT_WALK_LEFT] =
    ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR1_G);
climb_front_arm_retract_rad[LIFT_WALK_RIGHT] =
    ClimbLift_GetMotorAngle(UNITREE_GO_M8010_6_MOTOR2_G);
climb_front_arm_pose_captured = 1U;
```

### 4.2 前臂放地目标

前臂放地目标是：

```c
target = retract_angle + CLIMB_FRONT_ARM_DEPLOY_*_OFFSET_RAD;
```

也就是从开始上楼时的小臂收回姿态，再向前转一个可调 offset。

### 4.3 启动平滑转动

放地和收回都用 `Motor_StartSmoothGotoMIT()` 启动一次平滑转动：

```c
Motor_StartSmoothGotoMIT(...)
```

启动后，后续周期由 `Motor_All_Control_Loop()` 继续执行平滑轨迹。`climb_front_arm_move_started` 用来防止每个周期重复启动轨迹。

### 4.4 到位判定

到位判定直接写在状态里，条件是：

```c
角度误差 <= CLIMB_FRONT_ARM_DONE_TOLERANCE_RAD
速度 <= CLIMB_FRONT_ARM_DONE_SPEED_RAD_S
```

到位后会用 `ClimbMotor_SetMIT()` 保持目标角度。

### 4.5 ClimbFrontWheel_SetLinearSpeed()

只给达妙前轮下发速度，当前用于前臂放地阶段的前轮预转。

换算关系：

```c
front_motor_rad_s = vy_mm_s / front_wheel_radius * motor_ratio
```

也就是说达妙前轮的线速度目标和 `cha_remote()` 给底盘的前进线速度一致。

### 4.6 ClimbDrive_SetForwardSpeed()

直接给达妙前轮和后轮下发前进速度，不经过 `LiftWalk_Update()`。

当前换算：

```c
front_motor_rad_s = vy_mm_s / front_wheel_radius * motor_ratio
rear_rpm = vy_mm_s * sin(45deg) * 60 / rear_wheel_circumference
```

下发对象：

```c
DM_JOINT_G
DM_FRONT_RIGHT_G
BLAZER_FOC_MOTOR1_G
BLAZER_FOC_MOTOR2_G
```

如果实车方向不对，需要调整 `ClimbDrive_SetForwardSpeed()` 里的符号。

### 4.7 ClimbLift_HoldRearSliderAndDrive()

在前臂收回阶段使用。

作用：

- 保持小米滑轨在上一阶段 `LiftWalk` 输出的位置。
- 直接给轮子前进速度。
- 不再让 `LiftWalk` 控制宇树小臂。

### 4.8 ClimbLift_RetractRearSlider()

后侧小米滑轨收回到抬升零点：

```c
climb_lift_ctrl.cfg.slider_zero_rad[LEFT]
climb_lift_ctrl.cfg.slider_zero_rad[RIGHT]
```

到位后返回 `1`。

## 5. LiftWalk 抬升层说明

`ClimbLift_RunStep()` 当前仍然是正式抬升动作的核心函数：

```c
static uint8_t ClimbLift_RunStep(void)
```

它会设置：

```c
climb_lift_in.vx_mm_s = 0.0f;
climb_lift_in.vy_mm_s = CLIMB_LIFT_FORWARD_MM_S;
climb_lift_in.target_height_mm = CLIMB_LIFT_TARGET_HEIGHT_MM;
climb_lift_in.dt_s = CLIMB_LIFT_DT_S;
climb_lift_in.enable_motor_output = 1U;
```

然后调用：

```c
LiftWalk_RunLiftAction(&climb_lift_ctrl,
                       &climb_lift_in,
                       CLIMB_LIFT_FORWARD_MM_S,
                       CLIMB_LIFT_START_HEIGHT_MM,
                       CLIMB_LIFT_DONE_TOLERANCE_MM);
```

`LiftWalk_RunLiftAction()` 会：

1. 第一次启动时 `LiftWalk_Reset(ctrl, start_height_mm)`。
2. 每周期调用 `LiftWalk_Update()`。
3. 判断高度参考值是否到达目标。
4. 到达目标并且高度速度足够小时返回 `1`。

`LiftWalk_Update()` 每周期做：

1. 检查 roll/pitch 安全角。
2. 生成平滑高度轨迹。
3. 计算四个支撑点高度。
4. 后侧支撑点高度换算为小米滑轨位置。
5. 前侧支撑点高度换算为宇树小臂角度。
6. 根据 `vy_mm_s` 计算前后轮速度。
7. `enable_motor_output != 0` 时下发电机命令。

当前在 `ClimbLift_InitOnce()` 中，roll/pitch/yaw PID 都被置零，所以抬升时默认是四角等高，不做 IMU 姿态纠偏。

## 6. 调试重点

1. 前臂落地角度先调 `CLIMB_FRONT_ARM_DEPLOY_LEFT_OFFSET_RAD` 和 `CLIMB_FRONT_ARM_DEPLOY_RIGHT_OFFSET_RAD`。
2. 前臂放地阶段通过 `cha_remote()` 控制底盘前进，通过 `ClimbFrontWheel_SetLinearSpeed()` 让达妙前轮按同等线速度预转。
3. 前轮落地之前不要调用 `ClimbLift_RunStep()`、`ClimbLift_DriveForward()` 或任何会触发 `ClimbLift_InitOnce()` 的函数。
4. `ClimbLift_InitOnce()` 现在应该在前臂落地后执行，才能把落地姿态作为抬升零点。
5. `CLIMB_STEP2_LIFT_AND_FORWARD` 的切状态条件是 `is_on_stair_edge(stair_id, face)`，不是 `ClimbLift_RunStep()` 返回值。
6. 前臂收回阶段不再用 `LiftWalk` 控制前臂，只保持后滑轨并直接给轮速。
7. 后滑轨收回时当前速度是 `0mm/s`，如果希望边收边走，可以调整 `CLIMB_REAR_SLIDER_RETRACT_FORWARD_MM_S`。
8. 如果车轮方向不对，前臂放地阶段先检查 `cha_remote()` 方向和 `ClimbFrontWheel_SetLinearSpeed()` 的达妙符号；后续直接轮速阶段再检查 `ClimbDrive_SetForwardSpeed()` 的达妙和 Blazer FOC 符号。
9. 如果小米滑轨收回不到位，检查 `CLIMB_REAR_SLIDER_DONE_TOLERANCE_RAD` 和 `slider_zero_rad[]` 是否符合机械实际。

## 7. 新上楼流程简图

```text
CLIMB_IDLE
  |
  | ClimbLift_Reset()
  | ClimbDrive_Stop()
  | 清除前臂记录
  v
CLIMB_STEP1_FRONT_ARM_DEPLOY
  |
  | 记录当前宇树角度为收回姿态
  | 宇树小臂向前转 offset，让达妙前轮落地
  | 底盘通过 cha_remote() 前进
  | 达妙前轮按同等线速度提前空转
  | 注意：此阶段不初始化 LiftWalk
  v
CLIMB_STEP2_LIFT_AND_FORWARD
  |
  | ClimbLift_RunStep()
  | 第一次执行时 InitOnce，把前轮落地姿态作为抬升零点
  | 目标高度 198mm，前进速度 200mm/s
  | 到台阶边缘后进入下一步
  v
CLIMB_STEP3_FRONT_ARM_RETRACT
  |
  | 后小米保持上一阶段位置
  | 轮子继续前进
  | 宇树小臂收回到初始记录角度
  v
CLIMB_STEP4_REAR_SLIDER_RETRACT
  |
  | 宇树小臂保持收回
  | 后小米滑轨回到抬升零点
  | 当前默认停住收回
  v
CLIMB_STEP5_CENTER_FORWARD
  |
  | 小臂保持收回
  | 继续前进到台阶中心
  v
CLIMB_COMPLETE
  |
  | Reset / Stop / 清记录
  v
CLIMB_IDLE
```
