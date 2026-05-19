# 底盘抬升行走控制说明

本文档对应当前工程中的 `lift_walk_controller` 代码，用来说明底盘抬升动作的执行步骤、各电机运动方式、当前默认参数以及调试观察重点。

## 1. 当前控制目标

当前抬升控制器实现的是：

1. 底盘四个角同步抬升到目标高度。
2. 前侧两个支撑点由宇树小臂实现。
3. 后侧两个支撑点由小米滑轨实现。
4. 抬升过程中只允许沿底盘 `+y` 方向前进。
5. 不做 `x` 方向横移，也不做自转。
6. 默认关闭 IMU 姿态矫正，四个角等高抬升。

注意：当前代码是“有限角度小臂逆解”模型，不是“小臂 360 度连续转圈，每次抬升转一整圈”的模型。

## 2. 一次抬升动作的逐步效果

本节按当前默认调试目标举例：

```c
target_height_mm = 198.0f;
vy_mm_s = 200.0f;
dt_s = 0.01f;
roll/pitch/yaw PID = 0;
wheel_arm_comp_gain = 1.0f;
```

也就是：底盘一边沿 `+y` 正方向慢速前进，一边把四个角从 `0 mm` 抬到 `198 mm`。

### 第 0 步：进入抬升动作

车身整体效果：

```c
底盘准备从当前高度开始抬升。
如果 start_height_mm = 0，则认为当前是未抬升状态。
```

代码动作：

```c
LiftWalk_RunLiftAction(...)
    action_in.vx_mm_s = 0.0f;
    action_in.vy_mm_s = vy_mm_s;
    LiftWalk_Reset(ctrl, start_height_mm);
```

电机动作：

```c
此时还没有立即跳到目标位置，只是把内部高度轨迹 height_ref_mm 初始化。
```

### 第 1 步：生成平滑抬升高度

车身整体效果：

```c
底盘四个角不会瞬间到 198 mm，而是慢慢抬升。
```

当前默认轨迹限制：

```c
最大抬升速度 lift_vmax_mm_s = 10 mm/s
最大抬升加速度 lift_amax_mm_s2 = 20 mm/s^2
```

从 `0 mm` 抬到 `198 mm` 的理论时间约为：

```c
加速到 10 mm/s 用时 0.5 s，位移 2.5 mm
减速停止用时 0.5 s，位移 2.5 mm
中间匀速位移 193 mm，用时 19.3 s
总时间约 20.3 s
```

电机动作：

```c
这一阶段只是生成 height_ref_mm 和 height_dot_ref_mm_s。
真正的电机目标会根据每个周期的 height_ref_mm 实时更新。
```

### 第 2 步：四个支撑点目标高度

车身整体效果：

```c
因为当前默认关闭 roll/pitch 矫正，所以四个角等高抬升。
```

目标高度：

```c
front_left_z  = height_ref_mm
front_right_z = height_ref_mm
rear_left_z   = height_ref_mm
rear_right_z  = height_ref_mm
```

最终到达目标时：

```c
front_left_z  = 198 mm
front_right_z = 198 mm
rear_left_z   = 198 mm
rear_right_z  = 198 mm
```

电机对应关系：

```c
前左角 -> 左宇树小臂
前右角 -> 右宇树小臂
后左角 -> 左小米滑轨
后右角 -> 右小米滑轨
```

### 第 3 步：后侧小米滑轨抬升

车身整体效果：

```c
底盘后左、后右两个角被小米滑轨向上顶起。
```

参与电机：

```c
XIAOMI_MOTOR1_G -> 左后滑轨
XIAOMI_MOTOR2_G -> 右后滑轨
```

当前几何参数：

```c
小米电机转 1 圈，滑块沿导轨移动 180 mm
导轨与竖直方向夹角 7.5 deg
竖直等效位移 = 180 * cos(7.5 deg) ≈ 178.46 mm/rev
```

到达 `198 mm` 抬升高度时，小米电机目标变化量：

```c
motor_rev = 198 / 178.46 ≈ 1.11 rev
motor_rad = 1.11 * 2π ≈ 6.97 rad
```

所以最终目标大约是：

```c
左小米目标 = slider_zero_rad[L] + slider_motor_sign[L] * 6.97 rad
右小米目标 = slider_zero_rad[R] + slider_motor_sign[R] * 6.97 rad
```

当前默认：

```c
slider_zero_rad[L/R] = 0.0f
slider_motor_sign[L/R] = 1.0f
```

所以默认观察值约为：

```c
左小米 slider_motor_rad ≈ +6.97 rad
右小米 slider_motor_rad ≈ +6.97 rad
```

如果实车方向反了，把对应侧改成：

```c
slider_motor_sign[side] = -1.0f;
```

### 第 4 步：前侧宇树小臂抬升

车身整体效果：

```c
底盘前左、前右两个角由小臂支撑并向上抬起。
小臂末端轮子保持接地，转轴随底盘升高。
```

参与电机：

```c
UNITREE_GO_M8010_6_MOTOR1_G -> 左前小臂
UNITREE_GO_M8010_6_MOTOR2_G -> 右前小臂
```

当前几何参数：

```c
小臂长度 L = 217.75 mm
未抬升时小臂转轴离地高度 = 60 mm
前轮直径 = 83 mm
前轮半径 = 41.5 mm
宇树电机角 / 小臂机构角 = 2 / 3
```

未抬升时几何零位：

```c
phi0 = asin((41.5 - 60) / 217.75)
phi0 ≈ -0.085 rad ≈ -4.87 deg
arm_zero_offset_rad = phi0
```

到达 `198 mm` 抬升高度时：

```c
pivot_height = 60 + 198 = 258 mm
sin_phi = (41.5 - 258) / 217.75 ≈ -0.994
phi ≈ -1.464 rad ≈ -83.86 deg

theta = phi - arm_zero_offset_rad
theta ≈ -1.464 - (-0.085)
theta ≈ -1.379 rad ≈ -78.98 deg
```

所以小臂机构目标变化量约为：

```c
左小臂 arm_theta_rad ≈ -1.38 rad
右小臂 arm_theta_rad ≈ -1.38 rad
```

换算成宇树电机目标变化量：

```c
unitree_motor_delta = theta * 2 / 3
unitree_motor_delta ≈ -1.379 * 2 / 3
unitree_motor_delta ≈ -0.919 rad
unitree_motor_delta ≈ -0.146 rev
```

最终宇树目标大约是：

```c
左宇树目标 = arm_motor_zero_rad[L] + arm_motor_sign[L] * (-0.919 rad)
右宇树目标 = arm_motor_zero_rad[R] + arm_motor_sign[R] * (-0.919 rad)
```

当前默认：

```c
arm_motor_zero_rad[L/R] = 0.0f
arm_motor_sign[L/R] = 1.0f
```

所以默认宇树电机目标约为：

```c
左宇树目标 ≈ -0.919 rad
右宇树目标 ≈ -0.919 rad
```

注意：这说明当前代码并没有让宇树每次抬升转一整圈，而是只转约 `0.146` 个电机圈。如果真实机构要求小臂每次抬升转 360 度，需要改成连续相位模型。

### 第 5 步：前轮达妙配合行走

车身整体效果：

```c
底盘抬升时仍沿 +y 方向向前走。
前轮安装在小臂末端，因此前轮速度 = 车身前进速度 + 小臂扫动补偿。
```

参与电机：

```c
DM_JOINT_G       -> 左前轮
DM_FRONT_RIGHT_G -> 右前轮
```

当前默认：

```c
vy_mm_s = 200 mm/s
front_wheel_radius_mm = 41.5 mm
wheel_arm_comp_gain = 1.0f
front_wheel_speed_limit_rad_s = 25.0f
达妙电机角速度 / 前轮角速度 = 74 / 24
```

如果先忽略小臂扫动补偿，前轮轮子角速度为：

```c
front_wheel_rad_s = 200 / 41.5 ≈ 4.82 rad/s
```

换算成达妙电机角速度：

```c
dm_motor_rad_s = 4.82 * 74 / 24 ≈ 14.86 rad/s
```

如果 `wheel_arm_comp_gain = 1.0f`，实际前轮速度还会叠加：

```c
arm_x_dot_mm_s
```

调试初期如果只想看纯前进速度，可以先设：

```c
wheel_arm_comp_gain = 0.0f;
```

### 第 6 步：后轮 Blazer FOC 配合行走

车身整体效果：

```c
后侧两个 45 度全向轮配合车身沿 +y 方向前进。
```

参与电机：

```c
BLAZER_FOC_MOTOR1_G -> 左后轮
BLAZER_FOC_MOTOR2_G -> 右后轮
```

当前默认：

```c
rear_wheel_radius_mm = 60 mm
左后轮驱动方向角 = +45 deg
右后轮驱动方向角 = -45 deg
rear_wheel_speed_limit_rpm = 3000 rpm
```

当 `vy_mm_s = 200 mm/s` 时：

```c
左后轮驱动线速度 = 200 * sin(+45 deg) ≈ +141.4 mm/s
右后轮驱动线速度 = 200 * sin(-45 deg) ≈ -141.4 mm/s
```

后轮周长：

```c
2π * 60 ≈ 377.0 mm
```

换算成 rpm：

```c
左后轮 rpm = 141.4 * 60 / 377.0 ≈ +22.5 rpm
右后轮 rpm = -141.4 * 60 / 377.0 ≈ -22.5 rpm
```

所以默认观察值约为：

```c
rear_wheel_rpm[L] ≈ +22.5 rpm
rear_wheel_rpm[R] ≈ -22.5 rpm
```

如果实际方向反了，改：

```c
rear_wheel_sign[side] = -1.0f;
```

### 第 7 步：动作完成

车身整体效果：

```c
底盘四个角达到目标高度附近，且高度速度降到接近 0。
```

完成判断：

```c
fabs(height_ref_mm - target_height_mm) <= done_tolerance_mm
fabs(height_dot_ref_mm_s) <= lift_amax_mm_s2 * dt_s
```

例如：

```c
target_height_mm = 198.0f
done_tolerance_mm = 2.0f
dt_s = 0.01f
```

则高度进入大约：

```c
196 mm ~ 200 mm
```

并且速度足够小时，`LiftWalk_RunLiftAction()` 返回：

```c
1
```

上层可以进入下一步，比如 `CLIMB_STEP4_REAR_FORWARD`。

## 3. 调用入口

推荐上层周期调用：

```c
done = LiftWalk_RunLiftAction(&lift_walk_ctrl,
                              &lift_walk_in,
                              vy_mm_s,
                              start_height_mm,
                              done_tolerance_mm);
```

参数含义：

```c
lift_walk_ctrl        // 抬升控制器对象
lift_walk_in          // 当前输入，包括目标高度、dt、姿态角等
vy_mm_s               // 抬升时沿底盘 +y 方向前进速度，单位 mm/s
start_height_mm       // 动作开始时的当前抬升高度，未抬升时填 0
done_tolerance_mm     // 判断动作完成的高度容差，单位 mm
```

该函数需要在 task 中周期调用。当前 `Task_chassis` 周期为：

```c
#define CHASSIS_TASK_PERIOD 10
```

所以对应：

```c
lift_walk_in.dt_s = 0.01f;
```

或者写成：

```c
lift_walk_in.dt_s = CHASSIS_TASK_PERIOD / 1000.0f;
```

## 4. 抬升总流程

`LiftWalk_RunLiftAction()` 内部会强制：

```c
action_in.vx_mm_s = 0.0f;
action_in.vy_mm_s = vy_mm_s;
```

然后调用 `LiftWalk_Update()`。当前单周期流程为：

1. 检查 roll/pitch 是否超过安全角度。
2. 根据 `target_height_mm` 生成平滑高度轨迹。
3. 计算 roll/pitch 姿态修正。
4. 得到四个支撑点抬升高度 `support_z_mm[]`。
5. 后侧支撑点高度转换成小米滑轨电机位置。
6. 前侧支撑点高度转换成宇树小臂角度。
7. 根据 `vy_mm_s` 计算前后轮速度。
8. 如果 `enable_motor_output != 0`，下发电机命令。

## 5. 当前默认设定值

### 4.1 机械硬参数

```c
LIFT_WALK_UNITREE_MOTOR_RAD_PER_ARM_RAD = 2.0f / 3.0f

LIFT_WALK_ARM_LENGTH_MM = 217.75f
LIFT_WALK_ARM_PIVOT_HEIGHT_MM = 60.0f

LIFT_WALK_FRONT_WHEEL_DIAMETER_MM = 83.0f
LIFT_WALK_FRONT_WHEEL_RADIUS_MM = 41.5f
LIFT_WALK_FRONT_WHEEL_MOTOR_RAD_PER_WHEEL_RAD = 74.0f / 24.0f

LIFT_WALK_REAR_WHEEL_RADIUS_MM = 60.0f

LIFT_WALK_SLIDER_TRAVEL_MM_PER_MOTOR_REV = 180.0f
LIFT_WALK_SLIDER_ANGLE_FROM_VERTICAL_RAD = 7.5 deg
```

### 4.2 抬升高度轨迹

```c
cfg.min_height_mm = 0.0f;
cfg.max_height_mm = 198.0f;
cfg.lift_vmax_mm_s = 10.0f;
cfg.lift_amax_mm_s2 = 20.0f;
```

含义：

```c
target_height_mm 最终会被限制在 0 ~ 198 mm。
height_ref_mm 按最大速度 10 mm/s、最大加速度 20 mm/s^2 平滑靠近目标高度。
```

### 4.3 姿态矫正

当前默认关闭姿态 PID：

```c
cfg.roll_kp = 0.0f;
cfg.roll_ki = 0.0f;
cfg.roll_kd = 0.0f;

cfg.pitch_kp = 0.0f;
cfg.pitch_ki = 0.0f;
cfg.pitch_kd = 0.0f;

cfg.yaw_kp = 0.0f;
cfg.yaw_kd = 0.0f;
```

所以四个支撑点默认等高：

```c
support_z_mm[FL] = height_ref_mm
support_z_mm[FR] = height_ref_mm
support_z_mm[RL] = height_ref_mm
support_z_mm[RR] = height_ref_mm
```

虽然 PID 关闭，安全保护仍然会检查：

```c
cfg.max_roll_rad = 0.17f;
cfg.max_pitch_rad = 0.17f;
```

如果输入的 roll 或 pitch 超过该值，会停止输出并返回保护状态。

## 6. 各电机运动方式

### 5.1 后侧小米滑轨电机

对应电机：

```c
cfg.xiaomi_slider_motor[LIFT_WALK_LEFT]  = XIAOMI_MOTOR1_G;
cfg.xiaomi_slider_motor[LIFT_WALK_RIGHT] = XIAOMI_MOTOR2_G;
```

控制方式：

```c
位置模式 set_position(position, vel_limit)
```

当前默认参数：

```c
cfg.slider_pitch_mm_per_rev = 180.0f;
cfg.slider_reduction_ratio = 1.0f;
cfg.slider_motor_sign[LIFT_WALK_LEFT] = 1.0f;
cfg.slider_motor_sign[LIFT_WALK_RIGHT] = 1.0f;
cfg.slider_zero_rad[LIFT_WALK_LEFT] = 0.0f;
cfg.slider_zero_rad[LIFT_WALK_RIGHT] = 0.0f;
cfg.slider_min_rad[...] = -1000.0f;
cfg.slider_max_rad[...] = 1000.0f;
cfg.slider_vel_limit_rad_s = 2.0f;
```

高度到电机角度换算：

```c
vertical_mm_per_rev = 180.0f * cos(7.5 deg);

slider_motor_rad =
    slider_zero_rad[side] +
    slider_motor_sign[side] *
    lift_mm * 2π / vertical_mm_per_rev;
```

调试重点：

```c
slider_zero_rad[]      // 未抬升时小米反馈零点，实车必须填
slider_motor_sign[]    // 抬升方向反了改成 -1
slider_vel_limit_rad_s // 小米位置模式速度限制
```

### 5.2 前侧宇树小臂电机

对应电机：

```c
cfg.unitree_arm_motor[LIFT_WALK_LEFT]  = UNITREE_GO_M8010_6_MOTOR1_G;
cfg.unitree_arm_motor[LIFT_WALK_RIGHT] = UNITREE_GO_M8010_6_MOTOR2_G;
```

控制方式：

```c
MIT 模式 set_mit(position, speed, kp, kd, torque)
```

当前默认参数：

```c
cfg.arm_length_mm[...] = 217.75f;
cfg.arm_pivot_z_mm[...] = 60.0f;
cfg.arm_motor_zero_rad[...] = 0.0f;
cfg.arm_motor_sign[...] = 1.0f;

cfg.arm_min_rad[...] = -1.5f;
cfg.arm_max_rad[...] = 1.3f;
cfg.arm_dead_cos_min = 0.01f;

cfg.arm_kp = 0.08f;
cfg.arm_kd = 0.01f;
cfg.arm_torque_ff = 0.0f;
```

未抬升时的几何零位：

```c
arm_zero_offset_rad =
    asinf((front_wheel_radius_mm - arm_pivot_z_mm) / arm_length_mm);
```

代入当前参数：

```c
front_wheel_radius_mm = 41.5 mm
arm_pivot_z_mm = 60.0 mm
arm_length_mm = 217.75 mm

arm_zero_offset_rad ≈ -0.085 rad ≈ -4.87 deg
```

高度到小臂角度换算：

```c
pivot_height_mm = arm_pivot_z_mm + lift_mm;
wheel_center_height_mm = front_wheel_radius_mm;

sin_phi = (wheel_center_height_mm - pivot_height_mm) / arm_length_mm;
phi = asinf(sin_phi);
theta = phi - arm_zero_offset_rad;
theta = clamp(theta, arm_min_rad, arm_max_rad);
```

小臂角度到宇树电机角度：

```c
unitree_motor_rad =
    arm_motor_zero_rad[side] +
    arm_motor_sign[side] * theta * 2.0f / 3.0f;
```

小臂角速度前馈：

```c
theta_dot_rad_s = -lift_dot_mm_s / (arm_length_mm * cos(phi));
unitree_motor_speed_rad_s = arm_motor_sign[side] * theta_dot_rad_s * 2.0f / 3.0f;
```

调试重点：

```c
arm_motor_zero_rad[]  // 未抬升时宇树反馈零点，实车必须填
arm_motor_sign[]      // 宇树方向反了改成 -1
arm_min/max_rad       // 当前有限角度摆臂模型的软限位
arm_kp/kd/torque_ff   // 宇树 MIT 参数
```

注意：当前代码不会累计小臂圈数。如果真实机构需要每次抬升旋转一整圈，需要改为连续相位/圈数累计模型。

### 5.3 前侧达妙轮电机

对应电机：

```c
cfg.front_wheel_motor[LIFT_WALK_LEFT]  = DM_JOINT_G;
cfg.front_wheel_motor[LIFT_WALK_RIGHT] = DM_FRONT_RIGHT_G;
```

控制方式：

```c
速度模式 set_speed(speed)
```

当前默认参数：

```c
cfg.front_wheel_radius_mm = 41.5f;
cfg.front_wheel_speed_limit_rad_s = 25.0f;
cfg.front_wheel_sign[...] = 1.0f;
cfg.wheel_arm_comp_gain = 1.0f;
```

前轮线速度：

```c
front_linear_mm_s =
    vy_mm_s + wheel_arm_comp_gain * arm_x_dot_mm_s;
```

前轮角速度：

```c
front_wheel_rad_s =
    front_linear_mm_s / front_wheel_radius_mm;
```

下发达妙前会再乘达妙到轮子的减速比：

```c
dm_motor_rad_s = front_wheel_rad_s * 74.0f / 24.0f;
```

调试初期如果不想加小臂扫动补偿，可以设：

```c
cfg.wheel_arm_comp_gain = 0.0f;
```

### 5.4 后侧 Blazer FOC 全向轮

对应电机：

```c
cfg.rear_wheel_motor[LIFT_WALK_LEFT]  = BLAZER_FOC_MOTOR1_G;
cfg.rear_wheel_motor[LIFT_WALK_RIGHT] = BLAZER_FOC_MOTOR2_G;
```

控制方式：

```c
速度模式 set_speed(rpm)
```

当前默认参数：

```c
cfg.rear_wheel_radius_mm = 60.0f;
cfg.rear_wheel_drive_angle_rad[LIFT_WALK_LEFT] = +pi / 4;
cfg.rear_wheel_drive_angle_rad[LIFT_WALK_RIGHT] = -pi / 4;
cfg.rear_wheel_speed_limit_rpm = 3000.0f;
cfg.rear_wheel_sign[...] = 1.0f;
```

当前抬升阶段只允许沿 `+y` 方向前进：

```c
forward_y_mm_s = vy_mm_s;
rear_drive_mm_s = forward_y_mm_s * sin(rear_wheel_drive_angle_rad[side]);
```

后轮转速：

```c
rear_wheel_rpm =
    rear_drive_mm_s * 60.0f / (2π * rear_wheel_radius_mm);
```

注意：这里使用的是速度投影，所以是乘 `sin(angle)`，不是除。

## 7. 当前默认目标高度下的典型量级

以目标高度 `198 mm` 为例：

```c
theta ≈ -1.38 rad
```

该值低于当前最小限位：

```c
arm_min_rad = -1.5f
```

所以当前限位允许抬升到 `198 mm` 附近。

理论几何极限约为：

```c
217.75 + 41.5 - 60 = 199.25 mm
```

当前 `max_height_mm = 198.0f`，留了约 `1.25 mm` 几何余量。

## 8. 使能和观察

输入结构体里：

```c
lift_walk_in.enable_motor_output = 0;
```

表示只计算、不下发电机，适合调试观察。

```c
lift_walk_in.enable_motor_output = 1;
```

表示计算后真正下发小米、宇树、达妙和 Blazer FOC 命令。

建议观察输出：

```c
out->height_ref_mm
out->height_dot_ref_mm_s
out->support_z_mm[FL/FR/RL/RR]
out->slider_motor_rad[L/R]
out->arm_theta_rad[L/R]
out->arm_theta_dot_rad_s[L/R]
out->arm_x_dot_mm_s[L/R]
out->front_wheel_rad_s[L/R]
out->rear_wheel_rpm[L/R]
out->status
```

当前 `Task_Printf.c` 中已经有一套独立观察用的 LiftWalk 打印逻辑：

```c
#define LIFT_WALK_PRINTF_DEBUG 1U
#define LIFT_WALK_DEBUG_TARGET_HEIGHT_MM 198.0f
#define LIFT_WALK_DEBUG_FORWARD_MM_S 200.0f
```

它使用：

```c
in.enable_motor_output = 0U;
```

所以只会打印，不会让电机动作。

## 9. 接入上楼梯状态机建议

当前 `ClimbStairs()` 的流程中，最适合先替换的是：

```c
CLIMB_STEP3_LIFT_UP
```

这一段原来是 3508 抬升整车。建议第一版只把该状态替换为：

```c
if (LiftWalk_RunLiftAction(&lift_walk_ctrl,
                           &lift_walk_in,
                           200.0f,
                           0.0f,
                           2.0f)) {
    LiftWalk_Stop(&lift_walk_ctrl);
    current_climb_state = CLIMB_STEP4_REAR_FORWARD;
}
```

接入前必须确认：

```c
slider_zero_rad[]      // 小米未抬升零点
arm_motor_zero_rad[]   // 宇树未抬升零点
slider_motor_sign[]    // 小米方向
arm_motor_sign[]       // 宇树方向
front/rear_wheel_sign[]// 前后轮方向
```

调试初期建议：

```c
enable_motor_output = 0;
wheel_arm_comp_gain = 0.0f;
roll/pitch/yaw PID = 0;
```

确认输出方向正确后，再逐步打开真实电机输出。
