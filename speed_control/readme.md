# speed_control 参数说明与修改指南

本文记录以下配置头文件中的主要参数含义和修改方法：

- `Core/Inc/ftepc_rs485.h`
- `Core/Inc/arm_angle.h`
- `Core/Inc/safety_limits.h`
- `Core/Inc/homing_controller.h`
- `Core/Inc/joystick_profile.h`
- `Core/Inc/speed_pid_config.h`

## 总体约定

### 轴命名

| 轴 | 对应机构 | 当前控制含义 |
| --- | --- | --- |
| `X1` | 回转 | 目标角速度，单位 `deg/s` |
| `X2` | 铲斗 | 目标线速度，单位 `mm/s` |
| `Y1` | 小臂 | 目标线速度，单位 `mm/s` |
| `Y2` | 大臂 | 目标线速度，单位 `mm/s` |

### 长度单位

代码中很多参数使用 `hundredths_mm`，表示 `0.01 mm`。

例如：

- `14000` 表示 `140.00 mm`
- `500` 表示 `5.00 mm`

修改这类参数时，先把实际长度换算成 `实际 mm * 100` 后再写入宏。

## `ftepc_rs485.h`

该文件配置拉线编码器 RS485 读取和脉冲到位移的换算。

| 参数 | 当前值 | 含义 | 修改方法 |
| --- | --- | --- | --- |
| `FTEPC485_CHANNEL_COUNT` | `3U` | RS485 拉线编码器通道数。当前读取大臂、小臂、铲斗三个线位移通道。 | 只有硬件通道数和解析代码一起改变时才修改；单独改这个值会影响数组长度和解析逻辑。 |
| `FTEPC485_HUNDREDTHS_MM_PER_PULSE_NUM` | `25` | 每个脉冲对应位移换算系数的分子，单位是 `0.01 mm/pulse`。 | 与下面的分母共同决定位移比例。 |
| `FTEPC485_HUNDREDTHS_MM_PER_PULSE_DEN` | `20` | 每个脉冲对应位移换算系数的分母。 | 当前比例为 `25 / 20 = 1.25` 个 `0.01 mm/pulse`，即 `0.0125 mm/pulse`。 |

### 相关数据结构

| 字段 | 含义 |
| --- | --- |
| `pulse[]` | 各通道原始脉冲数。 |
| `length_hundredths_mm[]` | 各通道换算后的位移，单位 `0.01 mm`。 |
| `speed_hundredths_mm_s[]` | 各通道速度，单位 `0.01 mm/s`。 |
| `slave_address` | RS485 从站地址。 |
| `timeout_ms` | 串口通信超时时间。 |
| `last_pulse[]` / `last_tick_ms` | 用于计算速度的上一次脉冲和时间戳。 |

修改拉线比例时，应先用已知位移标定：记录某段实际位移 `L mm` 内的脉冲变化 `P`，则目标比例为：

```text
hundredths_mm_per_pulse = L * 100 / P
```

再把该小数写成分子 / 分母。例如 `1.25` 可写成 `25 / 20`。

## `arm_angle.h`

该文件配置由拉线伸长量估算角度时使用的机构几何尺寸。

| 参数 | 当前值 | 含义 | 修改方法 |
| --- | --- | --- | --- |
| `ARM_ANGLE_OA_MM` | `420.0f` | 角度计算模型中的 OA 边长度，单位 `mm`。 | 根据实际铰点到拉线固定点的机械尺寸修改。 |
| `ARM_ANGLE_OB_MM` | `120.0f` | 角度计算模型中的 OB 边长度，单位 `mm`。 | 根据实际机械尺寸修改。 |
| `ARM_ANGLE_BASE_LENGTH_MM` | `250.0f` | 角度计算模型中的基准长度，单位 `mm`。 | 根据机构零位或标定长度修改。 |

这些参数会影响 `ArmAngle_FromExtensionHundredthsMm()` 和 `ArmAngle_Channel2HundredthsDeg()` 的角度输出。只有在机械安装尺寸、拉线固定点或零位定义改变后才应修改。

## `safety_limits.h`

该文件配置各轴安全限位和正方向定义。安全限位会在输出到执行机构前拦截继续越界的控制量。

### 线位移限位

| 参数 | 当前值 | 实际值 | 含义 |
| --- | --- | --- | --- |
| `SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM` | `14000` | `140.00 mm` | 大臂允许最小拉线位移。 |
| `SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM` | `19000` | `190.00 mm` | 大臂允许最大拉线位移。 |
| `SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM` | `6000` | `60.00 mm` | 小臂允许最小拉线位移。 |
| `SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM` | `16000` | `160.00 mm` | 小臂允许最大拉线位移。 |
| `SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM` | `6000` | `60.00 mm` | 铲斗允许最小拉线位移。 |
| `SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM` | `22000` | `220.00 mm` | 铲斗允许最大拉线位移。 |

### 回转限位

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SAFETY_LIMIT_SWING_MIN_DEG` | `-60.0f` | 回转允许最小航向角，单位 `deg`。 |
| `SAFETY_LIMIT_SWING_MAX_DEG` | `60.0f` | 回转允许最大航向角，单位 `deg`。 |

航向角通过 `SafetyLimits_NormalizeYawDeg()` 归一化到 `-180` 到 `180` 附近的范围后再判断，因此回转限位建议写成 `-90.0f` 到 `90.0f` 这类跨零范围，不要写成 `270` 到 `90`。

### 正方向定义

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW` | `1U` | `1` 表示 X1 正输出会使航向角增大；`0` 表示正输出会使航向角减小。 |
| `SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH` | `0U` | `1` 表示 X2 正输出会使铲斗拉线位移增大；`0` 表示正输出会使位移减小。 |
| `SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH` | `1U` | `1` 表示 Y1 正输出会使小臂拉线位移增大；`0` 表示正输出会使位移减小。 |
| `SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH` | `1U` | `1` 表示 Y2 正输出会使大臂拉线位移增大；`0` 表示正输出会使位移减小。 |

修改正方向时，必须用低速手动验证：

1. 给该轴一个小的正目标速度。
2. 观察对应反馈值是增大还是减小。
3. 如果正目标速度导致反馈增大，则宏应为 `1U`；否则应为 `0U`。

如果这里的方向设置错误，安全限位可能会拦截错误方向，甚至放行继续越界的方向。

## `homing_controller.h`

该文件配置回中控制目标和判定条件。

| 参数 | 当前值 | 含义 | 修改方法 |
| --- | --- | --- | --- |
| `HOMING_CONTROLLER_SPEED` | `0.4f` | 回中时输出到 `JoystickProfileSample` 的目标速度幅值。当前速度闭环架构下，X1 按 `deg/s` 理解，X2/Y1/Y2 按 `mm/s` 理解。 | 如果回中太慢，增大该值；如果回中冲击大或容易过冲，减小该值。 |
| `HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM` | `16000` | 大臂回中目标位移，`160.00 mm`。 | 按实际安全中位设置。 |
| `HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM` | `12000` | 小臂回中目标位移，`120.00 mm`。 | 按实际安全中位设置。 |
| `HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM` | `12000` | 铲斗回中目标位移，`120.00 mm`。 | 按实际安全中位设置。 |
| `HOMING_CONTROLLER_LINEAR_TOLERANCE_HUNDREDTHS_MM` | `500` | 线位移回中容差，`5.00 mm`。 | 容差越小越精确，但更容易来回调整；容差越大越容易完成。 |
| `HOMING_CONTROLLER_YAW_TARGET_DEG` | `0.0f` | 回转回中目标角度，单位 `deg`。 | 通常保持为 `0.0f`。 |
| `HOMING_CONTROLLER_YAW_TOLERANCE_DEG` | `3.0f` | 回转回中角度容差，单位 `deg`。 | 过冲明显时可适当增大，要求更准时可减小。 |
| `HOMING_CONTROLLER_STABLE_TIME_MS` | `500U` | 所有轴进入容差范围后，需要连续稳定的时间。 | 增大可避免瞬时进入容差就判定完成。 |
| `HOMING_CONTROLLER_TIMEOUT_MS` | `20000U` | 回中超时时间。 | 如果机械行程长或速度小，可适当增大。 |

注意：`HOMING_CONTROLLER_SPEED` 当前值 `0.4f` 如果进入 PID 速度闭环，就是 `0.4 mm/s` 或 `0.4 deg/s` 级别，可能非常慢。若希望回中速度接近日常曲线速度，应按目标速度重新设置，而不是沿用旧的摇杆轴值经验。

## `joystick_profile.h`

该文件生成四轴目标速度曲线。当前曲线不是摇杆轴值，而是 PID 的目标速度输入。

### 轴开关

| 参数 | 当前值 | 含义 | 修改方法 |
| --- | --- | --- | --- |
| `JOYSTICK_PROFILE_X1_ENABLED` | `0U` | 是否默认生成回转 X1 速度曲线。 | `1U` 开启，`0U` 关闭。 |
| `JOYSTICK_PROFILE_X2_ENABLED` | `0U` | 是否默认生成铲斗 X2 速度曲线。 | `1U` 开启，`0U` 关闭。 |
| `JOYSTICK_PROFILE_Y1_ENABLED` | `0U` | 是否默认生成小臂 Y1 速度曲线。 | `1U` 开启，`0U` 关闭。 |
| `JOYSTICK_PROFILE_Y2_ENABLED` | `1U` | 是否默认生成大臂 Y2 速度曲线。 | `1U` 开启，`0U` 关闭。 |

这些宏使用 `#ifndef` 包裹，因此也可以通过编译器预定义宏覆盖默认值。

### 最大目标速度

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `JOYSTICK_PROFILE_X1_MAX_SPEED_DEG_S` | `20.0f` | X1 回转速度曲线最大幅值，单位 `deg/s`。 |
| `JOYSTICK_PROFILE_X2_MAX_SPEED_MM_S` | `25.0f` | X2 铲斗速度曲线最大幅值，单位 `mm/s`。 |
| `JOYSTICK_PROFILE_Y1_MAX_SPEED_MM_S` | `25.0f` | Y1 小臂速度曲线最大幅值，单位 `mm/s`。 |
| `JOYSTICK_PROFILE_Y2_MAX_SPEED_MM_S` | `15.0f` | Y2 大臂速度曲线最大幅值，单位 `mm/s`。 |

### 曲线周期

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `JOYSTICK_PROFILE_CYCLE_MS` | `8000U` | 一轮速度曲线周期，单位 `ms`。 |

当前 `joystick_profile.c` 中一轮 8 秒曲线为：

| 时间段 | 输出 |
| --- | --- |
| `0-1 s` | 从 `0` 线性升到 `+最大速度` |
| `1-3 s` | 保持 `+最大速度` |
| `3-4 s` | 从 `+最大速度` 线性降到 `0` |
| `4-5 s` | 从 `0` 线性降到 `-最大速度` |
| `5-7 s` | 保持 `-最大速度` |
| `7-8 s` | 从 `-最大速度` 线性回到 `0` |

如果只想采集单轴数据，应只开启一个轴，其余轴设为 `0U`。如果某轴曲线关闭，该轴目标速度为 `0`，PID 会复位并输出 `0`。

## `speed_pid_config.h`

该文件配置四个速度闭环 PID 和前馈参数。PID 输出是最终轴值，通常会被限制在控制输出范围内。

PID 计算形式为：

```text
output = target_speed * feedforward_gain
       + kp * (target_speed - measured_speed)
       + ki * integral(error)
       + kd * derivative(error)
```

当 `target_speed == 0` 时，PID 会复位积分并输出 `0`。

### X1 回转

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SPEED_PID_X1_KP` | `0.02f` | X1 比例增益。误差越大，修正越大。 |
| `SPEED_PID_X1_KI` | `0.0f` | X1 积分增益。用于消除稳态误差。 |
| `SPEED_PID_X1_KD` | `0.0f` | X1 微分增益。用于抑制快速变化，但容易放大噪声。 |
| `SPEED_PID_X1_FEEDFORWARD` | `1.0f / 40.0f` | X1 对称前馈。目标速度乘以该系数后直接形成基础输出。 |

### X2 铲斗

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SPEED_PID_X2_KP` | `0.015f` | X2 比例增益。 |
| `SPEED_PID_X2_KI` | `0.001f` | X2 积分增益。 |
| `SPEED_PID_X2_KD` | `0.0f` | X2 微分增益。 |
| `SPEED_PID_X2_POSITIVE_FEEDFORWARD` | `1.0f / 38.0f` | X2 正目标速度前馈。 |
| `SPEED_PID_X2_NEGATIVE_FEEDFORWARD` | `1.0f / 34.0f` | X2 负目标速度前馈。 |

### Y1 小臂

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SPEED_PID_Y1_KP` | `0.015f` | Y1 比例增益。 |
| `SPEED_PID_Y1_KI` | `0.001f` | Y1 积分增益。 |
| `SPEED_PID_Y1_KD` | `0.0f` | Y1 微分增益。 |
| `SPEED_PID_Y1_POSITIVE_FEEDFORWARD` | `1.0f / 35.0f` | Y1 正目标速度前馈。 |
| `SPEED_PID_Y1_NEGATIVE_FEEDFORWARD` | `1.0f / 38.0f` | Y1 负目标速度前馈。 |

### Y2 大臂

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SPEED_PID_Y2_KP` | `0.01f` | Y2 比例增益。 |
| `SPEED_PID_Y2_KI` | `0.001f` | Y2 积分增益。 |
| `SPEED_PID_Y2_KD` | `0.0f` | Y2 微分增益。 |
| `SPEED_PID_Y2_POSITIVE_FEEDFORWARD` | `1.0f / 17.5f` | Y2 正目标速度前馈。 |
| `SPEED_PID_Y2_NEGATIVE_FEEDFORWARD` | `1.0f / 40.0f` | Y2 负目标速度前馈。 |

### PID 参数修改原则

1. 先调前馈，再调 PID。
   - 固定一个目标速度，观察稳定后的实际速度。
   - 如果实际速度偏慢，增大前馈系数；写法上等价于减小分母。
   - 如果实际速度偏快，减小前馈系数；写法上等价于增大分母。

2. 正反向分开调。
   - `POSITIVE_FEEDFORWARD` 只作用于正目标速度。
   - `NEGATIVE_FEEDFORWARD` 只作用于负目标速度。
   - 如果负向过快，应优先减小负向前馈，也就是把 `1.0f / N` 中的 `N` 调大。

3. 再小幅调整 `KP`。
   - 响应慢、跟踪误差大：适当增大 `KP`。
   - 速度抖动、来回修正明显：适当减小 `KP`。

4. `KI` 只用于修正稳定偏差。
   - 稳态长期偏慢或偏快时，少量增加 `KI`。
   - 出现越调越大、接近限位还持续累积、换向后恢复慢时，减小 `KI`。

5. `KD` 默认保持 `0`。
   - 速度反馈噪声较大时不建议启用。
   - 只有在确认反馈平滑、且需要抑制超调时再小量尝试。

6. 每次只改一个轴、一个方向、一个参数。
   - 修改后重新编译下载。
   - 从小速度开始测试。
   - 先确认安全限位方向正确，再提高速度幅值。

## 修改后的检查流程

1. 检查 `safety_limits.h` 中限位范围是否小于真实机械极限，并留有安全余量。
2. 低速测试每个轴正方向，确认方向宏正确。
3. 单轴开启 `joystick_profile`，确认目标速度曲线和反馈速度符号一致。
4. 调整 `speed_pid_config.h` 前馈，使稳定速度接近目标速度。
5. 小幅调整 `KP` 和 `KI`，避免振荡和积分累积。
6. 确认到达限位后控制输出会被置零。
7. 修改完成后重新编译、下载，并用 USART2 采集实际数据复核。
