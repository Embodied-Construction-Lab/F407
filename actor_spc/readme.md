# actor_spc — 工程机械运动控制器

基于 STM32F407VET6 的工程机械（挖掘机类）运动控制器固件。采用速度闭环架构，通过 RS485 拉线编码器和 IMU 获取反馈，输出 PCA9685 PWM 驱动液压比例阀，支持上位机通过 USART2 发送速度指令。

## 硬件平台

| 组件 | 型号/接口 | 用途 |
| --- | --- | --- |
| MCU | STM32F407VET6 (Cortex-M4) | 运动控制算法与通信 |
| IMU | USART1 (DMA) | 航向角与回转角速度反馈 |
| 拉线编码器 | RS485 (USART3, Modbus RTU) | 大臂/小臂/铲斗位移与速度反馈 |
| 角度传感器 | ADS1115 (I2C1) | DWJ 电位计读取（大臂角度、铲斗角度） |
| PWM 输出 | PCA9685 (I2C1) | 8 通道 PWM 驱动液压比例阀 |
| OLED | SSD1306 (I2C2) | 状态显示 |
| 上位机指令 | USART2 (DMA) | 接收速度控制命令 |
| 遥测输出 | USART1 | CSV 格式实时数据 |

## 软件架构

### 控制流程图

```
上位机 (USART2)
    ↓ SpeedCommand (v_boom, v_stick, v_bucket, yaw_rate)
SpeedCommandReceiver  ←  USART2 DMA 空闲中断
    ↓ SpeedCommand
SpeedCommandPreprocess_Apply   (预处理：死区、平滑、限幅)
    ↓ SpeedCommand
SafetyLimits_ApplyAxisEnable   (按轴使能开关置零)
SafetyLimits_Apply             (安全限位截断)
    ↓ AxisControlSample (latest_speed_target)
PidController_Update ×4        (速度闭环 PID + 前馈)
    ↓ AxisControlSample (latest_axis_output)
SafetyLimits_Apply             (输出层安全限位)
    ↓ AxisControlSample
JoystickServoMap_Compute       (轴值 → 舵机 PWM)
    ↓ JoystickServoTargets
Pca9685_SetPwm ×N              (I2C 输出)
```

### 回中流程

上电后先执行回中（Homing Controller），驱动各轴到安全中位，完成后才进入正常速度控制模式。

```
HomingController_Update
    ↓ 各轴以固定速度运行至目标位置
    ↓ 所有轴进入容差范围并稳定 → 标记完成
    ↓ 初始化 PID 控制器
    → 进入正常控制循环
```

### 模块说明

| 模块 | 文件 | 功能 |
| --- | --- | --- |
| **speed_command_receiver** | `Core/Inc/speed_command_receiver.h` | USART2 数据帧接收与解析。ISR 中存入环形队列，主循环出队解析为 `SpeedCommand`（含 `v_boom`, `v_stick`, `v_bucket`, `yaw_rate`）。 |
| **speed_command_preprocess** | `Core/Inc/speed_command_preprocess.h` | 速度命令预处理（可扩展死区、平滑滤波、加速度限制）。 |
| **safety_limits** | `Core/Inc/safety_limits.h` | 安全限位：各轴软限位范围、正方向定义、轴独立使能。在目标速度和最终输出两处截断。 |
| **pid_controller** | `Core/Inc/pid_controller.h` | 通用 PID 控制器，支持对称/非对称前馈。输出范围归一化到 [-1.0, 1.0]。 |
| **speed_feedback** | `Core/Inc/speed_feedback.h` | 速度反馈聚合：IMU 陀螺仪 Y 轴角速度 + 拉线编码器三通道线速度。 |
| **speed_pid_config** | `Core/Inc/speed_pid_config.h` | 四轴 PID 参数配置（KP/KI/KD/前馈），可独立配置正反向。 |
| **ftepc_rs485** | `Core/Inc/ftepc_rs485.h` | FTEPC 拉线编码器 RS485 Modbus RTU 读取。原始脉冲 → 位移/速度换算。 |
| **arm_angle** | `Core/Inc/arm_angle.h` | 拉线伸长量 → 关节角度估算，基于三角形几何模型。 |
| **dwj_reader** | `Core/Inc/dwj_reader.h` | DWJ 角度传感器（ADS1115 ADC）读取，电位计电压 → 角度换算。 |
| **imu_parser** | `Core/Inc/imu_parser.h` | IMU 数据帧解析，输出角速度 `gz` 和时间戳。 |
| **homing_controller** | `Core/Inc/homing_controller.h` | 上电回中控制：驱动各轴到预设安全位置，判定完成/超时。 |
| **joystick_servo_map** | `Core/Inc/joystick_servo_map.h` | 轴控制量到 PCA9685 PWM 通道的映射（8 通道，含死区）。 |
| **pca9685** | `Core/Inc/pca9685.h` | PCA9685 PWM 驱动器 I2C 初始化与占空比设置。 |
| **oled_ssd1306** | `Core/Inc/oled_ssd1306.h` | SSD1306 OLED I2C 显示驱动。 |
| **motion_telemetry** | `Core/Inc/motion_telemetry.h` | 运动数据遥测 CSV 格式打包（含目标/反馈/状态）。 |
| **status_led** | `Core/Inc/status_led.h` | LED 状态指示（心跳、IMU 状态、RS485 状态）。 |

## 通信协议

### 上位机指令 (USART2)

速度命令通过 USART2（115200 8N1）以文本帧形式发送：

```
$V <v_boom> <v_stick> <v_bucket> <yaw_rate>\n
```

| 字段 | 类型 | 范围 | 含义 |
| --- | --- | --- | --- |
| `v_boom` | float | mm/s | 大臂目标线速度 |
| `v_stick` | float | mm/s | 小臂目标线速度 |
| `v_bucket` | float | mm/s | 铲斗目标线速度 |
| `yaw_rate` | float | deg/s | 回转目标角速度 |

- 帧以 `$V` 开头，空格分隔参数，换行结束。
- 超过 500ms 未收到有效帧会自动清零目标速度并进入安全状态。

### 遥测输出 (USART1)

USART1 以 CSV 格式输出运动数据，默认周期 100ms：

```
t_ms,x1,x2,y1,y2,s_boom,s_stick,s_bucket,v_boom,v_stick,v_bucket,a_boom,a_stick,a_bucket,yaw,yaw_rate,rs485_ok,adc_ok,imu_ok
```

## 轴约定

| 轴名称 | 机构 | 反馈源 | 速度单位 |
| --- | --- | --- | --- |
| X1 | 回转 (Swing) | IMU 陀螺仪 | deg/s |
| X2 | 铲斗 (Bucket) | RS485 拉线编码器 Ch3 | mm/s |
| Y1 | 小臂 (Stick) | RS485 拉线编码器 Ch2 | mm/s |
| Y2 | 大臂 (Boom) | RS485 拉线编码器 Ch1 | mm/s |

## 编译与烧录

1. 使用 STM32CubeIDE 打开项目根目录。
2. 选择配置（Debug / Release），编译。
3. 通过 ST-Link 烧录至 STM32F407VET6。

也可使用命令行工具链：

```bash
# 使用 arm-none-eabi-gcc 编译
arm-none-eabi-gcc -c Core/Src/*.c -I Core/Inc -I Drivers/STM32F4xx_HAL_Driver/Inc -I Drivers/CMSIS/Include -mcpu=cortex-m4 -mthumb -O2 -DUSE_HAL_DRIVER -DSTM32F407xx
```

## 参数调优

各轴 PID 参数、安全限位、回中配置、编码器比例、角度估算系数的详细说明和修改方法，请参见下方章节。

---

*以下为参数配置说明，由 `Core/Inc/` 下的头文件宏定义驱动。*

## 总体约定

### 长度单位

代码中参数使用 `hundredths_mm`，表示 `0.01 mm`。

例如：`14000` 表示 `140.00 mm`，`500` 表示 `5.00 mm`。

修改这类参数时，先把实际长度换算成 `实际 mm × 100` 后再写入宏。

## ftepc_rs485.h — 拉线编码器 RS485

该文件配置拉线编码器 RS485 读取和脉冲到位移的换算。

| 参数 | 当前值 | 含义 | 修改方法 |
| --- | --- | --- | --- |
| `FTEPC485_CHANNEL_COUNT` | `3U` | RS485 拉线编码器通道数 | 只有硬件通道数和解析代码一起改变时才修改 |
| `FTEPC485_HUNDREDTHS_MM_PER_PULSE_NUM` | `25` | 脉冲→位移分子（0.01 mm/pulse） | 见下方标定方法 |
| `FTEPC485_HUNDREDTHS_MM_PER_PULSE_DEN` | `20` | 脉冲→位移分母 | 当前比例 25/20 = 1.25 个 0.01 mm/pulse |

脉冲到位移换算标定：记录某段实际位移 `L mm` 内的脉冲变化 `P`，则
```
hundredths_mm_per_pulse = L × 100 / P
```

### 相关数据结构

| 字段 | 含义 |
| --- | --- |
| `pulse[]` | 各通道原始脉冲数 |
| `length_hundredths_mm[]` | 各通道换算后位移 |
| `speed_hundredths_mm_s[]` | 各通道速度 |
| `slave_address` | 从站地址 |
| `timeout_ms` | 串口超时 |

## arm_angle.h — 角度估算

拉线伸长量 → 关节角度，基于三角形几何模型。

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `ARM_ANGLE_OA_MM` | `420.0f` | 铰点到拉线固定点 O→A |
| `ARM_ANGLE_OB_MM` | `120.0f` | 铰点到拉线固定点 O→B |
| `ARM_ANGLE_BASE_LENGTH_MM` | `250.0f` | 基准长度（零位） |

## safety_limits.h — 安全限位

### 线位移限位

| 参数 | 当前值 | 实际值 |
| --- | --- | --- |
| `SAFETY_LIMIT_BOOM_MIN_HUNDREDTHS_MM` | `14000` | 140.00 mm |
| `SAFETY_LIMIT_BOOM_MAX_HUNDREDTHS_MM` | `19000` | 190.00 mm |
| `SAFETY_LIMIT_STICK_MIN_HUNDREDTHS_MM` | `6000` | 60.00 mm |
| `SAFETY_LIMIT_STICK_MAX_HUNDREDTHS_MM` | `16000` | 160.00 mm |
| `SAFETY_LIMIT_BUCKET_MIN_HUNDREDTHS_MM` | `6000` | 60.00 mm |
| `SAFETY_LIMIT_BUCKET_MAX_HUNDREDTHS_MM` | `22000` | 220.00 mm |

### 回转限位

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `SAFETY_LIMIT_SWING_MIN_DEG` | `-60.0f` | 最小航向角 |
| `SAFETY_LIMIT_SWING_MAX_DEG` | `60.0f` | 最大航向角 |

航向角通过 `SafetyLimits_NormalizeYawDeg()` 归一化到 [-180, 180] 后再判断，建议写成跨零范围。

### 正方向定义

| 参数 | 含义 |
| --- | --- |
| `SAFETY_LIMIT_SWING_POSITIVE_INCREASES_YAW` | 1: 正输出 → 航向角增大 |
| `SAFETY_LIMIT_BUCKET_POSITIVE_INCREASES_LENGTH` | 1: 正输出 → 铲斗位移增大 |
| `SAFETY_LIMIT_STICK_POSITIVE_INCREASES_LENGTH` | 1: 正输出 → 小臂位移增大 |
| `SAFETY_LIMIT_BOOM_POSITIVE_INCREASES_LENGTH` | 1: 正输出 → 大臂位移增大 |

修改后须低速验证：给正目标速度，观察对应反馈值是否增大。

### 轴使能

`SAFETY_LIMIT_AXIS_X1_ENABLED` ~ `Y2_ENABLED`：`1U` 开启，`0U` 关闭。关闭的轴在目标速度和最终输出两处均置零。

## homing_controller.h — 回中控制

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `HOMING_CONTROLLER_SPEED` | `0.6f` | 回中速度幅值 |
| `HOMING_CONTROLLER_BOOM_TARGET_HUNDREDTHS_MM` | `16000` | 大臂回中目标 160.00 mm |
| `HOMING_CONTROLLER_STICK_TARGET_HUNDREDTHS_MM` | `16000` | 小臂回中目标 160.00 mm |
| `HOMING_CONTROLLER_BUCKET_TARGET_HUNDREDTHS_MM` | `16000` | 铲斗回中目标 160.00 mm |
| `HOMING_CONTROLLER_LINEAR_TOLERANCE_HUNDREDTHS_MM` | `500` | 线位移容差 5.00 mm |
| `HOMING_CONTROLLER_YAW_TARGET_DEG` | `0.0f` | 回转回中目标 |
| `HOMING_CONTROLLER_YAW_TOLERANCE_DEG` | `3.0f` | 回转容差 |
| `HOMING_CONTROLLER_STABLE_TIME_MS` | `500U` | 稳定判定时间 |
| `HOMING_CONTROLLER_TIMEOUT_MS` | `20000U` | 回中超时 |

## speed_pid_config.h — PID 参数

PID 计算形式：
```
output = target_speed × feedforward_gain
       + kp × (target_speed − measured_speed)
       + ki × integral(error)
       + kd × derivative(error)
```

`target_speed == 0` 时复位积分并输出 0。

### 各轴 PID 当前值

| 轴 | KP | KI | KD | 正向前馈 | 反向前馈 |
| --- | --- | --- | --- | --- | --- |
| X1 回转 | 0.02 | 0.0 | 0.0 | 1/40（对称） | 同左 |
| X2 铲斗 | 0.015 | 0.001 | 0.0 | 1/38 | 1/34 |
| Y1 小臂 | 0.015 | 0.001 | 0.0 | 1/35 | 1/38 |
| Y2 大臂 | 0.01 | 0.001 | 0.0 | 1/17.5 | 1/40 |

### PID 调参原则

1. **先调前馈，再调 PID** — 固定目标速度，观察稳定后实际速度，调整前馈系数使接近目标。
2. **正反向分开调** — 正负前馈独立配置。
3. **小幅调整 KP** — 响应慢增大，抖动减小。
4. **KI 只用于稳态偏差** — 稳定偏慢时少量增加，出现积分累积（超调、限位堆积）时减小。
5. **KD 默认保持 0** — 反馈噪声大时不启用。
6. **每次只改一个轴、一个方向、一个参数** — 编译→下载→从小速度开始测试。

## 修改后检查流程

1. 检查 `safety_limits.h` 限位范围留有安全余量。
2. 低速测试各轴正方向，确认方向宏正确。
3. 通过 USART2 发送单轴速度命令，确认目标与反馈速度符号一致。
4. 调整 PID 前馈使稳定速度接近目标。
5. 小幅调整 KP/KI 避免振荡。
6. 确认到达限位后控制输出置零。
7. 重新编译下载，用 USART2 遥测复核。
