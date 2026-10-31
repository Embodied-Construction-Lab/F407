# Unified STM32 control firmware

`data_celect` is the single active STM32 firmware for the current RL + IL
experiment. One flash supports three upper-layer uses without changing the
low-level image:

- human demonstration collection: normalized manual action;
- ACT inference: normalized manual action;
- RL Follow: physical velocity reference with STM32 speed PID.

The STM32 does not run RL or ACT. It owns command interpretation, the calibrated
velocity PID, valve mapping, command timeout, actuator limits and the final PWM
write boundary.

## Windows 更新、编译与烧录（2026-10-31 当前入口）

本次同步分支是 **`codex/ea-orin-adaptation`**，工程是 **`data_celect`**。
以下步骤替代本文历史段落中的 `change_pwd` 分支操作。

在 Windows 的 F407 仓库目录执行（保留本机未提交修改；若 Git 报冲突先停止，不强制覆盖）：

```powershell
git fetch origin
git switch codex/ea-orin-adaptation
git pull --ff-only origin codex/ea-orin-adaptation
git log -1 --oneline
```

1. 用 **STM32CubeIDE** 导入/刷新 `data_celect` 工程。不要选择 `pulse`、`actor_spc` 或 `cmd_control`。
2. 工程右键 **Build Configurations → Set Active → Release**，然后 Clean / Build。
   `Debug` 也可用于调试；两者使用同一控制逻辑，不需要额外模式宏。
3. 构建成功后使用本次新生成的 **`data_celect/Release/data_celect.elf`**。
4. 退出上位机控制程序、关闭液压动力，使用 **STM32CubeProgrammer** 下载该 ELF 并校验。
   CubeProgrammer 用于编程/校验，不负责本 C 工程的编译；它支持 ELF，无需另行转换 HEX。
5. 重启控制板后先由匹配的新 Orin 软件读取状态：应为遥测 v3，未下发控制命令时使能为 0。
   完成接口核对后再进行单轴与 RL 现场验收。

本配置不修改三缸限位、PWM或速度比例。新固件的手动/ACT模式也执行已有方向性限位，
旧杆量模式可达的部分位置可能被限制；这不是重复零位标定要求。
旧速度v1客户端不能搭配本固件，新Orin客户端需使用速度v2。

验证：host 回归覆盖普通构建的手动/速度服务、启动零命令交接、超时归零、重启、限位和遥测。
配置测试核验两个 CubeIDE 配置及独立输出目录。本机另以 ARM GCC 完成源码构建；
Windows CubeIDE 导入/构建仍由现场执行。不将编译通过称为已烧录或实机运动通过。

工具说明：[ST官方 STM32CubeProgrammer 文档](https://www.st.com/en/development-tools/stm32cubeprog.html)。

### 2026-10-31 烧录前复查修复

- 命令只接受两种现有 schema 的完整 JSON 对象；拒绝截断、重复字段、非法数值与超范围手动杆量。
- 中位输出遇到 I²C 写入失败时，在后续控制周期继续重试，不把写入失败当作已归零。
- IMU 非有限值与超过 500 ms 未收到有效新帧会使反馈失效；角度归一化使用有界计算，不修改漂移补偿或零点。
- `.ioc` 工程名与 `data_celect` 统一。仅修复元数据，不重新生成外设代码。

本轮保留速度 v2、手动 v1、遥测 v3、PID、阀方向、PWM 和机械限位。
烧录应使用本轮重新构建的产物；旧构建不能包含这些修复。
验证与产物身份见 [烧录前复查记录](../../EvaluationReport/2026-10-31_f407_preflash_review/README.md)。

## 2026-10-31：IMU 静止漂移修复

当前固件开机后需要机器静止约 **10 秒**，用于估计本次陀螺仪零偏。
期间不要操作手柄或转动上车；OLED 显示 `IMU calibrating / Keep still 10s`，
`imu_ok=0` 是尚未校准完成的状态。稳定窗口通过后开始输出修正后的角速度和相对回转角。
这一步不需要拉线传感器重新清零，也不需要改三缸标定。

- IMU 每个解码采样逐帧积分，避免 DMA 批量数据只使用最后一帧。
- 启动零偏按采样时间加权；确认后固定，不在线学习真实回转，也不增加小速度死区。
- 未完成校准时，有效非零控制请求或明显不稳定数据会重置静止窗口。
- UART 接收恢复由主循环处理，保留已确认零偏和累计角；重复、回退、长间隔不跨缺口积分。
- 通信仍为遥测 v3、速度 v2、手动 v1；Python 和 RViz 继续使用同一板端角度及修正角速度。

烧录后先保持静止等待校准，再做只读漂移对比。开机方向仍是本次相对零位；
零偏补偿不等于长期绝对航向定位，也不能恢复丢帧期间未知的实际转角。
本轮未改变 PID、阀方向、PWM、机械限位或拉线映射。
诊断、测试与构建身份见 [IMU 漂移记录](../../EvaluationReport/2026-10-31_imu_drift/README.md)。

## 2026-10-29 更新：V5 RL 速度协议 v2

速度命令改为 `stm32_velocity_command.v2`：boom、stick、bucket 均以油缸伸长为正，
单位 m/s；回转 rad/s 方向沿用既有实现。STM32 将三缸速度转换为已标定阀方向的
内部目标 `[+boom, +stick, -bucket] × 1000`，速度回报 `*_v_ref_mmps` 再转换回
伸长为正的物理目标。原 PID、方向相关前馈、测速、机械限位、PWM 和手动／ACT 映射不变。

旧速度 v1（包括零速度包）一律拒绝，不在同一版本下静默反向，也不提供 v1 回退。
EA 的 `devices/stm32_commands.py` 已配套发 v2，需与新固件配套验收；手动／ACT 仍为 v1。
当日仅改源码和离线验证，**未烧录、未运动**。

已通过命令解析、三缸正反向与测速一致性、四轴两端限位及反向退回、零速度清空 PID、
实际控制服务的遥测符号与归零测试。EA 跨端测试直接编译本工程 C 解析器验证协议。
CubeIDE 的 Debug／Release 构建路径及项目元数据已从历史 `receive_stick` 改为
`data_celect`，与 `.project` 一致；当日尚无 ARM 工具链，后于2026-10-31完成ARM构建，见上方当前入口。
详细证据与既有修改保护见
[本轮迁移记录](../../EvaluationReport/2026-10-29_f407_v5_velocity_migration/README.md)。

## 遥测协议

遥测使用 `stm32_control_telemetry.v3`，保持原 55 列及其顺序。
`estop` 为保留字段，固定为 0，不表示实测硬件状态；`homing_complete` 的历史占位值
不能用于认定已回零或已标定。控制状态以有效命令、输出结果和当前模式为依据。

## 2026-10-21 更新：手动动作接入已有机械限位

`ServiceControl20Hz` 的手动／ACT 分支现在先调用与速度分支相同的
`SafetyLimits_Apply`，再计算阀与 PWM：达到端点时禁止继续越限，允许反向退回。
非零手动动作要求现有三缸编码器、IMU 有效标志成立；传感标志无效时输出中位。
全零手动包（含既有 ±0.15 杆量死区语义）仍可直接使输出回到中位，不依赖传感器是否可用。

现有限位常量、极性、动作顺序及速度模式均未调整。新增 host 回归直接编译
`main.c` 的活动控制服务体，验证四轴两端限位与退回、原映射、传感标志失效归中位和
传感缺失下的零动作（包括四轴和两个履带输入的死区边界）。完整 host 测试通过；
本轮未完成 ARM 全工程构建、烧录或真机运动。

`homing_complete` 占位值及手动 `limit_mask` 的现状仍未改，不能将其当作已完成标定的证据。

## 2026-09-09 更新：角度 ADC 转换就绪检查

本次更新修复 ADS1115 切换大臂/铲斗通道后，仅固定等待便读取转换寄存器的时序缺口。
静止采样曾出现铲斗角度由约 212.6° 短暂跳至 135°，而铲斗缸位移没有对应变化；异常值与
大臂电压按铲斗标定换算的结果吻合。因此怀疑读取了上一通道的转换结果，现场根因仍待烧录复验。

具体修改：

- 启动转换成功后重新读取系统时钟，避免将 I²C 通信耗时计入转换等待。
- 读取转换值前检查配置寄存器的 OS 完成位与预期 MUX 通道。
- 转换尚未完成时继续等待；通道不符、通信失败或等待达到 30 ms 时标记采集异常。
- 保留现有角度标定、动作协议、PID、PWM 映射和运动安全逻辑。

验证状态：主机测试全部通过，新增测试覆盖未就绪时拒读旧值、通道不符、通信错误、
启动计时、状态机超时和时钟回绕。尚未完成 ARM 全工程编译、烧录或现场稳定性验收。

以下为2026-09-09历史更新步骤；当前更新请使用本文开头的分支和构建配置：

1. 在 F407 仓库切换到 `change_pwd` 分支，执行 `git pull --ff-only origin change_pwd`。
2. 在 STM32CubeIDE 打开/刷新本目录的 `data_celect` 工程，执行 Clean 和 Build。
3. 编译成功后用 ST-Link 烧录本次新生成的固件，不使用历史构建产物。
4. 保持运动禁用，完成传感器初始化后先做静止采样；确认角度跳变及采集异常情况后再开展运动测试。

主机回归命令（在 F407 仓库根目录执行）：

```bash
bash data_celect/Tests/run_host_tests.sh
```

## Wire contracts

Both commands use USART2 at `460800`, 8N1, newline-delimited ASCII JSON and a
shared uint32 command sequence.

Manual/ACT command (unchanged):

```json
{"schema_version":"stm32_manual_command.v1","X1":0.0,"Y1":0.0,"Z1":0.0,"X2":0.0,"Y2":0.0,"Z2":0.0,"command_seq":0,"command_source_stamp_ms":0}
```

The expert-action contract remains:

```text
[boom, stick, bucket, swing] = [Y2, Y1, X2, X1]
```

RL velocity command:

```json
{"schema_version":"stm32_velocity_command.v2","boom_mps":0.0,"stick_mps":0.0,"bucket_mps":0.0,"swing_radps":0.0,"command_seq":0,"command_source_stamp_ms":0}
```

Orin sends physical velocity values without sign change or scaling. Cylinder
extension is positive. STM32 maps these values into the calibrated valve
directions while retaining PID gains, asymmetric feedforward and position limits.
Legacy velocity v1 is rejected; it must not be used with this firmware.

Telemetry uses the 55-field `stm32_control_telemetry.v3` contract. Its
`control_mode` is:

- `1`: manual/ACT normalized action;
- `2`: RL physical velocity reference;
- `3`: safe zero.

## Mode transition

Firmware boots in safe zero. A target mode must first be claimed with a zero
command of that schema. A nonzero command from another mode is rejected and
returns the supervisor to safe zero. Sequence rejection, malformed input and a
command timeout also produce safe zero. PID state is reset on every transition.

Only one process may own `/dev/ttyTHS1`:

```text
RL Follow ends and sends terminal velocity zero
→ exit the active earthmoving-autonomy runtime and release its serial port
→ the next teleop/ACT runtime opens that port and synchronizes command_seq
→ manual zero claims manual mode
→ demonstration or ACT motion may begin
```

No STM32 reboot or reflash is required. The firmware deliberately does not
auto-home at boot because collection must never move merely because power was
applied. If an experiment needs a standardized start pose, reach it explicitly
with the authorized manual preposition step before RL or collection.

## Host verification

From Linux:

```bash
cd /home/zhaoshuai/workspace_uinty/RL_prj/F407/data_celect
bash Tests/run_host_tests.sh
```

This checks both command schemas, zero-only mode transitions, the physical
velocity sign/unit conversion, PID outputs, manual action mapping, telemetry,
timing and OLED behavior.

## CubeIDE build and flash

Open the existing `data_celect` project in STM32CubeIDE. Do not create another
sibling project and do not regenerate `main.c` from the `.ioc` until the
generated diff has been reviewed.

1. Select `Project > Clean`.
2. Build the Debug configuration.
3. Confirm zero compiler errors and that the new sources under `Core/Src` are
   compiled (`control_command`, `control_mode_supervisor`, `pid_controller`,
   `safety_limits`, `velocity_control`).
4. Flash and verify the programmer reports `Download verified successfully`.
5. Keep the engine off for the first serial and zero-command soak.

The pre-change rollback point is the annotated Git tag
`before-unified-control-20260814` (commit `17c5a2a`).

## Hardware acceptance order

Do not skip stages:

1. engine off: telemetry v3 at about 20 Hz, no parse failures;
2. engine off: no command stays mode 3 and PWM is neutral;
3. engine off: manual zero claims mode 1, velocity zero claims mode 2;
4. engine off: malformed, stale and cross-mode nonzero commands return mode 3;
5. engine on: short manual deadman motion at existing `±0.15` upper-layer cap;
6. engine on: one-axis low-speed RL command and terminal zero;
7. RL Follow terminal zero → stop RL serial owner → guided collection without
   rebooting STM32.
