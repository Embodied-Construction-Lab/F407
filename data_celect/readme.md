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
2. 工程右键 **Build Configurations → Set Active → Release_ExternalCutoff**，然后 Clean / Build。
   该配置明确设置 `BOARD_EXTERNAL_POWER_CUTOFF_ENABLED=1`、`BOARD_ESTOP_INPUT_ENABLED=0`，
   对应已确认的外部断动力按钮。普通 `Debug` / `Release` 仍默认禁止运动。
3. 确认编译控制台包含上述两个 `-D` 定义，构建成功后使用本次新生成的
   **`data_celect/Release_ExternalCutoff/data_celect.elf`**。
4. 退出上位机控制程序、关闭液压动力，使用 **STM32CubeProgrammer** 下载该 ELF 并校验。
   CubeProgrammer 用于编程/校验，不负责本 C 工程的编译；它支持 ELF，无需另行转换 HEX。
5. 重启控制板后先由匹配的新 Orin 软件读取状态：应为遥测v3、外部断电模式；
   未下发控制命令时使能为0。完成接口核对后再进行单轴与RL现场验收。

本配置不修改三缸限位、PWM或速度比例。新固件的手动/ACT模式也执行已有方向性限位，
旧杆量模式可达的部分位置可能被限制；这不是重复零位标定要求。
旧速度v1客户端不能搭配本固件，新Orin客户端需使用速度v2。

验证：host回归覆盖三种停止构建模式；配置测试用真实 `board_safety.h` 核验三个CubeIDE配置
的有效宏以及独立输出目录。本机另以ARM GCC完成源码构建；Windows CubeIDE导入/构建仍由现场执行。
不将编译通过称为已烧录或实机运动通过。

工具说明：[ST官方 STM32CubeProgrammer 文档](https://www.st.com/en/development-tools/stm32cubeprog.html)。

## 2026-10-31 更新：外部动力断电按钮模式

用户已确认新增位于PC附近的独立按钮，用于切断挖掘机动力；使用后人工重启STM32、
Orin和挖掘机。该按钮没有STM32反馈线。本次增加显式构建模式，不把无反馈写成“已测急停清除”。

- 默认 `BOARD_ESTOP_INPUT_ENABLED=0`、`BOARD_EXTERNAL_POWER_CUTOFF_ENABLED=0`，仍禁止运动。
- 已接入GPIO反馈的设备继续使用原GPIO模式，读取实际输入。
- 本次外部断电架构显式编译 `BOARD_EXTERNAL_POWER_CUTOFF_ENABLED=1`；不能同时启用GPIO模式，
  两个开关只接受0或1。这只声明部署架构，不能由软件验证断电按钮是否接通或已按下。
- 外部模式保持 `BoardSafetyInput.available=0`，遥测bit5表示急停反馈不可用；同时新增
  `MOTION_TELEMETRY_FLAG_EXTERNAL_POWER_CUTOFF=0x40`（bit6）声明外部断电模式。
  `estop=0`仅表示没有板端输入抑制，**不是物理按钮处于松开状态的测量**。上位机必须保留
  物理急停状态unknown，并显式匹配模式；只看到 `estop=0` 不足以允许自动运动。
- v3的55列和顺序不变。bit6是现有 `fault_flags` 位图中的架构标志；bit5仍保留，因此旧版上位机
  会拒绝这种模式，不会静默当成GPIO已配置。无模式配置仍报告 `estop=1`、bit5及使能0。

三处板端检查统一使用 `BoardSafetyAllowsCommands`，但控制使能仍需要有效命令、正常输出及
相应控制模式条件。命令300ms超时、序号检查、限位、模式切换和PID归零行为不变。
上电仍为safe zero，必须先收到目标模式的零命令才能接收非零命令；外部断电之后须遵守上述
人工重启流程并重新启动上位机授权会话。板端不能检测只有液压动力被切断的事件，因而不能
证明“仅恢复动力而不重启控制器”不会恢复输出，也不支持将其作为操作流程。

host测试覆盖生产服务的GPIO、默认禁用、外部断电三种构建；外部模式覆盖手动/速度输出、
正确位图、上电非零拒绝、watchdog归零及重启后非零拒绝；同时检查非法/互斥编译配置拒绝。
测试不模拟实物动力切断有效性。本节对应源码变更没有自行烧录或运动。

## 2026-10-29 更新：V5 RL 速度协议 v2

速度命令改为 `stm32_velocity_command.v2`：boom、stick、bucket 均以油缸伸长为正，
单位 m/s；回转 rad/s 方向沿用既有实现。STM32 将三缸速度转换为已标定阀方向的
内部目标 `[+boom, +stick, -bucket] × 1000`，速度回报 `*_v_ref_mmps` 再转换回
伸长为正的物理目标。原 PID、方向相关前馈、测速、机械限位、PWM 和手动／ACT 映射不变。

旧速度 v1（包括零速度包）一律拒绝，不在同一版本下静默反向，也不提供 v1 回退。
EA 的 `devices/stm32_commands.py` 已配套发 v2，需与新固件配套验收；手动／ACT 仍为 v1。
当日仅改源码和离线验证，**未烧录、未运动**。当时用户确认现场没有专用急停，遇到问题由人工
断电，也没有 STM32 急停反馈线。它不被等同为已验证的急停系统；不再要求不存在的 GPIO。
已有 v3 急停输入未配置，默认构建不能运动，不能直接替换现场已通过的遥操作固件。
本轮保留默认禁用，不通过常量“急停已清除”或新增旁路放行；下节 GPIO 接入仅适用于以后确有该硬件时。

已通过命令解析、三缸正反向与测速一致性、四轴两端限位及反向退回、零速度清空 PID、
实际控制服务的遥测符号与归零测试。EA 跨端测试直接编译本工程 C 解析器验证协议。
CubeIDE 的 Debug／Release 构建路径及项目元数据已从历史 `receive_stick` 改为
`data_celect`，与 `.project` 一致；当日尚无 ARM 工具链，后于2026-10-31完成ARM构建，见上方当前入口。
详细证据与既有修改保护见
[本轮迁移记录](../../EvaluationReport/2026-10-29_f407_v5_velocity_migration/README.md)。

## 2026-10-22 更新：实时急停输入与遥测 v3

遥测使用 `stm32_control_telemetry.v3`，保持原 55 列及其顺序。`estop` 来自本次板端
GPIO 读取，不再固定为 0；`fault_flags` 新增
`MOTION_TELEMETRY_FAULT_ESTOP_INPUT_UNAVAILABLE = 0x20`（bit 5）。输入未配置时报告
`estop=1`、该故障位及 `control_enabled=0`，不能据人工勾选项把它当作急停已清除。

`Core/Inc/board_safety.h` 默认 `BOARD_ESTOP_INPUT_ENABLED=0`，端口、引脚、有效电平
均未指定，**默认固件不可运动**。只有现场确认实际电路、有效电平、断线行为，并在
CubeMX／`MX_GPIO_Init` 正确配置输入模式、所需上拉／下拉和端口时钟后，
才显式配置 `BOARD_ESTOP_GPIO_PORT`、`BOARD_ESTOP_GPIO_PIN`、
`BOARD_ESTOP_ACTIVE_STATE` 并启用、编译、经授权烧录。不提供猜测引脚或默认接线。
可用标志表示已显式配置此输入，不代表软件能自动检测接错线或断线，电路行为仍需现场验收。

命令接收时和每个控制周期在手动／速度分支之前均读取急停。输入不可用或触发时拒收命令，
废弃活动命令、清空模式与 PID，
按原输出路径写全中位；解除急停不会恢复原命令。之后必须重新接收有效零命令取得模式，再接收新
非零命令；Orin 本地会话重启与显式运动授权仍由上位机管理。原命令 watchdog、限位和映射保留。
`homing_complete` 的历史占位值未改，不能据此认定已回零或已标定。

验证：实际生产 GPIO 读取、命令接收、控制服务、遥测装配均纳入 host 回归；分别编译默认未配置
与虚拟已配置 GPIO 的测试，检查不可用／触发／清除、停止后不可自动恢复、两种控制模式和 v3
55 列／bit 5。11 个 host 测试程序通过。本轮未完成 ARM 全工程构建、烧录或现场运动。

## 2026-10-21 更新：手动动作接入已有机械限位

`ServiceControl20Hz` 的手动／ACT 分支现在先调用与速度分支相同的
`SafetyLimits_Apply`，再计算阀与 PWM：达到端点时禁止继续越限，允许反向退回。
非零手动动作要求现有三缸编码器、IMU 有效标志成立；传感标志无效时输出中位。
全零手动包（含既有 ±0.15 杆量死区语义）仍可直接使输出回到中位，不依赖传感器是否可用。

现有限位常量、极性、动作顺序及速度模式均未调整。新增 host 回归直接编译
`main.c` 的活动控制服务体，验证四轴两端限位与退回、原映射、传感标志失效归中位和
传感缺失下的零动作（包括四轴和两个履带输入的死区边界）。完整 host 测试通过；
本轮未完成 ARM 全工程构建、烧录或真机运动。

该改动不证明当前编码器长度基准、回转前向、IMU 持续采样或急停有效：这些仍需现场核对。
当日实现尚保留急停占位，已由上方 v3 更新替代；`homing_complete` 占位值及手动
`limit_mask` 的现状仍未改，不能将其当作已经完成安全标定的证据。

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
→ stop orin_state_sender.py
→ collector/ACT opens the same serial port and synchronizes command_seq
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

The motion stages require the explicitly selected physical stopping architecture
described above: verified GPIO input, or the confirmed external power-cut button
with manual controller/system restart before resuming. The default build still
only supports fail-closed/read-only checks. External mode provides no measured
button state and must not be interpreted as a GPIO clearance signal.

1. engine off: telemetry v3 at about 20 Hz, no parse failures;
2. engine off: no command stays mode 3 and PWM is neutral;
3. engine off: manual zero claims mode 1, velocity zero claims mode 2;
4. engine off: malformed, stale and cross-mode nonzero commands return mode 3;
5. engine on: short manual deadman motion at existing `±0.15` upper-layer cap;
6. engine on: one-axis low-speed RL command and terminal zero;
7. RL Follow terminal zero → stop RL serial owner → guided collection without
   rebooting STM32.
