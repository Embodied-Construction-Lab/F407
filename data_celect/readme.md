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

另一台电脑更新与烧录：

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
{"schema_version":"stm32_velocity_command.v1","boom_mps":0.0,"stick_mps":0.0,"bucket_mps":0.0,"swing_radps":0.0,"command_seq":0,"command_source_stamp_ms":0}
```

Orin sends physical velocity values without sign change or scaling. STM32
preserves the calibrated `actor_spc` unit/sign conversion, PID gains and
position limits.

Telemetry remains the 55-field `stm32_control_telemetry.v2` contract. Its
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

1. engine off: telemetry v2 at about 20 Hz, no parse failures;
2. engine off: no command stays mode 3 and PWM is neutral;
3. engine off: manual zero claims mode 1, velocity zero claims mode 2;
4. engine off: malformed, stale and cross-mode nonzero commands return mode 3;
5. engine on: short manual deadman motion at existing `±0.15` upper-layer cap;
6. engine on: one-axis low-speed RL command and terminal zero;
7. RL Follow terminal zero → stop RL serial owner → guided collection without
   rebooting STM32.
