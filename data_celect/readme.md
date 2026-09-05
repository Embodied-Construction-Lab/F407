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

The swing safety limit is measured from the heading present when the STM32 is
powered or reset. A separate unwrapped yaw accumulator gates both manual/ACT
and RL commands at `-100 deg` and `+100 deg`: an outward command is suppressed
at the boundary while a command back toward zero remains available. The
existing `[0, 360)` yaw telemetry and OLED display format are unchanged. If the
IMU is unavailable, manual/ACT swing is suppressed while the three linear axes
remain available.

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
velocity sign/unit conversion, PID outputs, the unwrapped `+/-100 deg` swing
limit, manual action mapping, telemetry, timing and OLED behavior.

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
