#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/f407-data-celect-host-tests"
mkdir -p "${build_dir}"

python3 "${project_root}/Tests/cubeide_config_test.py"

common=(-std=c11 -Wall -Wextra -Werror -I"${project_root}/Core/Inc")

cc "${common[@]}" \
  "${project_root}/Tests/stick_receiver_test.c" \
  "${project_root}/Core/Src/stick_receiver.c" \
  "${project_root}/Core/Src/control_command.c" \
  -o "${build_dir}/stick_receiver_test"

cc "${common[@]}" \
  "${project_root}/Tests/control_command_test.c" \
  "${project_root}/Core/Src/control_command.c" \
  -o "${build_dir}/control_command_test"

cc "${common[@]}" \
  "${project_root}/Tests/control_mode_supervisor_test.c" \
  "${project_root}/Core/Src/control_command.c" \
  "${project_root}/Core/Src/control_mode_supervisor.c" \
  -o "${build_dir}/control_mode_supervisor_test"

cc "${common[@]}" \
  "${project_root}/Tests/velocity_control_test.c" \
  "${project_root}/Core/Src/velocity_control.c" \
  "${project_root}/Core/Src/pid_controller.c" \
  "${project_root}/Core/Src/safety_limits.c" \
  -o "${build_dir}/velocity_control_test"

cc "${common[@]}" \
  "${project_root}/Tests/manual_action_test.c" \
  "${project_root}/Core/Src/joystick_servo_map.c" \
  "${project_root}/Core/Src/servo_control.c" \
  "${project_root}/Core/Src/pwm_timing.c" \
  -o "${build_dir}/manual_action_test"

cc "${common[@]}" \
  "${project_root}/Tests/motion_telemetry_v2_test.c" \
  "${project_root}/Core/Src/motion_telemetry.c" \
  -o "${build_dir}/motion_telemetry_v2_test"

cc "${common[@]}" -I"${project_root}/Tests/fakes" \
  "${project_root}/Tests/oled_ssd1306_test.c" \
  "${project_root}/Core/Src/oled_ssd1306.c" \
  -o "${build_dir}/oled_ssd1306_test"

cc "${common[@]}" \
  "${project_root}/Tests/collection_timing_test.c" \
  -o "${build_dir}/collection_timing_test"

"${build_dir}/stick_receiver_test"
"${build_dir}/control_command_test"
"${build_dir}/control_mode_supervisor_test"
"${build_dir}/velocity_control_test"
"${build_dir}/manual_action_test"
"${build_dir}/motion_telemetry_v2_test"
"${build_dir}/oled_ssd1306_test"
"${build_dir}/collection_timing_test"

# Exercise the production service body without importing MCU startup/peripherals.
python3 - "${project_root}/Core/Src/main.c" "${build_dir}/dwj_service_under_test.inc" <<'PYCODE'
import pathlib
import sys
source = pathlib.Path(sys.argv[1]).read_text()
start = source.index("static void ServiceDwjReader(void)\n{")
end = source.index("\nstatic void ServiceOled", start)
pathlib.Path(sys.argv[2]).write_text(source[start:end])
PYCODE
cc "${common[@]}" -I"${project_root}/Tests/fakes" -I"${build_dir}" \
  "${project_root}/Tests/dwj_reader_test.c" \
  "${project_root}/Core/Src/dwj_reader.c" \
  -o "${build_dir}/dwj_reader_test"
"${build_dir}/dwj_reader_test"

python3 - "${project_root}/Core/Src/main.c" "${build_dir}/control_service_under_test.inc" <<'PYCODE'
import pathlib
import sys
source = pathlib.Path(sys.argv[1]).read_text()
input_start = source.index("static BoardSafetyInput ReadBoardSafetyInput(void)\n{")
input_end = source.index("\nstatic uint8_t ControlCommandIsValid", input_start)
valid_start = source.index("static uint8_t ControlCommandIsValid(uint32_t now_ms)\n{")
service_end = source.index("\nstatic void Imu_NormalizeYaw", valid_start)
telemetry_start = source.index("static void FillMotionTelemetry(MotionTelemetry *telemetry, uint32_t now_ms)\n{")
telemetry_end = source.index("\nstatic void ServiceTelemetry", telemetry_start)
fault_defines = "\n".join(line for line in source.splitlines()
                          if line.startswith("#define TELEMETRY_FAULT_")) + "\n"
pathlib.Path(sys.argv[2]).write_text(
    fault_defines + source[input_start:input_end] + source[valid_start:service_end] +
    source[telemetry_start:telemetry_end])
PYCODE
for stop_mode in gpio disabled external; do
estop_unavailable=0
external_cutoff=0
if [[ "${stop_mode}" == disabled ]]; then estop_unavailable=1; fi
if [[ "${stop_mode}" == external ]]; then external_cutoff=1; fi
cc "${common[@]}" -I"${project_root}/Tests/fakes" -I"${build_dir}" \
  -DTEST_BOARD_ESTOP_UNAVAILABLE="${estop_unavailable}" \
  -DTEST_EXTERNAL_POWER_CUTOFF="${external_cutoff}" \
  "${project_root}/Tests/manual_control_service_test.c" \
  "${project_root}/Core/Src/control_command.c" \
  "${project_root}/Core/Src/stick_receiver.c" \
  "${project_root}/Core/Src/control_mode_supervisor.c" \
  "${project_root}/Core/Src/velocity_control.c" \
  "${project_root}/Core/Src/pid_controller.c" \
  "${project_root}/Core/Src/safety_limits.c" \
  "${project_root}/Core/Src/joystick_servo_map.c" \
  "${project_root}/Core/Src/servo_control.c" \
  "${project_root}/Core/Src/pwm_timing.c" \
  -o "${build_dir}/manual_control_service_test_${stop_mode}"
"${build_dir}/manual_control_service_test_${stop_mode}"
done

# Board safety modes are an explicit compile-time choice, never a permissive default.
for invalid in both bad_gpio bad_external; do
  options=(-DBOARD_ESTOP_INPUT_ENABLED=0 -DBOARD_EXTERNAL_POWER_CUTOFF_ENABLED=0)
  case "${invalid}" in
    both) options=(-DBOARD_ESTOP_INPUT_ENABLED=1 -DBOARD_EXTERNAL_POWER_CUTOFF_ENABLED=1
                   -DBOARD_ESTOP_GPIO_PORT=0 -DBOARD_ESTOP_GPIO_PIN=1 -DBOARD_ESTOP_ACTIVE_STATE=0);;
    bad_gpio) options=(-DBOARD_ESTOP_INPUT_ENABLED=2);;
    bad_external) options=(-DBOARD_EXTERNAL_POWER_CUTOFF_ENABLED=2);;
  esac
  if cc "${common[@]}" "${options[@]}" -include board_safety.h -x c -fsyntax-only /dev/null \
       >"${build_dir}/board_${invalid}.log" 2>&1; then
    echo "Invalid board safety configuration accepted: ${invalid}" >&2
    exit 1
  fi
done

echo "data_celect host tests passed"
