#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/f407-data-celect-host-tests"
mkdir -p "${build_dir}"

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

echo "data_celect host tests passed"
