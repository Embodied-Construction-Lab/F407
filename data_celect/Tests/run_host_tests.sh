#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${TMPDIR:-/tmp}/f407-data-celect-host-tests"
mkdir -p "${build_dir}"

common=(-std=c11 -Wall -Wextra -Werror -I"${project_root}/Core/Inc")

cc "${common[@]}" \
  "${project_root}/Tests/stick_receiver_test.c" \
  "${project_root}/Core/Src/stick_receiver.c" \
  -o "${build_dir}/stick_receiver_test"

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

"${build_dir}/stick_receiver_test"
"${build_dir}/manual_action_test"
"${build_dir}/motion_telemetry_v2_test"

echo "data_celect host tests passed"
