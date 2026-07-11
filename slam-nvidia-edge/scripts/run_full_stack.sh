#!/usr/bin/env bash
# Run the full local stack: frontend + VIO + LIO + fusion + map + health + recorder.
set -euo pipefail
cd "$(dirname "$0")/.."
export SLAM_CONFIG_DIR="${SLAM_CONFIG_DIR:-$PWD/configs}"
export SLAM_LOG_DIR="${SLAM_LOG_DIR:-$PWD/logs}"
# shellcheck disable=SC1091
source ros2_ws/install/setup.bash
exec ros2 launch slam_bringup full_stack.launch.py
