#!/usr/bin/env bash
# Run VIO only (camera + IMU).
set -euo pipefail
cd "$(dirname "$0")/.."
export SLAM_CONFIG_DIR="${SLAM_CONFIG_DIR:-$PWD/configs}"
# shellcheck disable=SC1091
source ros2_ws/install/setup.bash
exec ros2 launch slam_bringup vio.launch.py
