#!/usr/bin/env bash
# Record sensor + SLAM topics to logs/mcap/ for later cloud upload
# (SageMaker training/evaluation ingest — see docs/sagemaker_boundary.md).
set -euo pipefail
cd "$(dirname "$0")/.."
LOG_DIR="${SLAM_LOG_DIR:-$PWD/logs}/mcap"
mkdir -p "$LOG_DIR"
STAMP="$(date -u +%Y%m%dT%H%M%S)"
# shellcheck disable=SC1091
source ros2_ws/install/setup.bash
exec ros2 bag record \
  --storage mcap \
  --output "$LOG_DIR/session_$STAMP" \
  /camera/image /camera/camera_info /imu/data /lidar/points \
  /gps/fix /wheel/odom \
  /slam/vio/pose /slam/vio/state /slam/vio/health \
  /slam/lio/pose /slam/lio/state /slam/lio/health \
  /slam/fused/pose /slam/fused/state /slam/fused/health \
  /slam/health/global /slam/sync/status
