#!/usr/bin/env bash
# Build the workspace and run the ROS-free core tests.
# Usage: ./scripts/build.sh [--cuda]   (--cuda adds -DENABLE_CUDA=ON -DENABLE_TENSORRT=ON)
set -euo pipefail
cd "$(dirname "$0")/.."

echo "== slam_core unit tests (ROS-free) =="
cmake -S tests -B tests/build -DCMAKE_BUILD_TYPE=Release
cmake --build tests/build -j"$(nproc)"
ctest --test-dir tests/build --output-on-failure

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found — skipping ROS 2 workspace build (core tests passed)."
  exit 0
fi

CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
if [[ "${1:-}" == "--cuda" ]]; then
  CMAKE_ARGS+=(-DENABLE_CUDA=ON -DENABLE_TENSORRT=ON)
fi

echo "== colcon build =="
# shellcheck disable=SC1091
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args "${CMAKE_ARGS[@]}"
