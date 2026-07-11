---
name: slam-build-test
description: Build and verify the slam-nvidia-edge stack; use for any build, test, or verification task, and as the gate before handing work back.
---

# Build & verify slam-nvidia-edge

All paths relative to `slam-nvidia-edge/`.

## Core tests (always available — no ROS required)

```bash
cmake -S tests -B tests/build -DCMAKE_BUILD_TYPE=Release
cmake --build tests/build -j"$(nproc)"
ctest --test-dir tests/build --output-on-failure
```

Requires only a C++20 compiler and Eigen (`apt-get install libeigen3-dev`).
All 4 suites must pass. Never hand back with a failing suite; if the failure
is outside your task's scope, that is an escalation trigger — stop and report.

## Header syntax gate (touched ROS-free headers)

```bash
g++ -std=c++20 -Wall -Wextra -Wpedantic -fsyntax-only -I/usr/include/eigen3 \
    -Iros2_ws/src/slam_core/include -Iros2_ws/src/<pkg>/include <header>
```

## Full ROS build (only where ROS 2 Jazzy exists: on-target, CI, or Docker)

```bash
./scripts/build.sh            # runs core tests, then colcon if available
docker build -f docker/Dockerfile.jetson .   # hermetic: tests + colcon inside image
```

`build.sh --cuda` adds `-DENABLE_CUDA=ON -DENABLE_TENSORRT=ON` (Jetson only).

## Python (launch files)

Black, line length 120 (repo pre-commit enforces): `black --line-length=120 <files>`.

## Reporting format

State exactly which gates ran and paste the final ctest/colcon summary lines.
"Tests pass" without pasted output does not count as verification.
