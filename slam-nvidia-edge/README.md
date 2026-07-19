# slam-nvidia-edge

Production-grade local VIO/LIO SLAM stack for NVIDIA Jetson AGX Orin.

- **Target platform:** NVIDIA Jetson AGX Orin, Ubuntu 24.04, ROS 2 Jazzy
- **Acceleration:** CUDA / TensorRT / Isaac ROS / NITROS (feature frontend, point cloud ops)
- **Boundary:** All localization runs locally. Cloud (AWS SageMaker) is used *only* for
  training, evaluation, and model registry. No cloud dependency exists in the
  real-time estimation loop.

## Layout

```
slam-nvidia-edge/
├── docker/                  # Jetson container build + compose
├── ros2_ws/src/
│   ├── slam_interfaces/     # PoseHealth, EstimatorState, SensorSyncStatus msgs
│   ├── slam_core/           # Header-only, ROS-free estimator math (Eigen only)
│   ├── vio_node/            # Camera + IMU visual-inertial odometry skeleton
│   ├── lio_node/            # LiDAR + IMU odometry skeleton
│   ├── fusion_node/         # VIO + LIO + GPS/wheel-odom fusion
│   ├── map_node/            # Local map aggregation / serving
│   ├── health_monitor_node/ # Global health aggregation and rules
│   ├── tensor_frontend_node/# TensorRT feature frontend (SuperPoint/LightGlue-style)
│   ├── recorder_node/       # Local log artifacts for later cloud upload
│   └── slam_bringup/        # Launch files
├── configs/                 # ROS parameter YAML files
├── scripts/                 # build / run / record helpers
├── tests/                   # ROS-free unit tests for slam_core (plain CMake + ctest)
└── docs/                    # architecture, dataflow, NVIDIA stack, SageMaker boundary, plan
```

`slam_core` is one addition on top of the base layout: shared math (SO(3),
IMU preintegration, health rules, factor residuals) lives in exactly one
header-only package instead of being duplicated across nodes. It has no ROS
dependency, which keeps the estimator math unit-testable off-robot.

## Build

On the Jetson (or inside `docker/Dockerfile.jetson`):

```bash
./scripts/build.sh            # colcon build + ROS-free core tests
```

Off-robot (no ROS 2 required) you can still build and run the estimator math tests:

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
```

## Run

```bash
./scripts/run_vio.sh          # camera + IMU only
./scripts/run_lio.sh          # LiDAR + IMU only
./scripts/run_full_stack.sh   # VIO + LIO + fusion + map + health + recorder
./scripts/record_mcap.sh      # record sensor + SLAM topics to logs/mcap/
```

Config files are resolved through `SLAM_CONFIG_DIR` (defaults to this repo's
`configs/`; the Docker image sets it to the baked-in copy).

## Health model

Every estimator publishes `slam_interfaces/PoseHealth`. `health_monitor_node`
aggregates them into `/slam/health/global` using deterministic threshold rules
(see `configs/health.yaml` and `docs/architecture.md`). Downstream consumers
(planner, safety layer) should gate on `is_lost` / `is_degenerate` / `status`.

## Cloud boundary

`recorder_node` writes MCAP bags, health JSONL, TUM trajectories, and
calibration snapshots under `logs/`. A separate, out-of-scope uploader ships
those to S3 for SageMaker training/evaluation. **No AWS credentials or SDK
appear anywhere in this runtime.** See `docs/sagemaker_boundary.md`.
