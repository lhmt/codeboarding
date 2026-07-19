# Implementation Plan: Skeleton → Production SLAM Stack

Status of the codebase after PR #1: every node builds and runs with the full
topic/health/launch topology in place, but each algorithmic core is a
dimension-correct placeholder marked with `TODO(estimator)`, `TODO(lio)`,
`TODO(fusion)`, or `TODO(trt)`. This plan sequences the replacement of those
placeholders into six phases. Phases 2–4 are independent of each other once
Phase 1 lands; Phase 5 unblocks the learned frontend; Phase 6 runs continuously
from Phase 2 onward.

## Phase 1 — Math foundations (blocks everything else)

The estimator math in `slam_core` is shared by VIO, LIO, and fusion, so it
hardens first.

1. **Replace the SO(3) placeholder with Sophus.**
   `ros2_ws/src/slam_core/include/slam_core/lie_group.hpp` is deliberately
   Sophus-API-compatible, and `docker/Dockerfile.jetson` already stages the
   Sophus headers. Swap the implementation, keep the existing
   `tests/test_lie_group.cpp` as the regression gate, and add exp/log
   round-trip and adjoint tests near the singularity (small-angle) branch.
2. **Complete IMU preintegration covariance propagation.** *Done.*
   `slam_core/imu_preintegration.hpp` propagates the 15×15 covariance and the
   d(delta)/d(bias) Jacobians per Forster et al., with `biasCorrectedDelta()`
   for first-order re-linearization; validated in
   `tests/test_imu_preintegration.cpp` against full re-integration at a
   perturbed bias and by positive-definiteness of the covariance.
3. **Finalize factor residuals.** *Done.*
   `slam_core/factor_residuals.hpp` now has the full between-states IMU factor
   (bias-corrected deltas vs. gravity-compensated state deltas over a
   `NavState` pair) and the analytic reprojection Jacobian, both checked
   against numeric references in `tests/test_factor_residuals.cpp`.
4. **Pin the Jetson base image.** Resolve the `TODO` in
   `docker/Dockerfile.jetson` and `docker/docker-compose.edge.yaml` to the
   JetPack release matching the target fleet (JetPack 6 / L4T r36.x for AGX
   Orin + ROS 2 Jazzy).

Exit criteria: all ROS-free tests pass off-robot (`cmake -S tests -B
tests/build && ctest`), covariance propagation matches numeric reference to
1e-6, Docker image builds on the pinned base.

## Phase 2 — LIO to a working odometry

LIO first among the estimators: it degrades more gracefully than VIO and
produces the local map that `map_node` and fusion consume.

1. **Scan deskew** (`lio_node/scan_deskew.hpp`, `TODO(lio)`): slerp
   Identity→rotation-over-scan by per-point time fraction using the IMU
   rotation delta from `slam_core` preintegration.
2. **Registration**: replace `point_to_plane_icp.hpp` with a real registration
   backend. Recommended: `small_gicp` (header-friendly, OpenMP, proven on
   Orin-class CPUs) behind the existing interface, with CUDA VGICP as a later
   drop-in. Keep the placeholder as a fallback selected by config.
3. **Degeneracy handling** (`degeneracy_detector.hpp`, `TODO(lio)`): compute
   the eigen-spectrum of the true ICP Hessian (available from the new
   registration backend) and project the update out of degenerate directions;
   wire the condition number into the existing `PoseHealth` fields.
4. **Local map manager** (`local_map_manager.hpp`): incremental voxel map with
   sliding spatial window; publish through the existing `map_node` relay.

Exit criteria: on a recorded MCAP corridor sequence, LIO produces a
non-diverging trajectory; degenerate geometry (long corridor) trips
`is_degenerate` without the estimate exploding; `run_lio.sh` runs at sensor
rate on Orin.

## Phase 3 — VIO to a working odometry

1. **Feature tracking**: keep the placeholder tracker
   (`vio_node/feature_tracker.hpp`) as the CPU fallback; make the primary path
   consume `tensor_frontend_node` output once Phase 5 lands. Interface already
   allows both (see `configs/camera_imu.yaml` note).
2. **Sliding-window estimator** (`sliding_window_estimator.hpp`): replace the
   placeholder with a fixed-lag smoother. Recommended: GTSAM
   `IncrementalFixedLagSmoother` with the Phase 1 IMU factors; implement
   marginalization of the oldest keyframe (`TODO(estimator)`) instead of
   dropping it. Ceres is the fallback if GTSAM's Jetson packaging fights back.
3. **Observability**: replace the feature-count conditioning proxy with the
   real marginal covariance from the smoother, feeding the existing
   `EstimatorState` covariance fields (currently placeholder per
   `slam_interfaces/msg/EstimatorState.msg`).

Exit criteria: VIO tracks through a handheld/robot sequence with bounded
drift; `is_lost` triggers on cover-the-camera tests and recovers; smoother
solve time fits the frame budget on Orin (profile with `ros2 topic hz` +
recorder timing fields).

## Phase 4 — Fusion backend

1. **Error-state EKF** implementing the existing `FusionBackend` interface
   (`fusion_node/fusion_backend.hpp`), replacing the weighted average. The
   config key already selects backends (`configs/fusion.yaml`); the warning at
   `fusion_node.cpp:40` ("not implemented yet") goes away.
2. **GPS anchoring** (`fusion_node.cpp`, `TODO(fusion)`): geodetic → local ENU
   conversion anchored at the first fix, with covariance gating before the
   update.
3. **Wheel-odom alignment** (`TODO(fusion)`): estimate the wheel-frame
   alignment online (or from calibration) before fusing; reject when slip is
   detected via chi-square gating.

Exit criteria: fused output stays continuous across single-estimator dropouts
(kill VIO mid-run → fusion degrades to LIO without a pose jump); health
transitions in `/slam/health/global` match `configs/health.yaml` rules.

## Phase 5 — TensorRT frontend

1. **Real `InferenceEngine`** (`tensor_frontend_node/inference_engine.cpp`,
   `TODO(trt)`): `createInferRuntime` + `deserializeCudaEngine`, H2D copy on
   the existing `cuda_stream.hpp` stream, `enqueueV3`, D2H, decode keypoints +
   descriptors. Engine path and precision (FP16 first) from
   `configs/nvidia_runtime.yaml`.
2. **Model artifact contract**: consume a SuperPoint/LightGlue-style ONNX from
   the SageMaker registry (see `docs/sagemaker_boundary.md`), convert to a
   TensorRT engine at deploy time on-device (engines are not portable across
   TensorRT versions).
3. **NITROS/Isaac ROS**: once correctness is proven with vanilla `rclcpp`,
   adopt NITROS zero-copy transport between camera, frontend, and VIO per
   `docs/nvidia_stack.md`.

Exit criteria: frontend sustains camera rate at FP16 on Orin with GPU util
headroom; VIO quality with learned features ≥ CPU fallback on the evaluation
set.

## Phase 6 — Validation and operations (continuous)

- **Replay evaluation harness**: script that replays recorded MCAP bags
  (`scripts/record_mcap.sh` output) through each launch profile and computes
  ATE/RPE against the recorded TUM trajectories from `recorder_node`. This is
  the regression gate for every phase above.
- **CI**: build the ROS-free `slam_core` tests on every PR (no Jetson
  required); nightly full colcon build in the Jetson container via emulation
  or a self-hosted Orin runner.
- **Health threshold tuning**: revisit `configs/health.yaml` thresholds with
  data from the replay harness once real estimators produce real condition
  numbers and reprojection errors.
- **Latency budget tracking**: extend `recorder_node` JSONL with per-node
  processing time so regressions show up in recorded runs.

## Sequencing summary

```
Phase 1 (slam_core + docker pin)
   ├──► Phase 2 (LIO)   ──┐
   ├──► Phase 3 (VIO)   ──┼──► Phase 4 (fusion EKF, needs ≥1 estimator real)
   └──► Phase 5 (TensorRT frontend, feeds Phase 3 step 1)
Phase 6 runs alongside from Phase 2 onward.
```

Phase 1 items 2–3 landed with this plan (pure `slam_core` + tests, no ROS
dependency, verified off-robot). Remaining Phase 1: the Sophus swap (item 1)
and the Jetson base-image pin (item 4).
