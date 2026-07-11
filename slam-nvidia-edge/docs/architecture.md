# Architecture

## Principle: local estimation, cloud learning

The localization loop is fully local and deterministic. The cloud never sits on
the critical path:

```
┌──────────────────────────── Jetson AGX Orin (local, real-time) ────────────────────────────┐
│                                                                                            │
│  camera ──► tensor_frontend_node (TensorRT) ──► vio_node ──┐                               │
│  IMU ──────────────────────────┬──────────────► vio_node   │                               │
│                                └──────────────► lio_node ──┼──► fusion_node ──► map_node   │
│  LiDAR ────────────────────────────────────────► lio_node  │         │                     │
│  GPS / wheel odom (optional) ───────────────────────────────► fusion_node                  │
│                                                                      │                     │
│  vio/lio/fusion health ──► health_monitor_node ──► /slam/health/global                     │
│  states + health + bags ──► recorder_node ──► logs/   (local disk only)                    │
└────────────────────────────────────────────────────────────────────────────────────────────┘
                                        │ offline, batched, out of band
                                        ▼
                    S3 ──► SageMaker (training / evaluation / model registry)
                                        │
                                        ▼
                    Versioned ONNX → TensorRT engine deployed back to the edge
```

## Nodes

| Node | Role | Real-time critical |
|---|---|---|
| `tensor_frontend_node` | GPU feature extraction/matching (SuperPoint/LightGlue-style) | yes |
| `vio_node` | Camera+IMU sliding-window odometry | yes |
| `lio_node` | LiDAR+IMU scan-to-map odometry | yes |
| `fusion_node` | Combines VIO/LIO/GPS/wheel into one fused state | yes |
| `map_node` | Maintains/serves the local map | soft |
| `health_monitor_node` | Aggregates estimator health into a global verdict | soft |
| `recorder_node` | Writes local artifacts for later upload | no |

## Determinism and observability

- Runtime nodes are single-threaded (`rclcpp::spin` with default single-threaded
  executor); all processing happens in subscription/timer callbacks with
  deterministic control flow. No RNG in the estimation path.
- Every estimator publishes `PoseHealth` at state rate: tracking quality,
  feature counts, reprojection error, IMU bias norms, condition number,
  degeneracy/lost flags.
- `health_monitor_node` applies fixed threshold rules (see `configs/health.yaml`)
  and publishes a single global status for the safety layer.

## Extension points

Skeleton modules mark exactly where real algorithms slot in:

- `vio_node`: feature tracker → replace with `tensor_frontend_node` output or
  ORB-style tracker; sliding-window estimator → replace with GTSAM/Ceres
  fixed-lag smoother.
- `lio_node`: point-to-plane ICP → replace with GICP/VGICP (CUDA), degeneracy
  detector → eigenvalue analysis of the ICP Hessian.
- `fusion_node`: `FusionBackend` interface → replace weighted average with an
  error-state EKF or factor graph.
- `tensor_frontend_node`: `InferenceEngine` interface → back with a real
  TensorRT engine built from a SageMaker-produced ONNX model.
