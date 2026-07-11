# SageMaker boundary

## Hard rule

**SageMaker never runs, gates, or influences the real-time localization loop.**
The Jetson stack is fully functional with zero network connectivity. Cloud
interaction is batch, asynchronous, and one file-drop away from the runtime.

## What runs where

| Concern | Edge (Jetson, this repo) | Cloud (SageMaker, out of scope here) |
|---|---|---|
| VIO/LIO/fusion estimation | ✔ real-time | ✘ never |
| Feature frontend inference | ✔ TensorRT engine | ✘ |
| Health monitoring | ✔ | ✘ |
| Log/bag capture | ✔ `recorder_node` → `logs/` | ✘ |
| Log upload | ✘ (separate uploader service) | S3 ingest |
| Frontend model training | ✘ | ✔ SageMaker training jobs |
| Trajectory evaluation (ATE/RPE vs. reference) | ✘ | ✔ SageMaker processing jobs |
| Model packaging/versioning | ✘ | ✔ SageMaker model registry |

## Data flow (one direction at a time)

```
edge logs/ ──(uploader, batched, retryable)──► S3
S3 ──► SageMaker training / evaluation ──► model registry (ONNX + metrics)
model registry ──(explicit, operator-approved pull)──► Jetson: ONNX → trtexec → .engine
```

## Enforcement in this repo

- No AWS SDK dependency in any `package.xml` or `CMakeLists.txt`.
- No credentials, endpoints, or bucket names in code or configs.
- `recorder_node` writes only local files under `logs/`:
  - `logs/mcap/` — sensor + SLAM topic bags (via `scripts/record_mcap.sh`)
  - `logs/health/` — `PoseHealth` samples as JSONL
  - `logs/trajectories/` — fused trajectory in TUM format (for cloud ATE/RPE)
  - `logs/calibration_snapshots/` — camera intrinsics snapshots per session
- Model updates arrive as files (`configs/nvidia_runtime.yaml` engine path);
  swapping a model is a config change + node restart, never a network call
  from the runtime.

## Why TUM-format trajectories

`timestamp tx ty tz qx qy qz qw` is what standard evaluation tooling (evo,
SageMaker processing containers running it) consumes directly — the cloud can
score every field run against references without custom parsers.
