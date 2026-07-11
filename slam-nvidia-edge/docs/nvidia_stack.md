# NVIDIA stack on Jetson AGX Orin

## Roles

| Layer | Used for | Where in this repo |
|---|---|---|
| CUDA | Streams for async preprocessing + inference, custom kernels later | `tensor_frontend_node` `CudaStream` abstraction |
| TensorRT | SuperPoint/LightGlue-style feature frontend engine | `tensor_frontend_node` `InferenceEngine` / `TrtEngine` |
| Isaac ROS | Hardware-accelerated ROS graph pieces (image_proc, visual_slam reference, nvblox) | drop-in alternatives documented below |
| NITROS | Zero-copy GPU transport between Isaac ROS nodes | adapter boundary in `tensor_frontend_node` |
| Argus / GXF | Camera capture pipeline | driver layer, outside this repo |

## tensor_frontend_node design

- `InferenceEngine` is a pure-virtual interface (`infer(image) -> features`).
  The default `NullInferenceEngine` returns an empty feature set so the whole
  stack compiles and runs without a model file or TensorRT installed.
- `TrtEngine` is compiled only when `ENABLE_TENSORRT` is defined (CMake option
  `-DENABLE_TENSORRT=ON` on a Jetson with TensorRT headers). It loads a
  serialized `.engine` or builds one from ONNX.
- `CudaStream` wraps `cudaStream_t` behind RAII; without CUDA it degrades to a
  no-op handle so host builds still compile.

## Engine lifecycle

1. SageMaker training job exports ONNX → model registry (cloud, see
   `docs/sagemaker_boundary.md`).
2. On the Jetson: `trtexec --onnx=frontend.onnx --saveEngine=frontend.engine --fp16`
   (engines are device/JetPack-specific — always build on-target).
3. `configs/nvidia_runtime.yaml` points `tensor_frontend_node` at the engine path.

## Isaac ROS interop

The skeleton nodes use vanilla `rclcpp` so they run anywhere. On Orin you can
substitute accelerated components without touching downstream nodes:

- `isaac_ros_image_proc` for rectification/debayer ahead of `vio_node`.
- `isaac_ros_visual_slam` (cuVSLAM) as a reference VIO to A/B against `vio_node`.
- `nvblox` alongside `map_node` for 3D reconstruction.

When inserting Isaac ROS nodes, keep them NITROS-adjacent (composed in the same
container process) so image transport stays zero-copy; the boundary back to
this stack is plain ROS messages.

## Jetson runtime settings

- `nvpmodel -m 0 && jetson_clocks` for MAXN during benchmarking.
- Pin the estimation nodes off the cores handling interrupts
  (see `docker-compose.edge.yaml` `cpuset`).
- Prefer `NVMe` for `logs/` — MCAP recording at LiDAR rates will saturate eMMC.
