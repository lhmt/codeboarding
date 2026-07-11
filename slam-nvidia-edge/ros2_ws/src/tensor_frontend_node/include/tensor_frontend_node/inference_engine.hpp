#pragma once

// Feature-frontend inference interface (SuperPoint/LightGlue-style).
//
// EXTENSION POINT — TrtEngine (compiled with -DENABLE_TENSORRT=ON):
//   1. deserialize the .engine with nvinfer1::IRuntime,
//   2. allocate device buffers per binding, enqueueV3 on the CudaStream,
//   3. decode the score/descriptor heads into FrontendFeatures.
// The Null engine keeps the node runnable with no model file, no TensorRT,
// and no GPU — it reports zero keypoints, which downstream treats as
// "frontend unavailable" rather than an error.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "tensor_frontend_node/cuda_stream.hpp"

namespace tensor_frontend {

struct Keypoint {
  float u{0.0F};
  float v{0.0F};
  float score{0.0F};
};

struct FrontendFeatures {
  double timestamp{0.0};
  std::vector<Keypoint> keypoints;
  // Row-major [num_keypoints x descriptor_dim]; empty until a real engine runs.
  std::vector<float> descriptors;
  int descriptor_dim{0};
};

struct EngineConfig {
  std::string engine_path;
  std::string onnx_path;
  bool use_fp16{true};
  int max_keypoints{512};
  float score_threshold{0.005F};
  int cuda_device_id{0};
};

class InferenceEngine {
 public:
  virtual ~InferenceEngine() = default;
  virtual bool ready() const = 0;
  virtual std::string describe() const = 0;
  // Grayscale row-major image -> features. Must be synchronous with respect
  // to `stream` (implementations synchronize before returning).
  virtual FrontendFeatures infer(const uint8_t* gray, uint32_t width, uint32_t height, double timestamp,
                                 const CudaStream& stream) = 0;
};

class NullInferenceEngine final : public InferenceEngine {
 public:
  bool ready() const override { return false; }
  std::string describe() const override { return "null (no engine configured)"; }
  FrontendFeatures infer(const uint8_t* /*gray*/, uint32_t /*width*/, uint32_t /*height*/, double timestamp,
                         const CudaStream& /*stream*/) override {
    FrontendFeatures f;
    f.timestamp = timestamp;
    return f;
  }
};

// Factory: returns a TrtEngine when built with TensorRT and a usable engine
// file is configured; otherwise the Null engine.
std::unique_ptr<InferenceEngine> makeInferenceEngine(const EngineConfig& config);

}  // namespace tensor_frontend
