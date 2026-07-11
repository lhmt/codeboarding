#include "tensor_frontend_node/inference_engine.hpp"

#ifdef ENABLE_TENSORRT
#include <NvInfer.h>
#endif

namespace tensor_frontend {

#ifdef ENABLE_TENSORRT
// Minimal TensorRT-backed engine. Deserializes a prebuilt .engine (build it
// on-target with trtexec; see docs/nvidia_stack.md). Inference is left as the
// marked extension point so this file links against TensorRT without yet
// claiming real feature output.
class TrtEngine final : public InferenceEngine {
 public:
  explicit TrtEngine(const EngineConfig& config) : config_(config) {
    // TODO(trt): nvinfer1::createInferRuntime + deserializeCudaEngine from
    // config_.engine_path, create execution context, allocate bindings.
  }

  bool ready() const override { return false; }  // flips true once deserialization lands
  std::string describe() const override { return "tensorrt: " + config_.engine_path; }

  FrontendFeatures infer(const uint8_t* /*gray*/, uint32_t /*width*/, uint32_t /*height*/, double timestamp,
                         const CudaStream& stream) override {
    FrontendFeatures f;
    f.timestamp = timestamp;
    // TODO(trt): H2D copy on stream.handle(), enqueueV3, D2H copy, decode
    // score map -> top-k keypoints above config_.score_threshold.
    stream.synchronize();
    return f;
  }

 private:
  EngineConfig config_;
};
#endif

std::unique_ptr<InferenceEngine> makeInferenceEngine(const EngineConfig& config) {
#ifdef ENABLE_TENSORRT
  if (!config.engine_path.empty()) {
    return std::make_unique<TrtEngine>(config);
  }
#else
  (void)config;
#endif
  return std::make_unique<NullInferenceEngine>();
}

}  // namespace tensor_frontend
