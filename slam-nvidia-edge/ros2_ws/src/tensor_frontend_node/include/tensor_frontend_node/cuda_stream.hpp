#pragma once

// RAII CUDA stream abstraction. Compiles without the CUDA toolkit: when
// ENABLE_CUDA is off the handle is a no-op so host builds and CI stay green.
// Build with -DENABLE_CUDA=ON on Jetson to get a real cudaStream_t.

#ifdef ENABLE_CUDA
#include <cuda_runtime.h>
#endif

namespace tensor_frontend {

class CudaStream {
 public:
  CudaStream() {
#ifdef ENABLE_CUDA
    cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
#endif
  }

  ~CudaStream() {
#ifdef ENABLE_CUDA
    if (stream_ != nullptr) cudaStreamDestroy(stream_);
#endif
  }

  CudaStream(const CudaStream&) = delete;
  CudaStream& operator=(const CudaStream&) = delete;
  CudaStream(CudaStream&&) = delete;
  CudaStream& operator=(CudaStream&&) = delete;

  void synchronize() const {
#ifdef ENABLE_CUDA
    cudaStreamSynchronize(stream_);
#endif
  }

#ifdef ENABLE_CUDA
  cudaStream_t handle() const { return stream_; }
#else
  void* handle() const { return nullptr; }
#endif

 private:
#ifdef ENABLE_CUDA
  cudaStream_t stream_{nullptr};
#endif
};

}  // namespace tensor_frontend
