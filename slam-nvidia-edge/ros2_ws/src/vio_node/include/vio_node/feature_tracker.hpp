#pragma once

// Placeholder feature tracker.
// EXTENSION POINT: replace with (a) tensor_frontend_node output (SuperPoint +
// LightGlue over /slam/frontend/features) or (b) a CPU KLT/ORB tracker. The
// TrackResult contract (ids, pixel positions, quality metrics) is stable.

#include <cmath>
#include <cstdint>
#include <vector>

#include "vio_node/image_preprocessor.hpp"

namespace vio_node {

struct TrackedFeature {
  uint64_t id{0};
  float u{0.0F};
  float v{0.0F};
};

struct TrackResult {
  double timestamp{0.0};
  std::vector<TrackedFeature> features;
  int num_tracked_from_previous{0};
  double median_reprojection_error_px{0.0};
};

class FeatureTracker {
 public:
  explicit FeatureTracker(int max_features = 200) : max_features_(max_features) {}

  // Deterministic grid "detections" so the downstream pipeline exercises real
  // data shapes. Reports full re-detection each frame (no true tracking yet).
  TrackResult track(const PreprocessedImage& image) {
    TrackResult result;
    result.timestamp = image.timestamp;
    if (image.width == 0 || image.height == 0) return result;

    const int grid = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(max_features_))));
    const float step_u = static_cast<float>(image.width) / static_cast<float>(grid + 1);
    const float step_v = static_cast<float>(image.height) / static_cast<float>(grid + 1);
    for (int gy = 1; gy <= grid && static_cast<int>(result.features.size()) < max_features_; ++gy) {
      for (int gx = 1; gx <= grid && static_cast<int>(result.features.size()) < max_features_; ++gx) {
        result.features.push_back({next_id_++, step_u * static_cast<float>(gx), step_v * static_cast<float>(gy)});
      }
    }
    result.num_tracked_from_previous = had_previous_ ? static_cast<int>(result.features.size()) : 0;
    result.median_reprojection_error_px = 0.0;  // real tracker fills this from KLT/matcher residuals
    had_previous_ = true;
    return result;
  }

 private:
  int max_features_;
  uint64_t next_id_{0};
  bool had_previous_{false};
};

}  // namespace vio_node
