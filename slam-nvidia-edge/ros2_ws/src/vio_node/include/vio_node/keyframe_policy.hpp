#pragma once

// Keyframe insertion policy: time OR motion triggered, whichever fires first.

#include <cmath>

#include <Eigen/Core>

namespace vio_node {

class KeyframePolicy {
 public:
  KeyframePolicy(double min_interval_s, double min_translation_m, double min_rotation_rad)
      : min_interval_s_(min_interval_s),
        min_translation_m_(min_translation_m),
        min_rotation_rad_(min_rotation_rad) {}

  bool shouldInsert(double timestamp, const Eigen::Vector3d& translation_since_last, double rotation_since_last) {
    if (!have_keyframe_) {
      accept(timestamp);
      return true;
    }
    const bool time_ok = (timestamp - last_keyframe_time_) >= min_interval_s_;
    const bool moved = translation_since_last.norm() >= min_translation_m_ ||
                       std::abs(rotation_since_last) >= min_rotation_rad_;
    if (time_ok || moved) {
      accept(timestamp);
      return true;
    }
    return false;
  }

 private:
  void accept(double timestamp) {
    last_keyframe_time_ = timestamp;
    have_keyframe_ = true;
  }

  double min_interval_s_;
  double min_translation_m_;
  double min_rotation_rad_;
  double last_keyframe_time_{0.0};
  bool have_keyframe_{false};
};

}  // namespace vio_node
