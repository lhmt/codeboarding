#pragma once

// Sliding-window estimator placeholder.
// EXTENSION POINT: replace update() internals with a fixed-lag smoother
// (GTSAM ISAM2 / Ceres) over ImuFactorResidual + ReprojectionFactorResidual
// (slam_core/factor_residuals.hpp). State layout and health outputs are final.

#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam_core/imu_preintegration.hpp"
#include "slam_core/lie_group.hpp"
#include "vio_node/feature_tracker.hpp"

namespace vio_node {

struct EstimatorOutput {
  double timestamp{0.0};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};   // world <- body
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  slam_core::ImuBias bias;
  double condition_number{1.0};
  bool is_lost{false};
};

class SlidingWindowEstimator {
 public:
  explicit SlidingWindowEstimator(std::size_t window_size, double gravity_magnitude)
      : window_size_(window_size), gravity_(0.0, 0.0, -gravity_magnitude) {}

  // Dead-reckons the preintegrated IMU delta from the last state. A real
  // backend would instead add a keyframe + factors and re-optimize the window.
  EstimatorOutput update(const TrackResult& tracks, const slam_core::ImuPreintegrator& preint, bool is_keyframe) {
    const double dt = preint.deltaTime();
    EstimatorOutput out = state_;
    out.timestamp = tracks.timestamp;
    if (dt > 0.0) {
      out.position = state_.position + state_.velocity * dt + 0.5 * gravity_ * dt * dt +
                     state_.rotation * preint.deltaPosition();
      out.velocity = state_.velocity + gravity_ * dt + state_.rotation * preint.deltaVelocity();
      out.rotation = state_.rotation * preint.deltaRotation();
    }
    // Placeholder observability proxy: fewer features => worse conditioning.
    out.condition_number = tracks.features.empty() ? 1e12 : 1e4 / static_cast<double>(tracks.features.size());
    out.is_lost = tracks.features.empty();

    if (is_keyframe) {
      window_.push_back(out);
      while (window_.size() > window_size_) window_.pop_front();
      // TODO(estimator): marginalize the oldest keyframe instead of dropping it.
    }
    state_ = out;
    return out;
  }

  std::size_t windowSize() const { return window_.size(); }

 private:
  std::size_t window_size_;
  Eigen::Vector3d gravity_;
  EstimatorOutput state_;
  std::deque<EstimatorOutput> window_;
};

}  // namespace vio_node
