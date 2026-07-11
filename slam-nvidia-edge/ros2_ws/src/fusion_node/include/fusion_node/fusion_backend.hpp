#pragma once

// Fusion backend interface.
// EXTENSION POINT: implement EkfBackend (error-state EKF over SE(3)+velocity
// +biases) or FactorGraphBackend (fixed-lag smoother) against this same
// interface; select via the `backend` parameter in configs/fusion.yaml.

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fusion_node {

struct SourceState {
  std::string name;  // "vio", "lio", "gps", "wheel"
  double timestamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  double weight{0.0};
  bool has_orientation{true};  // GPS provides position only
};

struct FusedState {
  double timestamp{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  int num_sources{0};
};

class FusionBackend {
 public:
  virtual ~FusionBackend() = default;
  // Sources are pre-filtered to alive (non-stale) entries by the node.
  virtual std::optional<FusedState> fuse(const std::vector<SourceState>& sources) = 0;
};

// Weighted average placeholder: renormalizes weights over alive sources.
// Orientation: sign-aligned normalized quaternion blend (adequate for nearby
// estimates; a real backend fuses on the manifold with covariances).
class WeightedAverageBackend final : public FusionBackend {
 public:
  std::optional<FusedState> fuse(const std::vector<SourceState>& sources) override {
    double weight_sum = 0.0;
    for (const auto& s : sources) weight_sum += s.weight;
    if (sources.empty() || weight_sum <= 0.0) return std::nullopt;

    FusedState out;
    Eigen::Vector4d q_acc = Eigen::Vector4d::Zero();
    double orientation_weight = 0.0;
    const Eigen::Quaterniond* reference = nullptr;
    for (const auto& s : sources) {
      const double w = s.weight / weight_sum;
      out.position += w * s.position;
      out.velocity += w * s.velocity;
      out.timestamp = std::max(out.timestamp, s.timestamp);
      if (s.has_orientation) {
        Eigen::Vector4d q = s.orientation.coeffs();
        if (reference == nullptr) reference = &s.orientation;
        if (reference->coeffs().dot(q) < 0.0) q = -q;  // hemisphere alignment
        q_acc += w * q;
        orientation_weight += w;
      }
      ++out.num_sources;
    }
    if (orientation_weight > 0.0 && q_acc.norm() > 1e-12) {
      out.orientation = Eigen::Quaterniond(q_acc.normalized());
    }
    return out;
  }
};

}  // namespace fusion_node
