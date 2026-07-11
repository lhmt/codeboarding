#pragma once

// Deterministic health threshold rules, shared by health_monitor_node and the
// per-estimator health publishers. Pure logic: no ROS, no clock, no I/O.

#include <string>
#include <vector>

namespace slam_core {

struct HealthThresholds {
  int min_tracked_features{30};
  double max_reprojection_error_median{3.0};  // px
  double max_gyro_bias_norm{0.1};             // rad/s
  double max_accel_bias_norm{1.0};            // m/s^2
  double max_condition_number{1e10};
};

struct HealthSample {
  int tracked_features{0};
  double reprojection_error_median{0.0};
  double gyro_bias_norm{0.0};
  double accel_bias_norm{0.0};
  double condition_number{1.0};
  bool is_lost{false};
};

struct HealthVerdict {
  std::vector<std::string> warnings;  // deterministic order (rule order below)
  bool is_degenerate{false};
  bool is_lost{false};

  bool ok() const { return warnings.empty() && !is_lost; }

  // Joined status string for PoseHealth.status. LOST dominates.
  std::string status() const {
    if (is_lost) return "LOST";
    if (warnings.empty()) return "OK";
    std::string out;
    for (const auto& w : warnings) {
      if (!out.empty()) out += ",";
      out += w;
    }
    return out;
  }
};

inline HealthVerdict evaluateHealth(const HealthSample& s, const HealthThresholds& t = {}) {
  HealthVerdict v;
  if (s.is_lost) {
    v.is_lost = true;
    return v;
  }
  if (s.tracked_features < t.min_tracked_features) v.warnings.emplace_back("WARN_TRACKING");
  if (s.reprojection_error_median > t.max_reprojection_error_median) v.warnings.emplace_back("WARN_REPROJECTION");
  if (s.gyro_bias_norm > t.max_gyro_bias_norm) v.warnings.emplace_back("WARN_IMU_GYRO");
  if (s.accel_bias_norm > t.max_accel_bias_norm) v.warnings.emplace_back("WARN_IMU_ACCEL");
  if (s.condition_number > t.max_condition_number) {
    v.warnings.emplace_back("WARN_DEGENERACY");
    v.is_degenerate = true;
  }
  return v;
}

}  // namespace slam_core
