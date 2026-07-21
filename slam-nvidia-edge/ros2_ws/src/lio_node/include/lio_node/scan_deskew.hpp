#pragma once

// Scan deskew: rotation-only correction using per-point time fractions.
// EXTENSION POINT: translation deskew (lever-arm motion during the sweep) is
// a future extension; only rotation is corrected here.

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam_core/lie_group.hpp"

namespace lio_node {

class ScanDeskewer {
 public:
  // rotation_over_scan: body rotation accumulated during the sweep
  // (from IMU preintegration); scan_period the sweep duration.
  // Passthrough: no per-point time fields available (driver limitation).
  std::vector<Eigen::Vector3d> deskew(std::vector<Eigen::Vector3d> points,
                                      const Eigen::Matrix3d& rotation_over_scan, double scan_period) const {
    (void)rotation_over_scan;
    (void)scan_period;
    return points;
  }

  // time_fractions[i]: capture time of points[i] as a fraction of the sweep,
  // 0 = scan start, 1 = scan end. Rotates each point into the scan-end frame
  // assuming constant angular velocity over the sweep (no translation term
  // yet; see EXTENSION POINT above).
  // Falls back to passthrough if sizes mismatch (deterministic degradation).
  std::vector<Eigen::Vector3d> deskew(std::vector<Eigen::Vector3d> points,
                                      const std::vector<double>& time_fractions,
                                      const Eigen::Matrix3d& rotation_over_scan) const {
    if (time_fractions.size() != points.size()) {
      return points;
    }
    const Eigen::Vector3d phi = slam_core::logSO3(rotation_over_scan);
    for (std::size_t i = 0; i < points.size(); ++i) {
      const double tau = std::clamp(time_fractions[i], 0.0, 1.0);
      const Eigen::Matrix3d r = slam_core::expSO3(-(1.0 - tau) * phi);
      points[i] = r * points[i];
    }
    return points;
  }
};

}  // namespace lio_node
