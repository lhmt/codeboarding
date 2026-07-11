#pragma once

// Scan deskew placeholder.
// EXTENSION POINT: with per-point timestamps (ring/time fields), rotate each
// point by the interpolated IMU rotation over the scan period. Until then the
// scan passes through untouched; the interface already carries the motion.

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lio_node {

class ScanDeskewer {
 public:
  // rotation_over_scan: body rotation accumulated during the sweep
  // (from IMU preintegration); scan_period the sweep duration.
  std::vector<Eigen::Vector3d> deskew(std::vector<Eigen::Vector3d> points,
                                      const Eigen::Matrix3d& rotation_over_scan, double scan_period) const {
    (void)rotation_over_scan;
    (void)scan_period;
    // TODO(lio): slerp Identity..rotation_over_scan by per-point time fraction
    // and unrotate each point into the scan-end frame.
    return points;
  }
};

}  // namespace lio_node
