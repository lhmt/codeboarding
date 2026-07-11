#pragma once

// SO(3) exponential/logarithm with small-angle-safe branches.
// Placeholder for a full Sophus dependency; keep the API Sophus-compatible so
// swapping to Sophus::SO3d later is mechanical.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace slam_core {

inline constexpr double kSmallAngleThreshold = 1e-8;

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

inline Eigen::Vector3d vee(const Eigen::Matrix3d& m) {
  return {m(2, 1), m(0, 2), m(1, 0)};
}

// Exp: so(3) -> SO(3). Rodrigues with Taylor fallback near zero.
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega) {
  const double theta2 = omega.squaredNorm();
  const Eigen::Matrix3d W = hat(omega);
  if (theta2 < kSmallAngleThreshold * kSmallAngleThreshold) {
    // R ≈ I + W + 0.5 W² keeps orthogonality to O(θ³).
    return Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
  }
  const double theta = std::sqrt(theta2);
  return Eigen::Matrix3d::Identity() + (std::sin(theta) / theta) * W +
         ((1.0 - std::cos(theta)) / theta2) * W * W;
}

// Log: SO(3) -> so(3).
inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  const double trace = R.trace();
  const double cos_theta = std::clamp(0.5 * (trace - 1.0), -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  const Eigen::Vector3d w = vee(R - R.transpose());
  if (theta < kSmallAngleThreshold) {
    return 0.5 * w;
  }
  if (theta > M_PI - 1e-6) {
    // Near π the antisymmetric part vanishes; recover the axis from R + I.
    Eigen::Vector3d axis;
    const Eigen::Matrix3d S = 0.5 * (R + Eigen::Matrix3d::Identity());
    axis = S.diagonal().cwiseMax(0.0).cwiseSqrt();
    // Fix signs from the off-diagonal terms relative to the largest component.
    int k = 0;
    axis.maxCoeff(&k);
    for (int i = 0; i < 3; ++i) {
      if (i != k && S(k, i) < 0.0) axis[i] = -axis[i];
    }
    return theta * axis.normalized();
  }
  return (theta / (2.0 * std::sin(theta))) * w;
}

// Right Jacobian of SO(3); used by IMU preintegration bias corrections.
inline Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& omega) {
  const double theta2 = omega.squaredNorm();
  const Eigen::Matrix3d W = hat(omega);
  if (theta2 < kSmallAngleThreshold * kSmallAngleThreshold) {
    return Eigen::Matrix3d::Identity() - 0.5 * W;
  }
  const double theta = std::sqrt(theta2);
  return Eigen::Matrix3d::Identity() - ((1.0 - std::cos(theta)) / theta2) * W +
         ((theta - std::sin(theta)) / (theta2 * theta)) * W * W;
}

}  // namespace slam_core
