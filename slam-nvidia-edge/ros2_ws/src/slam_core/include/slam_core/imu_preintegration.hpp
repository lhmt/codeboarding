#pragma once

// On-manifold IMU preintegration (Forster et al., RSS'15) — accumulation only.
// Covariance propagation and first-order bias-update Jacobians are the marked
// extension points; the delta terms here are exact for piecewise-constant
// measurements and are what the sliding-window / LIO estimators consume.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "slam_core/lie_group.hpp"

namespace slam_core {

struct ImuSample {
  double timestamp{0.0};       // seconds
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};   // rad/s, body frame
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};  // m/s^2, body frame
};

struct ImuBias {
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

class ImuPreintegrator {
 public:
  explicit ImuPreintegrator(const ImuBias& bias = {}) : bias_(bias) {}

  // Integrate one sample held constant over dt. dt <= 0 is rejected so a
  // repeated or out-of-order timestamp cannot corrupt the deltas.
  bool integrate(const ImuSample& sample, double dt) {
    if (dt <= 0.0) return false;
    const Eigen::Vector3d w = sample.gyro - bias_.gyro;
    const Eigen::Vector3d a = sample.accel - bias_.accel;

    // Order matters: position uses the velocity/rotation at interval start.
    delta_p_ += delta_v_ * dt + 0.5 * delta_r_ * a * dt * dt;
    delta_v_ += delta_r_ * a * dt;
    delta_r_ = delta_r_ * expSO3(w * dt);

    // TODO(estimator): propagate 15x15 preintegration covariance and the
    // d(delta)/d(bias) Jacobians here (Forster eq. 62-64).

    dt_sum_ += dt;
    ++num_samples_;
    return true;
  }

  void reset(const ImuBias& bias) {
    bias_ = bias;
    delta_r_.setIdentity();
    delta_v_.setZero();
    delta_p_.setZero();
    dt_sum_ = 0.0;
    num_samples_ = 0;
  }

  const Eigen::Matrix3d& deltaRotation() const { return delta_r_; }
  const Eigen::Vector3d& deltaVelocity() const { return delta_v_; }
  const Eigen::Vector3d& deltaPosition() const { return delta_p_; }
  const ImuBias& bias() const { return bias_; }
  double deltaTime() const { return dt_sum_; }
  std::size_t numSamples() const { return num_samples_; }

 private:
  ImuBias bias_;
  Eigen::Matrix3d delta_r_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d delta_v_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_p_{Eigen::Vector3d::Zero()};
  double dt_sum_{0.0};
  std::size_t num_samples_{0};
};

}  // namespace slam_core
