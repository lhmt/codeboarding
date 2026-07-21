#pragma once

// On-manifold IMU preintegration (Forster et al., RSS'15).
// Implements delta accumulation, 9x9 measurement-noise covariance propagation
// (state order [dphi, dv, dp]), and first-order bias-update Jacobians
// (eq. 59-64). Bias random walk is deliberately NOT folded into this
// covariance — it belongs to the separate bias-evolution factor.

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

struct ImuNoiseParams {
  double gyro_noise_density{0.0};   // rad/s/sqrt(Hz); 0 => covariance stays zero
  double accel_noise_density{0.0};  // m/s^2/sqrt(Hz)
};

// Deltas re-expressed at a perturbed bias via the stored Jacobians.
struct CorrectedDelta {
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

class ImuPreintegrator {
 public:
  explicit ImuPreintegrator(const ImuBias& bias = {}, const ImuNoiseParams& noise = {})
      : bias_(bias), noise_(noise) {}

  // Integrate one sample held constant over dt. dt <= 0 is rejected so a
  // repeated or out-of-order timestamp cannot corrupt the deltas.
  bool integrate(const ImuSample& sample, double dt) {
    if (dt <= 0.0) return false;
    const Eigen::Vector3d w = sample.gyro - bias_.gyro;
    const Eigen::Vector3d a = sample.accel - bias_.accel;
    const Eigen::Matrix3d r_inc = expSO3(w * dt);
    const Eigen::Matrix3d jr = rightJacobianSO3(w * dt);
    const Eigen::Matrix3d a_hat = hat(a);

    // Covariance propagation: Sigma <- A Sigma A^T + B Q_d B^T, evaluated at
    // the pre-update state (delta_r_ is still Delta R_k here).
    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    A.block<3, 3>(0, 0) = r_inc.transpose();
    A.block<3, 3>(3, 0) = -delta_r_ * a_hat * dt;
    A.block<3, 3>(6, 0) = -0.5 * delta_r_ * a_hat * dt * dt;
    A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
    B.block<3, 3>(0, 0) = jr * dt;
    B.block<3, 3>(3, 3) = delta_r_ * dt;
    B.block<3, 3>(6, 3) = 0.5 * delta_r_ * dt * dt;

    // Discrete sample covariance of a white-noise density: sigma^2 / dt.
    const double gyro_var = noise_.gyro_noise_density * noise_.gyro_noise_density / dt;
    const double accel_var = noise_.accel_noise_density * noise_.accel_noise_density / dt;
    Eigen::Matrix<double, 6, 6> q = Eigen::Matrix<double, 6, 6>::Zero();
    q.topLeftCorner<3, 3>() = gyro_var * Eigen::Matrix3d::Identity();
    q.bottomRightCorner<3, 3>() = accel_var * Eigen::Matrix3d::Identity();

    covariance_ = A * covariance_ * A.transpose() + B * q * B.transpose();

    // Bias-update Jacobians (position first: it consumes the pre-update
    // velocity/rotation Jacobians; rotation last).
    j_p_bg_ += j_v_bg_ * dt - 0.5 * delta_r_ * a_hat * j_r_bg_ * dt * dt;
    j_p_ba_ += j_v_ba_ * dt - 0.5 * delta_r_ * dt * dt;
    j_v_bg_ += -delta_r_ * a_hat * j_r_bg_ * dt;
    j_v_ba_ += -delta_r_ * dt;
    j_r_bg_ = r_inc.transpose() * j_r_bg_ - jr * dt;

    // Delta accumulation (position uses interval-start velocity/rotation).
    delta_p_ += delta_v_ * dt + 0.5 * delta_r_ * a * dt * dt;
    delta_v_ += delta_r_ * a * dt;
    delta_r_ = delta_r_ * r_inc;

    dt_sum_ += dt;
    ++num_samples_;
    return true;
  }

  // First-order re-expression of the deltas at bias_ + (delta_bg, delta_ba).
  CorrectedDelta biasCorrectedDelta(const Eigen::Vector3d& delta_bg, const Eigen::Vector3d& delta_ba) const {
    CorrectedDelta out;
    out.rotation = delta_r_ * expSO3(j_r_bg_ * delta_bg);
    out.velocity = delta_v_ + j_v_bg_ * delta_bg + j_v_ba_ * delta_ba;
    out.position = delta_p_ + j_p_bg_ * delta_bg + j_p_ba_ * delta_ba;
    return out;
  }

  void reset(const ImuBias& bias) {
    bias_ = bias;
    delta_r_.setIdentity();
    delta_v_.setZero();
    delta_p_.setZero();
    covariance_.setZero();
    j_r_bg_.setZero();
    j_v_bg_.setZero();
    j_v_ba_.setZero();
    j_p_bg_.setZero();
    j_p_ba_.setZero();
    dt_sum_ = 0.0;
    num_samples_ = 0;
  }

  const Eigen::Matrix3d& deltaRotation() const { return delta_r_; }
  const Eigen::Vector3d& deltaVelocity() const { return delta_v_; }
  const Eigen::Vector3d& deltaPosition() const { return delta_p_; }
  const Eigen::Matrix<double, 9, 9>& covariance() const { return covariance_; }
  const Eigen::Matrix3d& rotationBiasJacobian() const { return j_r_bg_; }
  const Eigen::Matrix3d& velocityGyroBiasJacobian() const { return j_v_bg_; }
  const Eigen::Matrix3d& velocityAccelBiasJacobian() const { return j_v_ba_; }
  const Eigen::Matrix3d& positionGyroBiasJacobian() const { return j_p_bg_; }
  const Eigen::Matrix3d& positionAccelBiasJacobian() const { return j_p_ba_; }
  const ImuBias& bias() const { return bias_; }
  double deltaTime() const { return dt_sum_; }
  std::size_t numSamples() const { return num_samples_; }

 private:
  ImuBias bias_;
  ImuNoiseParams noise_;
  Eigen::Matrix3d delta_r_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d delta_v_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_p_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 9, 9> covariance_{Eigen::Matrix<double, 9, 9>::Zero()};
  Eigen::Matrix3d j_r_bg_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d j_v_bg_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d j_v_ba_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d j_p_bg_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d j_p_ba_{Eigen::Matrix3d::Zero()};
  double dt_sum_{0.0};
  std::size_t num_samples_{0};
};

}  // namespace slam_core
