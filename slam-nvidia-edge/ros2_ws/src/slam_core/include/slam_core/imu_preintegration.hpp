#pragma once

// On-manifold IMU preintegration (Forster et al., RSS'15) with 15x15
// covariance propagation and first-order bias-update Jacobians.
// State/error ordering: [dR (3), dv (3), dp (3), dbg (3), dba (3)].

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

// Continuous-time noise densities; discretized per-sample inside integrate().
struct ImuNoiseParams {
  double gyro_noise_density{1.7e-4};    // rad/s/sqrt(Hz)
  double accel_noise_density{2.0e-3};   // m/s^2/sqrt(Hz)
  double gyro_bias_random_walk{1.9e-5};  // rad/s^2/sqrt(Hz)
  double accel_bias_random_walk{3.0e-3}; // m/s^3/sqrt(Hz)
};

class ImuPreintegrator {
 public:
  static constexpr int kStateDim = 15;

  explicit ImuPreintegrator(const ImuBias& bias = {}, const ImuNoiseParams& noise = {})
      : bias_(bias), noise_(noise) {}

  // Integrate one sample held constant over dt. dt <= 0 is rejected so a
  // repeated or out-of-order timestamp cannot corrupt the deltas.
  bool integrate(const ImuSample& sample, double dt) {
    if (dt <= 0.0) return false;
    const Eigen::Vector3d w = sample.gyro - bias_.gyro;
    const Eigen::Vector3d a = sample.accel - bias_.accel;

    const Eigen::Matrix3d dR = expSO3(w * dt);
    const Eigen::Matrix3d Jr = rightJacobianSO3(w * dt);
    const Eigen::Matrix3d a_hat = hat(a);
    const Eigen::Matrix3d& R = delta_r_;  // ΔR_ik, pre-update
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

    // Error-state transition A (15x15) and noise map B (15x12) for
    // n = [η_g, η_a, η_bg, η_ba] (Forster eq. 62-64 plus bias random walk).
    Eigen::Matrix<double, 15, 15> A = Eigen::Matrix<double, 15, 15>::Identity();
    A.block<3, 3>(0, 0) = dR.transpose();
    A.block<3, 3>(0, 9) = -Jr * dt;
    A.block<3, 3>(3, 0) = -R * a_hat * dt;
    A.block<3, 3>(3, 12) = -R * dt;
    A.block<3, 3>(6, 0) = -0.5 * R * a_hat * dt * dt;
    A.block<3, 3>(6, 3) = I3 * dt;
    A.block<3, 3>(6, 12) = -0.5 * R * dt * dt;

    Eigen::Matrix<double, 15, 12> B = Eigen::Matrix<double, 15, 12>::Zero();
    B.block<3, 3>(0, 0) = Jr * dt;
    B.block<3, 3>(3, 3) = R * dt;
    B.block<3, 3>(6, 3) = 0.5 * R * dt * dt;
    B.block<3, 3>(9, 6) = I3;
    B.block<3, 3>(12, 9) = I3;

    // White noise discretizes as σ²/dt; bias random walk as σ²·dt.
    Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Zero();
    const double g2 = noise_.gyro_noise_density * noise_.gyro_noise_density / dt;
    const double a2 = noise_.accel_noise_density * noise_.accel_noise_density / dt;
    const double bg2 = noise_.gyro_bias_random_walk * noise_.gyro_bias_random_walk * dt;
    const double ba2 = noise_.accel_bias_random_walk * noise_.accel_bias_random_walk * dt;
    Q.diagonal() << g2, g2, g2, a2, a2, a2, bg2, bg2, bg2, ba2, ba2, ba2;

    covariance_ = A * covariance_ * A.transpose() + B * Q * B.transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();

    // Bias Jacobian recursion; position/velocity rows consume pre-update values.
    dp_dbg_ += dv_dbg_ * dt - 0.5 * R * a_hat * dr_dbg_ * dt * dt;
    dp_dba_ += dv_dba_ * dt - 0.5 * R * dt * dt;
    dv_dbg_ += -R * a_hat * dr_dbg_ * dt;
    dv_dba_ += -R * dt;
    dr_dbg_ = dR.transpose() * dr_dbg_ - Jr * dt;

    // Order matters: position uses the velocity/rotation at interval start.
    delta_p_ += delta_v_ * dt + 0.5 * delta_r_ * a * dt * dt;
    delta_v_ += delta_r_ * a * dt;
    delta_r_ = delta_r_ * dR;

    dt_sum_ += dt;
    ++num_samples_;
    return true;
  }

  void reset(const ImuBias& bias) {
    bias_ = bias;
    delta_r_.setIdentity();
    delta_v_.setZero();
    delta_p_.setZero();
    covariance_.setZero();
    dr_dbg_.setZero();
    dv_dbg_.setZero();
    dv_dba_.setZero();
    dp_dbg_.setZero();
    dp_dba_.setZero();
    dt_sum_ = 0.0;
    num_samples_ = 0;
  }

  // First-order re-linearization of the deltas at a new bias estimate, so the
  // estimator can update biases without re-integrating raw samples.
  void biasCorrectedDelta(const ImuBias& new_bias, Eigen::Matrix3d& delta_r, Eigen::Vector3d& delta_v,
                          Eigen::Vector3d& delta_p) const {
    const Eigen::Vector3d dbg = new_bias.gyro - bias_.gyro;
    const Eigen::Vector3d dba = new_bias.accel - bias_.accel;
    delta_r = delta_r_ * expSO3(dr_dbg_ * dbg);
    delta_v = delta_v_ + dv_dbg_ * dbg + dv_dba_ * dba;
    delta_p = delta_p_ + dp_dbg_ * dbg + dp_dba_ * dba;
  }

  const Eigen::Matrix3d& deltaRotation() const { return delta_r_; }
  const Eigen::Vector3d& deltaVelocity() const { return delta_v_; }
  const Eigen::Vector3d& deltaPosition() const { return delta_p_; }
  const Eigen::Matrix<double, kStateDim, kStateDim>& covariance() const { return covariance_; }
  const Eigen::Matrix3d& rotationBiasJacobian() const { return dr_dbg_; }
  const Eigen::Matrix3d& velocityGyroBiasJacobian() const { return dv_dbg_; }
  const Eigen::Matrix3d& velocityAccelBiasJacobian() const { return dv_dba_; }
  const Eigen::Matrix3d& positionGyroBiasJacobian() const { return dp_dbg_; }
  const Eigen::Matrix3d& positionAccelBiasJacobian() const { return dp_dba_; }
  const ImuBias& bias() const { return bias_; }
  const ImuNoiseParams& noiseParams() const { return noise_; }
  double deltaTime() const { return dt_sum_; }
  std::size_t numSamples() const { return num_samples_; }

 private:
  ImuBias bias_;
  ImuNoiseParams noise_;
  Eigen::Matrix3d delta_r_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d delta_v_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_p_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, kStateDim, kStateDim> covariance_{
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero()};
  Eigen::Matrix3d dr_dbg_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d dv_dbg_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d dv_dba_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d dp_dbg_{Eigen::Matrix3d::Zero()};
  Eigen::Matrix3d dp_dba_{Eigen::Matrix3d::Zero()};
  double dt_sum_{0.0};
  std::size_t num_samples_{0};
};

}  // namespace slam_core
