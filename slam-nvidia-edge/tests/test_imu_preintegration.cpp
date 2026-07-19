#include "slam_core/imu_preintegration.hpp"

#include <Eigen/Eigenvalues>

#include "test_framework.hpp"

using slam_core::ImuBias;
using slam_core::ImuPreintegrator;
using slam_core::ImuSample;

static void run_tests() {
  constexpr double dt = 0.005;  // 200 Hz
  constexpr int steps = 200;    // 1 s

  // Constant acceleration, no rotation: dv = a t, dp = 0.5 a t^2
  {
    ImuPreintegrator pre;
    ImuSample s;
    s.accel = Eigen::Vector3d(1.0, 0.0, 0.0);
    for (int i = 0; i < steps; ++i) CHECK(pre.integrate(s, dt));
    CHECK_NEAR(pre.deltaTime(), 1.0, 1e-12);
    CHECK(pre.numSamples() == steps);
    CHECK_NEAR(pre.deltaVelocity().x(), 1.0, 1e-9);
    // Discrete sum of 0.5 a dt^2 terms converges to 0.5 a t^2 at O(dt)
    CHECK_NEAR(pre.deltaPosition().x(), 0.5, 0.5 * dt + 1e-9);
    CHECK((pre.deltaRotation() - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  }

  // Constant gyro rate about z: accumulated rotation angle = w t
  {
    ImuPreintegrator pre;
    ImuSample s;
    s.gyro = Eigen::Vector3d(0.0, 0.0, 0.5);
    for (int i = 0; i < steps; ++i) pre.integrate(s, dt);
    const Eigen::Vector3d phi = slam_core::logSO3(pre.deltaRotation());
    CHECK_NEAR(phi.z(), 0.5, 1e-9);
    CHECK_NEAR(phi.head<2>().norm(), 0.0, 1e-12);
  }

  // Bias subtraction: measurement equal to bias integrates to identity/zero
  {
    ImuBias bias;
    bias.gyro = Eigen::Vector3d(0.01, -0.02, 0.03);
    bias.accel = Eigen::Vector3d(0.1, 0.2, -0.3);
    ImuPreintegrator pre(bias);
    ImuSample s;
    s.gyro = bias.gyro;
    s.accel = bias.accel;
    for (int i = 0; i < steps; ++i) pre.integrate(s, dt);
    CHECK((pre.deltaRotation() - Eigen::Matrix3d::Identity()).norm() < 1e-12);
    CHECK(pre.deltaVelocity().norm() < 1e-12);
    CHECK(pre.deltaPosition().norm() < 1e-12);
  }

  // Invalid dt is rejected and does not corrupt state
  {
    ImuPreintegrator pre;
    ImuSample s;
    s.accel = Eigen::Vector3d(1.0, 0.0, 0.0);
    CHECK(!pre.integrate(s, 0.0));
    CHECK(!pre.integrate(s, -0.01));
    CHECK(pre.numSamples() == 0);
    CHECK(pre.deltaVelocity().norm() == 0.0);
  }

  // reset() clears deltas and installs the new bias
  {
    ImuPreintegrator pre;
    ImuSample s;
    s.accel = Eigen::Vector3d(1.0, 0.0, 0.0);
    pre.integrate(s, dt);
    ImuBias new_bias;
    new_bias.gyro = Eigen::Vector3d(0.1, 0.0, 0.0);
    pre.reset(new_bias);
    CHECK(pre.numSamples() == 0);
    CHECK(pre.deltaTime() == 0.0);
    CHECK(pre.deltaPosition().norm() == 0.0);
    CHECK((pre.bias().gyro - new_bias.gyro).norm() == 0.0);
    CHECK(pre.covariance().norm() == 0.0);
    CHECK(pre.rotationBiasJacobian().norm() == 0.0);
  }

  // Covariance: zero initially, then symmetric with strictly growing trace,
  // and full-rank across all 15 dims once noise has been injected
  {
    ImuPreintegrator pre;
    ImuSample s;
    s.gyro = Eigen::Vector3d(0.2, -0.1, 0.3);
    s.accel = Eigen::Vector3d(0.5, 9.8, -0.2);
    CHECK(pre.covariance().norm() == 0.0);
    double prev_trace = 0.0;
    for (int i = 0; i < steps; ++i) {
      pre.integrate(s, dt);
      CHECK(pre.covariance().trace() > prev_trace);
      prev_trace = pre.covariance().trace();
    }
    const auto& P = pre.covariance();
    CHECK((P - P.transpose()).norm() < 1e-15);
    for (int i = 0; i < 15; ++i) CHECK(P(i, i) > 0.0);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eig(P);
    CHECK(eig.eigenvalues().minCoeff() > 0.0);
  }

  // Bias Jacobians: first-order correction matches re-integration at a
  // perturbed bias, and beats the uncorrected deltas by a wide margin
  {
    ImuBias bias;
    bias.gyro = Eigen::Vector3d(0.01, -0.02, 0.005);
    bias.accel = Eigen::Vector3d(0.05, -0.1, 0.08);
    ImuPreintegrator pre(bias);
    ImuPreintegrator pre_perturbed;  // integrates with the perturbed bias directly

    ImuBias perturbed = bias;
    perturbed.gyro += Eigen::Vector3d(1e-3, -2e-3, 1.5e-3);
    perturbed.accel += Eigen::Vector3d(5e-3, 1e-2, -8e-3);
    pre_perturbed.reset(perturbed);

    for (int i = 0; i < steps; ++i) {
      ImuSample s;
      // Time-varying motion so the bias Jacobians see generic excitation.
      const double t = i * dt;
      s.gyro = Eigen::Vector3d(0.3 * std::sin(t), 0.2 * std::cos(t), 0.4);
      s.accel = Eigen::Vector3d(1.0 + 0.5 * std::sin(2.0 * t), -0.3, 9.8 * 0.1 * std::cos(t));
      pre.integrate(s, dt);
      pre_perturbed.integrate(s, dt);
    }

    Eigen::Matrix3d r_corr;
    Eigen::Vector3d v_corr, p_corr;
    pre.biasCorrectedDelta(perturbed, r_corr, v_corr, p_corr);

    const double r_err = slam_core::logSO3(pre_perturbed.deltaRotation().transpose() * r_corr).norm();
    const double r_err_raw =
        slam_core::logSO3(pre_perturbed.deltaRotation().transpose() * pre.deltaRotation()).norm();
    CHECK(r_err < 1e-5);
    CHECK(r_err < 0.01 * r_err_raw);
    CHECK((v_corr - pre_perturbed.deltaVelocity()).norm() <
          0.01 * (pre.deltaVelocity() - pre_perturbed.deltaVelocity()).norm());
    CHECK((p_corr - pre_perturbed.deltaPosition()).norm() <
          0.01 * (pre.deltaPosition() - pre_perturbed.deltaPosition()).norm());
  }

  // biasCorrectedDelta at the linearization bias returns the raw deltas
  {
    ImuPreintegrator pre;
    ImuSample s;
    s.gyro = Eigen::Vector3d(0.1, 0.2, -0.1);
    s.accel = Eigen::Vector3d(1.0, 0.0, 9.8);
    for (int i = 0; i < steps; ++i) pre.integrate(s, dt);
    Eigen::Matrix3d r;
    Eigen::Vector3d v, p;
    pre.biasCorrectedDelta(pre.bias(), r, v, p);
    CHECK((r - pre.deltaRotation()).norm() < 1e-14);
    CHECK((v - pre.deltaVelocity()).norm() < 1e-14);
    CHECK((p - pre.deltaPosition()).norm() < 1e-14);
  }
}

TEST_MAIN()
