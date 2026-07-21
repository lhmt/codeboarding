#include "slam_core/imu_preintegration.hpp"

#include <Eigen/Eigenvalues>

#include "test_framework.hpp"

using slam_core::ImuBias;
using slam_core::ImuNoiseParams;
using slam_core::ImuPreintegrator;
using slam_core::ImuSample;

// Deterministic, dynamics-rich measurement stream (rotation + acceleration
// varying over time) so the bias Jacobians have non-trivial structure.
static ImuSample richSample(int i, double dt) {
  ImuSample s;
  const double t = i * dt;
  s.gyro = Eigen::Vector3d(0.3 * std::sin(2.0 * t), -0.2 * std::cos(3.0 * t), 0.4);
  s.accel = Eigen::Vector3d(1.0 + 0.5 * std::sin(t), -0.7 * std::cos(2.0 * t), 9.8 + 0.1 * std::sin(5.0 * t));
  return s;
}

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

  // First-order bias correction matches re-integration with a perturbed bias
  {
    ImuBias bias;
    bias.gyro = Eigen::Vector3d(0.01, -0.02, 0.015);
    bias.accel = Eigen::Vector3d(0.05, 0.1, -0.08);
    ImuPreintegrator nominal(bias);
    for (int i = 0; i < steps; ++i) nominal.integrate(richSample(i, dt), dt);

    const Eigen::Vector3d delta_bg(2e-4, -1e-4, 1.5e-4);
    const Eigen::Vector3d delta_ba(3e-4, 1e-4, -2e-4);
    ImuBias perturbed = bias;
    perturbed.gyro += delta_bg;
    perturbed.accel += delta_ba;
    ImuPreintegrator reintegrated(perturbed);
    for (int i = 0; i < steps; ++i) reintegrated.integrate(richSample(i, dt), dt);

    const auto corrected = nominal.biasCorrectedDelta(delta_bg, delta_ba);
    // First-order approximation: residual error is O(|delta_b|^2) ~ 1e-7.
    CHECK((corrected.velocity - reintegrated.deltaVelocity()).norm() < 1e-6);
    CHECK((corrected.position - reintegrated.deltaPosition()).norm() < 1e-6);
    CHECK(slam_core::logSO3(corrected.rotation.transpose() * reintegrated.deltaRotation()).norm() < 1e-6);
    // And the correction must actually matter (guards against zero Jacobians).
    CHECK((nominal.deltaVelocity() - reintegrated.deltaVelocity()).norm() > 1e-5);
  }

  // Covariance: zero without noise, PSD and growing with noise
  {
    ImuNoiseParams noise;
    noise.gyro_noise_density = 1.7e-4;
    noise.accel_noise_density = 2.0e-3;
    ImuPreintegrator noisy({}, noise);
    ImuPreintegrator noiseless;  // default params: zero densities
    Eigen::Matrix<double, 9, 9> half_way = Eigen::Matrix<double, 9, 9>::Zero();
    for (int i = 0; i < steps; ++i) {
      noisy.integrate(richSample(i, dt), dt);
      noiseless.integrate(richSample(i, dt), dt);
      if (i == steps / 2) half_way = noisy.covariance();
    }
    CHECK(noiseless.covariance().norm() == 0.0);
    const auto& cov = noisy.covariance();
    CHECK((cov - cov.transpose()).norm() < 1e-18);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> solver(cov);
    CHECK(solver.eigenvalues().minCoeff() > -1e-18);
    CHECK(cov.trace() > half_way.trace());  // uncertainty accumulates
    // Identical deltas regardless of noise params (covariance is bookkeeping).
    CHECK((noisy.deltaPosition() - noiseless.deltaPosition()).norm() == 0.0);
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
  }
}

TEST_MAIN()
