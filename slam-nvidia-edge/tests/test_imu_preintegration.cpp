#include "slam_core/imu_preintegration.hpp"

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
  }
}

TEST_MAIN()
