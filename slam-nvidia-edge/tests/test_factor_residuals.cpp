#include "slam_core/factor_residuals.hpp"

#include "test_framework.hpp"

using slam_core::ImuFactorResidual;
using slam_core::ImuPreintegrator;
using slam_core::ImuSample;
using slam_core::NavState;
using slam_core::PointToPlaneFactorResidual;
using slam_core::ReprojectionFactorResidual;

static void run_tests() {
  // Residual dimensions are the contract with the future solver
  static_assert(ImuFactorResidual::kDim == 15);
  static_assert(ReprojectionFactorResidual::kDim == 2);
  static_assert(PointToPlaneFactorResidual::kDim == 1);

  // IMU factor: identical measured/predicted deltas give zero residual
  {
    const Eigen::Matrix3d r = slam_core::expSO3(Eigen::Vector3d(0.1, 0.2, -0.3));
    const auto res = ImuFactorResidual::evaluate(r, r, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    CHECK(res.rows() == 15);
    CHECK(res.norm() < 1e-12);
  }

  // IMU factor: rotation mismatch lands in the first 3 components only
  {
    const Eigen::Matrix3d meas = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d pred = slam_core::expSO3(Eigen::Vector3d(0.0, 0.0, 0.2));
    const auto res = ImuFactorResidual::evaluate(meas, pred, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                 Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    CHECK_NEAR(res.segment<3>(0).norm(), 0.2, 1e-10);
    CHECK(res.tail<12>().norm() < 1e-12);
  }

  // Reprojection: point on the optical axis projects to the principal point
  {
    const auto res = ReprojectionFactorResidual::evaluate(Eigen::Vector3d(0, 0, 2.0), Eigen::Vector2d(320, 240),
                                                          400.0, 400.0, 320.0, 240.0);
    CHECK(res.norm() < 1e-12);
  }

  // Reprojection: point behind the camera yields the large sentinel residual
  {
    const auto res = ReprojectionFactorResidual::evaluate(Eigen::Vector3d(0, 0, -1.0), Eigen::Vector2d(0, 0),
                                                          400.0, 400.0, 320.0, 240.0);
    CHECK(res.x() == 1e6 && res.y() == 1e6);
  }

  // Point-to-plane: signed distance to the plane z = 1 (n=(0,0,1), d=-1)
  {
    CHECK_NEAR(PointToPlaneFactorResidual::evaluate(Eigen::Vector3d(5, 7, 3.0), Eigen::Vector3d(0, 0, 1), -1.0),
               2.0, 1e-12);
    CHECK_NEAR(PointToPlaneFactorResidual::evaluate(Eigen::Vector3d(0, 0, 1.0), Eigen::Vector3d(0, 0, 1), -1.0),
               0.0, 1e-12);
  }

  // Between-states IMU factor: state_j derived from state_i and the
  // preintegrated deltas under gravity gives zero residual
  {
    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
    ImuPreintegrator pre;
    for (int i = 0; i < 200; ++i) {
      ImuSample s;
      const double t = i * 0.005;
      s.gyro = Eigen::Vector3d(0.2 * std::sin(t), -0.1, 0.3);
      s.accel = Eigen::Vector3d(1.0, 0.5 * std::cos(t), 9.9);
      pre.integrate(s, 0.005);
    }

    NavState state_i;
    state_i.rotation = slam_core::expSO3(Eigen::Vector3d(0.1, -0.2, 0.3));
    state_i.position = Eigen::Vector3d(5.0, -2.0, 1.0);
    state_i.velocity = Eigen::Vector3d(0.5, 0.1, -0.3);

    const double t = pre.deltaTime();
    NavState state_j;
    state_j.rotation = state_i.rotation * pre.deltaRotation();
    state_j.velocity = state_i.velocity + gravity * t + state_i.rotation * pre.deltaVelocity();
    state_j.position = state_i.position + state_i.velocity * t + 0.5 * gravity * t * t +
                       state_i.rotation * pre.deltaPosition();

    const auto res = ImuFactorResidual::evaluate(state_i, state_j, pre, gravity);
    CHECK(res.norm() < 1e-10);

    // A rotation perturbation of state_j lands in the rotation block only
    NavState state_j_rot = state_j;
    state_j_rot.rotation = state_j.rotation * slam_core::expSO3(Eigen::Vector3d(0.0, 0.0, 0.05));
    const auto res_rot = ImuFactorResidual::evaluate(state_i, state_j_rot, pre, gravity);
    CHECK_NEAR(res_rot.segment<3>(0).norm(), 0.05, 1e-10);
    CHECK(res_rot.tail<12>().norm() < 1e-10);

    // A bias difference between the states lands in the bias blocks
    NavState state_j_bias = state_j;
    state_j_bias.bias.gyro = Eigen::Vector3d(0.01, 0.0, 0.0);
    const auto res_bias = ImuFactorResidual::evaluate(state_i, state_j_bias, pre, gravity);
    CHECK_NEAR(res_bias.segment<3>(9).norm(), 0.01, 1e-12);
  }

  // Reprojection Jacobian matches central differences
  {
    const Eigen::Vector3d p(0.3, -0.2, 2.5);
    const Eigen::Vector2d obs(300.0, 250.0);
    const double fx = 400.0, fy = 410.0, cx = 320.0, cy = 240.0;
    const auto J = ReprojectionFactorResidual::jacobian(p, fx, fy);
    const double eps = 1e-7;
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d dp = Eigen::Vector3d::Zero();
      dp[k] = eps;
      const Eigen::Vector2d num =
          (ReprojectionFactorResidual::evaluate(p + dp, obs, fx, fy, cx, cy) -
           ReprojectionFactorResidual::evaluate(p - dp, obs, fx, fy, cx, cy)) /
          (2.0 * eps);
      CHECK((num - J.col(k)).norm() < 1e-5);
    }
    // Behind-plane branch has the zero Jacobian matching the clamped residual
    CHECK(ReprojectionFactorResidual::jacobian(Eigen::Vector3d(0, 0, -1.0), fx, fy).norm() == 0.0);
  }
}

TEST_MAIN()
