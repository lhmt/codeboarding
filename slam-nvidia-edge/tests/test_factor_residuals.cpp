#include "slam_core/factor_residuals.hpp"

#include "test_framework.hpp"

using slam_core::ImuFactorResidual;
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
}

TEST_MAIN()
