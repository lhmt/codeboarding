#include "slam_core/lie_group.hpp"

#include "test_framework.hpp"

using slam_core::expSO3;
using slam_core::hat;
using slam_core::logSO3;
using slam_core::rightJacobianInvSO3;
using slam_core::rightJacobianSO3;
using slam_core::vee;

static void run_tests() {
  // hat/vee are inverse
  const Eigen::Vector3d v(0.1, -0.2, 0.3);
  CHECK((vee(hat(v)) - v).norm() < 1e-15);

  // Exp of zero is identity
  CHECK((expSO3(Eigen::Vector3d::Zero()) - Eigen::Matrix3d::Identity()).norm() < 1e-15);

  // Exp/Log roundtrip at a generic angle
  const Eigen::Vector3d w(0.3, -0.5, 0.7);
  CHECK((logSO3(expSO3(w)) - w).norm() < 1e-10);

  // Small-angle branch: roundtrip and orthonormality near zero
  const Eigen::Vector3d tiny(1e-10, -2e-10, 3e-10);
  const Eigen::Matrix3d r_tiny = expSO3(tiny);
  CHECK((r_tiny * r_tiny.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  CHECK((logSO3(r_tiny) - tiny).norm() < 1e-12);

  // Rotation is orthonormal with det +1 at a large angle
  const Eigen::Matrix3d r = expSO3(Eigen::Vector3d(2.0, 1.0, -1.5));
  CHECK((r * r.transpose() - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  CHECK_NEAR(r.determinant(), 1.0, 1e-12);

  // Known value: pi/2 about z maps x to y
  const Eigen::Matrix3d rz = expSO3(Eigen::Vector3d(0, 0, M_PI / 2));
  CHECK((rz * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitY()).norm() < 1e-12);

  // Log near pi recovers the angle magnitude
  const Eigen::Vector3d near_pi = (M_PI - 1e-9) * Eigen::Vector3d::UnitX();
  CHECK_NEAR(logSO3(expSO3(near_pi)).norm(), near_pi.norm(), 1e-6);

  // Right Jacobian: Exp(w + dw) ≈ Exp(w) Exp(Jr(w) dw) to first order
  const Eigen::Vector3d dw(1e-6, -2e-6, 1.5e-6);
  const Eigen::Matrix3d lhs = expSO3(w + dw);
  const Eigen::Matrix3d rhs = expSO3(w) * expSO3(rightJacobianSO3(w) * dw);
  CHECK((lhs - rhs).norm() < 1e-10);

  // Inverse right Jacobian: Jr(w)⁻¹ Jr(w) = I at generic and tiny angles
  CHECK((rightJacobianInvSO3(w) * rightJacobianSO3(w) - Eigen::Matrix3d::Identity()).norm() < 1e-12);
  const Eigen::Vector3d w_tiny(1e-10, 2e-10, -1e-10);
  CHECK((rightJacobianInvSO3(w_tiny) * rightJacobianSO3(w_tiny) - Eigen::Matrix3d::Identity()).norm() < 1e-12);
}

TEST_MAIN()
