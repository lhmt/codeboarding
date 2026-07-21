#include "lio_node/point_to_plane_icp.hpp"

#include <cmath>

#include "lio_node/degeneracy_detector.hpp"
#include "slam_core/lie_group.hpp"
#include "test_framework.hpp"

using lio_node::DegeneracyDetector;
using lio_node::IcpResult;
using lio_node::PointToPlaneIcp;

namespace {

// Synthetic map: three mutually orthogonal grid planes through the origin
// (z=0, x=0, y=0), spacing 0.2, extent +/-2m (4m span). Deterministic nested
// loops, no RNG.
//
// A thin strip is excluded around each pair of shared axes (|a| < margin or
// |b| < margin): right at those lines, brute-force KNN legitimately pulls in
// points from the adjacent (perpendicular) plane, producing a mixed-normal
// local plane fit. That's a real artifact of intersecting-plane geometry with
// brute-force correspondence, not a solver bug -- excluding the strip keeps
// the fixture's plane fits clean while still fully constraining all 6 DOF.
std::vector<Eigen::Vector3d> buildCornerPlanes() {
  std::vector<Eigen::Vector3d> pts;
  constexpr double spacing = 0.2;
  constexpr double extent = 2.0;
  constexpr double margin = 0.3;
  const int n = static_cast<int>(extent / spacing);

  for (int i = -n; i <= n; ++i) {
    for (int j = -n; j <= n; ++j) {
      const double a = i * spacing;
      const double b = j * spacing;
      if (std::fabs(a) < margin || std::fabs(b) < margin) continue;
      pts.emplace_back(a, b, 0.0);  // z = 0 plane
      pts.emplace_back(0.0, a, b);  // x = 0 plane
      pts.emplace_back(a, 0.0, b);  // y = 0 plane
    }
  }
  return pts;
}

std::vector<Eigen::Vector3d> buildSinglePlane() {
  std::vector<Eigen::Vector3d> pts;
  constexpr double spacing = 0.2;
  constexpr double extent = 4.0;
  const int n = static_cast<int>(extent / spacing);
  for (int i = -n; i <= n; ++i) {
    for (int j = -n; j <= n; ++j) {
      pts.emplace_back(i * spacing, j * spacing, 0.0);
    }
  }
  return pts;
}

std::vector<Eigen::Vector3d> transformCloud(const std::vector<Eigen::Vector3d>& cloud, const Eigen::Matrix3d& R,
                                             const Eigen::Vector3d& t) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(cloud.size());
  for (const auto& p : cloud) out.push_back(R * p + t);
  return out;
}

}  // namespace

static void run_tests() {
  // --- Recovery test: three orthogonal planes, known small rigid perturbation.
  {
    const auto target = buildCornerPlanes();
    const Eigen::Matrix3d R_true = slam_core::expSO3(Eigen::Vector3d(0.02, -0.015, 0.03));
    const Eigen::Vector3d t_true(0.05, -0.04, 0.03);
    const auto source = transformCloud(target, R_true, t_true);

    const PointToPlaneIcp icp(30, 0.3);
    const IcpResult result = icp.align(source, target);

    CHECK(result.converged);
    CHECK(result.fitness > 0.9);

    // Composed with the known perturbation, (R_est, t_est) should undo it:
    // R_est * R_true ≈ I, t_est + R_est * t_true ≈ 0.
    const Eigen::Matrix3d R_compose = result.rotation_increment * R_true;
    const Eigen::Vector3d t_compose = result.translation_increment + result.rotation_increment * t_true;

    const double angle_error = slam_core::logSO3(R_compose).norm();
    const double position_error = t_compose.norm();

    CHECK(angle_error < 5e-3);
    CHECK(position_error < 5e-3);

    // Hessian symmetric PSD.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(result.hessian);
    CHECK(solver.eigenvalues().minCoeff() > -1e-9);
    CHECK((result.hessian - result.hessian.transpose()).norm() < 1e-9);
  }

  // --- Degenerate test: single plane only -> translation along the plane is
  // unobservable, condition number should blow up.
  {
    const auto target = buildSinglePlane();
    const Eigen::Matrix3d R_true = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d t_true(0.0, 0.0, 0.02);  // small translation along the plane normal only
    const auto source = transformCloud(target, R_true, t_true);

    const PointToPlaneIcp icp(30, 0.3);
    const IcpResult result = icp.align(source, target);

    const DegeneracyDetector detector(1e6);
    const auto report = detector.analyze(result.hessian);
    CHECK(report.is_degenerate);
  }

  // --- Empty inputs.
  {
    const PointToPlaneIcp icp(10, 0.3);
    const std::vector<Eigen::Vector3d> empty;
    const std::vector<Eigen::Vector3d> nonempty = {Eigen::Vector3d(0, 0, 0)};

    {
      const auto result = icp.align(empty, nonempty);
      CHECK(!result.converged);
      CHECK_NEAR(result.fitness, 0.0, 1e-12);
    }
    {
      const auto result = icp.align(nonempty, empty);
      CHECK(!result.converged);
      CHECK_NEAR(result.fitness, 0.0, 1e-12);
    }
    {
      const auto result = icp.align(empty, empty);
      CHECK(!result.converged);
    }
  }
}

TEST_MAIN()
