#include "lio_node/scan_deskew.hpp"

#include "test_framework.hpp"

using lio_node::ScanDeskewer;

static void run_tests() {
  const ScanDeskewer deskewer;

  const std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 2.0, 0.0),
      Eigen::Vector3d(0.5, -0.5, 1.5),
  };

  // 90 degrees about z over the scan.
  const Eigen::Matrix3d rotation_over_scan = slam_core::expSO3(Eigen::Vector3d(0, 0, M_PI / 2));

  // tau = 1 for all points (captured at scan end) -> unchanged.
  {
    const std::vector<double> time_fractions = {1.0, 1.0, 1.0};
    const auto out = deskewer.deskew(points, time_fractions, rotation_over_scan);
    CHECK(out.size() == points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      CHECK((out[i] - points[i]).norm() < 1e-12);
    }
  }

  // tau = 0 for all points (captured at scan start) -> rotated by
  // Exp(-phi) == rotation_over_scan^T.
  {
    const std::vector<double> time_fractions = {0.0, 0.0, 0.0};
    const auto out = deskewer.deskew(points, time_fractions, rotation_over_scan);
    for (std::size_t i = 0; i < points.size(); ++i) {
      const Eigen::Vector3d expected = rotation_over_scan.transpose() * points[i];
      CHECK((out[i] - expected).norm() < 1e-10);
    }
  }

  // Known value: 90 degrees about z, tau = 0.5 -> point rotated by -45
  // degrees about z.
  {
    const std::vector<Eigen::Vector3d> single = {Eigen::Vector3d(1.0, 0.0, 0.0)};
    const std::vector<double> time_fractions = {0.5};
    const auto out = deskewer.deskew(single, time_fractions, rotation_over_scan);
    const Eigen::Matrix3d minus_45_about_z = slam_core::expSO3(Eigen::Vector3d(0, 0, -M_PI / 4));
    const Eigen::Vector3d expected = minus_45_about_z * single[0];
    CHECK((out[0] - expected).norm() < 1e-10);
  }

  // Mismatched time_fractions size -> passthrough.
  {
    const std::vector<double> time_fractions = {0.0, 0.5};  // wrong size
    const auto out = deskewer.deskew(points, time_fractions, rotation_over_scan);
    CHECK(out.size() == points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      CHECK((out[i] - points[i]).norm() < 1e-15);
    }
  }

  // Empty input -> empty output.
  {
    const std::vector<Eigen::Vector3d> empty_points;
    const std::vector<double> empty_fractions;
    const auto out = deskewer.deskew(empty_points, empty_fractions, rotation_over_scan);
    CHECK(out.empty());
  }
}

TEST_MAIN()
