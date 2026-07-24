#include "fusion_node/ekf_backend.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>

#include "slam_core/lie_group.hpp"
#include "test_framework.hpp"

using fusion_node::ErrorStateEkfBackend;
using fusion_node::SourceState;

namespace {

SourceState makeSource(const std::string& name, const Eigen::Vector3d& position, double weight,
                        const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero(),
                        bool has_orientation = false,
                        const Eigen::Quaterniond& orientation = Eigen::Quaterniond::Identity()) {
  SourceState s;
  s.name = name;
  s.timestamp = 1.0;
  s.position = position;
  s.velocity = velocity;
  s.orientation = orientation;
  s.weight = weight;
  s.has_orientation = has_orientation;
  return s;
}

}  // namespace

static void run_tests() {
  // Empty / all-zero-weight sources -> nullopt, no state change.
  {
    ErrorStateEkfBackend backend;
    CHECK(!backend.fuse({}).has_value());

    std::vector<SourceState> zero_weight = {makeSource("gps", Eigen::Vector3d(1, 2, 3), 0.0)};
    CHECK(!backend.fuse(zero_weight).has_value());
  }

  // Single source converges to its position over repeated updates.
  {
    ErrorStateEkfBackend backend;
    const Eigen::Vector3d target(1.0, 2.0, 3.0);
    std::vector<SourceState> src = {makeSource("lio", target, 0.8)};
    std::optional<fusion_node::FusedState> result;
    for (int i = 0; i < 20; ++i) result = backend.fuse(src);
    CHECK(result.has_value());
    CHECK((result->position - target).norm() < 1e-2);
    CHECK(result->num_sources == 1);
  }

  // Two agreeing sources: fused position matches, and fusing two sources
  // leaves less uncertainty than fusing just one.
  {
    const Eigen::Vector3d target(2.0, -1.0, 0.5);

    ErrorStateEkfBackend one_source;
    auto r1 = one_source.fuse({makeSource("vio", target, 0.5)});
    CHECK(r1.has_value());
    const double trace_one = one_source.covariance().topLeftCorner<3, 3>().trace();

    ErrorStateEkfBackend two_sources;
    auto r2 = two_sources.fuse(
        {makeSource("vio", target, 0.5), makeSource("lio", target, 0.5)});
    CHECK(r2.has_value());
    CHECK((r2->position - target).norm() < 0.5);
    const double trace_two = two_sources.covariance().topLeftCorner<3, 3>().trace();

    CHECK(trace_two < trace_one);
  }

  // Disagreeing sources with different weights: fused estimate leans toward
  // the higher-weight source.
  {
    ErrorStateEkfBackend backend;
    std::vector<SourceState> src = {makeSource("vio", Eigen::Vector3d(0, 0, 0), 0.2),
                                     makeSource("lio", Eigen::Vector3d(10, 0, 0), 0.8)};
    std::optional<fusion_node::FusedState> result;
    for (int i = 0; i < 5; ++i) result = backend.fuse(src);
    CHECK(result.has_value());
    CHECK(result->position.x() > 5.0);
  }

  // Orientation measurement pulls the fused quaternion to the source's.
  {
    ErrorStateEkfBackend backend;
    const Eigen::Quaterniond target(Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()));
    std::vector<SourceState> src = {
        makeSource("vio", Eigen::Vector3d::Zero(), 0.7, Eigen::Vector3d::Zero(), true, target)};
    std::optional<fusion_node::FusedState> result;
    for (int i = 0; i < 20; ++i) result = backend.fuse(src);
    CHECK(result.has_value());
    const Eigen::Vector3d rot_err =
        slam_core::logSO3((result->orientation.conjugate() * target).toRotationMatrix());
    CHECK(rot_err.norm() < 1e-2);
  }

  // Covariance stays symmetric and PSD across all the updates above.
  {
    ErrorStateEkfBackend backend;
    std::vector<SourceState> src = {makeSource("vio", Eigen::Vector3d(0.1, 0.2, 0.3), 0.3),
                                     makeSource("lio", Eigen::Vector3d(0.2, 0.1, 0.4), 0.7)};
    for (int i = 0; i < 10; ++i) backend.fuse(src);
    const auto& P = backend.covariance();
    CHECK((P - P.transpose()).norm() < 1e-9);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> solver(P);
    CHECK(solver.eigenvalues().minCoeff() > -1e-9);
  }
}

TEST_MAIN()
