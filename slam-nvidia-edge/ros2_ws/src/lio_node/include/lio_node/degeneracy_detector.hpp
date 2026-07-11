#pragma once

// Degeneracy detector: condition number of the registration Hessian.
// Long corridors / tunnels collapse one translational eigenvalue; the
// condition number blowing up is the canonical LOAM-style signal.

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace lio_node {

struct DegeneracyReport {
  double condition_number{1.0};
  bool is_degenerate{false};
};

class DegeneracyDetector {
 public:
  explicit DegeneracyDetector(double condition_number_threshold) : threshold_(condition_number_threshold) {}

  DegeneracyReport analyze(const Eigen::Matrix<double, 6, 6>& hessian) const {
    DegeneracyReport report;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(hessian);
    const auto& ev = solver.eigenvalues();
    const double min_ev = ev.minCoeff();
    const double max_ev = ev.maxCoeff();
    report.condition_number = (min_ev <= 0.0) ? 1e18 : max_ev / min_ev;
    report.is_degenerate = report.condition_number > threshold_;
    // TODO(lio): project the ICP update out of the degenerate eigen-directions
    // (Zhang & Singh solution remapping) instead of only flagging.
    return report;
  }

 private:
  double threshold_;
};

}  // namespace lio_node
