#pragma once

// Degeneracy detector: condition number of the registration Hessian.
// Long corridors / tunnels collapse one translational eigenvalue; the
// condition number blowing up is the canonical LOAM-style signal.

#include <limits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace lio_node {

struct DegeneracyReport {
  double condition_number{1.0};
  bool is_degenerate{false};
  int num_degenerate_directions{0};
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
    const double cutoff = observabilityCutoff(max_ev);
    for (int i = 0; i < ev.size(); ++i) {
      if (ev[i] < cutoff) ++report.num_degenerate_directions;
    }
    return report;
  }

  // Projection onto the observable subspace: the span of eigenvectors whose
  // eigenvalue clears max_ev / threshold_ (a direction weaker than that would,
  // on its own, blow the condition number past threshold). Directions below the
  // cutoff are unobservable and get projected out.
  Eigen::Matrix<double, 6, 6> observableProjection(const Eigen::Matrix<double, 6, 6>& hessian) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(hessian);
    const auto& ev = solver.eigenvalues();
    const auto& V = solver.eigenvectors();
    const double cutoff = observabilityCutoff(ev.maxCoeff());
    Eigen::Matrix<double, 6, 6> projection = Eigen::Matrix<double, 6, 6>::Zero();
    for (int i = 0; i < ev.size(); ++i) {
      if (ev[i] >= cutoff) projection += V.col(i) * V.col(i).transpose();
    }
    return projection;
  }

  // Zhang & Singh solution remapping: keep the ICP increment only along
  // observable eigen-directions; the caller retains its prediction elsewhere.
  // `update` is the 6-DoF increment in [rotation(3), translation(3)] order,
  // matching the Hessian's basis.
  Eigen::Matrix<double, 6, 1> remap(const Eigen::Matrix<double, 6, 1>& update,
                                    const Eigen::Matrix<double, 6, 6>& hessian) const {
    return observableProjection(hessian) * update;
  }

 private:
  // A non-positive max eigenvalue means nothing is observable — cutoff above
  // any finite eigenvalue so the projection collapses to zero.
  double observabilityCutoff(double max_ev) const {
    if (max_ev <= 0.0) return std::numeric_limits<double>::infinity();
    return max_ev / threshold_;
  }

  double threshold_;
};

}  // namespace lio_node
