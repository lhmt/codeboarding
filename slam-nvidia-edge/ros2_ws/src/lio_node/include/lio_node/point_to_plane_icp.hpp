#pragma once

// Point-to-plane ICP: brute-force correspondence + Gauss-Newton refinement.
// EXTENSION POINT: replace align() with a real solver (small_gicp / VGICP-CUDA
// / KISS-ICP registration). The IcpResult contract — increment, fitness,
// Hessian proxy for degeneracy analysis — is what lio_node consumes.

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "slam_core/lie_group.hpp"

namespace lio_node {

struct IcpResult {
  Eigen::Matrix3d rotation_increment{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_increment{Eigen::Vector3d::Zero()};
  double fitness{0.0};              // fraction of source points with a valid correspondence
  int iterations{0};
  bool converged{false};
  // 6x6 Gauss-Newton Hessian (J^T J) from the final iteration; the degeneracy
  // detector reads its spectrum.
  Eigen::Matrix<double, 6, 6> hessian{Eigen::Matrix<double, 6, 6>::Identity()};
};

class PointToPlaneIcp {
 public:
  PointToPlaneIcp(int max_iterations, double max_correspondence_dist, double epsilon = 1e-3)
      : max_iterations_(max_iterations), max_correspondence_dist_(max_correspondence_dist), epsilon_(epsilon) {}

  IcpResult align(const std::vector<Eigen::Vector3d>& source_world_predicted,
                  const std::vector<Eigen::Vector3d>& target_map) const {
    IcpResult result;

    if (source_world_predicted.empty() || target_map.empty()) {
      result.iterations = 0;
      result.converged = false;
      result.fitness = 0.0;
      result.hessian = Eigen::Matrix<double, 6, 6>::Identity();
      return result;
    }

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();

    static constexpr int kNeighbors = 6;
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
    std::size_t matched = 0;
    int iterations_run = 0;
    bool converged = false;

    for (int iter = 0; iter < max_iterations_; ++iter) {
      Eigen::Matrix<double, 6, 6> h_accum = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> b_accum = Eigen::Matrix<double, 6, 1>::Zero();
      matched = 0;

      for (const auto& p : source_world_predicted) {
        const Eigen::Vector3d transformed = R * p + t;

        // EXTENSION POINT: replace this brute-force scan with a KD-tree /
        // ikd-tree nearest-neighbor query for O(log M) correspondence search.
        std::array<double, kNeighbors> best_dist2{};
        std::array<int, kNeighbors> best_idx{};
        int found = 0;
        best_dist2.fill(std::numeric_limits<double>::max());
        best_idx.fill(-1);

        for (std::size_t j = 0; j < target_map.size(); ++j) {
          const double d2 = (target_map[j] - transformed).squaredNorm();
          if (d2 >= best_dist2[kNeighbors - 1]) continue;
          // Insertion sort into the fixed-size k-best buffer.
          int pos = found < kNeighbors ? found : kNeighbors - 1;
          if (found < kNeighbors) ++found;
          while (pos > 0 && best_dist2[pos - 1] > d2) {
            best_dist2[pos] = best_dist2[pos - 1];
            best_idx[pos] = best_idx[pos - 1];
            --pos;
          }
          best_dist2[pos] = d2;
          best_idx[pos] = static_cast<int>(j);
        }

        if (found < 3) continue;  // not enough neighbors for a plane fit
        if (std::sqrt(best_dist2[0]) > max_correspondence_dist_) continue;

        // Local plane fit: centroid + smallest-eigenvector normal of the
        // k-neighbor scatter.
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (int k = 0; k < found; ++k) centroid += target_map[static_cast<std::size_t>(best_idx[k])];
        centroid /= static_cast<double>(found);

        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        for (int k = 0; k < found; ++k) {
          const Eigen::Vector3d d = target_map[static_cast<std::size_t>(best_idx[k])] - centroid;
          scatter += d * d.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> plane_solver(scatter);
        const Eigen::Vector3d normal = plane_solver.eigenvectors().col(0);  // smallest eigenvalue first

        const double residual = normal.dot(transformed - centroid);

        // Left-perturbation derivative of Exp(dphi)(Rp + t) + dt: the rotation
        // block acts on the fully transformed point, not just R*p.
        Eigen::Matrix<double, 1, 6> J;
        J.head<3>() = -(normal.transpose() * slam_core::hat(transformed));
        J.tail<3>() = normal.transpose();

        h_accum += J.transpose() * J;
        b_accum += J.transpose() * residual;
        ++matched;
      }

      iterations_run = iter + 1;
      H = h_accum + 1e-9 * Eigen::Matrix<double, 6, 6>::Identity();

      const Eigen::Matrix<double, 6, 1> dx = H.ldlt().solve(-b_accum);

      R = slam_core::expSO3(dx.head<3>()) * R;
      t = slam_core::expSO3(dx.head<3>()) * t + dx.tail<3>();

      if (dx.norm() < epsilon_) {
        converged = true;
        break;
      }
    }

    result.rotation_increment = R;
    result.translation_increment = t;
    result.fitness = static_cast<double>(matched) / static_cast<double>(source_world_predicted.size());
    result.iterations = iterations_run;
    result.converged = converged;
    result.hessian = H;
    return result;
  }

 private:
  int max_iterations_;
  double max_correspondence_dist_;
  double epsilon_;
};

}  // namespace lio_node
