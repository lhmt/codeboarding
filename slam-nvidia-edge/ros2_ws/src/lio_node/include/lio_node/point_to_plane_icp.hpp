#pragma once

// Point-to-plane ICP placeholder.
// EXTENSION POINT: replace align() with a real solver (small_gicp / VGICP-CUDA
// / KISS-ICP registration). The IcpResult contract — increment, fitness,
// Hessian proxy for degeneracy analysis — is what lio_node consumes.

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lio_node {

struct IcpResult {
  Eigen::Matrix3d rotation_increment{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_increment{Eigen::Vector3d::Zero()};
  double fitness{0.0};              // fraction of source points with a valid correspondence
  int iterations{0};
  bool converged{false};
  // 6x6 Gauss-Newton Hessian approximation (J^T J); the degeneracy detector
  // reads its spectrum. Placeholder fills a scatter-based proxy.
  Eigen::Matrix<double, 6, 6> hessian{Eigen::Matrix<double, 6, 6>::Identity()};
};

class PointToPlaneIcp {
 public:
  PointToPlaneIcp(int max_iterations, double max_correspondence_dist)
      : max_iterations_(max_iterations), max_correspondence_dist_(max_correspondence_dist) {}

  IcpResult align(const std::vector<Eigen::Vector3d>& source_world_predicted,
                  const std::vector<Eigen::Vector3d>& target_map) const {
    IcpResult result;
    result.iterations = 0;
    result.converged = !source_world_predicted.empty() && !target_map.empty();
    result.fitness = result.converged ? 1.0 : 0.0;

    // Geometric observability proxy while the real solver is absent: the
    // translational block gets the source scatter; rotation block identity.
    if (!source_world_predicted.empty()) {
      Eigen::Vector3d mean = Eigen::Vector3d::Zero();
      for (const auto& p : source_world_predicted) mean += p;
      mean /= static_cast<double>(source_world_predicted.size());
      Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
      for (const auto& p : source_world_predicted) scatter += (p - mean) * (p - mean).transpose();
      scatter /= static_cast<double>(source_world_predicted.size());
      result.hessian.topLeftCorner<3, 3>() = scatter + 1e-9 * Eigen::Matrix3d::Identity();
    }
    (void)max_iterations_;
    (void)max_correspondence_dist_;
    return result;
  }

 private:
  int max_iterations_;
  double max_correspondence_dist_;
};

}  // namespace lio_node
