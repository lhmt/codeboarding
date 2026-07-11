#pragma once

// Rolling local map: voxel-deduplicated world-frame points cropped to a
// radius around the current position, capped for bounded memory.
// EXTENSION POINT: replace with an ikd-tree / VDB structure for real ICP.

#include <vector>

#include <Eigen/Core>

#include "lio_node/voxel_downsampler.hpp"

namespace lio_node {

class LocalMapManager {
 public:
  LocalMapManager(double voxel_size, double radius, std::size_t max_points)
      : downsampler_(voxel_size), radius_(radius), max_points_(max_points) {}

  void insert(const std::vector<Eigen::Vector3d>& world_points, const Eigen::Vector3d& current_position) {
    map_.insert(map_.end(), world_points.begin(), world_points.end());
    map_ = downsampler_.filter(map_);

    // Radius crop, preserving order.
    std::vector<Eigen::Vector3d> kept;
    kept.reserve(map_.size());
    for (const auto& p : map_) {
      if ((p - current_position).norm() <= radius_) kept.push_back(p);
    }
    map_ = std::move(kept);

    // Cap: drop oldest first (front of the vector is oldest insertion).
    if (map_.size() > max_points_) {
      map_.erase(map_.begin(), map_.begin() + static_cast<std::ptrdiff_t>(map_.size() - max_points_));
    }
  }

  const std::vector<Eigen::Vector3d>& points() const { return map_; }
  bool empty() const { return map_.empty(); }

 private:
  VoxelDownsampler downsampler_;
  double radius_;
  std::size_t max_points_;
  std::vector<Eigen::Vector3d> map_;
};

}  // namespace lio_node
