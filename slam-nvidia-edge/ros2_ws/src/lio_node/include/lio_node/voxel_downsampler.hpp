#pragma once

// Centroid voxel-grid downsampler over a hash grid. Deterministic: output
// order follows first-touch order of voxels in the input sequence.

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace lio_node {

class VoxelDownsampler {
 public:
  explicit VoxelDownsampler(double voxel_size) : inv_size_(1.0 / voxel_size) {}

  std::vector<Eigen::Vector3d> filter(const std::vector<Eigen::Vector3d>& points) const {
    struct Acc {
      Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
      int count{0};
      std::size_t order{0};
    };
    std::unordered_map<uint64_t, Acc> voxels;
    voxels.reserve(points.size());
    std::size_t next_order = 0;
    for (const auto& p : points) {
      auto& acc = voxels[key(p)];
      if (acc.count == 0) acc.order = next_order++;
      acc.sum += p;
      ++acc.count;
    }
    std::vector<Eigen::Vector3d> out(voxels.size());
    for (const auto& [k, acc] : voxels) {
      out[acc.order] = acc.sum / static_cast<double>(acc.count);
    }
    return out;
  }

 private:
  uint64_t key(const Eigen::Vector3d& p) const {
    // 21 bits per axis, offset to keep coordinates positive within ±~1e5 m.
    const auto ix = static_cast<uint64_t>(static_cast<int64_t>(std::floor(p.x() * inv_size_)) + (1 << 20));
    const auto iy = static_cast<uint64_t>(static_cast<int64_t>(std::floor(p.y() * inv_size_)) + (1 << 20));
    const auto iz = static_cast<uint64_t>(static_cast<int64_t>(std::floor(p.z() * inv_size_)) + (1 << 20));
    return (ix & 0x1FFFFF) | ((iy & 0x1FFFFF) << 21) | ((iz & 0x1FFFFF) << 42);
  }

  double inv_size_;
};

}  // namespace lio_node
