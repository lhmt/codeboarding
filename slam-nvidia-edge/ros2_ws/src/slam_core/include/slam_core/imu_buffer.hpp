#pragma once

// Time-ordered IMU sample buffer shared by VIO and LIO.
// Out-of-order samples are dropped (drivers occasionally replay under load);
// dropping keeps downstream integration monotonic and deterministic.

#include <deque>
#include <vector>

#include "slam_core/imu_preintegration.hpp"

namespace slam_core {

class ImuBuffer {
 public:
  explicit ImuBuffer(double horizon_seconds = 10.0) : horizon_(horizon_seconds) {}

  bool push(const ImuSample& sample) {
    if (!buffer_.empty() && sample.timestamp <= buffer_.back().timestamp) {
      return false;
    }
    buffer_.push_back(sample);
    while (!buffer_.empty() && buffer_.back().timestamp - buffer_.front().timestamp > horizon_) {
      buffer_.pop_front();
    }
    return true;
  }

  // Samples with t0 < timestamp <= t1, oldest first.
  std::vector<ImuSample> range(double t0, double t1) const {
    std::vector<ImuSample> out;
    for (const auto& s : buffer_) {
      if (s.timestamp > t1) break;
      if (s.timestamp > t0) out.push_back(s);
    }
    return out;
  }

  bool empty() const { return buffer_.empty(); }
  std::size_t size() const { return buffer_.size(); }
  double latestTimestamp() const { return buffer_.empty() ? 0.0 : buffer_.back().timestamp; }

 private:
  double horizon_;
  std::deque<ImuSample> buffer_;
};

}  // namespace slam_core
