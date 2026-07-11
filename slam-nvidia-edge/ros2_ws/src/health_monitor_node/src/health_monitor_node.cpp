// Aggregates per-estimator PoseHealth into /slam/health/global.
// Rule logic lives in slam_core/health_rules.hpp (pure, unit-tested); this
// node only handles ROS plumbing and staleness tracking.

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "slam_core/health_rules.hpp"
#include "slam_interfaces/msg/pose_health.hpp"

namespace slam_edge {

class HealthMonitorNode : public rclcpp::Node {
 public:
  HealthMonitorNode() : Node("health_monitor_node") {
    const auto topics = declare_parameter<std::vector<std::string>>(
        "input_topics", {"/slam/vio/health", "/slam/lio/health", "/slam/fused/health"});
    thresholds_.min_tracked_features = static_cast<int>(declare_parameter<int64_t>("min_tracked_features", 30));
    thresholds_.max_reprojection_error_median = declare_parameter<double>("max_reprojection_error_median", 3.0);
    thresholds_.max_gyro_bias_norm = declare_parameter<double>("max_gyro_bias_norm", 0.1);
    thresholds_.max_accel_bias_norm = declare_parameter<double>("max_accel_bias_norm", 1.0);
    thresholds_.max_condition_number = declare_parameter<double>("max_condition_number", 1e10);
    staleness_timeout_ = declare_parameter<double>("staleness_timeout_s", 1.0);
    const double rate = declare_parameter<double>("publish_rate_hz", 10.0);

    global_pub_ = create_publisher<slam_interfaces::msg::PoseHealth>("/slam/health/global", 10);
    for (const auto& topic : topics) {
      subs_.push_back(create_subscription<slam_interfaces::msg::PoseHealth>(
          topic, 10,
          [this, topic](slam_interfaces::msg::PoseHealth::ConstSharedPtr msg) { onHealth(topic, msg); }));
      last_msg_[topic] = std::nullopt;
    }
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate, 1.0)),
                               [this] { publishGlobal(); });
  }

 private:
  struct SourceRecord {
    slam_interfaces::msg::PoseHealth msg;
    rclcpp::Time received;
  };

  void onHealth(const std::string& topic, slam_interfaces::msg::PoseHealth::ConstSharedPtr msg) {
    last_msg_[topic] = SourceRecord{*msg, now()};
  }

  void publishGlobal() {
    slam_interfaces::msg::PoseHealth global;
    global.header.stamp = now();
    global.header.frame_id = "global";

    // Worst-case aggregation across sources, evaluated in fixed topic order so
    // the output status string is deterministic for a given input set.
    std::vector<std::string> parts;
    bool any_lost = false;
    bool any_degenerate = false;
    bool any_alive = false;
    double worst_quality = 1.0;
    int32_t min_features = std::numeric_limits<int32_t>::max();

    for (auto& [topic, record] : last_msg_) {
      if (!record.has_value()) {
        parts.push_back(sourceName(topic) + ":STALE");
        continue;
      }
      if ((now() - record->received).seconds() > staleness_timeout_) {
        parts.push_back(sourceName(topic) + ":STALE");
        continue;
      }
      any_alive = true;
      const auto& m = record->msg;
      slam_core::HealthSample sample;
      sample.tracked_features = m.tracked_features;
      sample.reprojection_error_median = m.reprojection_error_median;
      sample.gyro_bias_norm = m.imu_gyro_bias_norm;
      sample.accel_bias_norm = m.imu_accel_bias_norm;
      sample.condition_number = m.condition_number;
      sample.is_lost = m.is_lost;
      const auto verdict = slam_core::evaluateHealth(sample, thresholds_);

      any_lost = any_lost || verdict.is_lost;
      any_degenerate = any_degenerate || verdict.is_degenerate || m.is_degenerate;
      worst_quality = std::min(worst_quality, m.tracking_quality);
      // Mirror the worst offender's raw metrics so downstream sees why.
      min_features = std::min(min_features, m.tracked_features);
      global.reprojection_error_median = std::max(global.reprojection_error_median, m.reprojection_error_median);
      global.imu_gyro_bias_norm = std::max(global.imu_gyro_bias_norm, m.imu_gyro_bias_norm);
      global.imu_accel_bias_norm = std::max(global.imu_accel_bias_norm, m.imu_accel_bias_norm);
      global.condition_number = std::max(global.condition_number, m.condition_number);
      parts.push_back(sourceName(topic) + ":" + verdict.status());
    }

    global.tracking_quality = any_alive ? worst_quality : 0.0;
    global.tracked_features = any_alive ? min_features : 0;
    global.is_lost = any_lost || !any_alive;
    global.is_degenerate = any_degenerate;
    global.status = global.is_lost ? "LOST" : joinParts(parts);
    global_pub_->publish(global);
  }

  static std::string sourceName(const std::string& topic) {
    // "/slam/vio/health" -> "vio"
    const auto end = topic.rfind('/');
    const auto begin = topic.rfind('/', end == 0 ? 0 : end - 1);
    if (end == std::string::npos || begin == std::string::npos || end <= begin + 1) return topic;
    return topic.substr(begin + 1, end - begin - 1);
  }

  static std::string joinParts(const std::vector<std::string>& parts) {
    std::string out;
    for (const auto& p : parts) {
      if (!out.empty()) out += ",";
      out += p;
    }
    return out.empty() ? "OK" : out;
  }

  slam_core::HealthThresholds thresholds_;
  double staleness_timeout_{1.0};
  std::map<std::string, std::optional<SourceRecord>> last_msg_;
  std::vector<rclcpp::Subscription<slam_interfaces::msg::PoseHealth>::SharedPtr> subs_;
  rclcpp::Publisher<slam_interfaces::msg::PoseHealth>::SharedPtr global_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::HealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
