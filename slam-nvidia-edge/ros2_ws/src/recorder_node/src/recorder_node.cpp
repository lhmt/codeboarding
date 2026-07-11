// Writes local artifacts for later (out-of-band) cloud upload:
//   logs/health/<session>.jsonl            PoseHealth samples
//   logs/trajectories/<session>.tum        fused trajectory, TUM format
//   logs/calibration_snapshots/<session>_camera.json  first CameraInfo seen
// MCAP bags are handled by scripts/record_mcap.sh into logs/mcap/.
// Hard boundary: local filesystem only — no AWS SDK, credentials, or network.

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "slam_interfaces/msg/estimator_state.hpp"
#include "slam_interfaces/msg/pose_health.hpp"

namespace slam_edge {

class RecorderNode : public rclcpp::Node {
 public:
  RecorderNode() : Node("recorder_node") {
    const auto output_dir = declare_parameter<std::string>("output_dir", "logs");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/slam/fused/state");
    const auto camera_info_topic = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    const auto health_topics = declare_parameter<std::vector<std::string>>(
        "health_topics", {"/slam/vio/health", "/slam/lio/health", "/slam/fused/health", "/slam/health/global"});

    session_ = sessionStamp();
    const std::filesystem::path root(output_dir);
    for (const char* sub : {"mcap", "health", "trajectories", "calibration_snapshots"}) {
      std::filesystem::create_directories(root / sub);
    }

    health_file_.open(root / "health" / (session_ + ".jsonl"), std::ios::app);
    trajectory_file_.open(root / "trajectories" / (session_ + ".tum"), std::ios::app);
    calibration_path_ = root / "calibration_snapshots" / (session_ + "_camera.json");
    if (!health_file_ || !trajectory_file_) {
      RCLCPP_ERROR(get_logger(), "Cannot open log files under '%s'", output_dir.c_str());
    }

    for (const auto& topic : health_topics) {
      health_subs_.push_back(create_subscription<slam_interfaces::msg::PoseHealth>(
          topic, 10,
          [this, topic](slam_interfaces::msg::PoseHealth::ConstSharedPtr msg) { onHealth(topic, *msg); }));
    }
    state_sub_ = create_subscription<slam_interfaces::msg::EstimatorState>(
        state_topic, 10,
        [this](slam_interfaces::msg::EstimatorState::ConstSharedPtr msg) { onState(*msg); });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { onCameraInfo(*msg); });
  }

 private:
  std::string sessionStamp() {
    // Wall-clock session id, filesystem-safe: YYYYMMDDTHHMMSS.
    const auto t = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&t, &tm);
    std::ostringstream out;
    out << std::put_time(&tm, "%Y%m%dT%H%M%S");
    return out.str();
  }

  static double toSeconds(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  }

  static std::string escapeJson(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (const char c : in) {
      if (c == '"' || c == '\\') out += '\\';
      out += c;
    }
    return out;
  }

  void onHealth(const std::string& topic, const slam_interfaces::msg::PoseHealth& m) {
    if (!health_file_) return;
    health_file_ << std::setprecision(17) << "{\"topic\":\"" << escapeJson(topic) << "\""
                 << ",\"t\":" << toSeconds(m.header.stamp) << ",\"tracking_quality\":" << m.tracking_quality
                 << ",\"tracked_features\":" << m.tracked_features
                 << ",\"reprojection_error_median\":" << m.reprojection_error_median
                 << ",\"gyro_bias_norm\":" << m.imu_gyro_bias_norm
                 << ",\"accel_bias_norm\":" << m.imu_accel_bias_norm
                 << ",\"condition_number\":" << m.condition_number
                 << ",\"is_degenerate\":" << (m.is_degenerate ? "true" : "false")
                 << ",\"is_lost\":" << (m.is_lost ? "true" : "false") << ",\"status\":\""
                 << escapeJson(m.status) << "\"}\n";
    health_file_.flush();
  }

  void onState(const slam_interfaces::msg::EstimatorState& m) {
    if (!trajectory_file_) return;
    // TUM: timestamp tx ty tz qx qy qz qw — consumed directly by evo/SageMaker.
    trajectory_file_ << std::setprecision(17) << toSeconds(m.header.stamp) << " " << m.pose.position.x << " "
                     << m.pose.position.y << " " << m.pose.position.z << " " << m.pose.orientation.x << " "
                     << m.pose.orientation.y << " " << m.pose.orientation.z << " " << m.pose.orientation.w
                     << "\n";
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo& m) {
    if (calibration_written_) return;
    std::ofstream out(calibration_path_);
    if (!out) return;
    out << std::setprecision(17) << "{\"width\":" << m.width << ",\"height\":" << m.height
        << ",\"distortion_model\":\"" << escapeJson(m.distortion_model) << "\",\"k\":[";
    for (std::size_t i = 0; i < m.k.size(); ++i) out << (i ? "," : "") << m.k[i];
    out << "],\"d\":[";
    for (std::size_t i = 0; i < m.d.size(); ++i) out << (i ? "," : "") << m.d[i];
    out << "]}\n";
    calibration_written_ = true;
    RCLCPP_INFO(get_logger(), "Calibration snapshot written to %s", calibration_path_.c_str());
  }

  std::string session_;
  std::ofstream health_file_;
  std::ofstream trajectory_file_;
  std::filesystem::path calibration_path_;
  bool calibration_written_{false};

  std::vector<rclcpp::Subscription<slam_interfaces::msg::PoseHealth>::SharedPtr> health_subs_;
  rclcpp::Subscription<slam_interfaces::msg::EstimatorState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::RecorderNode>());
  rclcpp::shutdown();
  return 0;
}
