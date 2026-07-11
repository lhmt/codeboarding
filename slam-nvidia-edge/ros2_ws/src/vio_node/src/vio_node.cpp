// VIO skeleton: camera + IMU -> pose/state/health.
// Pipeline per frame: preprocess -> track -> IMU preintegrate (last frame ->
// this frame) -> keyframe policy -> sliding-window update -> publish.

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "slam_core/health_rules.hpp"
#include "slam_core/imu_buffer.hpp"
#include "slam_core/imu_preintegration.hpp"
#include "slam_core/lie_group.hpp"
#include "slam_interfaces/msg/estimator_state.hpp"
#include "slam_interfaces/msg/pose_health.hpp"
#include "slam_interfaces/msg/sensor_sync_status.hpp"
#include "vio_node/feature_tracker.hpp"
#include "vio_node/image_preprocessor.hpp"
#include "vio_node/keyframe_policy.hpp"
#include "vio_node/sliding_window_estimator.hpp"

namespace slam_edge {

class VioNode : public rclcpp::Node {
 public:
  VioNode() : Node("vio_node") {
    const auto camera_topic = declare_parameter<std::string>("camera_topic", "/camera/image");
    const auto camera_info_topic = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const auto max_features = static_cast<int>(declare_parameter<int64_t>("max_features", 200));
    const auto imu_horizon = declare_parameter<double>("imu_buffer_seconds", 10.0);
    const auto gravity = declare_parameter<double>("gravity_magnitude", 9.80665);
    const auto window_size = static_cast<std::size_t>(declare_parameter<int64_t>("window_size", 10));
    const auto kf_interval = declare_parameter<double>("keyframe_min_interval_s", 0.25);
    const auto kf_translation = declare_parameter<double>("keyframe_min_translation_m", 0.15);
    const auto kf_rotation = declare_parameter<double>("keyframe_min_rotation_rad", 0.15);
    time_offset_ms_ = declare_parameter<double>("time_offset_camera_to_imu_ms", 0.0);
    max_sync_offset_ms_ = declare_parameter<double>("max_sync_offset_ms", 5.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");

    tracker_ = std::make_unique<vio_node::FeatureTracker>(max_features);
    imu_buffer_ = std::make_unique<slam_core::ImuBuffer>(imu_horizon);
    keyframe_policy_ = std::make_unique<vio_node::KeyframePolicy>(kf_interval, kf_translation, kf_rotation);
    estimator_ = std::make_unique<vio_node::SlidingWindowEstimator>(window_size, gravity);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/vio/pose", 10);
    state_pub_ = create_publisher<slam_interfaces::msg::EstimatorState>("/slam/vio/state", 10);
    health_pub_ = create_publisher<slam_interfaces::msg::PoseHealth>("/slam/vio/health", 10);
    sync_pub_ = create_publisher<slam_interfaces::msg::SensorSyncStatus>("/slam/sync/status", 10);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(*msg); });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { camera_info_ = *msg; });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { onImu(*msg); });
  }

 private:
  static double toSeconds(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  }

  void onImu(const sensor_msgs::msg::Imu& msg) {
    slam_core::ImuSample sample;
    sample.timestamp = toSeconds(msg.header.stamp);
    sample.gyro = {msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
    sample.accel = {msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};
    imu_buffer_->push(sample);
  }

  void onImage(const sensor_msgs::msg::Image& msg) {
    const auto image = preprocessor_.process(msg);
    if (!image) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported image encoding '%s'",
                           msg.encoding.c_str());
      return;
    }
    const double frame_time = image->timestamp + 1e-3 * time_offset_ms_;

    // Preintegrate IMU covering (last frame, this frame].
    slam_core::ImuPreintegrator preint(current_bias_);
    if (last_frame_time_ > 0.0) {
      double prev_t = last_frame_time_;
      for (const auto& s : imu_buffer_->range(last_frame_time_, frame_time)) {
        preint.integrate(s, s.timestamp - prev_t);
        prev_t = s.timestamp;
      }
    }

    const auto tracks = tracker_->track(*image);
    const Eigen::Vector3d translation = preint.deltaPosition();
    const double rotation_angle = slam_core::logSO3(preint.deltaRotation()).norm();
    const bool is_keyframe = keyframe_policy_->shouldInsert(frame_time, translation, rotation_angle);
    const auto state = estimator_->update(tracks, preint, is_keyframe);

    publishOutputs(msg.header.stamp, state, tracks);
    publishSyncStatus(msg.header.stamp, frame_time);
    last_frame_time_ = frame_time;
  }

  void publishOutputs(const builtin_interfaces::msg::Time& stamp, const vio_node::EstimatorOutput& state,
                      const vio_node::TrackResult& tracks) {
    const Eigen::Quaterniond q(state.rotation);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = state.position.x();
    pose.pose.position.y = state.position.y();
    pose.pose.position.z = state.position.z();
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose_pub_->publish(pose);

    slam_interfaces::msg::EstimatorState es;
    es.header = pose.header;
    es.pose = pose.pose;
    es.velocity.linear.x = state.velocity.x();
    es.velocity.linear.y = state.velocity.y();
    es.velocity.linear.z = state.velocity.z();
    for (auto& c : es.covariance_position_orientation) c = 0.0;
    es.estimator_mode = state.is_lost ? "LOST" : (last_frame_time_ > 0.0 ? "TRACKING" : "INITIALIZING");
    state_pub_->publish(es);

    slam_core::HealthSample sample;
    sample.tracked_features = static_cast<int>(tracks.features.size());
    sample.reprojection_error_median = tracks.median_reprojection_error_px;
    sample.gyro_bias_norm = state.bias.gyro.norm();
    sample.accel_bias_norm = state.bias.accel.norm();
    sample.condition_number = state.condition_number;
    sample.is_lost = state.is_lost;
    const auto verdict = slam_core::evaluateHealth(sample);

    slam_interfaces::msg::PoseHealth health;
    health.header = pose.header;
    health.tracked_features = sample.tracked_features;
    health.reprojection_error_median = sample.reprojection_error_median;
    health.imu_gyro_bias_norm = sample.gyro_bias_norm;
    health.imu_accel_bias_norm = sample.accel_bias_norm;
    health.condition_number = sample.condition_number;
    health.is_degenerate = verdict.is_degenerate;
    health.is_lost = verdict.is_lost;
    health.status = verdict.status();
    health.tracking_quality = verdict.ok() ? 1.0 : (verdict.is_lost ? 0.0 : 0.5);
    health_pub_->publish(health);
  }

  void publishSyncStatus(const builtin_interfaces::msg::Time& stamp, double frame_time) {
    slam_interfaces::msg::SensorSyncStatus sync;
    sync.header.stamp = stamp;
    sync.header.frame_id = body_frame_;
    const double imu_latest = imu_buffer_->latestTimestamp();
    sync.camera_imu_offset_ms = imu_buffer_->empty() ? 1e6 : 1e3 * (imu_latest - frame_time);
    sync.lidar_imu_offset_ms = 0.0;  // owned by lio_node; kept for message completeness
    sync.sync_ok = !imu_buffer_->empty() && std::abs(sync.camera_imu_offset_ms) < max_sync_offset_ms_ * 10.0;
    sync_pub_->publish(sync);
  }

  vio_node::ImagePreprocessor preprocessor_;
  std::unique_ptr<vio_node::FeatureTracker> tracker_;
  std::unique_ptr<slam_core::ImuBuffer> imu_buffer_;
  std::unique_ptr<vio_node::KeyframePolicy> keyframe_policy_;
  std::unique_ptr<vio_node::SlidingWindowEstimator> estimator_;
  slam_core::ImuBias current_bias_;  // updated by the estimator once bias estimation lands
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;

  double last_frame_time_{0.0};
  double time_offset_ms_{0.0};
  double max_sync_offset_ms_{5.0};
  std::string world_frame_;
  std::string body_frame_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<slam_interfaces::msg::EstimatorState>::SharedPtr state_pub_;
  rclcpp::Publisher<slam_interfaces::msg::PoseHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<slam_interfaces::msg::SensorSyncStatus>::SharedPtr sync_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::VioNode>());
  rclcpp::shutdown();
  return 0;
}
