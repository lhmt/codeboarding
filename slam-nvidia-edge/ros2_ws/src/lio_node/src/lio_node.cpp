// LIO skeleton: LiDAR + IMU -> pose/state/health/local_map.
// Per scan: parse xyz -> IMU preintegrate over the scan gap -> deskew ->
// voxel downsample -> predict -> point-to-plane ICP vs local map ->
// degeneracy check -> map insert -> publish.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "lio_node/degeneracy_detector.hpp"
#include "lio_node/local_map_manager.hpp"
#include "lio_node/point_to_plane_icp.hpp"
#include "lio_node/scan_deskew.hpp"
#include "lio_node/voxel_downsampler.hpp"
#include "slam_core/health_rules.hpp"
#include "slam_core/imu_buffer.hpp"
#include "slam_core/imu_preintegration.hpp"
#include "slam_core/lie_group.hpp"
#include "slam_interfaces/msg/estimator_state.hpp"
#include "slam_interfaces/msg/pose_health.hpp"

namespace slam_edge {

class LioNode : public rclcpp::Node {
 public:
  LioNode() : Node("lio_node") {
    const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/lidar/points");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    scan_period_ = declare_parameter<double>("scan_period_s", 0.1);
    const auto scan_voxel = declare_parameter<double>("scan_voxel_size_m", 0.25);
    const auto map_voxel = declare_parameter<double>("map_voxel_size_m", 0.5);
    const auto icp_iters = static_cast<int>(declare_parameter<int64_t>("icp_max_iterations", 20));
    const auto icp_dist = declare_parameter<double>("icp_max_correspondence_dist_m", 1.0);
    const auto map_radius = declare_parameter<double>("local_map_radius_m", 100.0);
    const auto map_max_points = static_cast<std::size_t>(declare_parameter<int64_t>("local_map_max_points", 500000));
    const auto map_period = declare_parameter<double>("map_publish_period_s", 1.0);
    const auto degeneracy_threshold = declare_parameter<double>("degeneracy_condition_number_threshold", 1e10);
    const auto imu_horizon = declare_parameter<double>("imu_buffer_seconds", 10.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");

    scan_downsampler_ = std::make_unique<lio_node::VoxelDownsampler>(scan_voxel);
    icp_ = std::make_unique<lio_node::PointToPlaneIcp>(icp_iters, icp_dist);
    local_map_ = std::make_unique<lio_node::LocalMapManager>(map_voxel, map_radius, map_max_points);
    degeneracy_ = std::make_unique<lio_node::DegeneracyDetector>(degeneracy_threshold);
    imu_buffer_ = std::make_unique<slam_core::ImuBuffer>(imu_horizon);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/lio/pose", 10);
    state_pub_ = create_publisher<slam_interfaces::msg::EstimatorState>("/slam/lio/state", 10);
    health_pub_ = create_publisher<slam_interfaces::msg::PoseHealth>("/slam/lio/health", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/slam/lio/local_map",
                                                               rclcpp::QoS(1).reliable());

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { onScan(*msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { onImu(*msg); });
    map_timer_ = create_wall_timer(std::chrono::duration<double>(map_period), [this] { publishMap(); });
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

  static std::vector<Eigen::Vector3d> parseXyz(const sensor_msgs::msg::PointCloud2& msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(msg.width) * msg.height);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      if (std::isfinite(*it_x) && std::isfinite(*it_y) && std::isfinite(*it_z)) {
        points.emplace_back(*it_x, *it_y, *it_z);
      }
    }
    return points;
  }

  void onScan(const sensor_msgs::msg::PointCloud2& msg) {
    const double scan_time = toSeconds(msg.header.stamp);
    auto points = parseXyz(msg);
    if (points.empty()) {
      publishOutputs(msg.header.stamp, /*fitness=*/0.0, /*condition_number=*/1e18, /*degenerate=*/true,
                     /*lost=*/true, 0);
      return;
    }

    // IMU motion since the previous scan (also covers the sweep for deskew).
    slam_core::ImuPreintegrator preint(bias_);
    if (last_scan_time_ > 0.0) {
      double prev_t = last_scan_time_;
      for (const auto& s : imu_buffer_->range(last_scan_time_, scan_time)) {
        preint.integrate(s, s.timestamp - prev_t);
        prev_t = s.timestamp;
      }
    }

    points = deskewer_.deskew(std::move(points), preint.deltaRotation(), scan_period_);
    const auto downsampled = scan_downsampler_->filter(points);

    // Predict with IMU, correct with ICP (identity increment until real solver lands).
    const Eigen::Matrix3d predicted_r = rotation_ * preint.deltaRotation();
    const Eigen::Vector3d predicted_p = position_ + rotation_ * preint.deltaPosition();
    std::vector<Eigen::Vector3d> world_points;
    world_points.reserve(downsampled.size());
    for (const auto& p : downsampled) world_points.push_back(predicted_r * p + predicted_p);

    const auto icp = icp_->align(world_points, local_map_->points());
    const auto degeneracy = degeneracy_->analyze(icp.hessian);

    // Apply the ICP increment, but along unobservable eigen-directions keep the
    // IMU prediction (Zhang & Singh remapping) so corridors/tunnels don't drift.
    Eigen::Matrix3d applied_r = icp.rotation_increment;
    Eigen::Vector3d applied_t = icp.translation_increment;
    if (degeneracy.is_degenerate) {
      Eigen::Matrix<double, 6, 1> dx;
      dx.head<3>() = slam_core::logSO3(icp.rotation_increment);
      dx.tail<3>() = icp.translation_increment;
      const Eigen::Matrix<double, 6, 1> dx_remapped = degeneracy_->remap(dx, icp.hessian);
      applied_r = slam_core::expSO3(dx_remapped.head<3>());
      applied_t = dx_remapped.tail<3>();
    }
    rotation_ = applied_r * predicted_r;
    position_ = applied_r * predicted_p + applied_t;
    local_map_->insert(world_points, position_);

    publishOutputs(msg.header.stamp, icp.fitness, degeneracy.condition_number, degeneracy.is_degenerate,
                   /*lost=*/!icp.converged && !local_map_->empty(), static_cast<int>(downsampled.size()));
    last_scan_time_ = scan_time;
  }

  void publishOutputs(const builtin_interfaces::msg::Time& stamp, double fitness, double condition_number,
                      bool degenerate, bool lost, int num_points) {
    const Eigen::Quaterniond q(rotation_);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = position_.x();
    pose.pose.position.y = position_.y();
    pose.pose.position.z = position_.z();
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose_pub_->publish(pose);

    slam_interfaces::msg::EstimatorState es;
    es.header = pose.header;
    es.pose = pose.pose;
    for (auto& c : es.covariance_position_orientation) c = 0.0;
    es.estimator_mode = lost ? "LOST" : (degenerate ? "DEGRADED" : "TRACKING");
    state_pub_->publish(es);

    slam_interfaces::msg::PoseHealth health;
    health.header = pose.header;
    // LIO reuses tracked_features for registered downsampled points.
    health.tracked_features = num_points;
    health.reprojection_error_median = 0.0;
    health.imu_gyro_bias_norm = bias_.gyro.norm();
    health.imu_accel_bias_norm = bias_.accel.norm();
    health.condition_number = condition_number;
    health.is_degenerate = degenerate;
    health.is_lost = lost;
    health.tracking_quality = lost ? 0.0 : fitness;
    health.status = lost ? "LOST" : (degenerate ? "WARN_DEGENERACY" : "OK");
    health_pub_->publish(health);
  }

  void publishMap() {
    const auto& pts = local_map_->points();
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = world_frame_;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(pts.size());
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    for (const auto& p : pts) {
      *it_x = static_cast<float>(p.x());
      *it_y = static_cast<float>(p.y());
      *it_z = static_cast<float>(p.z());
      ++it_x;
      ++it_y;
      ++it_z;
    }
    map_pub_->publish(msg);
  }

  lio_node::ScanDeskewer deskewer_;
  std::unique_ptr<lio_node::VoxelDownsampler> scan_downsampler_;
  std::unique_ptr<lio_node::PointToPlaneIcp> icp_;
  std::unique_ptr<lio_node::LocalMapManager> local_map_;
  std::unique_ptr<lio_node::DegeneracyDetector> degeneracy_;
  std::unique_ptr<slam_core::ImuBuffer> imu_buffer_;
  slam_core::ImuBias bias_;

  Eigen::Matrix3d rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  double last_scan_time_{0.0};
  double scan_period_{0.1};
  std::string world_frame_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<slam_interfaces::msg::EstimatorState>::SharedPtr state_pub_;
  rclcpp::Publisher<slam_interfaces::msg::PoseHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::LioNode>());
  rclcpp::shutdown();
  return 0;
}
