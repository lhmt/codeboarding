// Fuses VIO + LIO (+ optional GPS / wheel odometry) into /slam/fused/*.
// Sources go stale after staleness_timeout_s and drop out of the weight
// renormalization; losing all sources reports LOST on /slam/fused/health.

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "fusion_node/fusion_backend.hpp"
#include "slam_interfaces/msg/estimator_state.hpp"
#include "slam_interfaces/msg/pose_health.hpp"

namespace slam_edge {

class FusionNode : public rclcpp::Node {
 public:
  FusionNode() : Node("fusion_node") {
    const auto vio_topic = declare_parameter<std::string>("vio_state_topic", "/slam/vio/state");
    const auto lio_topic = declare_parameter<std::string>("lio_state_topic", "/slam/lio/state");
    const bool use_gps = declare_parameter<bool>("use_gps", false);
    const auto gps_topic = declare_parameter<std::string>("gps_topic", "/gps/fix");
    const bool use_wheel = declare_parameter<bool>("use_wheel_odom", false);
    const auto wheel_topic = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
    const auto backend_name = declare_parameter<std::string>("backend", "weighted");
    vio_weight_ = declare_parameter<double>("vio_weight", 0.5);
    lio_weight_ = declare_parameter<double>("lio_weight", 0.5);
    staleness_timeout_ = declare_parameter<double>("staleness_timeout_s", 0.5);
    const double rate = declare_parameter<double>("publish_rate_hz", 50.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");

    if (backend_name != "weighted") {
      RCLCPP_WARN(get_logger(), "Backend '%s' not implemented yet; using weighted average",
                  backend_name.c_str());
    }
    backend_ = std::make_unique<fusion_node::WeightedAverageBackend>();

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/fused/pose", 10);
    state_pub_ = create_publisher<slam_interfaces::msg::EstimatorState>("/slam/fused/state", 10);
    health_pub_ = create_publisher<slam_interfaces::msg::PoseHealth>("/slam/fused/health", 10);

    vio_sub_ = create_subscription<slam_interfaces::msg::EstimatorState>(
        vio_topic, 10,
        [this](slam_interfaces::msg::EstimatorState::ConstSharedPtr msg) { onEstimatorState("vio", *msg); });
    lio_sub_ = create_subscription<slam_interfaces::msg::EstimatorState>(
        lio_topic, 10,
        [this](slam_interfaces::msg::EstimatorState::ConstSharedPtr msg) { onEstimatorState("lio", *msg); });
    if (use_gps) {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
          gps_topic, rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { onGps(*msg); });
    }
    if (use_wheel) {
      wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          wheel_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onWheelOdom(*msg); });
    }

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate, 1.0)),
                               [this] { publishFused(); });
  }

 private:
  struct TimedSource {
    fusion_node::SourceState state;
    rclcpp::Time received;
  };

  static double toSeconds(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  }

  void onEstimatorState(const std::string& name, const slam_interfaces::msg::EstimatorState& msg) {
    fusion_node::SourceState s;
    s.name = name;
    s.timestamp = toSeconds(msg.header.stamp);
    s.position = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
    s.orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z);
    s.velocity = {msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z};
    s.weight = (name == "vio") ? vio_weight_ : lio_weight_;
    if (msg.estimator_mode == "LOST") s.weight = 0.0;
    store(name, s);
  }

  void onGps(const sensor_msgs::msg::NavSatFix& msg) {
    // TODO(fusion): geodetic -> local ENU conversion anchored at first fix.
    // Until then GPS participates with zero weight so plumbing is exercised
    // without corrupting the fused local-frame estimate.
    fusion_node::SourceState s;
    s.name = "gps";
    s.timestamp = toSeconds(msg.header.stamp);
    s.has_orientation = false;
    s.weight = 0.0;
    store("gps", s);
  }

  void onWheelOdom(const nav_msgs::msg::Odometry& msg) {
    fusion_node::SourceState s;
    s.name = "wheel";
    s.timestamp = toSeconds(msg.header.stamp);
    s.position = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
    s.orientation = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                       msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    s.velocity = {msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z};
    // TODO(fusion): wheel odom drifts in its own frame; needs alignment before
    // it can carry weight. Zero weight keeps it observable but inert.
    s.weight = 0.0;
    store("wheel", s);
  }

  void store(const std::string& name, const fusion_node::SourceState& s) {
    auto it = std::find_if(sources_.begin(), sources_.end(),
                           [&](const TimedSource& t) { return t.state.name == name; });
    if (it == sources_.end()) {
      sources_.push_back({s, now()});
    } else {
      *it = {s, now()};
    }
  }

  void publishFused() {
    std::vector<fusion_node::SourceState> alive;
    for (const auto& t : sources_) {
      if ((now() - t.received).seconds() <= staleness_timeout_) alive.push_back(t.state);
    }
    const auto fused = backend_->fuse(alive);

    slam_interfaces::msg::PoseHealth health;
    health.header.stamp = now();
    health.header.frame_id = world_frame_;

    if (!fused) {
      health.is_lost = true;
      health.tracking_quality = 0.0;
      health.status = "LOST";
      health_pub_->publish(health);
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = health.header;
    pose.pose.position.x = fused->position.x();
    pose.pose.position.y = fused->position.y();
    pose.pose.position.z = fused->position.z();
    pose.pose.orientation.w = fused->orientation.w();
    pose.pose.orientation.x = fused->orientation.x();
    pose.pose.orientation.y = fused->orientation.y();
    pose.pose.orientation.z = fused->orientation.z();
    pose_pub_->publish(pose);

    slam_interfaces::msg::EstimatorState es;
    es.header = pose.header;
    es.pose = pose.pose;
    es.velocity.linear.x = fused->velocity.x();
    es.velocity.linear.y = fused->velocity.y();
    es.velocity.linear.z = fused->velocity.z();
    for (auto& c : es.covariance_position_orientation) c = 0.0;
    es.estimator_mode = fused->num_sources >= 2 ? "TRACKING" : "DEGRADED";
    state_pub_->publish(es);

    health.tracked_features = fused->num_sources;
    health.tracking_quality = fused->num_sources >= 2 ? 1.0 : 0.5;
    health.status = fused->num_sources >= 2 ? "OK" : "WARN_TRACKING";
    health_pub_->publish(health);
  }

  std::unique_ptr<fusion_node::FusionBackend> backend_;
  std::vector<TimedSource> sources_;  // small fixed set; linear scan is deterministic
  double vio_weight_{0.5};
  double lio_weight_{0.5};
  double staleness_timeout_{0.5};
  std::string world_frame_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<slam_interfaces::msg::EstimatorState>::SharedPtr state_pub_;
  rclcpp::Publisher<slam_interfaces::msg::PoseHealth>::SharedPtr health_pub_;
  rclcpp::Subscription<slam_interfaces::msg::EstimatorState>::SharedPtr vio_sub_;
  rclcpp::Subscription<slam_interfaces::msg::EstimatorState>::SharedPtr lio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::FusionNode>());
  rclcpp::shutdown();
  return 0;
}
