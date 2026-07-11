// Local map aggregation/serving skeleton: relays the LIO local map on
// /slam/map/points at a bounded rate.
// EXTENSION POINT: replace the relay with a persistent tiled map (voxel
// hashing / nvblox), keyframe-based map maintenance, and map queries.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace slam_edge {

class MapNode : public rclcpp::Node {
 public:
  MapNode() : Node("map_node") {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/slam/lio/local_map");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/slam/map/points");
    const auto publish_period = declare_parameter<double>("publish_period_s", 1.0);

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(1).reliable());
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::QoS(1).reliable(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { latest_ = msg; });
    timer_ = create_wall_timer(std::chrono::duration<double>(publish_period), [this] {
      if (latest_) map_pub_->publish(*latest_);
    });
  }

 private:
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::MapNode>());
  rclcpp::shutdown();
  return 0;
}
