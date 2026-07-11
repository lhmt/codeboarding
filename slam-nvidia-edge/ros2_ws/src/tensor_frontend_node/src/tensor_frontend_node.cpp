// TensorRT feature frontend: /camera/image -> /slam/frontend/features.
// Features publish as a PointCloud2 with fields (u, v, score); descriptors
// move to a dedicated message once the real engine produces them.

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "tensor_frontend_node/cuda_stream.hpp"
#include "tensor_frontend_node/inference_engine.hpp"

namespace slam_edge {

class TensorFrontendNode : public rclcpp::Node {
 public:
  TensorFrontendNode() : Node("tensor_frontend_node") {
    const auto camera_topic = declare_parameter<std::string>("camera_topic", "/camera/image");
    const auto features_topic = declare_parameter<std::string>("features_topic", "/slam/frontend/features");
    tensor_frontend::EngineConfig config;
    config.engine_path = declare_parameter<std::string>("engine_path", "");
    config.onnx_path = declare_parameter<std::string>("onnx_path", "");
    config.use_fp16 = declare_parameter<bool>("use_fp16", true);
    config.max_keypoints = static_cast<int>(declare_parameter<int64_t>("max_keypoints", 512));
    config.score_threshold = static_cast<float>(declare_parameter<double>("score_threshold", 0.005));
    config.cuda_device_id = static_cast<int>(declare_parameter<int64_t>("cuda_device_id", 0));

    engine_ = tensor_frontend::makeInferenceEngine(config);
    RCLCPP_INFO(get_logger(), "Inference engine: %s (ready=%d)", engine_->describe().c_str(),
                static_cast<int>(engine_->ready()));

    features_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(features_topic, 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(*msg); });
  }

 private:
  void onImage(const sensor_msgs::msg::Image& msg) {
    if (msg.encoding != "mono8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Frontend expects mono8 (got '%s'); insert isaac_ros_image_proc upstream",
                           msg.encoding.c_str());
      return;
    }
    const double t = static_cast<double>(msg.header.stamp.sec) + 1e-9 * msg.header.stamp.nanosec;
    const auto features = engine_->infer(msg.data.data(), msg.width, msg.height, t, stream_);
    publishFeatures(msg.header, features);
  }

  void publishFeatures(const std_msgs::msg::Header& header, const tensor_frontend::FrontendFeatures& features) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header = header;
    msg.height = 1;
    msg.width = static_cast<uint32_t>(features.keypoints.size());
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(features.keypoints.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");
    for (const auto& kp : features.keypoints) {
      *it_x = kp.u;
      *it_y = kp.v;
      *it_z = kp.score;  // z carries the detection score
      ++it_x;
      ++it_y;
      ++it_z;
    }
    features_pub_->publish(msg);
  }

  std::unique_ptr<tensor_frontend::InferenceEngine> engine_;
  tensor_frontend::CudaStream stream_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr features_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

}  // namespace slam_edge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam_edge::TensorFrontendNode>());
  rclcpp::shutdown();
  return 0;
}
