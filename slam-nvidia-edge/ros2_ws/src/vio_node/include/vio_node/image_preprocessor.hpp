#pragma once

// Converts incoming images to grayscale working buffers.
// EXTENSION POINT: swap for isaac_ros_image_proc (GPU rectify/debayer/CLAHE)
// or a VPI pipeline; keep PreprocessedImage as the stable output contract.

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/image.hpp>

namespace vio_node {

struct PreprocessedImage {
  double timestamp{0.0};
  uint32_t width{0};
  uint32_t height{0};
  std::vector<uint8_t> gray;  // row-major, width*height
};

class ImagePreprocessor {
 public:
  std::optional<PreprocessedImage> process(const sensor_msgs::msg::Image& msg) const {
    PreprocessedImage out;
    out.timestamp = static_cast<double>(msg.header.stamp.sec) + 1e-9 * msg.header.stamp.nanosec;
    out.width = msg.width;
    out.height = msg.height;
    out.gray.resize(static_cast<std::size_t>(msg.width) * msg.height);

    if (msg.encoding == "mono8") {
      for (uint32_t r = 0; r < msg.height; ++r) {
        const uint8_t* src = msg.data.data() + static_cast<std::size_t>(r) * msg.step;
        std::copy(src, src + msg.width, out.gray.data() + static_cast<std::size_t>(r) * msg.width);
      }
      return out;
    }
    if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
      // Integer BT.601 luma; channel order does not matter enough for tracking.
      for (uint32_t r = 0; r < msg.height; ++r) {
        const uint8_t* src = msg.data.data() + static_cast<std::size_t>(r) * msg.step;
        uint8_t* dst = out.gray.data() + static_cast<std::size_t>(r) * msg.width;
        for (uint32_t c = 0; c < msg.width; ++c) {
          const uint8_t* px = src + 3 * c;
          dst[c] = static_cast<uint8_t>((77 * px[0] + 150 * px[1] + 29 * px[2]) >> 8);
        }
      }
      return out;
    }
    return std::nullopt;  // unsupported encoding; caller logs and skips frame
  }
};

}  // namespace vio_node
