#pragma once

#include "frame.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <bit>
#include <cstring>

namespace raisin::tactile {

// One atomic message: channel 0 is normal force [N], channel 1 validity {0,1}.
// Separate force/validity topics can combine different acquisition instants.
// Versioned topic tactile/v1/<sensor>, encoding 32FC2; not legacy aggregate7.
inline sensor_msgs::msg::Image toImage(const Frame& frame, const Layout& layout) {
  frame.validate(layout);
  if (frame.stamp_s >= std::numeric_limits<std::int32_t>::max())
    throw std::invalid_argument("timestamp exceeds Image seconds range");
  sensor_msgs::msg::Image image;
  image.header.frame_id = layout.frame;
  image.header.stamp.sec = static_cast<std::int32_t>(frame.stamp_s);
  image.header.stamp.nanosec = static_cast<std::uint32_t>(
      (frame.stamp_s - image.header.stamp.sec) * 1e9);
  image.height = layout.rows;
  image.width = layout.cols;
  image.encoding = "32FC2";
  image.is_bigendian = std::endian::native == std::endian::big;
  image.step = layout.cols * 2 * sizeof(float);
  image.data.resize(layout.cells() * 2 * sizeof(float));
  for (std::size_t i = 0; i < layout.cells(); ++i) {
    const float pair[2] = {frame.normal_force_n[i], static_cast<float>(frame.valid[i])};
    std::memcpy(image.data.data() + i * sizeof(pair), pair, sizeof(pair));
  }
  return image;
}

inline Frame fromImage(const sensor_msgs::msg::Image& image, const Layout& layout) {
  layout.validate();
  if (image.header.frame_id != layout.frame || image.encoding != "32FC2" ||
      image.height != static_cast<std::uint32_t>(layout.rows) ||
      image.width != static_cast<std::uint32_t>(layout.cols) ||
      image.step != layout.cols * 2 * sizeof(float) ||
      image.data.size() != layout.cells() * 2 * sizeof(float) ||
      image.header.stamp.sec < 0 || image.header.stamp.nanosec >= 1000000000 ||
      image.is_bigendian > 1)
    throw std::invalid_argument("tactile Image contract mismatch");
  Frame frame;
  frame.stamp_s = image.header.stamp.sec + image.header.stamp.nanosec * 1e-9;
  frame.normal_force_n.resize(layout.cells());
  frame.valid.resize(layout.cells());
  for (std::size_t i = 0; i < layout.cells(); ++i) {
    float pair[2];
    for (std::size_t channel = 0; channel < 2; ++channel) {
      std::uint8_t bytes[sizeof(float)];
      std::memcpy(bytes, image.data.data() + (2 * i + channel) * sizeof(float), sizeof(float));
      if (image.is_bigendian != (std::endian::native == std::endian::big))
        std::reverse(bytes, bytes + sizeof(float));
      std::memcpy(&pair[channel], bytes, sizeof(float));
    }
    if (pair[1] != 0.f && pair[1] != 1.f)
      throw std::invalid_argument("invalid wire validity");
    frame.normal_force_n[i] = pair[0];
    frame.valid[i] = static_cast<std::uint8_t>(pair[1]);
  }
  frame.validate(layout);
  return frame;
}

}  // namespace raisin::tactile
