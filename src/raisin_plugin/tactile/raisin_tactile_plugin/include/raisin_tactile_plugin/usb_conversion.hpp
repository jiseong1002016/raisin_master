#pragma once

#include "raisin_tactile_plugin/usb_backend.hpp"
#include "raisin_tactile_sensor/frame_v2.hpp"
#include "raisin_tactile_sensor/msg/tactile_frame.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace raisin::tactile::usb {

struct ConversionConfig {
  float code_to_force_n_per_code{0.001F};
  std::string calibration_id{"provisional_linear_0p001"};
  std::array<std::int8_t, 2> polarity{1, 1};
};

class FrameConverter {
 public:
  explicit FrameConverter(ConversionConfig config) : config_(std::move(config)) {
    if (!std::isfinite(config_.code_to_force_n_per_code) ||
        config_.code_to_force_n_per_code <= 0 || config_.calibration_id.empty() ||
        (config_.polarity[0] != 1 && config_.polarity[0] != -1) ||
        (config_.polarity[1] != 1 && config_.polarity[1] != -1))
      throw std::invalid_argument("invalid tactile force conversion configuration");
  }

  bool requestBaseline(const std::string& request) {
    std::lock_guard lock(mutex_);
    if (request == "capture_baseline:left") {
      sensors_[0].capture_requested = true;
      return true;
    }
    if (request == "capture_baseline:right") {
      sensors_[1].capture_requested = true;
      return true;
    }
    if (request == "capture_baseline:both") {
      sensors_[0].capture_requested = true;
      sensors_[1].capture_requested = true;
      return true;
    }
    return false;
  }

  raisin_tactile_sensor::msg::TactileFrame convert(const RawFrame& frame) {
    std::lock_guard lock(mutex_);
    const auto index = sensorIndex(frame.sensor);
    auto& state = sensors_[index];
    if (state.session_id != frame.session_id) {
      state.session_id = frame.session_id;
      state.baseline.fill(0);
      state.baseline_valid = false;
      state.capture_requested = false;
    }
    if (state.capture_requested && frame.packet_mask == kFullPacketMask) {
      state.baseline = frame.raw_code;
      state.baseline_valid = true;
      state.capture_requested = false;
    }

    raisin_tactile_sensor::msg::TactileFrame message;
    message.header.frame_id = "tactile_" + frame.sensor + "_frame";
    const auto stamp_sec = frame.stamp_monotonic_ns / 1000000000ULL;
    if (stamp_sec > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()))
      throw std::invalid_argument("monotonic timestamp exceeds Raisin Header range");
    message.header.stamp.sec = static_cast<std::int32_t>(stamp_sec);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(
        frame.stamp_monotonic_ns % 1000000000ULL);
    message.sensor_name = frame.sensor;
    message.session_id = frame.session_id;
    message.sequence = frame.sequence;
    message.clock_domain = "CLOCK_MONOTONIC";
    message.rows = kRows;
    message.cols = kCols;
    message.code_to_force_n_per_code = config_.code_to_force_n_per_code;
    message.polarity = config_.polarity[index];
    message.calibration_id = config_.calibration_id;
    message.calibration_status = "provisional";
    message.baseline_status = state.baseline_valid
                                  ? "operator_confirmed_unloaded"
                                  : "awaiting_unloaded_baseline";
    message.baseline_valid = state.baseline_valid;
    message.force_unit = "N_estimate";
    message.raw_code.assign(frame.raw_code.begin(), frame.raw_code.end());
    message.baseline_code.assign(state.baseline.begin(), state.baseline.end());
    message.normal_force_n.resize(kMapPoints, 0.F);
    message.valid.resize(kMapPoints, 0);
    for (std::size_t row = 0; row < kRows; ++row) {
      for (std::size_t col = 0; col < kCols; ++col) {
        const auto cell = row * kCols + col;
        const auto packet = (col * kRows + row) / kPointsPerPacket;
        if (!state.baseline_valid || !isUsbRoiCell(row, col) || col == 40 ||
            (frame.packet_mask & (1U << packet)) == 0)
          continue;
        const auto delta = static_cast<std::int32_t>(config_.polarity[index]) *
                           (static_cast<std::int32_t>(frame.raw_code[cell]) -
                            static_cast<std::int32_t>(state.baseline[cell]));
        message.normal_force_n[cell] = config_.code_to_force_n_per_code *
                                       static_cast<float>(std::max(delta, 0));
        message.valid[cell] = 1;
      }
    }
    return message;
  }

 private:
  struct SensorState {
    std::uint64_t session_id{};
    bool baseline_valid{};
    bool capture_requested{};
    Grid baseline{};
  };

  static std::size_t sensorIndex(const std::string& name) {
    if (name == "left") return 0;
    if (name == "right") return 1;
    throw std::invalid_argument("unknown tactile sensor identity");
  }

  ConversionConfig config_;
  std::array<SensorState, 2> sensors_{};
  std::mutex mutex_;
};

}  // namespace raisin::tactile::usb
