#pragma once

#include "raisin_tactile_sensor/msg/tactile_frame.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace raisin::tactile {

inline constexpr std::uint32_t kUsbRows = 71;
inline constexpr std::uint32_t kUsbCols = 41;
inline constexpr std::size_t kUsbCells = kUsbRows * kUsbCols;
inline constexpr std::uint32_t kUsbRoiRxFirst = 36;
inline constexpr std::uint32_t kUsbRoiRxLast = 70;
inline constexpr std::uint32_t kUsbRoiTxFirst = 18;
inline constexpr std::uint32_t kUsbRoiTxLast = 39;
inline constexpr std::uint32_t kUsbRoiRxCount = kUsbRoiRxLast - kUsbRoiRxFirst + 1;
inline constexpr std::uint32_t kUsbRoiTxCount = kUsbRoiTxLast - kUsbRoiTxFirst + 1;
inline constexpr std::size_t kUsbRoiCells = kUsbRoiRxCount * kUsbRoiTxCount;

constexpr bool isUsbRoiCell(std::uint32_t row, std::uint32_t col) noexcept {
  return row >= kUsbRoiRxFirst && row <= kUsbRoiRxLast &&
         col >= kUsbRoiTxFirst && col <= kUsbRoiTxLast;
}

struct FrameV2 {
  std::string sensor_name;
  std::string frame_id;
  std::string clock_domain;
  std::string calibration_id;
  std::string calibration_status;
  std::string baseline_status;
  std::string force_unit;
  std::uint64_t stamp_monotonic_ns{};
  std::uint64_t session_id{};
  std::uint64_t sequence{};
  std::uint32_t rows{};
  std::uint32_t cols{};
  float code_to_force_n_per_code{};
  std::int8_t polarity{};
  bool baseline_valid{};
  std::vector<std::int16_t> raw_code;
  std::vector<std::int16_t> baseline_code;
  std::vector<float> normal_force_n;
  std::vector<std::uint8_t> valid;

  void validate(const std::string& expected_sensor) const {
    if ((expected_sensor != "left" && expected_sensor != "right") ||
        sensor_name != expected_sensor || frame_id != "tactile_" + expected_sensor + "_frame" ||
        clock_domain != "CLOCK_MONOTONIC" || session_id == 0 || stamp_monotonic_ns == 0 ||
        stamp_monotonic_ns / 1000000000ULL >
            static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max()) ||
        rows != kUsbRows ||
        cols != kUsbCols || raw_code.size() != kUsbCells ||
        baseline_code.size() != kUsbCells || normal_force_n.size() != kUsbCells ||
        valid.size() != kUsbCells || !std::isfinite(code_to_force_n_per_code) ||
        code_to_force_n_per_code <= 0 || (polarity != 1 && polarity != -1) ||
        calibration_id.empty() || calibration_status != "provisional" ||
        force_unit != "N_estimate" ||
        (baseline_valid && baseline_status != "operator_confirmed_unloaded") ||
        (!baseline_valid && baseline_status != "awaiting_unloaded_baseline"))
      throw std::invalid_argument("invalid tactile v2 frame contract");
    for (std::size_t row = 0; row < rows; ++row) {
      for (std::size_t col = 0; col < cols; ++col) {
        const auto cell = row * cols + col;
        if (valid[cell] > 1 || !std::isfinite(normal_force_n[cell]) ||
            normal_force_n[cell] < 0 || (!baseline_valid && valid[cell] != 0) ||
            (col == cols - 1 && valid[cell] != 0))
          throw std::invalid_argument("invalid tactile v2 force map or validity mask");
      }
    }
  }
};

inline FrameV2 fromMessage(const raisin_tactile_sensor::msg::TactileFrame& message,
                           const std::string& expected_sensor) {
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000)
    throw std::invalid_argument("invalid tactile v2 timestamp");
  FrameV2 frame;
  frame.sensor_name = message.sensor_name;
  frame.frame_id = message.header.frame_id;
  frame.clock_domain = message.clock_domain;
  frame.calibration_id = message.calibration_id;
  frame.calibration_status = message.calibration_status;
  frame.baseline_status = message.baseline_status;
  frame.force_unit = message.force_unit;
  frame.stamp_monotonic_ns = static_cast<std::uint64_t>(message.header.stamp.sec) *
                                 1000000000ULL + message.header.stamp.nanosec;
  frame.session_id = message.session_id;
  frame.sequence = message.sequence;
  frame.rows = message.rows;
  frame.cols = message.cols;
  frame.code_to_force_n_per_code = message.code_to_force_n_per_code;
  frame.polarity = message.polarity;
  frame.baseline_valid = message.baseline_valid;
  frame.raw_code = message.raw_code;
  frame.baseline_code = message.baseline_code;
  frame.normal_force_n = message.normal_force_n;
  frame.valid = message.valid;
  frame.validate(expected_sensor);
  return frame;
}

inline raisin_tactile_sensor::msg::TactileFrame toMessage(const FrameV2& frame) {
  frame.validate(frame.sensor_name);
  raisin_tactile_sensor::msg::TactileFrame message;
  message.header.frame_id = frame.frame_id;
  message.header.stamp.sec = static_cast<std::int32_t>(frame.stamp_monotonic_ns / 1000000000ULL);
  message.header.stamp.nanosec = static_cast<std::uint32_t>(frame.stamp_monotonic_ns % 1000000000ULL);
  message.sensor_name = frame.sensor_name;
  message.session_id = frame.session_id;
  message.sequence = frame.sequence;
  message.clock_domain = frame.clock_domain;
  message.rows = frame.rows;
  message.cols = frame.cols;
  message.code_to_force_n_per_code = frame.code_to_force_n_per_code;
  message.polarity = frame.polarity;
  message.calibration_id = frame.calibration_id;
  message.calibration_status = frame.calibration_status;
  message.baseline_status = frame.baseline_status;
  message.baseline_valid = frame.baseline_valid;
  message.force_unit = frame.force_unit;
  message.raw_code = frame.raw_code;
  message.baseline_code = frame.baseline_code;
  message.normal_force_n = frame.normal_force_n;
  message.valid = frame.valid;
  return message;
}

class SnapshotV2 {
 public:
  explicit SnapshotV2(std::string sensor) : sensor_(std::move(sensor)) {
    if (sensor_ != "left" && sensor_ != "right")
      throw std::invalid_argument("tactile v2 receiver requires left or right identity");
  }

  bool set(FrameV2 frame) {
    frame.validate(sensor_);
    std::lock_guard guard(mutex_);
    if (latest_ && (frame.stamp_monotonic_ns <= latest_->stamp_monotonic_ns ||
        (frame.session_id == latest_->session_id && frame.sequence <= latest_->sequence)))
      return false;
    latest_ = std::move(frame);
    return true;
  }

  std::optional<FrameV2> latest() const {
    std::lock_guard guard(mutex_);
    return latest_;
  }

  void reset() { std::lock_guard guard(mutex_); latest_.reset(); }

 private:
  std::string sensor_;
  mutable std::mutex mutex_;
  std::optional<FrameV2> latest_;
};

struct BilateralFrameV2 {
  FrameV2 left;
  FrameV2 right;
  std::uint64_t skew_ns{};
};

class PairAdmitter {
 public:
  std::optional<BilateralFrameV2> admit(const FrameV2& left, const FrameV2& right,
                                        std::uint64_t now_ns,
                                        std::uint64_t stale_timeout_ns = 150000000ULL,
                                        std::uint64_t max_skew_ns = 50000000ULL) {
    left.validate("left");
    right.validate("right");
    if (stale_timeout_ns == 0 || left.stamp_monotonic_ns > now_ns ||
        right.stamp_monotonic_ns > now_ns || now_ns - left.stamp_monotonic_ns > stale_timeout_ns ||
        now_ns - right.stamp_monotonic_ns > stale_timeout_ns || !left.baseline_valid ||
        !right.baseline_valid || countValid(left) == 0 || countValid(right) == 0)
      return std::nullopt;
    const auto skew = left.stamp_monotonic_ns > right.stamp_monotonic_ns
                          ? left.stamp_monotonic_ns - right.stamp_monotonic_ns
                          : right.stamp_monotonic_ns - left.stamp_monotonic_ns;
    if (skew > max_skew_ns) return std::nullopt;
    if (emitted_ && !isNew(left, last_left_session_, last_left_sequence_) ) return std::nullopt;
    if (emitted_ && !isNew(right, last_right_session_, last_right_sequence_)) return std::nullopt;
    emitted_ = true;
    last_left_session_ = left.session_id;
    last_left_sequence_ = left.sequence;
    last_right_session_ = right.session_id;
    last_right_sequence_ = right.sequence;
    return BilateralFrameV2{left, right, skew};
  }

  void reset() noexcept { emitted_ = false; }

 private:
  static std::size_t countValid(const FrameV2& frame) {
    return static_cast<std::size_t>(std::count(frame.valid.begin(), frame.valid.end(), 1));
  }
  static bool isNew(const FrameV2& frame, std::uint64_t session,
                    std::uint64_t sequence) {
    return frame.session_id != session || frame.sequence > sequence;
  }

  bool emitted_{};
  std::uint64_t last_left_session_{};
  std::uint64_t last_left_sequence_{};
  std::uint64_t last_right_session_{};
  std::uint64_t last_right_sequence_{};
};

}  // namespace raisin::tactile
