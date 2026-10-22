#pragma once

#include "raisin_tactile_plugin/usb_protocol.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace raisin::tactile::usb {

struct RawFrame {
  std::string sensor;
  Grid raw_code{};
  std::uint64_t session_id{};
  std::uint64_t sequence{};
  std::uint64_t stamp_monotonic_ns{};
  std::uint16_t packet_mask{kFullPacketMask};
};

struct SensorStatus {
  std::string sensor;
  std::string port;
  std::string state;
  std::string error;
  bool online{};
  std::uint64_t session_id{};
  std::uint64_t next_sequence{};
  std::uint64_t packets{};
  std::uint64_t frames{};
  std::uint64_t disconnects{};
  std::uint64_t frame_timeouts{};
  std::uint64_t last_receive_monotonic_ns{};
  ParserStats parser{};
};

class UsbBackend {
 public:
  UsbBackend(std::string port, int sensor_count, std::filesystem::path record_dir,
             int rate_hz = 20,
             int baud_rate = 115200,
             double frame_timeout_s = 0.15, double retry_s = 1.0);
  ~UsbBackend();
  UsbBackend(const UsbBackend&) = delete;
  UsbBackend& operator=(const UsbBackend&) = delete;

  std::vector<RawFrame> poll();
  std::array<SensorStatus, 2> status() const;
  void stop() noexcept;

 private:
  struct Stream;
  std::unique_ptr<Stream> stream_;
  std::uint8_t rate_subcommand_{};
  int baud_rate_{};
  std::uint64_t frame_timeout_ns_{};
  std::uint64_t retry_ns_{};
  bool stopped_{};
};

std::uint64_t monotonicNowNs() noexcept;

}  // namespace raisin::tactile::usb
