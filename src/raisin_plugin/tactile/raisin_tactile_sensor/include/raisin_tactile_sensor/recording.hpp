#pragma once

#include "frame_v2.hpp"

#include <array>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace raisin::tactile {

class FrameRecorder {
 public:
  void start(const std::filesystem::path& directory) {
    if (active_ || directory.empty())
      throw std::invalid_argument("tactile recorder is active or directory is empty");
    std::filesystem::create_directories(directory);
    directory_ = directory;
    for (std::size_t i = 0; i < files_.size(); ++i) {
      const auto sensor = i == 0 ? "left" : "right";
      files_[i].open(directory_ / (std::string(sensor) + ".tfr2"),
                     std::ios::binary | std::ios::out | std::ios::trunc);
      if (!files_[i]) {
        stop();
        throw std::runtime_error("cannot open tactile frame recording");
      }
      files_[i].write("TFR2", 4);
      writeU16(files_[i], 1);
      files_[i].put(static_cast<char>(std::char_traits<char>::length(sensor)));
      files_[i].write(sensor, static_cast<std::streamsize>(
                                  std::char_traits<char>::length(sensor)));
      if (!files_[i]) {
        stop();
        throw std::runtime_error("cannot write tactile frame recording header");
      }
    }
    active_ = true;
  }

  bool write(const FrameV2& frame) {
    if (!active_) return false;
    frame.validate(frame.sensor_name);
    auto& file = files_[sensorIndex(frame.sensor_name)];
    raisin_tactile_sensor::msg::TactileFrame message = toMessage(frame);
    std::vector<unsigned char> bytes;
    message.setBuffer(bytes);
    if (bytes.empty() || bytes.size() > 1024 * 1024)
      throw std::runtime_error("invalid tactile frame record size");
    file.write("TFM2", 4);
    writeU32(file, static_cast<std::uint32_t>(bytes.size()));
    file.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
    file.flush();
    if (!file) throw std::runtime_error("tactile frame recording write failed");
    return true;
  }

  void stop() noexcept {
    for (auto& file : files_) {
      if (file.is_open()) {
        file.flush();
        file.close();
      }
    }
    active_ = false;
  }

  bool active() const noexcept { return active_; }
  const std::filesystem::path& directory() const noexcept { return directory_; }

  ~FrameRecorder() { stop(); }

 private:
  static std::size_t sensorIndex(const std::string& name) {
    if (name == "left") return 0;
    if (name == "right") return 1;
    throw std::invalid_argument("unknown tactile sensor recording identity");
  }
  static void writeU16(std::ostream& file, std::uint16_t value) {
    file.put(static_cast<char>(value));
    file.put(static_cast<char>(value >> 8U));
  }
  static void writeU32(std::ostream& file, std::uint32_t value) {
    for (int i = 0; i < 4; ++i) file.put(static_cast<char>(value >> (8U * i)));
  }

  std::filesystem::path directory_;
  std::array<std::ofstream, 2> files_;
  bool active_{};
};

inline std::vector<FrameV2> readRecordedFrames(const std::filesystem::path& path,
                                                const std::string& expected_sensor) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("cannot open tactile frame recording");
  std::array<char, 4> magic{};
  file.read(magic.data(), magic.size());
  if (!file || std::string(magic.data(), magic.size()) != "TFR2")
    throw std::runtime_error("invalid tactile frame recording header");
  const auto readU16 = [&]() -> std::uint16_t {
    const auto lo = file.get();
    const auto hi = file.get();
    if (lo < 0 || hi < 0) throw std::runtime_error("truncated tactile recording header");
    return static_cast<std::uint16_t>(lo | (hi << 8));
  };
  if (readU16() != 1) throw std::runtime_error("unsupported tactile recording version");
  const auto sensor_length = file.get();
  if (sensor_length <= 0 || sensor_length > 16)
    throw std::runtime_error("invalid tactile recording sensor name");
  std::string sensor(static_cast<std::size_t>(sensor_length), '\0');
  file.read(sensor.data(), sensor.size());
  if (!file || sensor != expected_sensor)
    throw std::runtime_error("tactile recording sensor identity mismatch");

  std::vector<FrameV2> frames;
  for (;;) {
    file.read(magic.data(), magic.size());
    if (file.eof() && file.gcount() == 0) break;
    if (!file || std::string(magic.data(), magic.size()) != "TFM2")
      throw std::runtime_error("invalid or truncated tactile frame record");
    std::uint32_t size = 0;
    for (int i = 0; i < 4; ++i) {
      const auto byte = file.get();
      if (byte < 0) throw std::runtime_error("truncated tactile frame record size");
      size |= static_cast<std::uint32_t>(byte) << (8U * i);
    }
    if (size == 0 || size > 1024 * 1024)
      throw std::runtime_error("invalid tactile frame record length");
    std::vector<unsigned char> bytes(size);
    file.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!file) throw std::runtime_error("truncated tactile frame message");
    raisin_tactile_sensor::msg::TactileFrame message;
    message.getBuffer(bytes);
    frames.push_back(fromMessage(message, expected_sensor));
  }
  return frames;
}

}  // namespace raisin::tactile
