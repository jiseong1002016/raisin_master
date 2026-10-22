#include "raisin_tactile_plugin/usb_backend.hpp"
#include "raisin_tactile_plugin/usb_conversion.hpp"

#include "raisin_network/raisin.hpp"
#include "raisin_util/raisin_directories.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace std_msgs = raisin::std_msgs;
namespace raisin_tactile_sensor = raisin::raisin_tactile_sensor;

std::string jsonString(std::string_view value) {
  std::ostringstream escaped;
  escaped << '"';
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (ch < 0x20)
          escaped << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(ch) << std::dec;
        else escaped << static_cast<char>(ch);
    }
  }
  escaped << '"';
  return escaped.str();
}

void writeRunManifest(const std::filesystem::path& directory,
                      const std::string& port, int sensor_count,
                      int rate_hz, int baud_rate, double timeout_s, double retry_s,
                      const std::string& topic_prefix, double poll_rate_hz,
                      float coefficient, const std::string& calibration_id,
                      std::int8_t left_polarity, std::int8_t right_polarity,
                      const std::string& single_sensor) {
  std::ofstream manifest(directory / "run_manifest.json", std::ios::out | std::ios::trunc);
  if (!manifest) throw std::runtime_error("cannot create tactile run manifest");
  manifest << std::setprecision(9)
      << "{\n  \"protocol\": \"Qt F1 01 / 494-byte x 12 packets\",\n"
      << "  \"rows\": 71,\n  \"cols\": 41,\n"
      << "  \"clock_domain\": \"CLOCK_MONOTONIC\",\n"
      << "  \"stream_port\": " << jsonString(port) << ",\n"
      << "  \"stream_sensor_count\": " << sensor_count << ",\n"
      << "  \"map_order\": " << (single_sensor.empty() ? "[\"right\", \"left\"]" : "[" + jsonString(single_sensor) + "]") << ",\n"
      << "  \"raw_packet_log\": \"stream.rawpkt\",\n"
      << "  \"rate_hz\": " << rate_hz << ",\n"
      << "  \"baud_rate\": " << baud_rate << ",\n"
      << "  \"frame_timeout_s\": " << timeout_s << ",\n"
      << "  \"reconnect_retry_s\": " << retry_s << ",\n"
      << "  \"topic_prefix\": " << jsonString(topic_prefix) << ",\n"
      << "  \"poll_rate_hz\": " << poll_rate_hz << ",\n"
      << "  \"code_to_force_n_per_code\": " << coefficient << ",\n"
      << "  \"calibration_id\": " << jsonString(calibration_id) << ",\n"
      << "  \"calibration_status\": \"provisional\",\n"
      << "  \"force_unit\": \"N_estimate\",\n"
      << "  \"left_polarity\": " << static_cast<int>(left_polarity) << ",\n"
      << "  \"right_polarity\": " << static_cast<int>(right_polarity) << ",\n"
      << "  \"runtime_install_prefix\": "
      << jsonString(raisin::getInstallDirectory()) << "\n}\n";
  if (!manifest) throw std::runtime_error("cannot write tactile run manifest");
}

class UsbPublisher {
 public:
  UsbPublisher(raisin::Node& node, const std::string& port, int sensor_count,
               const std::filesystem::path& record_dir, int rate_hz, int baud_rate,
               double timeout_s, double retry_s, const std::string& topic_prefix,
               double poll_rate_hz, float coefficient, const std::string& calibration_id,
               std::int8_t left_polarity, std::int8_t right_polarity,
               std::string single_sensor)
      : single_sensor_(std::move(single_sensor)), backend_(port, sensor_count, record_dir, rate_hz, baud_rate, timeout_s, retry_s),
        converter_(raisin::tactile::usb::ConversionConfig{
            coefficient, calibration_id, {left_polarity, right_polarity}}) {
    writeRunManifest(record_dir, port, sensor_count, rate_hz, baud_rate, timeout_s, retry_s,
                     topic_prefix, poll_rate_hz, coefficient, calibration_id,
                     left_polarity, right_polarity, single_sensor_);
    for (std::size_t i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      if (!single_sensor_.empty() && single_sensor_ != side) continue;
      frames_[i] = node.createPublisher<raisin_tactile_sensor::msg::TactileFrame>(topic_prefix + "/" + side, 5);
      statuses_[i] = node.createPublisher<std_msgs::msg::String>(topic_prefix + "/status/" + side, 5);
    }
    baseline_command_ = node.createSubscriber<std_msgs::msg::String>(
        topic_prefix + "/command", nullptr,
        [this](const std_msgs::msg::String::SharedPtr message) {
          if (!converter_.requestBaseline(message->data))
            std::cerr << "ignored unknown tactile command: " << message->data << '\n';
        }, "main");
    node.createTimedLoop("tactile_usb_poll", [this]() { poll(); }, poll_rate_hz);
  }

 private:
  void poll() {
    for (auto frame : backend_.poll()) {
      if (!single_sensor_.empty()) frame.sensor = single_sensor_;
      const auto index = frame.sensor == "left" ? 0U : 1U;
      frames_[index]->publish(converter_.convert(frame));
    }
    const auto now = raisin::tactile::usb::monotonicNowNs();
    if (now - last_status_ns_ < 100000000ULL) return;
    last_status_ns_ = now;
    const auto current = backend_.status();
    for (std::size_t i = 0; i < current.size(); ++i) {
      if (!statuses_[i]) continue;
      const auto& status = current[single_sensor_.empty() ? i : 1];
      std_msgs::msg::String message;
      std::ostringstream out;
      out << "sensor=" << (single_sensor_.empty() ? status.sensor : single_sensor_) << " state=" << status.state
          << " online=" << (status.online ? "true" : "false")
          << " port=" << status.port
          << " session_id=" << status.session_id << " next_sequence=" << status.next_sequence
          << " packets=" << status.packets << " frames=" << status.frames
          << " disconnects=" << status.disconnects << " frame_timeouts=" << status.frame_timeouts
          << " last_receive_monotonic_ns=" << status.last_receive_monotonic_ns
          << " age_ms=" << (status.last_receive_monotonic_ns == 0
                                  ? -1.0
                                  : static_cast<double>(now - status.last_receive_monotonic_ns) / 1e6)
          << " crc_errors=" << status.parser.crc_errors
          << " malformed_packets=" << status.parser.malformed_packets
          << " order_errors=" << status.parser.order_errors
          << " incomplete_sets=" << status.parser.incomplete_sets;
      if (!status.error.empty()) out << " error=" << status.error;
      message.data = out.str();
      statuses_[i]->publish(message);
    }
  }

  std::string single_sensor_;
  raisin::tactile::usb::UsbBackend backend_;
  raisin::tactile::usb::FrameConverter converter_;
  std::array<raisin::Publisher<raisin_tactile_sensor::msg::TactileFrame>::SharedPtr, 2> frames_;
  std::array<raisin::Publisher<std_msgs::msg::String>::SharedPtr, 2> statuses_;
  std::shared_ptr<raisin::Subscriber<std_msgs::msg::String>> baseline_command_;
  std::uint64_t last_status_ns_{};
};

int parseInt(const char* value, const char* name) {
  std::size_t used = 0;
  const int result = std::stoi(value, &used);
  if (value[used] != '\0') throw std::invalid_argument(std::string("invalid ") + name);
  return result;
}

double parseDouble(const char* value, const char* name) {
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (value[used] != '\0') throw std::invalid_argument(std::string("invalid ") + name);
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 14 && argc != 15) {
    std::cerr << "usage: raisin_tactile_usb_process PORT STREAM_SENSOR_COUNT RECORD_DIR RATE_HZ BAUD "
                 "FRAME_TIMEOUT_S RETRY_S TOPIC_PREFIX POLL_RATE_HZ COEFFICIENT "
                 "CALIBRATION_ID LEFT_POLARITY RIGHT_POLARITY [left|right]\n";
    return 2;
  }
  try {
    const std::string single_sensor = argc == 15 ? argv[14] : "";
    if ((!single_sensor.empty() && single_sensor != "left" && single_sensor != "right") ||
        (!single_sensor.empty() && parseInt(argv[2], "sensor count") != 1))
      throw std::invalid_argument("an assigned USB side requires one sensor and left or right identity");
    raisin::raisinInit();
    auto pool = std::make_shared<raisin::ThreadPool>(
        std::vector<std::vector<std::string>>{{"main"}}, false);
    raisin::Node node(pool);
    const std::filesystem::path run_root(argv[3]);
    const auto run_dir = run_root / ("usb_" + std::to_string(
        raisin::tactile::usb::monotonicNowNs()));
    UsbPublisher publisher(node, argv[1], parseInt(argv[2], "stream sensor count"), run_dir,
                           parseInt(argv[4], "rate"), parseInt(argv[5], "baud"),
                           parseDouble(argv[6], "timeout"), parseDouble(argv[7], "retry"),
                           argv[8], parseDouble(argv[9], "poll rate"),
                           static_cast<float>(parseDouble(argv[10], "force coefficient")),
                           argv[11], static_cast<std::int8_t>(parseInt(argv[12], "left polarity")),
                           static_cast<std::int8_t>(parseInt(argv[13], "right polarity")), single_sensor);
    pool->getWorker(0)->run();
  } catch (const std::exception& error) {
    std::cerr << "tactile USB process: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
