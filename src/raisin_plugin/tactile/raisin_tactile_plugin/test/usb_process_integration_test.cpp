#include "raisin_network/raisin.hpp"
#include "raisin_tactile_plugin/usb_protocol.hpp"
#include "raisin_tactile_sensor/frame_v2.hpp"
#include "raisin_tactile_sensor/msg/tactile_frame.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <memory>
#include <poll.h>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <stdlib.h>
#include <iterator>

using namespace std::chrono_literals;
namespace tactile_msg = raisin::raisin_tactile_sensor::msg;
namespace string_msg = raisin::std_msgs::msg;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class Pty {
 public:
  Pty() {
    master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0)
      throw std::runtime_error("cannot create PTY");
    std::array<char, 256> path{};
    if (::ptsname_r(master, path.data(), path.size()) != 0)
      throw std::runtime_error("cannot read PTY path");
    slave_path = path.data();
  }
  ~Pty() { if (master >= 0) ::close(master); }
  int master{-1};
  std::string slave_path;
};

class NodeCleanup {
 public:
  explicit NodeCleanup(raisin::Node& node) : node_(node) {}
  ~NodeCleanup() { cleanup(); }
  void cleanup() {
    if (!cleaned_) {
      node_.cleanupResources();
      cleaned_ = true;
    }
  }
 private:
  raisin::Node& node_;
  bool cleaned_{};
};

class ChildProcess {
 public:
  explicit ChildProcess(const char* executable, const std::array<std::string, 13>& args, const std::string& side = "") {
    pid_ = ::fork();
    if (pid_ < 0) throw std::runtime_error("fork failed");
    if (pid_ == 0) {
      ::execl(executable, executable,
          args[0].c_str(), args[1].c_str(), args[2].c_str(), args[3].c_str(),
          args[4].c_str(), args[5].c_str(), args[6].c_str(), args[7].c_str(),
          args[8].c_str(), args[9].c_str(), args[10].c_str(), args[11].c_str(),
          args[12].c_str(), side.empty() ? nullptr : side.c_str(), static_cast<char*>(nullptr));
      _exit(127);
    }
  }
  ~ChildProcess() { stop(); }
  ChildProcess(const ChildProcess&) = delete;
  ChildProcess& operator=(const ChildProcess&) = delete;
  void stop() {
    if (pid_ <= 0) return;
    ::kill(pid_, SIGTERM);
    int status = 0;
    while (::waitpid(pid_, &status, 0) < 0 && errno == EINTR) {}
    pid_ = -1;
  }
 private:
  pid_t pid_{-1};
};

std::vector<std::uint8_t> readFixture() {
  std::ifstream file(TACTILE_QT_FRAME_FIXTURE, std::ios::binary);
  if (!file) throw std::runtime_error("cannot read Qt fixture");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::vector<std::uint8_t> offsetFixture(const std::vector<std::uint8_t>& source, int offset) {
  auto result = source;
  for (std::size_t packet_index = 0; packet_index < 12; ++packet_index) {
    auto* packet = result.data() + packet_index * 494;
    for (std::size_t i = 0; i < 243; ++i) {
      const auto at = 6 + 2 * i;
      const auto bits = static_cast<std::uint16_t>(
          (static_cast<std::uint16_t>(packet[at]) << 8U) | packet[at + 1]);
      const auto value = static_cast<std::int16_t>(std::bit_cast<std::int16_t>(bits) + offset);
      const auto changed = std::bit_cast<std::uint16_t>(value);
      packet[at] = static_cast<std::uint8_t>(changed >> 8U);
      packet[at + 1] = static_cast<std::uint8_t>(changed);
    }
    packet[492] = raisin::tactile::usb::crc8(
        std::span<const std::uint8_t>(packet + 1, 491));
  }
  return result;
}

void writeAll(int fd, std::span<const std::uint8_t> bytes) {
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto result = ::write(fd, bytes.data() + offset, bytes.size() - offset);
    if (result > 0) offset += static_cast<std::size_t>(result);
    else if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd descriptor{fd, POLLOUT, 0};
      if (::poll(&descriptor, 1, 1000) <= 0) throw std::runtime_error("PTY write timed out");
    } else throw std::runtime_error("PTY write failed");
  }
}

std::vector<std::uint8_t> readAvailable(int fd) {
  std::vector<std::uint8_t> result;
  std::array<std::uint8_t, 32> buffer{};
  for (;;) {
    const auto count = ::read(fd, buffer.data(), buffer.size());
    if (count > 0) result.insert(result.end(), buffer.begin(), buffer.begin() + count);
    else if (count < 0 && errno == EINTR) continue;
    else break;
  }
  return result;
}

void expectCommand(int fd, std::array<std::uint8_t, 7> expected) {
  std::vector<std::uint8_t> received;
  for (int attempt = 0; attempt < 30 && received.size() < expected.size(); ++attempt) {
    pollfd descriptor{fd, POLLIN, 0};
    ::poll(&descriptor, 1, 25);
    auto part = readAvailable(fd);
    received.insert(received.end(), part.begin(), part.end());
  }
  require(received.size() == expected.size() &&
              std::equal(received.begin(), received.end(), expected.begin()),
          "child process USB startup command mismatch");
}

void expectStarted(int stream) {
  std::this_thread::sleep_for(210ms);
  expectCommand(stream, raisin::tactile::usb::buildCommand(0xF2, 0x01));
  std::this_thread::sleep_for(310ms);
  expectCommand(stream, raisin::tactile::usb::buildCommand(0xF3, 0x02));
  std::this_thread::sleep_for(310ms);
  expectCommand(stream, raisin::tactile::usb::buildCommand(0xF1, 0x01));
}

std::vector<std::filesystem::path> runDirectories(const std::filesystem::path& root) {
  std::vector<std::filesystem::path> result;
  for (const auto& entry : std::filesystem::directory_iterator(root))
    if (entry.is_directory()) result.push_back(entry.path());
  return result;
}

void verifyRunDirectory(const std::filesystem::path& directory) {
  require(std::filesystem::exists(directory / "stream.rawpkt"),
          "run directory is missing the shared raw packet log");
  std::ifstream manifest_file(directory / "run_manifest.json");
  const std::string manifest(std::istreambuf_iterator<char>(manifest_file), {});
  require(!manifest.empty() && manifest.find("\"stream_sensor_count\": 2") != std::string::npos &&
              manifest.find("\"map_order\": [\"right\", \"left\"]") != std::string::npos &&
              manifest.find("\"rate_hz\": 20") != std::string::npos &&
              manifest.find("\"code_to_force_n_per_code\": 0.001") != std::string::npos &&
              manifest.find("tactile/v2/process_test") != std::string::npos,
          "run manifest did not preserve transport and conversion settings");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) throw std::runtime_error("expected staged USB process executable path [--dual]");
  const bool separate = argc == 3 && std::string(argv[2]) == "--dual";
  char temp_template[] = "/tmp/raisin-tactile-process-test-XXXXXX";
  const auto* temp = ::mkdtemp(temp_template);
  if (!temp) throw std::runtime_error("cannot create test directory");
  const std::filesystem::path root(temp);
  try {
    Pty stream, left_stream;
    const std::string prefix = "tactile/v2/process_test_" + std::to_string(::getpid());

    raisin::raisinInit();
    auto pool = std::make_shared<raisin::ThreadPool>(
        std::vector<std::vector<std::string>>{{"main"}}, true);
    raisin::Node node(pool);
    NodeCleanup nodeCleanup(node);
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<tactile_msg::TactileFrame> left_frames, right_frames;
    raisin::tactile::SnapshotV2 left_core("left"), right_core("right");
    std::size_t core_left_accepted = 0, core_right_accepted = 0;
    auto left_sub = node.createSubscriber<tactile_msg::TactileFrame>(std::string(prefix) + "/left",
        nullptr, [&](const tactile_msg::TactileFrame::SharedPtr message) {
          const bool accepted = left_core.set(raisin::tactile::fromMessage(*message, "left"));
          std::lock_guard lock(mutex);
          left_frames.push_back(*message);
          if (accepted) ++core_left_accepted;
          changed.notify_all();
        });
    auto right_sub = node.createSubscriber<tactile_msg::TactileFrame>(std::string(prefix) + "/right",
        nullptr, [&](const tactile_msg::TactileFrame::SharedPtr message) {
          const bool accepted = right_core.set(raisin::tactile::fromMessage(*message, "right"));
          std::lock_guard lock(mutex);
          right_frames.push_back(*message);
          if (accepted) ++core_right_accepted;
          changed.notify_all();
        });
    auto command_pub = node.createPublisher<string_msg::String>(std::string(prefix) + "/command");

    ChildProcess child(argv[1], {stream.slave_path, separate ? "1" : "2", (root / "record").string(),
        "20", "115200", "0.5", "0.05", prefix,
        "100", "0.001", "provisional_linear_0p001", "1", "1"}, separate ? "right" : "");
    expectStarted(stream.master);
    std::unique_ptr<ChildProcess> left_child;
    if (separate) {
      left_child = std::make_unique<ChildProcess>(argv[1], std::array<std::string, 13>{
          left_stream.slave_path, "1", (root / "left_record").string(),
          "20", "115200", "0.5", "0.05", prefix,
          "100", "0.001", "provisional_linear_0p001", "1", "1"}, "left");
      expectStarted(left_stream.master);
    }
    const auto fixture = readFixture();
    const auto left_initial = offsetFixture(fixture, 1234);
    writeAll(stream.master, fixture);
    {
      std::unique_lock lock(mutex);
      require(changed.wait_for(lock, 2s, [&] { return !right_frames.empty(); }),
              "first map did not reach the right topic");
      require(right_frames.back().raw_code[0] == -1458,
              "right-first stream map value changed");
    }
    writeAll(separate ? left_stream.master : stream.master, left_initial);
    {
      std::unique_lock lock(mutex);
      require(changed.wait_for(lock, 2s, [&] {
        return !left_frames.empty() && !right_frames.empty() &&
               core_left_accepted >= 1 && core_right_accepted >= 1;
      }), "Raisin topics did not receive both raw-derived frames");
      require(!left_frames.back().baseline_valid && !right_frames.back().baseline_valid &&
                  left_frames.back().clock_domain == "CLOCK_MONOTONIC" &&
                  left_frames.back().sensor_name == "left" &&
                  right_frames.back().sensor_name == "right" &&
                  left_core.latest()->raw_code == left_frames.back().raw_code &&
                  right_core.latest()->raw_code == right_frames.back().raw_code,
              "initial producer message lost identity, clock, or pending-baseline metadata");
    }

    string_msg::String command;
    command.data = "capture_baseline:both";
    command_pub->publish(command);
    std::this_thread::sleep_for(30ms);
    writeAll(stream.master, offsetFixture(fixture, 100));
    writeAll(separate ? left_stream.master : stream.master, offsetFixture(fixture, 1334));
    {
      std::unique_lock lock(mutex);
      require(changed.wait_for(lock, 2s, [&] {
        return left_frames.size() >= 2 && right_frames.size() >= 2 &&
               left_frames.back().baseline_valid && right_frames.back().baseline_valid;
      }), "explicit dual baseline command did not reach the USB process");
      require(left_frames.back().sequence == 1 && right_frames.back().sequence == 1 &&
                  left_frames.back().raw_code == left_frames.back().baseline_code &&
                  right_frames.back().raw_code == right_frames.back().baseline_code,
              "captured baselines were not per-sensor raw maps");
    }

    writeAll(stream.master, offsetFixture(fixture, 150));
    writeAll(separate ? left_stream.master : stream.master, offsetFixture(fixture, 1384));
    {
      std::unique_lock lock(mutex);
      require(changed.wait_for(lock, 2s, [&] {
        return left_frames.size() >= 3 && right_frames.size() >= 3 &&
               left_frames.back().sequence >= 2 && right_frames.back().sequence >= 2;
      }), "post-baseline force maps were not published");
      constexpr std::size_t firstRoiCell =
          raisin::tactile::kUsbRoiRxFirst * raisin::tactile::kUsbCols +
          raisin::tactile::kUsbRoiTxFirst;
      const auto left_force = left_frames.back().normal_force_n[firstRoiCell];
      const auto right_force = right_frames.back().normal_force_n[firstRoiCell];
      require(std::abs(left_force - 0.05F) < 1e-6F &&
                  std::abs(right_force - 0.05F) < 1e-6F &&
                  left_frames.back().valid[firstRoiCell] == 1 &&
                  right_frames.back().valid[firstRoiCell] == 1 &&
                  left_frames.back().normal_force_n[0] == 0.F &&
                  right_frames.back().normal_force_n[0] == 0.F &&
                  left_frames.back().valid[0] == 0 && right_frames.back().valid[0] == 0 &&
                  left_frames.back().valid[40] == 0 && right_frames.back().valid[40] == 0,
              "Raisin topic values/mask differ from raw delta conversion in the active ROI");
    }

    if (separate) {
      child.stop();
      std::size_t previous_left, previous_right;
      { std::lock_guard lock(mutex); previous_left = left_frames.size(); previous_right = right_frames.size(); }
      writeAll(left_stream.master, offsetFixture(fixture, 1400));
      {
        std::unique_lock lock(mutex);
        require(changed.wait_for(lock, 2s, [&] { return left_frames.size() > previous_left; }),
                "left USB stopped when the independent right process disconnected");
        require(right_frames.size() == previous_right, "left USB published a right sensor frame");
      }
      left_child->stop();
      nodeCleanup.cleanup(); pool.reset(); std::filesystem::remove_all(root);
      std::cout << "PASS: two independent USB streams preserve side identity, baseline, force and disconnect isolation\n";
      return 0;
    }

    std::size_t previous_left_count = 0, previous_right_count = 0;
    std::uint64_t previous_left_session = 0, previous_right_session = 0;
    {
      std::lock_guard lock(mutex);
      previous_left_count = left_frames.size();
      previous_right_count = right_frames.size();
      previous_left_session = left_frames.back().session_id;
      previous_right_session = right_frames.back().session_id;
    }
    child.stop();
    ChildProcess reloaded(argv[1], {stream.slave_path, "2", (root / "record_reloaded").string(),
        "20", "115200", "0.5", "0.05", prefix,
        "100", "0.001", "provisional_linear_0p001", "1", "1"});
    expectStarted(stream.master);
    writeAll(stream.master, fixture);
    {
      std::unique_lock lock(mutex);
      require(changed.wait_for(lock, 2s, [&] {
        return right_frames.size() > previous_right_count;
      }), "reloaded USB process did not publish the first right map");
    }
    writeAll(separate ? left_stream.master : stream.master, left_initial);
    {
      std::unique_lock lock(mutex);
      require(changed.wait_for(lock, 2s, [&] {
        return left_frames.size() > previous_left_count;
      }), "reloaded USB process did not publish the second left map");
      require(left_frames.back().session_id != previous_left_session &&
                  right_frames.back().session_id != previous_right_session &&
                  left_frames.back().sequence == 0 && right_frames.back().sequence == 0 &&
                  !left_frames.back().baseline_valid && !right_frames.back().baseline_valid &&
                  !left_core.latest()->baseline_valid && !right_core.latest()->baseline_valid,
              "reload must start new sessions, reset sequences, and require fresh baselines");
    }
    reloaded.stop();
    const auto first_runs = runDirectories(root / "record");
    const auto reloaded_runs = runDirectories(root / "record_reloaded");
    require(first_runs.size() == 1 && reloaded_runs.size() == 1 &&
                first_runs.front() != reloaded_runs.front(),
            "USB process restart must keep each capture in a separate run directory");
    verifyRunDirectory(first_runs.front());
    verifyRunDirectory(reloaded_runs.front());
    nodeCleanup.cleanup();
    pool.reset();
    std::filesystem::remove_all(root);
    std::cout << "PASS: staged USB process demultiplexes one stream right-first, captures per-side baselines, and reloads with new sessions\n";
  } catch (...) {
    std::filesystem::remove_all(root);
    throw;
  }
}
