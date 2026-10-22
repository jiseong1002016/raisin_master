#include "raisin_tactile_plugin/usb_backend.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <stdexcept>
#include <string_view>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace raisin::tactile::usb {
namespace {

enum class StartPhase : std::uint8_t { offline, wait_dtr, wait_stop, wait_rate, streaming };
constexpr std::size_t kLeftSide = 0;
constexpr std::size_t kRightSide = 1;
constexpr std::size_t kNoSide = 2;

const char* sideName(std::size_t side) {
  return side == kLeftSide ? "left" : "right";
}

bool startsMap(const std::array<std::uint8_t, kPacketSize>& packet) {
  const auto length = (static_cast<std::size_t>(packet[1]) << 8U) | packet[2];
  return packet[0] == kStart && packet[kPacketSize - 1] == kEnd && length == 490 &&
         packet[3] == kDataCommand && packet[4] == 0x01 && packet[5] == 0;
}

std::uint8_t rateSubcommand(int rate_hz) {
  switch (rate_hz) {
    case 30: return 0x01;
    case 20: return 0x02;
    case 15: return 0x03;
    case 10: return 0x04;
    default: throw std::invalid_argument("USB tactile rate must be 10, 15, 20 or 30 Hz");
  }
}

void writeU16(std::ostream& stream, std::uint16_t value) {
  const char bytes[2]{static_cast<char>(value), static_cast<char>(value >> 8U)};
  stream.write(bytes, sizeof(bytes));
}

void writeU64(std::ostream& stream, std::uint64_t value) {
  char bytes[8];
  for (int i = 0; i < 8; ++i) bytes[i] = static_cast<char>(value >> (8U * i));
  stream.write(bytes, sizeof(bytes));
}

speed_t baudConstant(int baud_rate) {
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw std::invalid_argument("unsupported USB tactile baud rate");
  }
}

void configurePort(int fd, int baud_rate) {
  termios options{};
  if (::tcgetattr(fd, &options) != 0)
    throw std::runtime_error("tcgetattr: " + std::string(std::strerror(errno)));
  ::cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~PARENB;
  options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  const auto speed = baudConstant(baud_rate);
  if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0 ||
      ::tcsetattr(fd, TCSANOW, &options) != 0)
    throw std::runtime_error("cannot configure tactile USB CDC port: " +
                             std::string(std::strerror(errno)));
  int lines = TIOCM_DTR | TIOCM_RTS;
  if (::ioctl(fd, TIOCMBIS, &lines) != 0 && errno != ENOTTY && errno != EINVAL)
    throw std::runtime_error("cannot assert tactile USB DTR/RTS: " +
                             std::string(std::strerror(errno)));
  if (::tcflush(fd, TCIOFLUSH) != 0)
    throw std::runtime_error("tcflush: " + std::string(std::strerror(errno)));
}

bool writeCommand(int fd, std::uint8_t command, std::uint8_t subcommand) {
  const auto bytes = buildCommand(command, subcommand);
  std::size_t written = 0;
  while (written < bytes.size()) {
    const auto result = ::write(fd, bytes.data() + written, bytes.size() - written);
    if (result > 0) {
      written += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
}

std::string phaseName(StartPhase phase) {
  switch (phase) {
    case StartPhase::offline: return "offline";
    case StartPhase::wait_dtr: return "starting_dtr";
    case StartPhase::wait_stop: return "starting_stop";
    case StartPhase::wait_rate: return "starting_rate";
    case StartPhase::streaming: return "streaming";
  }
  return "unknown";
}

}  // namespace

struct UsbBackend::Stream {
  Stream(std::string port_path, int count, const std::filesystem::path& record_dir)
      : path(std::move(port_path)), sensor_count(count) {
    log.open(record_dir / "stream.rawpkt", std::ios::binary | std::ios::out | std::ios::trunc);
    if (!log) throw std::runtime_error("cannot open raw tactile stream log");
    log.write("TACR", 4);
    writeU16(log, 1);
    constexpr std::string_view name = "stream";
    writeU16(log, static_cast<std::uint16_t>(name.size()));
    log.write(name.data(), static_cast<std::streamsize>(name.size()));
  }

  std::string path;
  int sensor_count{};
  int fd{-1};
  StartPhase phase{StartPhase::offline};
  std::uint64_t deadline_ns{};
  std::uint64_t retry_at_ns{};
  std::uint64_t session_id{};
  std::array<std::uint64_t, 2> sequence{};
  std::uint64_t packets{};
  std::array<std::uint64_t, 2> frames{};
  std::uint64_t disconnects{};
  std::uint64_t frame_timeouts{};
  std::array<std::uint64_t, 2> last_receive_ns{};
  std::uint64_t assembly_started_ns{};
  std::uint64_t pending_started_ns{};
  std::uint64_t connections{};
  std::size_t next_side{kRightSide};
  std::size_t assembly_side{kNoSide};
  std::string error;
  std::vector<std::uint8_t> pending;
  FrameAssembler assembler;
  std::ofstream log;
};

std::uint64_t monotonicNowNs() noexcept {
  timespec now{};
  if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
  return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(now.tv_nsec);
}

UsbBackend::UsbBackend(std::string port, int sensor_count,
                       std::filesystem::path record_dir, int rate_hz,
                       int baud_rate, double frame_timeout_s, double retry_s)
    : rate_subcommand_(rateSubcommand(rate_hz)), baud_rate_(baud_rate) {
  (void)baudConstant(baud_rate_);
  if (port.empty()) throw std::invalid_argument("USB tactile backend requires one explicit port path");
  if (sensor_count != 1 && sensor_count != 2)
    throw std::invalid_argument("USB tactile stream_sensor_count must be 1 or 2");
  if (!std::isfinite(frame_timeout_s) || frame_timeout_s <= 0 ||
      !std::isfinite(retry_s) || retry_s <= 0)
    throw std::invalid_argument("USB tactile timeout and reconnect retry must be positive");
  if (record_dir.empty()) throw std::invalid_argument("USB tactile raw record_dir is required");
  frame_timeout_ns_ = static_cast<std::uint64_t>(frame_timeout_s * 1e9);
  retry_ns_ = static_cast<std::uint64_t>(retry_s * 1e9);
  std::filesystem::create_directories(record_dir);
  stream_ = std::make_unique<Stream>(std::move(port), sensor_count, record_dir);
}

UsbBackend::~UsbBackend() { stop(); }

std::vector<RawFrame> UsbBackend::poll() {
  std::vector<RawFrame> frames;
  if (stopped_) return frames;
  auto now = monotonicNowNs();
  auto& sensor = *stream_;
  if (sensor.phase == StartPhase::offline) {
    if (now < sensor.retry_at_ns) return frames;
    sensor.fd = ::open(sensor.path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (sensor.fd < 0) {
      sensor.error = "open: " + std::string(std::strerror(errno));
      sensor.retry_at_ns = now + retry_ns_;
      return frames;
    }
    try {
      configurePort(sensor.fd, baud_rate_);
    } catch (const std::exception& e) {
      sensor.error = e.what();
      ::close(sensor.fd);
      sensor.fd = -1;
      sensor.retry_at_ns = now + retry_ns_;
      return frames;
    }
    sensor.pending.clear();
    sensor.pending_started_ns = 0;
    sensor.assembly_started_ns = 0;
    sensor.assembler.reset();
    sensor.next_side = kRightSide;
    sensor.assembly_side = kNoSide;
    sensor.phase = StartPhase::wait_dtr;
    sensor.deadline_ns = now + 200000000ULL;
    sensor.error.clear();
    return frames;
  }

  if (sensor.phase != StartPhase::streaming && now >= sensor.deadline_ns) {
    bool command_ok = true;
    if (sensor.phase == StartPhase::wait_dtr) {
      command_ok = writeCommand(sensor.fd, 0xF2, 0x01);
      sensor.phase = StartPhase::wait_stop;
      sensor.deadline_ns = now + 300000000ULL;
    } else if (sensor.phase == StartPhase::wait_stop) {
      command_ok = ::tcflush(sensor.fd, TCIOFLUSH) == 0 &&
                   writeCommand(sensor.fd, 0xF3, rate_subcommand_);
      sensor.phase = StartPhase::wait_rate;
      sensor.deadline_ns = now + 300000000ULL;
    } else if (sensor.phase == StartPhase::wait_rate) {
      command_ok = ::tcflush(sensor.fd, TCIOFLUSH) == 0 &&
                   writeCommand(sensor.fd, 0xF1, 0x01);
      if (command_ok) {
        sensor.phase = StartPhase::streaming;
        sensor.session_id = monotonicNowNs();
        sensor.sequence.fill(0);
        sensor.frames.fill(0);
        sensor.last_receive_ns.fill(0);
        sensor.next_side = kRightSide;
        sensor.assembly_side = kNoSide;
        ++sensor.connections;
      }
    }
    if (!command_ok) {
      sensor.error = "USB command/flush failed: " + std::string(std::strerror(errno));
      ::close(sensor.fd);
      sensor.fd = -1;
      sensor.phase = StartPhase::offline;
      sensor.retry_at_ns = now + retry_ns_;
      ++sensor.disconnects;
    }
    return frames;
  }

  if (sensor.phase != StartPhase::streaming) return frames;

    if (sensor.assembly_started_ns != 0 &&
        now - sensor.assembly_started_ns > frame_timeout_ns_) {
      sensor.assembler.reset();
      sensor.assembly_started_ns = 0;
      ++sensor.frame_timeouts;
    }
    if (!sensor.pending.empty() && sensor.pending_started_ns != 0 &&
        now - sensor.pending_started_ns > frame_timeout_ns_) {
      sensor.pending.clear();
      sensor.pending_started_ns = 0;
      sensor.assembler.reset();
      sensor.assembly_started_ns = 0;
      ++sensor.frame_timeouts;
    }

    pollfd descriptor{sensor.fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 0);
    if (ready < 0 && errno != EINTR) {
      sensor.error = "poll: " + std::string(std::strerror(errno));
      ::close(sensor.fd);
      sensor.fd = -1;
      sensor.phase = StartPhase::offline;
      sensor.retry_at_ns = now + retry_ns_;
      ++sensor.disconnects;
      sensor.session_id = 0;
      sensor.assembler.reset();
      sensor.pending.clear();
      sensor.pending_started_ns = 0;
      sensor.assembly_started_ns = 0;
      return frames;
    }
    if (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      sensor.error = "USB port disconnected";
      ::close(sensor.fd);
      sensor.fd = -1;
      sensor.phase = StartPhase::offline;
      sensor.retry_at_ns = now + retry_ns_;
      ++sensor.disconnects;
      sensor.session_id = 0;
      sensor.assembler.reset();
      sensor.pending.clear();
      sensor.pending_started_ns = 0;
      sensor.assembly_started_ns = 0;
      return frames;
    }
    if (ready <= 0 || !(descriptor.revents & POLLIN)) return frames;

    std::array<std::uint8_t, 8192> chunk{};
    for (;;) {
      const auto count = ::read(sensor.fd, chunk.data(), chunk.size());
      if (count > 0) {
        if (sensor.pending.empty()) sensor.pending_started_ns = monotonicNowNs();
        sensor.pending.insert(sensor.pending.end(), chunk.begin(), chunk.begin() + count);
        continue;
      }
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (count == 0) break;  // VMIN=0 raw nonblocking ports return zero when drained.
      sensor.error = "read: " + std::string(std::strerror(errno));
      ::close(sensor.fd);
      sensor.fd = -1;
      sensor.phase = StartPhase::offline;
      sensor.retry_at_ns = monotonicNowNs() + retry_ns_;
      ++sensor.disconnects;
      sensor.session_id = 0;
      sensor.assembler.reset();
      sensor.pending.clear();
      sensor.pending_started_ns = 0;
      sensor.assembly_started_ns = 0;
      break;
    }

    while (sensor.phase == StartPhase::streaming && sensor.pending.size() >= 3) {
      if (sensor.pending.front() != kStart) {
        const auto next = std::find(sensor.pending.begin() + 1, sensor.pending.end(), kStart);
        if (next == sensor.pending.end()) {
          sensor.pending.clear();
          sensor.pending_started_ns = 0;
          break;
        }
        sensor.pending.erase(sensor.pending.begin(), next);
        sensor.pending_started_ns = monotonicNowNs();
        continue;
      }
      const auto length = (static_cast<std::size_t>(sensor.pending[1]) << 8U) |
                          sensor.pending[2];
      if (length != 490) {
        sensor.pending.erase(sensor.pending.begin());
        continue;
      }
      if (sensor.pending.size() < kPacketSize) break;

      std::array<std::uint8_t, kPacketSize> packet{};
      std::copy_n(sensor.pending.begin(), kPacketSize, packet.begin());
      sensor.pending.erase(sensor.pending.begin(), sensor.pending.begin() + kPacketSize);
      sensor.pending_started_ns = sensor.pending.empty() ? 0 : monotonicNowNs();
      const auto packet_time = monotonicNowNs();
      ++sensor.packets;
      sensor.log.write("TPKT", 4);
      writeU64(sensor.log, sensor.session_id);
      writeU64(sensor.log, packet_time);
      writeU16(sensor.log, static_cast<std::uint16_t>(packet.size()));
      sensor.log.write(reinterpret_cast<const char*>(packet.data()),
                       static_cast<std::streamsize>(packet.size()));

      const auto index = packet[5];
      if (startsMap(packet)) {
        sensor.assembly_side = sensor.next_side;
        if (sensor.sensor_count == 2)
          sensor.next_side = sensor.next_side == kRightSide ? kLeftSide : kRightSide;
      }
      Grid grid{};
      const auto result = sensor.assembler.accept(packet, grid);
      if (result == FrameResult::rejected) {
        sensor.assembly_started_ns = 0;
      } else if (result == FrameResult::incomplete) {
        if (index == 0) sensor.assembly_started_ns = packet_time;
      } else {
        sensor.assembly_started_ns = 0;
        sensor.log.flush();
        if (sensor.assembly_side != kNoSide) {
          const auto side = sensor.assembly_side;
          ++sensor.frames[side];
          sensor.last_receive_ns[side] = packet_time;
          frames.push_back(RawFrame{sideName(side), std::move(grid), sensor.session_id,
                                    sensor.sequence[side]++, packet_time,
                                    sensor.assembler.packetMask()});
        }
        sensor.assembly_side = kNoSide;
      }
    }
  return frames;
}

std::array<SensorStatus, 2> UsbBackend::status() const {
  std::array<SensorStatus, 2> result{};
  const auto& sensor = *stream_;
  for (std::size_t side = kLeftSide; side <= kRightSide; ++side) {
    const bool enabled = sensor.sensor_count == 2 || side == kRightSide;
    result[side] = SensorStatus{
        sideName(side), sensor.path, enabled ? phaseName(sensor.phase) : "disabled",
        enabled ? sensor.error : "", enabled && sensor.phase == StartPhase::streaming,
        enabled ? sensor.session_id : 0, sensor.sequence[side], sensor.packets,
        sensor.frames[side], sensor.disconnects, sensor.frame_timeouts,
        sensor.last_receive_ns[side], sensor.assembler.stats()};
  }
  return result;
}

void UsbBackend::stop() noexcept {
  if (stopped_) return;
  stopped_ = true;
  auto& sensor = *stream_;
  if (sensor.fd >= 0) {
    if (sensor.phase == StartPhase::streaming) writeCommand(sensor.fd, 0xF2, 0x01);
    ::close(sensor.fd);
    sensor.fd = -1;
  }
  sensor.phase = StartPhase::offline;
  sensor.assembler.reset();
  sensor.pending.clear();
  sensor.log.flush();
  sensor.log.close();
}

}  // namespace raisin::tactile::usb
