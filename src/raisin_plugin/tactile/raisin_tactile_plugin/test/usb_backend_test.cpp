#include "raisin_tactile_plugin/usb_backend.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <stdlib.h>

using namespace raisin::tactile::usb;
using namespace std::chrono_literals;

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
      throw std::runtime_error("cannot get PTY path");
    slave_path = path.data();
  }
  ~Pty() { closeMaster(); }
  Pty(const Pty&) = delete;
  Pty& operator=(const Pty&) = delete;

  void closeMaster() {
    if (master >= 0) ::close(master);
    master = -1;
  }

  int master{-1};
  std::string slave_path;
};

std::vector<std::uint8_t> readFixture() {
  std::ifstream file(TACTILE_QT_FRAME_FIXTURE, std::ios::binary);
  if (!file) throw std::runtime_error("cannot open Qt packet fixture");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::vector<std::uint8_t> offsetFixture(const std::vector<std::uint8_t>& source,
                                        int offset) {
  auto result = source;
  for (std::size_t frame = 0; frame < kPacketCount; ++frame) {
    auto* packet = result.data() + frame * kPacketSize;
    for (std::size_t i = 0; i < kPointsPerPacket; ++i) {
      const auto at = 6 + 2 * i;
      const auto bits = static_cast<std::uint16_t>(
          (static_cast<std::uint16_t>(packet[at]) << 8U) | packet[at + 1]);
      const auto value = static_cast<std::int16_t>(std::bit_cast<std::int16_t>(bits) + offset);
      const auto changed = std::bit_cast<std::uint16_t>(value);
      packet[at] = static_cast<std::uint8_t>(changed >> 8U);
      packet[at + 1] = static_cast<std::uint8_t>(changed);
    }
    packet[kPacketSize - 2] = crc8(std::span<const std::uint8_t>(packet + 1, 491));
  }
  return result;
}

void writeAll(int fd, std::span<const std::uint8_t> data) {
  std::size_t offset = 0;
  while (offset < data.size()) {
    const auto result = ::write(fd, data.data() + offset, data.size() - offset);
    if (result > 0) {
      offset += static_cast<std::size_t>(result);
    } else if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      pollfd descriptor{fd, POLLOUT, 0};
      if (::poll(&descriptor, 1, 1000) <= 0) throw std::runtime_error("PTY write timed out");
    } else {
      throw std::runtime_error("PTY write failed");
    }
  }
}

std::vector<std::uint8_t> readAvailable(int fd) {
  std::vector<std::uint8_t> bytes;
  std::array<std::uint8_t, 64> buffer{};
  for (;;) {
    const auto count = ::read(fd, buffer.data(), buffer.size());
    if (count > 0) bytes.insert(bytes.end(), buffer.begin(), buffer.begin() + count);
    else if (count < 0 && errno == EINTR) continue;
    else break;
  }
  return bytes;
}

std::array<std::uint8_t, 7> command(std::uint8_t opcode, std::uint8_t subcode) {
  return buildCommand(opcode, subcode);
}

void expectCommand(int fd, std::uint8_t opcode, std::uint8_t subcode) {
  const auto expected = command(opcode, subcode);
  std::vector<std::uint8_t> received;
  for (int attempt = 0; attempt < 20 && received.size() < expected.size(); ++attempt) {
    pollfd descriptor{fd, POLLIN, 0};
    ::poll(&descriptor, 1, 25);
    auto part = readAvailable(fd);
    received.insert(received.end(), part.begin(), part.end());
  }
  require(received.size() == expected.size() &&
              std::equal(received.begin(), received.end(), expected.begin()),
          "USB startup command bytes or order differ from Qt");
}

void startStream(UsbBackend& backend, int stream) {
  std::this_thread::sleep_for(210ms);
  backend.poll();
  expectCommand(stream, 0xF2, 0x01);
  std::this_thread::sleep_for(310ms);
  backend.poll();
  expectCommand(stream, 0xF3, 0x02);
  std::this_thread::sleep_for(310ms);
  backend.poll();
  expectCommand(stream, 0xF1, 0x01);
}

std::vector<RawFrame> readFrames(UsbBackend& backend, std::size_t expected) {
  std::vector<RawFrame> frames;
  for (int attempt = 0; attempt < 100 && frames.size() < expected; ++attempt) {
    auto current = backend.poll();
    frames.insert(frames.end(), std::make_move_iterator(current.begin()),
                  std::make_move_iterator(current.end()));
    std::this_thread::sleep_for(1ms);
  }
  return frames;
}

void sendSplitFrame(UsbBackend& backend, int stream,
                    const std::vector<std::uint8_t>& bytes) {
  constexpr std::size_t split = 137;
  writeAll(stream, std::span<const std::uint8_t>(bytes).first(split));
  require(backend.poll().empty(), "partial serial frame was published");
  writeAll(stream, std::span<const std::uint8_t>(bytes).subspan(split));
}

}  // namespace

int main() {
  char temp_template[] = "/tmp/raisin-tactile-usb-test-XXXXXX";
  const auto* temp = ::mkdtemp(temp_template);
  if (!temp) throw std::runtime_error("cannot create test directory");
  const std::filesystem::path root(temp);
  try {
    Pty stream_pty;
    Pty replacement_pty;
    const auto stream_link = root / "stream-port";
    std::filesystem::create_symlink(stream_pty.slave_path, stream_link);

    UsbBackend backend(stream_link.string(), 2, root / "record", 20,
                       115200, 0.2, 0.05);
    require(backend.poll().empty(), "connection setup produced a frame");
    startStream(backend, stream_pty.master);
    auto statuses = backend.status();
    require(statuses[0].online && statuses[1].online &&
                statuses[0].session_id == statuses[1].session_id &&
                statuses[0].port == statuses[1].port,
            "logical sides did not share one USB stream session");

    const auto fixture = readFixture();
    require(fixture.size() == kPacketCount * kPacketSize,
            "fixture does not contain one complete frame");
    const auto left_fixture = offsetFixture(fixture, 1234);
    sendSplitFrame(backend, stream_pty.master, fixture);
    auto frames = readFrames(backend, 1);
    require(frames.size() == 1 && frames[0].sensor == "right" &&
                frames[0].raw_code[0] == -1458 && frames[0].sequence == 0 &&
                frames[0].packet_mask == kFullPacketMask,
            "first complete map was not assigned to the right side");
    sendSplitFrame(backend, stream_pty.master, left_fixture);
    frames = readFrames(backend, 1);
    require(frames.size() == 1 && frames[0].sensor == "left" &&
                frames[0].raw_code[0] == -224 && frames[0].sequence == 0 &&
                frames[0].session_id == statuses[1].session_id,
            "second complete map was not assigned to the left side in the shared session");

    const auto packet0 = std::span<const std::uint8_t>(fixture).first(kPacketSize);
    const auto packet2 = std::span<const std::uint8_t>(fixture).subspan(2 * kPacketSize,
                                                                        kPacketSize);
    writeAll(stream_pty.master, packet0);
    backend.poll();
    writeAll(stream_pty.master, packet2);
    backend.poll();
    require(backend.status()[0].parser.order_errors == 1,
            "missing packet was not detected on the shared stream");
    sendSplitFrame(backend, stream_pty.master, fixture);
    frames = readFrames(backend, 1);
    require(frames.size() == 1 && frames[0].sensor == "left",
            "a partial right map did not leave the next map assigned left");

    writeAll(stream_pty.master, packet0);
    backend.poll();
    std::this_thread::sleep_for(220ms);
    backend.poll();
    require(backend.status()[1].frame_timeouts > 0,
            "incomplete map did not time out");
    sendSplitFrame(backend, stream_pty.master, left_fixture);
    frames = readFrames(backend, 1);
    require(frames.size() == 1 && frames[0].sensor == "left",
            "map order did not continue after a timed-out map");

    const auto old_session = backend.status()[1].session_id;
    stream_pty.closeMaster();
    backend.poll();
    statuses = backend.status();
    require(!statuses[0].online && !statuses[1].online && statuses[0].disconnects == 1,
            "shared USB disconnect did not fault both logical sides");
    std::filesystem::remove(stream_link);
    std::filesystem::create_symlink(replacement_pty.slave_path, stream_link);
    std::this_thread::sleep_for(60ms);
    backend.poll();
    startStream(backend, replacement_pty.master);
    statuses = backend.status();
    require(statuses[0].online && statuses[1].online &&
                statuses[0].session_id != old_session && statuses[1].session_id != old_session,
            "shared USB stream did not reconnect into a new session");
    sendSplitFrame(backend, replacement_pty.master, fixture);
    frames = readFrames(backend, 1);
    require(frames.size() == 1 && frames[0].sensor == "right" && frames[0].sequence == 0,
            "reconnected stream did not restart map order and sequence on right");

    const auto raw_log = root / "record/stream.rawpkt";
    require(std::filesystem::file_size(raw_log) > 8,
            "shared raw packet stream log was not written");
    std::ifstream log(raw_log, std::ios::binary);
    std::array<char, 4> magic{};
    log.read(magic.data(), magic.size());
    require(std::string(magic.data(), magic.size()) == "TACR",
            "raw recording header is invalid");
    backend.stop();
    require(!backend.status()[0].online && !backend.status()[1].online,
            "stop did not close the shared USB session");

    Pty single_pty;
    UsbBackend single(single_pty.slave_path, 1, root / "single_record", 20,
                      115200, 0.2, 0.05);
    single.poll();
    startStream(single, single_pty.master);
    sendSplitFrame(single, single_pty.master, fixture);
    auto single_frames = readFrames(single, 1);
    sendSplitFrame(single, single_pty.master, left_fixture);
    auto next_single_frames = readFrames(single, 1);
    require(single_frames.size() == 1 && next_single_frames.size() == 1 &&
                single_frames[0].sensor == "right" && next_single_frames[0].sensor == "right" &&
                single_frames[0].sequence == 0 && next_single_frames[0].sequence == 1 &&
                single.status()[0].state == "disabled",
            "one-sensor mode must route every shared map to right only");
    single.stop();
    std::filesystem::remove_all(root);
    std::cout << "PASS: one-port startup, right-first map routing, one/two-sensor modes, timeout, reconnect, raw log, stop\n";
  } catch (...) {
    std::filesystem::remove_all(root);
    throw;
  }
}
