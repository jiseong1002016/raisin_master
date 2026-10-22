#include "raisin_tactile_plugin/usb_protocol.hpp"

#include <bit>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

using namespace raisin::tactile::usb;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::vector<std::uint8_t> readFixture() {
  std::ifstream file(TACTILE_QT_FRAME_FIXTURE, std::ios::binary);
  if (!file) throw std::runtime_error("cannot open Qt packet fixture");
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::vector<std::uint8_t> offsetFixture(const std::vector<std::uint8_t>& source) {
  auto result = source;
  for (std::size_t frame = 0; frame < kPacketCount; ++frame) {
    auto* packet = result.data() + frame * kPacketSize;
    for (std::size_t i = 0; i < kPointsPerPacket; ++i) {
      const auto offset = 6 + 2 * i;
      const auto bits = static_cast<std::uint16_t>(
          (static_cast<std::uint16_t>(packet[offset]) << 8U) | packet[offset + 1]);
      const auto value = static_cast<std::int16_t>(
          std::bit_cast<std::int16_t>(bits) + 1234);
      const auto shifted_bits = std::bit_cast<std::uint16_t>(value);
      packet[offset] = static_cast<std::uint8_t>(
          shifted_bits >> 8U);
      packet[offset + 1] = static_cast<std::uint8_t>(shifted_bits);
    }
    packet[kPacketSize - 2] = crc8(std::span<const std::uint8_t>(packet + 1, 491));
  }
  return result;
}

std::span<const std::uint8_t> packetAt(const std::vector<std::uint8_t>& bytes,
                                       std::size_t index) {
  return std::span<const std::uint8_t>(bytes).subspan(index * kPacketSize,
                                                       kPacketSize);
}

void checkMap(const Grid& grid, int offset) {
  require(grid.size() == 71 * 41, "map dimensions changed");
  for (std::size_t row = 0; row < kRows; ++row) {
    for (std::size_t col = 0; col < kCols; ++col) {
      const auto wire_index = col * kRows + row;
      const auto expected = col == 40 ? 0 : static_cast<int>(wire_index) - 1458;
      require(grid[row * kCols + col] == expected + offset,
              "Qt wire ordering or signed big-endian decode changed");
    }
  }
}

}  // namespace

int main() {
  const auto command = buildCommand(0xF1, 0x01);
  const std::array<std::uint8_t, 7> expected_command{0x02, 0x00, 0x03,
                                                     0xF1, 0x01, 0xE7, 0x03};
  require(command == expected_command, "start command differs from Qt");

  const auto left_bytes = readFixture();
  require(left_bytes.size() == kPacketCount * kPacketSize,
          "fixture is not one complete 12-packet Qt frame");
  const auto right_bytes = offsetFixture(left_bytes);
  FrameAssembler left;
  FrameAssembler right;
  Grid left_map{}, right_map{};
  for (std::size_t index = 0; index < kPacketCount; ++index) {
    const auto expected = index + 1 == kPacketCount ? FrameResult::complete
                                                    : FrameResult::incomplete;
    require(left.accept(packetAt(left_bytes, index), left_map) == expected,
            "left packet assembly failed");
    require(right.accept(packetAt(right_bytes, index), right_map) == expected,
            "right packet assembly failed");
  }
  checkMap(left_map, 0);
  checkMap(right_map, 1234);
  require(left.stats().completed_frames == 1 && right.stats().completed_frames == 1,
          "each identity must complete exactly one frame");

  FrameAssembler damaged;
  Grid ignored{};
  auto corrupt = std::vector<std::uint8_t>(packetAt(left_bytes, 0).begin(),
                                           packetAt(left_bytes, 0).end());
  corrupt[kPacketSize - 2] ^= 0x01;
  require(damaged.accept(corrupt, ignored) == FrameResult::rejected,
          "bad CRC packet was accepted");
  require(damaged.stats().crc_errors == 1 && damaged.stats().completed_frames == 0,
          "bad CRC statistics are wrong");

  FrameAssembler reordered;
  require(reordered.accept(packetAt(left_bytes, 0), ignored) == FrameResult::incomplete,
          "packet zero did not start a frame");
  require(reordered.accept(packetAt(left_bytes, 2), ignored) == FrameResult::incomplete,
          "out-of-order packet was discarded");
  require(reordered.accept(packetAt(left_bytes, 1), ignored) == FrameResult::incomplete,
          "late packet was discarded");
  for (std::size_t index = 3; index < kPacketCount; ++index) {
    const auto expected = index + 1 == kPacketCount ? FrameResult::complete
                                                    : FrameResult::incomplete;
    require(reordered.accept(packetAt(left_bytes, index), ignored) == expected,
            "out-of-order full frame failed to assemble");
  }
  checkMap(ignored, 0);
  require(reordered.packetMask() == kFullPacketMask &&
              reordered.stats().order_errors == 2 &&
              reordered.stats().completed_frames == 1,
          "out-of-order full-frame accounting is wrong");

  FrameAssembler partial;
  for (std::size_t index = 0; index < kPacketCount; ++index) {
    if (index == 5) continue;
    const auto expected = index == kPacketCount - 1 ? FrameResult::complete
                                                    : FrameResult::incomplete;
    require(partial.accept(packetAt(left_bytes, index), ignored) == expected,
            "partial Qt-style frame did not emit on its final packet");
  }
  require(partial.packetMask() == (kFullPacketMask & ~(1U << 5)) &&
              partial.stats().incomplete_sets == 1 &&
              partial.stats().completed_frames == 1,
          "missing packet mask or partial-frame accounting is wrong");
  for (std::size_t row = 0; row < kRows; ++row) {
    for (std::size_t col = 0; col < kCols; ++col) {
      const auto wire_index = col * kRows + row;
      const auto expected = col == 40 || wire_index / kPointsPerPacket == 5
                                ? 0
                                : static_cast<int>(wire_index) - 1458;
      require(ignored[row * kCols + col] == expected,
              "missing packet data was not zero-filled");
    }
  }

  FrameAssembler resumed;
  for (std::size_t index = 0; index < kPacketCount; ++index)
    require(resumed.accept(packetAt(left_bytes, index), ignored) ==
                (index + 1 == kPacketCount ? FrameResult::complete
                                           : FrameResult::incomplete),
            "complete frame failed after a rejected partial set");
  checkMap(ignored, 0);
  std::cout << "PASS: Qt CRC/command, indexed packet assembly, partial masks, endian/order/padding, and two isolated parser states\n";
}
