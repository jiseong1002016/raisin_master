#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace raisin::tactile::usb {

inline constexpr std::size_t kPacketSize = 494;
inline constexpr std::size_t kPointsPerPacket = 243;
inline constexpr std::size_t kPacketCount = 12;
inline constexpr std::size_t kRows = 71;
inline constexpr std::size_t kCols = 41;
inline constexpr std::size_t kMapPoints = kRows * kCols;
inline constexpr std::size_t kWirePoints = kPointsPerPacket * kPacketCount;
inline constexpr std::uint16_t kFullPacketMask = (1U << kPacketCount) - 1U;
inline constexpr std::uint8_t kStart = 0x02;
inline constexpr std::uint8_t kEnd = 0x03;
inline constexpr std::uint8_t kDataCommand = 0xF1;

using Grid = std::array<std::int16_t, kMapPoints>;

struct Packet {
  std::uint8_t index{};
  std::array<std::int16_t, kPointsPerPacket> values{};
};

enum class PacketError : std::uint8_t {
  none,
  size,
  marker,
  length,
  command,
  crc,
  index,
};

std::uint8_t crc8(std::span<const std::uint8_t> bytes) noexcept;
std::array<std::uint8_t, 7> buildCommand(std::uint8_t command,
                                        std::uint8_t subcommand) noexcept;
PacketError decodePacket(std::span<const std::uint8_t> bytes,
                         Packet& packet) noexcept;

enum class FrameResult : std::uint8_t { rejected, incomplete, complete };

struct ParserStats {
  std::uint64_t crc_errors{};
  std::uint64_t malformed_packets{};
  std::uint64_t order_errors{};
  std::uint64_t incomplete_sets{};
  std::uint64_t completed_frames{};
};

// One instance belongs to one USB identity; never share it between sensors.
class FrameAssembler {
 public:
  FrameResult accept(std::span<const std::uint8_t> bytes, Grid& output) noexcept;
  void reset() noexcept;
  const ParserStats& stats() const noexcept { return stats_; }
  std::uint16_t packetMask() const noexcept { return packet_mask_; }

 private:
  void noteIncomplete() noexcept;
  void discardAssembly() noexcept;

  std::array<std::int16_t, kWirePoints> wire_{};
  std::uint8_t expected_index_{};
  std::uint16_t packet_mask_{};
  bool active_{};
  bool incomplete_counted_{};
  ParserStats stats_{};
};

}  // namespace raisin::tactile::usb
