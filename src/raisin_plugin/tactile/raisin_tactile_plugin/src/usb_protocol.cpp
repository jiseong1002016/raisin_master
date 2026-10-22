#include "raisin_tactile_plugin/usb_protocol.hpp"

#include <bit>

namespace raisin::tactile::usb {

std::uint8_t crc8(std::span<const std::uint8_t> bytes) noexcept {
  std::uint8_t crc = 0;
  for (const auto byte : bytes) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) ? static_cast<std::uint8_t>((crc >> 1U) ^ 0x8CU)
                       : static_cast<std::uint8_t>(crc >> 1U);
  }
  return crc;
}

std::array<std::uint8_t, 7> buildCommand(std::uint8_t command,
                                        std::uint8_t subcommand) noexcept {
  std::array<std::uint8_t, 7> packet{kStart, 0x00, 0x03, command, subcommand, 0, kEnd};
  packet[5] = crc8(std::span<const std::uint8_t>(packet).subspan(1, 4));
  return packet;
}

PacketError decodePacket(std::span<const std::uint8_t> bytes,
                         Packet& packet) noexcept {
  if (bytes.size() != kPacketSize) return PacketError::size;
  if (bytes.front() != kStart || bytes.back() != kEnd) return PacketError::marker;
  const std::size_t length = (static_cast<std::size_t>(bytes[1]) << 8U) | bytes[2];
  if (length + 4 != kPacketSize || length != 490) return PacketError::length;
  if (bytes[3] != kDataCommand || bytes[4] != 0x01) return PacketError::command;
  if (crc8(bytes.subspan(1, length + 1)) != bytes[kPacketSize - 2])
    return PacketError::crc;
  if (bytes[5] >= kPacketCount) return PacketError::index;

  packet.index = bytes[5];
  for (std::size_t i = 0; i < kPointsPerPacket; ++i) {
    const auto offset = 6 + 2 * i;
    const auto bits = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(bytes[offset]) << 8U) | bytes[offset + 1]);
    packet.values[i] = std::bit_cast<std::int16_t>(bits);
  }
  return PacketError::none;
}

void FrameAssembler::noteIncomplete() noexcept {
  if (active_ && packet_mask_ != 0 && packet_mask_ != kFullPacketMask &&
      !incomplete_counted_) {
    ++stats_.incomplete_sets;
    incomplete_counted_ = true;
  }
}

void FrameAssembler::discardAssembly() noexcept {
  noteIncomplete();
  wire_.fill(0);
  expected_index_ = 0;
  packet_mask_ = 0;
  active_ = false;
  incomplete_counted_ = false;
}

FrameResult FrameAssembler::accept(std::span<const std::uint8_t> bytes,
                                   Grid& output) noexcept {
  Packet packet;
  const auto error = decodePacket(bytes, packet);
  if (error != PacketError::none) {
    if (error == PacketError::crc) ++stats_.crc_errors;
    else ++stats_.malformed_packets;
    discardAssembly();
    return FrameResult::rejected;
  }

  if (packet.index == 0) {
    noteIncomplete();
    wire_.fill(0);
    packet_mask_ = 0;
    expected_index_ = 0;
    active_ = true;
    incomplete_counted_ = false;
  } else if (!active_) {
    ++stats_.order_errors;
    return FrameResult::rejected;
  }

  if (packet.index != expected_index_ ||
      (packet_mask_ & (1U << packet.index)) != 0)
    ++stats_.order_errors;

  const auto begin = static_cast<std::size_t>(packet.index) * kPointsPerPacket;
  for (std::size_t i = 0; i < kPointsPerPacket; ++i)
    wire_[begin + i] = packet.values[i];
  packet_mask_ |= static_cast<std::uint16_t>(1U << packet.index);
  if (packet.index >= expected_index_)
    expected_index_ = static_cast<std::uint8_t>(packet.index + 1U);

  if (packet.index != kPacketCount - 1) return FrameResult::incomplete;

  if (packet_mask_ != kFullPacketMask) noteIncomplete();
  for (std::size_t row = 0; row < kRows; ++row)
    for (std::size_t col = 0; col < kCols; ++col)
      output[row * kCols + col] = wire_[col * kRows + row];

  ++stats_.completed_frames;
  return FrameResult::complete;
}

void FrameAssembler::reset() noexcept { discardAssembly(); }

}  // namespace raisin::tactile::usb
