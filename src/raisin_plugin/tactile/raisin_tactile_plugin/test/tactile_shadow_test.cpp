#include "raisin_tactile_sensor/shadow_consumer.hpp"

#include <iostream>
#include <stdexcept>

using raisin::tactile::FrameV2;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

FrameV2 frame(const char* sensor, std::uint64_t session, std::uint64_t sequence,
              std::uint64_t stamp_ns) {
  FrameV2 value;
  value.sensor_name = sensor;
  value.frame_id = std::string("tactile_") + sensor + "_frame";
  value.clock_domain = "CLOCK_MONOTONIC";
  value.calibration_id = "provisional_linear_0p001";
  value.calibration_status = "provisional";
  value.baseline_status = "operator_confirmed_unloaded";
  value.force_unit = "N_estimate";
  value.stamp_monotonic_ns = stamp_ns;
  value.session_id = session;
  value.sequence = sequence;
  value.rows = raisin::tactile::kUsbRows;
  value.cols = raisin::tactile::kUsbCols;
  value.code_to_force_n_per_code = 0.001F;
  value.polarity = 1;
  value.baseline_valid = true;
  value.raw_code.resize(raisin::tactile::kUsbCells);
  value.baseline_code.resize(raisin::tactile::kUsbCells);
  value.normal_force_n.resize(raisin::tactile::kUsbCells);
  value.valid.assign(raisin::tactile::kUsbCells, 1);
  for (std::size_t row = 0; row < value.rows; ++row)
    value.valid[row * value.cols + value.cols - 1] = 0;
  value.validate(sensor);
  return value;
}

}  // namespace

int main() {
  raisin::tactile::ShadowPairConsumer consumer;
  std::optional<FrameV2> left, right;
  std::size_t emitted = 0;
  for (std::uint64_t tick = 0; tick < 100; ++tick) {
    const auto now = 1000000000ULL + tick * 10000000ULL;
    if (tick % 5 == 0) {
      const auto sequence = tick / 5;
      left = frame("left", 11, sequence, now - 3000000ULL);
      right = frame("right", 22, sequence, now - 1000000ULL);
    }
    const auto event = consumer.tick(left, right, now);
    if (event) {
      ++emitted;
      require(event->left.sequence == event->right.sequence,
              "shadow event paired different 20 Hz sample indices");
    }
  }
  require(emitted == 20, "100 Hz shadow loop must emit once for each of 20 new pairs");

  raisin::tactile::ShadowPairConsumer gates;
  auto fresh_left = frame("left", 31, 0, 2000000000ULL);
  auto fresh_right = frame("right", 32, 0, 2020000000ULL);
  require(gates.tick(fresh_left, fresh_right, 2030000000ULL).has_value(),
          "fresh dual baseline pair was not admitted");
  require(!gates.tick(fresh_left, fresh_right, 2030000000ULL),
          "unchanged pair was emitted twice");

  auto one_side = fresh_left;
  one_side.sequence = 1;
  one_side.stamp_monotonic_ns += 10000000ULL;
  require(!gates.tick(one_side, fresh_right, 2040000000ULL),
          "one-sided sequence update emitted a shadow event");
  auto other_side = fresh_right;
  other_side.sequence = 1;
  other_side.stamp_monotonic_ns += 15000000ULL;
  require(gates.tick(one_side, other_side, 2040000000ULL).has_value(),
          "new pair was not admitted after both sides advanced");

  auto stale_left = one_side;
  auto stale_right = other_side;
  stale_left.sequence++;
  stale_right.sequence++;
  require(!gates.tick(stale_left, stale_right, 2300000000ULL),
          "stale pair emitted a shadow event");
  auto future_left = stale_left;
  auto future_right = stale_right;
  future_left.sequence++;
  future_right.sequence++;
  future_left.stamp_monotonic_ns = 2400000000ULL;
  future_right.stamp_monotonic_ns = 2410000000ULL;
  require(!gates.tick(future_left, future_right, 2350000000ULL),
          "future-stamped pair emitted a shadow event");
  std::cout << "PASS: 100 Hz shadow loop emits once per fresh 20 Hz bilateral pair; duplicate, one-sided, stale and future pairs emit zero events\n";
}
