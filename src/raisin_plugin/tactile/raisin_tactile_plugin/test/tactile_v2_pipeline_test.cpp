#include "raisin_tactile_plugin/usb_conversion.hpp"
#include "raisin_tactile_sensor/frame_v2.hpp"
#include "raisin_tactile_sensor/recording.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

using namespace raisin::tactile;
using namespace raisin::tactile::usb;

namespace raisin_tactile_sensor = raisin::raisin_tactile_sensor;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

RawFrame rawFrame(const char* sensor, std::uint64_t session, std::uint64_t sequence,
                  std::uint64_t stamp_ns, std::int16_t value) {
  RawFrame frame;
  frame.sensor = sensor;
  frame.session_id = session;
  frame.sequence = sequence;
  frame.stamp_monotonic_ns = stamp_ns;
  frame.raw_code.fill(value);
  return frame;
}

constexpr std::size_t cellIndex(std::size_t row, std::size_t col) {
  return row * kUsbCols + col;
}

FrameV2 checkedFrame(const raisin_tactile_sensor::msg::TactileFrame& message,
                     const char* sensor) {
  return fromMessage(message, sensor);
}

}  // namespace

int main() {
  FrameConverter converter(ConversionConfig{0.001F, "provisional_linear_0p001", {1, -1}});
  auto left = rawFrame("left", 11, 0, 1000000000ULL, 100);
  auto pending = converter.convert(left);
  require(!pending.baseline_valid && pending.baseline_status == "awaiting_unloaded_baseline" &&
              pending.force_unit == "N_estimate" && pending.calibration_status == "provisional" &&
              pending.valid.size() == kUsbCells && pending.valid[0] == 0,
          "unconfirmed baseline must preserve raw data and suppress force validity");

  require(converter.requestBaseline("capture_baseline:left"),
          "left baseline command was rejected");
  left.sequence = 1;
  left.stamp_monotonic_ns += 50000000ULL;
  left.packet_mask &= static_cast<std::uint16_t>(~(1U << 1));
  auto partial_baseline = converter.convert(left);
  require(!partial_baseline.baseline_valid && partial_baseline.valid[0] == 0,
          "partial packet set was accepted as an unloaded baseline");
  left.sequence = 2;
  left.stamp_monotonic_ns += 10000000ULL;
  left.packet_mask = kFullPacketMask;
  auto left_baseline = converter.convert(left);
  require(left_baseline.baseline_valid && left_baseline.baseline_code == left_baseline.raw_code &&
              left_baseline.baseline_status == "operator_confirmed_unloaded" &&
              static_cast<std::size_t>(std::count(left_baseline.valid.begin(),
                                                  left_baseline.valid.end(), 1)) == kUsbRoiCells &&
              left_baseline.valid[cellIndex(35, 18)] == 0 &&
              left_baseline.valid[cellIndex(36, 17)] == 0 &&
              left_baseline.valid[cellIndex(36, 18)] == 1 &&
              left_baseline.valid[cellIndex(70, 39)] == 1 &&
              left_baseline.valid[cellIndex(70, 40)] == 0,
          "left baseline capture or inclusive 35x22 ROI mask is wrong");
  left.sequence = 3;
  left.stamp_monotonic_ns += 10000000ULL;
  left.raw_code[0] = 120;
  left.raw_code[cellIndex(36, 18)] = 120;
  left.raw_code[cellIndex(36, 19)] = 80;
  auto left_msg = converter.convert(left);
  require(std::abs(left_msg.normal_force_n[cellIndex(36, 18)] - 0.02F) < 1e-6F &&
              left_msg.normal_force_n[cellIndex(36, 19)] == 0.F &&
              left_msg.valid[cellIndex(36, 18)] == 1 &&
              left_msg.valid[cellIndex(36, 19)] == 1 &&
              left_msg.normal_force_n[0] == 0.F && left_msg.valid[0] == 0 &&
              left_msg.raw_code[0] == 120 && left_msg.baseline_code[0] == 100 &&
              left_msg.raw_code[cellIndex(36, 18)] == 120 &&
              left_msg.raw_code.size() == kUsbCells,
          "ROI force conversion or full-shape raw/baseline retention failed");
  left.sequence = 4;
  left.stamp_monotonic_ns += 10000000ULL;
  const auto missing_roi_packet = (18 * kUsbRows + 36) / kPointsPerPacket;
  const auto retained_roi_packet = (22 * kUsbRows + 36) / kPointsPerPacket;
  left.packet_mask = static_cast<std::uint16_t>(kFullPacketMask &
                                                 ~(1U << missing_roi_packet));
  auto partial_frame = converter.convert(left);
  require(partial_frame.valid[cellIndex(36, 18)] == 0 &&
              partial_frame.normal_force_n[cellIndex(36, 18)] == 0.F &&
              partial_frame.valid[cellIndex(36, 22)] == 1 &&
              retained_roi_packet != missing_roi_packet &&
              partial_frame.valid[0] == 0,
          "partial packet did not mask only the missing ROI packet");
  for (std::size_t row = 0; row < kUsbRows; ++row) {
    for (std::size_t col = 0; col < kUsbCols; ++col) {
      const auto index = cellIndex(row, col);
      const auto packet = (col * kUsbRows + row) / kPointsPerPacket;
      if (!isUsbRoiCell(row, col))
        require(partial_frame.valid[index] == 0 && partial_frame.normal_force_n[index] == 0.F,
                "partial frame exposed force outside the configured ROI");
      if (partial_frame.valid[index])
        require(partial_frame.valid[index] == 1 && isUsbRoiCell(row, col) &&
                    (left.packet_mask & (1U << packet)) != 0,
                "partial frame marked a cell outside the ROI or in a missing packet valid");
    }
  }
  auto reconnected = rawFrame("left", 12, 0, 1200000000ULL, 300);
  auto needs_new_baseline = converter.convert(reconnected);
  require(!needs_new_baseline.baseline_valid && needs_new_baseline.valid[0] == 0 &&
              needs_new_baseline.raw_code[0] == 300,
          "new source session reused a stale baseline");

  auto right = rawFrame("right", 22, 0, 1050000000ULL, 200);
  auto right_pending = converter.convert(right);
  require(!right_pending.baseline_valid && right_pending.valid[0] == 0,
          "left baseline leaked into the right sensor");
  require(converter.requestBaseline("capture_baseline:right"),
          "right baseline command was rejected");
  right.sequence = 1;
  right.stamp_monotonic_ns += 10000000ULL;
  converter.convert(right);
  right.sequence = 2;
  right.stamp_monotonic_ns += 20000000ULL;
  right.raw_code[0] = 190;
  right.raw_code[cellIndex(36, 18)] = 190;
  auto right_msg = converter.convert(right);
  require(std::abs(right_msg.normal_force_n[cellIndex(36, 18)] - 0.01F) < 1e-6F &&
              right_msg.valid[cellIndex(36, 18)] == 1 && right_msg.normal_force_n[0] == 0.F &&
              right_msg.raw_code[0] == 190 && right_msg.polarity == -1,
          "right-side polarity was not applied independently");

  std::vector<unsigned char> serialized;
  left_msg.setBuffer(serialized);
  raisin_tactile_sensor::msg::TactileFrame decoded;
  decoded.getBuffer(serialized);
  require(decoded == left_msg, "generated Raisin v2 message failed serialization round trip");

  auto left_frame = checkedFrame(left_msg, "left");
  auto right_frame = checkedFrame(right_msg, "right");
  char record_template[] = "/tmp/raisin-tactile-recording-test-XXXXXX";
  const auto* record_root = ::mkdtemp(record_template);
  require(record_root != nullptr, "cannot create tactile recording test directory");
  const std::filesystem::path record_path(record_root);
  FrameRecorder recorder;
  recorder.start(record_path);
  require(recorder.write(left_frame) && recorder.write(right_frame),
          "frame recording did not accept both sensors");
  recorder.stop();
  const auto left_replay = readRecordedFrames(record_path / "left.tfr2", "left");
  const auto right_replay = readRecordedFrames(record_path / "right.tfr2", "right");
  require(left_replay.size() == 1 && right_replay.size() == 1 &&
              left_replay[0].raw_code == left_frame.raw_code &&
              left_replay[0].baseline_code == left_frame.baseline_code &&
              left_replay[0].normal_force_n == left_frame.normal_force_n &&
              left_replay[0].valid == left_frame.valid &&
              left_replay[0].raw_code.size() == kUsbCells &&
              left_replay[0].rows == kUsbRows && left_replay[0].cols == kUsbCols &&
              right_replay[0].session_id == right_frame.session_id,
          "record/replay did not preserve full raw shape, ROI mask, map, or identity");
  std::filesystem::remove_all(record_path);

  SnapshotV2 left_snapshot("left");
  require(left_snapshot.set(left_frame), "first left session frame was rejected");
  auto duplicate = left_frame;
  duplicate.stamp_monotonic_ns += 1;
  require(!left_snapshot.set(duplicate), "duplicate left sequence was accepted");
  auto out_of_order = left_frame;
  out_of_order.sequence = 1;
  out_of_order.stamp_monotonic_ns += 2;
  require(!left_snapshot.set(out_of_order), "out-of-order left sequence was accepted");
  auto restarted = left_frame;
  restarted.session_id = 12;
  restarted.sequence = 0;
  restarted.stamp_monotonic_ns += 1000000ULL;
  require(left_snapshot.set(restarted), "new session sequence zero was not admitted");

  PairAdmitter pairer;
  const auto now = right_frame.stamp_monotonic_ns + 20000000ULL;
  auto pair = pairer.admit(left_frame, right_frame, now);
  require(pair && pair->skew_ns == 10000000ULL,
          "fresh baseline-valid left/right frames did not form a pair");
  require(!pairer.admit(left_frame, right_frame, now),
          "same pair was emitted repeatedly without new sequences");
  auto left_next = left_frame;
  left_next.sequence++;
  left_next.stamp_monotonic_ns += 20000000ULL;
  require(!pairer.admit(left_next, right_frame, now + 20000000ULL),
          "one new side was admitted without a new frame from the other side");
  auto right_next = right_frame;
  right_next.sequence++;
  right_next.stamp_monotonic_ns += 25000000ULL;
  require(pairer.admit(left_next, right_next, now + 25000000ULL).has_value(),
          "a fresh pair was not admitted after both sides advanced");
  require(!pairer.admit(left_next, right_next, now + 25000000ULL + 150000001ULL),
          "stale pair was admitted");
  auto future = right_next;
  future.stamp_monotonic_ns += 100000000ULL;
  require(!pairer.admit(left_next, future, now + 25000000ULL),
          "future-stamped pair was admitted");
  auto skewed = right_next;
  skewed.sequence++;
  skewed.stamp_monotonic_ns = left_next.stamp_monotonic_ns + 50000001ULL;
  auto left_new = left_next;
  left_new.sequence++;
  require(!pairer.admit(left_new, skewed, skewed.stamp_monotonic_ns + 1000000ULL),
          "pair beyond configured skew was admitted");
  auto no_baseline = left_next;
  no_baseline.session_id++;
  no_baseline.sequence = 0;
  no_baseline.baseline_valid = false;
  no_baseline.baseline_status = "awaiting_unloaded_baseline";
  std::fill(no_baseline.valid.begin(), no_baseline.valid.end(), 0);
  require(!pairer.admit(no_baseline, right_next, now + 30000000ULL),
          "pair without both unloaded baselines was admitted");

  auto wrong_domain = left_frame;
  wrong_domain.clock_domain = "UNIX_TIME";
  bool rejected_domain = false;
  try { wrong_domain.validate("left"); }
  catch (const std::invalid_argument&) { rejected_domain = true; }
  require(rejected_domain, "non-monotonic clock domain was accepted");
  std::cout << "PASS: v2 raw/baseline/N_estimate conversion, message serialization, receiver ordering, bilateral freshness/skew gates\n";
}
