#include "ee_maintenance_sequence.hpp"

#include <cassert>
#include <limits>
#include <set>
#include <utility>

using Sequence = raisin::robot::EeMaintenanceSequence;
using namespace std::chrono_literals;

namespace {
Sequence::Config config() {
  return {{-0.7, 0.}, {0., 0.45}, 0.001, 20ms, 1000ms, 100ms};
}
Sequence::Readback readback(Sequence::TimePoint now, Sequence::Target position = {}) {
  return {position, {true, true}, {now, now}, {false, false}};
}
void testCoupledMotion() {
  const Sequence::TimePoint origin{};
  for (std::size_t lead = 0; lead < 2; ++lead) {
    const auto follow = 1 - lead;
    for (double relative : {0.743, 0.05}) {
      auto coupled = config();
      coupled.lower_limit_rad[lead] = 0.;
      coupled.upper_limit_rad[lead] = 0.96;
      coupled.lower_limit_rad[follow] = -0.96;
      coupled.upper_limit_rad[follow] = 0.;
      coupled.first_test_axis = lead;
      coupled.max_relative_angle_rad = relative;
      const auto feasible = [&](const Sequence::Target& point) {
        for (std::size_t axis = 0; axis < 2; ++axis) {
          assert(point[axis] >= coupled.lower_limit_rad[axis] - 1.e-12);
          assert(point[axis] <= coupled.upper_limit_rad[axis] + 1.e-12);
        }
        const double difference = point[lead] + point[follow];
        assert(difference >= -1.e-12 && difference <= relative + 1.e-12);
      };
      for (const auto mode : {Sequence::Mode::CLOSE, Sequence::Mode::TEST,
                              Sequence::Mode::GET_READY}) {
        Sequence sequence(coupled);
        Sequence::Target position{};
        position[lead] = 0.96;
        position[follow] = -0.96;
        sequence.start(mode, readback(origin, position), origin);
        auto now = origin;
        bool openedTogether = false;
        for (int step = 0; step < 1000 && !sequence.done() &&
             sequence.status() != Sequence::Status::HOLD; ++step) {
          const auto target = sequence.target();
          for (int sample = 0; sample <= 100; ++sample) {
            Sequence::Target interpolated{};
            for (std::size_t axis = 0; axis < 2; ++axis)
              interpolated[axis] = position[axis] +
                  (target[axis] - position[axis]) * sample / 100.;
            feasible(interpolated);
          }
          if (sequence.status() == Sequence::Status::TEST &&
              target[lead] > 0. && target[follow] < 0.)
            openedTogether = true;
          position = target;
          now += 20ms;
          sequence.update(now, readback(now, position));
          assert(!sequence.failed());
        }
        assert(sequence.target() == Sequence::Target{});
        assert(sequence.done() || sequence.status() == Sequence::Status::HOLD);
        assert(openedTogether == (mode == Sequence::Mode::TEST));
      }
      Sequence sequence(coupled);
      sequence.start(Sequence::Mode::TEST, readback(origin), origin);
      auto now = origin + 20ms;
      sequence.update(now, readback(now));
      for (const auto& scale : std::array<Sequence::Target, 3>{{
               {1., 0.}, {1., -1.}, {0., 0.}}}) {
        const double excursion = std::min(std::acos(-1.) / 6., relative);
        const auto target = sequence.target();
        assert(target[lead] == scale[0] * excursion);
        assert(target[follow] == scale[1] * excursion);
        for (int sample = 0; sample < 2; ++sample) {
          now += 20ms;
          sequence.update(now, readback(now, target));
        }
      }
      assert(sequence.done());
      coupled.upper_limit_rad[lead] = 0.08;
      coupled.lower_limit_rad[follow] = -0.03;
      Sequence narrow(coupled);
      narrow.start(Sequence::Mode::TEST, readback(origin), origin);
      now = origin;
      for (int step = 0; step < 100 && !narrow.done(); ++step) {
        feasible(narrow.target());
        now += 20ms;
        narrow.update(now, readback(now, narrow.target()));
      }
      assert(narrow.done());
    }
  }
  for (double relative : {0., -0.1, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid = config();
    invalid.first_test_axis = 1;
    invalid.max_relative_angle_rad = relative;
    bool rejected = false;
    try { Sequence sequence(invalid); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
  }
}

}

int main() {
  testCoupledMotion();
  const Sequence::TimePoint origin{};
  Sequence::Config slowConfig{{-0.2, 0.}, {0., 0.2}, 0.01, 500ms, 10000ms, 150ms};
  slowConfig.command_velocity_rad_s = {0.02, 0.02};
  Sequence slow(slowConfig);
  auto slowPosition = Sequence::Target{-0.2, 0.2};
  slow.start(Sequence::Mode::CLOSE, readback(origin, slowPosition), origin);
  for (int tick = 1; tick <= 1200 && !slow.done() && !slow.failed(); ++tick) {
    for (auto& position : slowPosition) {
      const double delta = std::clamp(-position, -0.0002, 0.0002);
      position += delta;
    }
    const auto now = origin + tick * 10ms;
    slow.update(now, readback(now, slowPosition));
  }
  assert(slow.done());
  slow.start(Sequence::Mode::CLOSE, readback(origin, {-0.2, 0.2}), origin);
  slow.update(origin + 11500ms, readback(origin + 11500ms, {-0.2, 0.2}));
  assert(slow.status() == Sequence::Status::TRACKING_TIMEOUT);
  auto physicalConfig = config();
  physicalConfig.first_test_axis = 1;
  Sequence physical(physicalConfig);
  physical.start(Sequence::Mode::TEST, readback(origin), origin);
  physical.update(origin + 20ms, readback(origin + 20ms));
  assert((physical.target() == Sequence::Target{0., 0.1}));
  physical.update(origin + 40ms, readback(origin + 40ms, physical.target()));
  physical.update(origin + 60ms, readback(origin + 60ms, physical.target()));
  assert((physical.target() == Sequence::Target{0., 0.2}));
  physicalConfig.first_test_axis = 2;
  bool badOrderRejected = false;
  try { Sequence badOrder(physicalConfig); }
  catch (const std::invalid_argument&) { badOrderRejected = true; }
  assert(badOrderRejected);
  Sequence sequence(config());
  assert(!sequence.commandValid());
  sequence.start(Sequence::Mode::CLOSE, readback(origin, {-0.3, 0.2}), origin);
  assert(sequence.target() == Sequence::Target{});
  sequence.update(origin + 10ms, readback(origin + 10ms));
  sequence.update(origin + 25ms, readback(origin + 25ms, {-0.01, 0.}));
  sequence.update(origin + 30ms, readback(origin + 30ms));
  sequence.update(origin + 49ms, readback(origin + 49ms));
  assert(!sequence.done());
  sequence.update(origin + 50ms, readback(origin + 50ms));
  assert(sequence.done());

  sequence.start(Sequence::Mode::GET_READY, readback(origin), origin);
  sequence.update(origin + 20ms, readback(origin + 20ms));
  assert(sequence.status() == Sequence::Status::HOLD);
  for (int tick = 1; tick <= 200; ++tick)
    sequence.update(origin + tick * 20ms, readback(origin + tick * 20ms));
  assert(sequence.status() == Sequence::Status::HOLD);
  sequence.update(origin + 4020ms, readback(origin + 4020ms, {-0.2, 0.}));
  sequence.update(origin + 5000ms, readback(origin + 5000ms, {-0.2, 0.}));
  assert(sequence.status() == Sequence::Status::TRACKING_TIMEOUT);
  assert(!sequence.commandValid());

  sequence.start(Sequence::Mode::TEST, readback(origin), origin);
  Sequence::TimePoint now = origin;
  std::set<std::pair<double, double>> visited;
  for (int step = 0; step < 200 && !sequence.done(); ++step) {
    const auto target = sequence.target();
    assert(target[0] >= -0.7 && target[0] <= 0.);
    assert(target[1] >= 0. && target[1] <= 0.45);
    visited.emplace(target[0], target[1]);
    now += 20ms;
    sequence.update(now, readback(now, target));
    assert(!sequence.failed());
  }
  assert(sequence.done() && sequence.target() == Sequence::Target{});
  for (const auto& target : visited) {
    assert(std::abs(target.first) <= 0.2 && std::abs(target.second) <= 0.2);
  }
  assert(visited.count({-0.1, 0.}) && visited.count({-0.2, 0.}));
  assert(visited.count({0., 0.1}) && visited.count({0., 0.2}));
  assert(visited.count({-0.2, 0.2}) && !visited.count({-0.7, 0.45}));

  sequence.start(Sequence::Mode::TEST, readback(origin), origin);
  now = origin + 20ms;
  sequence.update(now, readback(now));
  const std::array<Sequence::Target, 8> first_targets{{
      {-0.1, 0.}, {-0.2, 0.}, {0., 0.}, {0., 0.1},
      {0., 0.2}, {0., 0.}, {-0.2, 0.2}, {0., 0.}}};
  for (const auto& expected : first_targets) {
    assert(sequence.target() == expected);
    now += 20ms;
    sequence.update(now, readback(now, expected));
    now += 20ms;
    sequence.update(now, readback(now, expected));
  }

  Sequence bilateral({{-0.4, -0.2}, {0.3, 0.6}, 0.001, 20ms, 1000ms, 100ms});
  bilateral.start(Sequence::Mode::TEST, readback(origin), origin);
  now = origin;
  visited.clear();
  for (int step = 0; step < 200 && !bilateral.done(); ++step) {
    const auto target = bilateral.target();
    assert(target[0] >= -0.4 && target[0] <= 0.3);
    assert(target[1] >= -0.2 && target[1] <= 0.6);
    visited.emplace(target[0], target[1]);
    now += 20ms;
    bilateral.update(now, readback(now, target));
  }
  assert(bilateral.done());
  assert(visited.count({-0.2, 0.}) && !visited.count({0.3, 0.}));
  assert(visited.count({0., 0.2}) && !visited.count({0., 0.6}));

  Sequence narrow({{-0.1, 0.}, {0., 0.08}, 0.001, 20ms, 1000ms, 100ms});
  narrow.start(Sequence::Mode::TEST, readback(origin), origin);
  now = origin;
  for (int step = 0; step < 200 && !narrow.done(); ++step) {
    const auto target = narrow.target();
    assert(target[0] >= -0.1 && target[0] <= 0.);
    assert(target[1] >= 0. && target[1] <= 0.08);
    now += 20ms;
    narrow.update(now, readback(now, target));
  }
  assert(narrow.done());

  sequence.start(Sequence::Mode::TEST, readback(origin), origin);
  sequence.update(origin + 20ms, readback(origin + 20ms));
  assert(sequence.status() == Sequence::Status::TEST);
  sequence.update(origin + 1020ms, readback(origin + 1020ms));
  assert(sequence.status() == Sequence::Status::TRACKING_TIMEOUT);

  for (std::size_t axis = 0; axis < 2; ++axis) {
    sequence.start(Sequence::Mode::CLOSE, readback(origin), origin);
    auto state = readback(origin + 101ms);
    state.feedback_time[axis] = origin;
    sequence.update(origin + 101ms, state);
    assert(sequence.status() == Sequence::Status::FEEDBACK_LOST);
    sequence.update(origin + 102ms, readback(origin + 102ms));
    assert(sequence.failed() && !sequence.commandValid());

    state = readback(origin);
    state.drive_fault[axis] = true;
    sequence.start(Sequence::Mode::CLOSE, state, origin);
    assert(sequence.status() == Sequence::Status::DRIVE_FAULT);
    state = readback(origin);
    state.feedback_valid[axis] = false;
    sequence.start(Sequence::Mode::CLOSE, state, origin);
    assert(sequence.status() == Sequence::Status::FEEDBACK_LOST);
    state = readback(origin);
    state.feedback_time[axis] = origin + 1ms;
    sequence.start(Sequence::Mode::CLOSE, state, origin);
    assert(sequence.status() == Sequence::Status::FEEDBACK_LOST);
    state = readback(origin);
    state.position_rad[axis] = std::numeric_limits<double>::quiet_NaN();
    sequence.start(Sequence::Mode::CLOSE, state, origin);
    assert(sequence.failed());
  }
  sequence.start(Sequence::Mode::TEST, readback(origin), origin);
  sequence.cancel();
  sequence.update(origin + 20ms, readback(origin + 20ms));
  assert(sequence.status() == Sequence::Status::CANCELLED && !sequence.commandValid());
  sequence.reset();
  assert(sequence.status() == Sequence::Status::IDLE && !sequence.commandValid());
  sequence.start(Sequence::Mode::CLOSE, readback(origin), origin);
  sequence.update(origin - 1ms, readback(origin - 1ms));
  assert(sequence.status() == Sequence::Status::INVALID_TIME);

  auto invalid = config();
  invalid.lower_limit_rad[1] = 0.1;
  bool rejected = false;
  try { Sequence bad(invalid); } catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
}
