#include "raisin_raipal_ee/mechanics.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Fn>
void requireThrows(Fn&& fn, const char* message) {
  bool rejected = false;
  try {
    fn();
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, message);
}

raisin::raipal_ee::HardwareConfiguration validEeSnapshot() {
  using namespace raisin::raipal_ee;
  HardwareConfiguration config;
  config.hardware_profile = "dual_arm_right_ee";
  config.selection_source = "yaml";
  config.arm_dof = 14;
  config.expected_slaves = 18;
  config.modules = {"raipal_R", "raipal-ee-visual_R", "raipal_L", "tool-empty_L"};
  config.gui_arm_modules = {"raipal_R", "stub-0_R", "raipal_L", "tool-empty_L"};
  config.raisim_config = "/raisim_config/flat_terrain_ee_visual_R.xml";
  config.ee_present = true;
  config.ee_communication = true;
  config.ee_actuation_qualified = false;
  config.ee_mount_mode = "single_R";
  config.ee_visual_resource = "resource/raisin_raipal_ee/resource/ee_visual.urdf";
  config.ee_mount_resource = "resource/raisin_raipal_ee/resource/ee_mounts.yaml";
  config.initial_motor_state = "COMM_DISABLED";
  return config;
}

raisin::raipal_ee::HardwareConfiguration validArmOnlySnapshot() {
  using namespace raisin::raipal_ee;
  HardwareConfiguration config;
  config.hardware_profile = "dual_arm";
  config.selection_source = "cli";
  config.arm_dof = 14;
  config.expected_slaves = 16;
  config.modules = {"raipal_R", "stub-0_R", "raipal_L", "tool-empty_L"};
  config.gui_arm_modules = config.modules;
  config.raisim_config = "/raisim_config/flat_terrain.xml";
  config.ee_present = false;
  config.ee_communication = false;
  config.ee_actuation_qualified = false;
  config.ee_mount_mode = "none";
  config.initial_motor_state = "COMM_DISABLED";
  return config;
}

void testPreviewMapping() {
  const auto preview = raisin::raipal_ee::mapDriveToPreview({0.2, -0.3});
  const std::array<double, 8> expected{0.2, -0.3, 0.08, -0.08, 0.2, -0.3, -0.12, 0.12};
  for (std::size_t i = 0; i < preview.size(); ++i)
    require(std::abs(preview[i] - expected[i]) < 1e-12,
            "canonical mimic coordinate mismatch");

  const auto neutral = raisin::raipal_ee::mapDriveToPreview({0.0, 0.0});
  for (const double value : neutral)
    require(value == 0.0, "uncalibrated neutral EE preview must not invent joint motion");

  requireThrows([&] {
    (void)raisin::raipal_ee::mapDriveToPreview(
        {std::numeric_limits<double>::quiet_NaN(), 0.0});
  }, "nonfinite drive position must be rejected");
}

void testSnapshotMountAuthority() {
  using namespace raisin::raipal_ee;
  const auto ee = parseHardwareConfiguration(encodeHardwareConfiguration(validEeSnapshot()));
  const auto arm = parseHardwareConfiguration(encodeHardwareConfiguration(validArmOnlySnapshot()));

  require(resolveVisualMountMode(ee, "single_L") == "single_R",
          "resolved EE snapshot must override a stale package mount_mode");
  require(resolveVisualMountMode(std::nullopt, "single_L") == "single_L",
          "legacy standalone package configuration should retain its mount mode");
  requireThrows([&] {
    (void)resolveVisualMountMode(arm, "single_L");
  }, "manual plugin load must reject a profile without an EE module");
}

void testQualifiedFreshVisualGate() {
  using namespace raisin::raipal_ee;
  State state;
  require(!hasQualifiedVisualFeedback(state),
          "unqualified raw state must not drive the source Jacobian");

  state.actuation_qualified = true;
  state.homed = true;
  require(!hasQualifiedVisualFeedback(state),
          "homing and profile flags alone must not qualify missing axis calibration");

  for (auto& axis : state.axes) {
    axis.calibrated_valid = true;
    axis.position_rad = 0.0;
  }
  require(hasQualifiedVisualFeedback(state),
          "fully calibrated, homed, finite feedback should be preview eligible");

  state.homed = false;
  state.actuation_qualified = false;
  for (auto& axis : state.axes) {
    axis.calibrated_valid = false;
    axis.position_valid = true;
  }
  require(hasQualifiedVisualFeedback(state),
          "position-only calibration must update visualization while actuation is disabled");

  state.axes[1].position_rad = std::numeric_limits<double>::infinity();
  require(!hasQualifiedVisualFeedback(state),
          "nonfinite calibrated feedback must not drive the source Jacobian");

  const auto start = std::chrono::steady_clock::time_point{} + std::chrono::seconds(10);
  require(visualStateIsFresh(start, start + std::chrono::milliseconds(499),
                             std::chrono::milliseconds(500)),
          "recently accepted preview state should remain fresh");
  require(!visualStateIsFresh(start, start + std::chrono::milliseconds(501),
                              std::chrono::milliseconds(500)),
          "stale preview state should freeze instead of updating");
  require(!visualStateIsFresh({}, start, std::chrono::milliseconds(500)),
          "missing preview state is not fresh");
}

void testFloatingAndFixedCoordinateIndices() {
  using raisin::raipal_ee::generalizedCoordinateIndex;
  require(generalizedCoordinateIndex(6, 15, 14) == 7,
          "floating 6+8-DOF EE needs qdim 15 and a +1 joint-coordinate offset");
  require(generalizedCoordinateIndex(13, 15, 14) == 14,
          "last floating EE joint index must stay within qdim 15");
  require(generalizedCoordinateIndex(7, 8, 8) == 7,
          "fixed-base visuals must not add a free-root coordinate offset");
  requireThrows([&] { (void)generalizedCoordinateIndex(6, 16, 14); },
                "unsupported generalized coordinate layout must be rejected");
  requireThrows([&] { (void)generalizedCoordinateIndex(14, 15, 14); },
                "out-of-range generalized velocity index must be rejected");
}

}  // namespace

int main() {
  try {
    testPreviewMapping();
    testSnapshotMountAuthority();
    testQualifiedFreshVisualGate();
    testFloatingAndFixedCoordinateIndices();
    std::cout << "PASS: EE preview mechanics, profile-authoritative mounting, "
                 "qualified/fresh feedback gating, and fixed/floating q-coordinate layouts\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
