#pragma once

#include "raisin_raipal_ee/contract.hpp"
#include "raisin_raipal_ee/hardware_configuration.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace raisin::raipal_ee {

inline constexpr std::array<std::string_view, 2> kDriveJointNames{
    "ee_01_left", "ee_02_right"};

inline constexpr std::array<std::string_view, 8> kPreviewJointNames{
    "ee_01_left", "ee_02_right", "ee_13_left", "ee_34_left",
    "ee_02_left", "ee_01_right", "ee_13_right", "ee_34_right"};

inline constexpr std::array<std::array<double, 2>, 8> kPreviewJacobian{{
    {{1., 0.}}, {{0., 1.}}, {{0.4, 0.}}, {{-0.4, 0.}},
    {{1., 0.}}, {{0., 1.}}, {{0., 0.4}}, {{0., -0.4}}}};

inline std::array<double, 8> mapDriveToPreview(const std::array<double, 2>& drive_rad) {
  if (!std::isfinite(drive_rad[0]) || !std::isfinite(drive_rad[1]))
    throw std::invalid_argument("EE preview drive positions must be finite");

  std::array<double, 8> joint_rad{};
  for (std::size_t row = 0; row < joint_rad.size(); ++row) {
    joint_rad[row] = kPreviewJacobian[row][0] * drive_rad[0] +
                     kPreviewJacobian[row][1] * drive_rad[1];
  }
  return joint_rad;
}

inline std::array<double, 8> mapLeftEthercatDriveToPreview(
    const std::array<double, 2>& drive_rad) {
  if (!std::isfinite(drive_rad[0]) || !std::isfinite(drive_rad[1]))
    throw std::invalid_argument("EE preview drive positions must be finite");
  const double joint8 = drive_rad[1];
  const double joint9 = drive_rad[0];
  return {joint8, joint9, -joint8 - joint9, joint8 + joint9,
          -joint9, -joint8, joint8 + joint9, -joint8 - joint9};
}

inline std::array<double, 8> mapDriveToPreviewForMount(
    const std::array<double, 2>& drive_rad, std::string_view mountMode) {
  if (mountMode == "single_L") return mapLeftEthercatDriveToPreview(drive_rad);
  if (mountMode == "single_R") return mapDriveToPreview(drive_rad);
  throw std::invalid_argument("EE preview requires a single_L or single_R mount mode");
}

inline std::string resolveVisualMountMode(
    const std::optional<HardwareConfiguration>& snapshot,
    const std::string& packageFallback) {
  if (!snapshot) return packageFallback;
  if (!snapshot->ee_present)
    throw std::invalid_argument("the selected RAIPAL hardware profile has no EE module");
  return snapshot->ee_mount_mode;
}

inline bool hasQualifiedVisualFeedback(const State& state) {
  return std::all_of(state.axes.begin(), state.axes.end(), [&state](const AxisState& axis) {
    return (axis.position_valid || (state.homed && axis.calibrated_valid)) &&
           std::isfinite(axis.position_rad);
  });
}

inline bool visualStateIsFresh(std::chrono::steady_clock::time_point received,
                               std::chrono::steady_clock::time_point now,
                               std::chrono::milliseconds timeout) {
  return received != std::chrono::steady_clock::time_point{} &&
         now >= received && now - received <= timeout;
}

inline std::size_t generalizedCoordinateIndex(std::size_t velocityIndex,
                                               std::size_t coordinateSize,
                                               std::size_t dof) {
  if (coordinateSize != dof && coordinateSize != dof + 1)
    throw std::runtime_error("unexpected generalized coordinate layout");
  if (velocityIndex >= dof)
    throw std::runtime_error("generalized velocity index is out of range");
  const auto coordinateIndex = velocityIndex + (coordinateSize - dof);
  if (coordinateIndex >= coordinateSize)
    throw std::runtime_error("generalized coordinate index is out of range");
  return coordinateIndex;
}

}  // namespace raisin::raipal_ee
