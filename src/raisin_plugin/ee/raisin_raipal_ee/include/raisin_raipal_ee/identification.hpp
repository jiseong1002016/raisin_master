#pragma once

#include "raisin_raipal_ee/contract.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace raisin::raipal_ee {

struct IdentificationAxis {
  double kp{};
  double kd{};
};

inline std::array<IdentificationAxis, 2> readIdentificationAxes(const YAML::Node& node) {
  if (!node.IsSequence() || node.size() != 2)
    throw std::invalid_argument("identification requires exactly two axes in left/right order");
  std::array<IdentificationAxis, 2> result;
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto item = node[index];
    detail::requireOnlyKeys(item, {"kp", "kd"});
    auto& axis = result[index];
    axis = {item["kp"].as<double>(), item["kd"].as<double>()};
    for (double value : {axis.kp, axis.kd}) {
      if (!std::isfinite(value)) throw std::invalid_argument("nonfinite identification parameter");
    }
    if (axis.kp < 0. || axis.kd < 0.)
      throw std::invalid_argument("identification gains must be nonnegative");
  }
  return result;
}

struct IdentificationRow {
  std::uint64_t monotonic_ns{};
  std::size_t axis{};
  double position{}, velocity{}, torque{}, target{}, slewed_target{};
  std::int32_t raw_position{}, raw_velocity{};
  std::int16_t raw_torque{};
  std::uint8_t status{}, error{};
  std::int32_t command_position{}, command_velocity{};
  std::int16_t command_torque{};
  std::uint16_t p_gain{}, d_gain{};
  std::uint8_t control{}, mode{};
};

inline YAML::Node identificationRow(const IdentificationRow& row) {
  YAML::Node node;
  node["monotonic_ns"] = row.monotonic_ns;
  node["axis"] = row.axis;
  node["position_rad"] = row.position;
  node["velocity_rad_s"] = row.velocity;
  node["torque_estimate_nm"] = row.torque;
  node["target_rad"] = row.target;
  node["slewed_target_rad"] = row.slewed_target;
  node["raw_position"] = row.raw_position;
  node["raw_velocity"] = row.raw_velocity;
  node["raw_torque"] = row.raw_torque;
  node["status_word"] = static_cast<unsigned>(row.status);
  node["error_code"] = static_cast<unsigned>(row.error);
  node["command_position"] = row.command_position;
  node["command_velocity"] = row.command_velocity;
  node["command_torque"] = row.command_torque;
  node["p_gain_raw"] = row.p_gain;
  node["d_gain_raw"] = row.d_gain;
  node["control_word"] = static_cast<unsigned>(row.control);
  node["mode"] = static_cast<unsigned>(row.mode);
  return node;
}

}  // namespace raisin::raipal_ee
