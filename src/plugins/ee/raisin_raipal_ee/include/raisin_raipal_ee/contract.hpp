#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace raisin::raipal_ee {

inline constexpr std::uint32_t kSchemaVersion = 1;
inline constexpr std::uint32_t kStateSchemaVersion = 3;

enum class Action { ENABLE, STOP, TARGET, SET_HOME, HEARTBEAT };
enum class Backend { DISABLED, MOCK, ETHERCAT };

struct Command {
  Action action{};
  std::string request_id;
  std::optional<std::uint64_t> sequence;
  std::optional<std::array<double, 2>> target_rad;
};

struct AxisState {
  std::string name;
  double position_rad{};
  double velocity_rad_s{};
  double torque_nm{};
  double target_rad{};
  double lower_limit_rad{};
  double upper_limit_rad{};
  std::uint16_t status_word{};
  std::uint16_t error_code{};
  std::int32_t raw_encoder_count{};
  bool raw_valid{};
  bool calibrated_valid{};
  bool position_valid{};
};

struct TactileState {
  std::string name;
  bool valid{};
  bool baseline_ready{};
  double age_ms{-1.};
  std::uint64_t sequence{};
};

struct State {
  std::uint64_t sequence{};
  std::uint64_t last_command_sequence{};
  Backend backend{Backend::DISABLED};
  bool enabled{};
  bool homed{};
  bool tactile_ready{};
  bool actuation_qualified{};
  std::string error;
  std::vector<std::string> gate_reasons;
  std::array<AxisState, 2> axes{{AxisState{"left"}, AxisState{"right"}}};
  std::array<TactileState, 2> tactile{{TactileState{"left"}, TactileState{"right"}}};
};

namespace detail {

inline std::string actionName(Action action) {
  switch (action) {
    case Action::ENABLE: return "enable";
    case Action::STOP: return "stop";
    case Action::TARGET: return "target";
    case Action::SET_HOME: return "set_home";
    case Action::HEARTBEAT: return "heartbeat";
  }
  throw std::invalid_argument("unknown EE action");
}

inline Action parseAction(const std::string& value) {
  if (value == "enable") return Action::ENABLE;
  if (value == "stop") return Action::STOP;
  if (value == "target") return Action::TARGET;
  if (value == "set_home") return Action::SET_HOME;
  if (value == "heartbeat") return Action::HEARTBEAT;
  throw std::invalid_argument("unknown EE action: " + value);
}

inline void requireOnlyKeys(const YAML::Node& node, const std::vector<std::string>& allowed) {
  if (!node.IsMap()) throw std::invalid_argument("EE YAML value must be a mapping");
  std::set<std::string> seen;
  for (const auto& entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!seen.insert(key).second)
      throw std::invalid_argument("duplicate EE YAML field: " + key);
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end())
      throw std::invalid_argument("unknown EE YAML field: " + key);
  }
}

inline std::array<double, 2> parseTarget(const YAML::Node& node) {
  if (!node.IsSequence() || node.size() != 2)
    throw std::invalid_argument("target_rad must contain exactly two values");
  std::array<double, 2> target{node[0].as<double>(), node[1].as<double>()};
  if (!std::isfinite(target[0]) || !std::isfinite(target[1]))
    throw std::invalid_argument("target_rad must be finite");
  return target;
}

inline Command parseCommand(const std::string& yaml, bool internal) {
  try {
    const auto node = YAML::Load(yaml);
    requireOnlyKeys(node, internal
        ? std::vector<std::string>{"schema_version", "sequence", "action", "target_rad"}
        : std::vector<std::string>{"schema_version", "request_id", "action", "target_rad"});
    if (!node["schema_version"] || node["schema_version"].as<std::uint32_t>() != kSchemaVersion)
      throw std::invalid_argument("unsupported EE command schema_version");
    if (!node["action"] || !node["action"].IsScalar())
      throw std::invalid_argument("EE command action is required");

    Command command;
    command.action = parseAction(node["action"].as<std::string>());
    if (internal) {
      if (!node["sequence"])
        throw std::invalid_argument("internal EE command sequence is required");
      command.sequence = node["sequence"].as<std::uint64_t>();
    } else if (node["request_id"]) {
      command.request_id = node["request_id"].as<std::string>();
      if (command.request_id.size() > 128)
        throw std::invalid_argument("request_id is too long");
    }

    if (command.action == Action::TARGET) {
      if (!node["target_rad"])
        throw std::invalid_argument("target_rad is required for target action");
      command.target_rad = parseTarget(node["target_rad"]);
    } else if (node["target_rad"]) {
      throw std::invalid_argument("target_rad is only valid for target action");
    }
    return command;
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("invalid EE YAML: ") + e.what());
  }
}

}  // namespace detail

inline Command parsePublicCommand(const std::string& yaml) {
  return detail::parseCommand(yaml, false);
}

inline Command parseInternalCommand(const std::string& yaml) {
  return detail::parseCommand(yaml, true);
}

inline std::string encodePublicCommand(const Command& command) {
  YAML::Node node;
  node["schema_version"] = kSchemaVersion;
  node["action"] = detail::actionName(command.action);
  if (!command.request_id.empty()) node["request_id"] = command.request_id;
  if (command.target_rad) {
    YAML::Node target(YAML::NodeType::Sequence);
    target.push_back((*command.target_rad)[0]);
    target.push_back((*command.target_rad)[1]);
    node["target_rad"] = target;
  }
  return YAML::Dump(node);
}

inline std::string encodeInternalCommand(const Command& command, std::uint64_t sequence) {
  YAML::Node node;
  node["schema_version"] = kSchemaVersion;
  node["sequence"] = sequence;
  node["action"] = detail::actionName(command.action);
  if (command.target_rad) {
    YAML::Node target(YAML::NodeType::Sequence);
    target.push_back((*command.target_rad)[0]);
    target.push_back((*command.target_rad)[1]);
    node["target_rad"] = target;
  }
  return YAML::Dump(node);
}

inline std::string toString(Backend backend) {
  switch (backend) {
    case Backend::DISABLED: return "disabled";
    case Backend::MOCK: return "mock";
    case Backend::ETHERCAT: return "ethercat";
  }
  throw std::invalid_argument("unknown EE backend");
}

inline Backend parseBackend(const std::string& value) {
  if (value == "disabled") return Backend::DISABLED;
  if (value == "mock") return Backend::MOCK;
  if (value == "ethercat") return Backend::ETHERCAT;
  throw std::invalid_argument("unknown EE backend: " + value);
}

inline std::string encodeState(const State& state) {
  YAML::Node node;
  node["schema_version"] = kStateSchemaVersion;
  node["sequence"] = state.sequence;
  node["last_command_sequence"] = state.last_command_sequence;
  node["backend"] = toString(state.backend);
  node["enabled"] = state.enabled;
  node["homed"] = state.homed;
  node["tactile_ready"] = state.tactile_ready;
  node["actuation_qualified"] = state.actuation_qualified;
  node["error"] = state.error;
  node["gate_reasons"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto& reason : state.gate_reasons) node["gate_reasons"].push_back(reason);
  for (const auto& axis : state.axes) {
    YAML::Node item;
    item["name"] = axis.name;
    item["position_rad"] = axis.position_valid || axis.calibrated_valid ? YAML::Node(axis.position_rad) : YAML::Node(YAML::NodeType::Null);
    item["velocity_rad_s"] = axis.calibrated_valid ? YAML::Node(axis.velocity_rad_s) : YAML::Node(YAML::NodeType::Null);
    item["torque_nm"] = axis.calibrated_valid ? YAML::Node(axis.torque_nm) : YAML::Node(YAML::NodeType::Null);
    item["target_rad"] = axis.calibrated_valid ? YAML::Node(axis.target_rad) : YAML::Node(YAML::NodeType::Null);
    item["lower_limit_rad"] = axis.calibrated_valid ? YAML::Node(axis.lower_limit_rad) : YAML::Node(YAML::NodeType::Null);
    item["upper_limit_rad"] = axis.calibrated_valid ? YAML::Node(axis.upper_limit_rad) : YAML::Node(YAML::NodeType::Null);
    item["status_word"] = axis.status_word;
    item["error_code"] = axis.error_code;
    item["raw_encoder_count"] = axis.raw_valid ? YAML::Node(axis.raw_encoder_count) : YAML::Node(YAML::NodeType::Null);
    item["raw_valid"] = axis.raw_valid;
    item["calibrated_valid"] = axis.calibrated_valid;
    item["position_valid"] = axis.position_valid;
    node["axes"].push_back(item);
  }
  for (const auto& tactile : state.tactile) {
    YAML::Node item;
    item["name"] = tactile.name;
    item["valid"] = tactile.valid;
    item["baseline_ready"] = tactile.baseline_ready;
    item["age_ms"] = tactile.age_ms;
    item["sequence"] = tactile.sequence;
    node["tactile"].push_back(item);
  }
  return YAML::Dump(node);
}

inline State parseState(const std::string& yaml) {
  try {
    const auto node = YAML::Load(yaml);
    if (!node.IsMap()) throw std::invalid_argument("EE state must be a YAML mapping");
    if (!node["schema_version"])
      throw std::invalid_argument("EE state schema_version is required");
    const auto schema = node["schema_version"].as<std::uint32_t>();
    if (schema != 1 && schema != 2 && schema != kStateSchemaVersion)
      throw std::invalid_argument("unsupported EE state schema_version");
    const bool version2 = schema >= 2;
    detail::requireOnlyKeys(node, version2
        ? std::vector<std::string>{"schema_version", "sequence", "last_command_sequence",
            "backend", "enabled", "homed", "tactile_ready", "actuation_qualified", "error",
            "gate_reasons", "axes", "tactile"}
        : std::vector<std::string>{"schema_version", "sequence", "last_command_sequence",
            "backend", "enabled", "homed", "tactile_ready", "error", "gate_reasons",
            "axes", "tactile"});
    State state;
    state.sequence = node["sequence"].as<std::uint64_t>();
    state.last_command_sequence = node["last_command_sequence"].as<std::uint64_t>();
    state.backend = parseBackend(node["backend"].as<std::string>());
    state.enabled = node["enabled"].as<bool>();
    state.homed = node["homed"].as<bool>();
    state.tactile_ready = node["tactile_ready"].as<bool>();
    state.actuation_qualified = version2 && node["actuation_qualified"].as<bool>();
    state.error = node["error"].as<std::string>();
    if (!node["gate_reasons"].IsSequence())
      throw std::invalid_argument("gate_reasons must be a sequence");
    for (const auto& item : node["gate_reasons"]) state.gate_reasons.push_back(item.as<std::string>());
    if (!node["axes"].IsSequence() || node["axes"].size() != state.axes.size() ||
        !node["tactile"].IsSequence() || node["tactile"].size() != state.tactile.size())
      throw std::invalid_argument("EE state must contain two axes and two tactile samples");
    for (std::size_t i = 0; i < state.axes.size(); ++i) {
      const auto item = node["axes"][i];
      auto axisKeys = version2
          ? std::vector<std::string>{"name", "position_rad", "velocity_rad_s", "torque_nm",
              "target_rad", "lower_limit_rad", "upper_limit_rad", "status_word", "error_code",
              "raw_encoder_count", "raw_valid", "calibrated_valid"}
          : std::vector<std::string>{"name", "position_rad", "velocity_rad_s", "torque_nm",
              "target_rad", "lower_limit_rad", "upper_limit_rad", "status_word", "error_code"};
      if (schema >= 3) axisKeys.push_back("position_valid");
      detail::requireOnlyKeys(item, axisKeys);
      auto& axis = state.axes[i];
      axis.name = item["name"].as<std::string>();
      axis.status_word = item["status_word"].as<std::uint16_t>();
      axis.error_code = item["error_code"].as<std::uint16_t>();
      if (version2) {
        axis.raw_valid = item["raw_valid"].as<bool>();
        axis.calibrated_valid = item["calibrated_valid"].as<bool>();
        axis.position_valid = schema >= 3 && item["position_valid"].as<bool>();
        if (axis.raw_valid) axis.raw_encoder_count = item["raw_encoder_count"].as<std::int32_t>();
        else if (!item["raw_encoder_count"].IsNull())
          throw std::invalid_argument("raw_encoder_count must be null when raw_valid is false");
        if (axis.position_valid || axis.calibrated_valid)
          axis.position_rad = item["position_rad"].as<double>();
        else if (!item["position_rad"].IsNull())
          throw std::invalid_argument("position_rad must be null when position feedback is invalid");
        if (axis.calibrated_valid) {
          axis.velocity_rad_s = item["velocity_rad_s"].as<double>();
          axis.torque_nm = item["torque_nm"].as<double>();
          axis.target_rad = item["target_rad"].as<double>();
          axis.lower_limit_rad = item["lower_limit_rad"].as<double>();
          axis.upper_limit_rad = item["upper_limit_rad"].as<double>();
        } else {
          for (const auto* key : {"velocity_rad_s", "torque_nm", "target_rad",
                                  "lower_limit_rad", "upper_limit_rad"}) {
            if (!item[key].IsNull())
              throw std::invalid_argument(std::string(key) + " must be null when calibrated_valid is false");
          }
        }
      } else {
        // v1 carried numeric placeholders but did not establish that they were calibrated.
        axis.position_rad = item["position_rad"].as<double>();
        axis.velocity_rad_s = item["velocity_rad_s"].as<double>();
        axis.torque_nm = item["torque_nm"].as<double>();
        axis.target_rad = item["target_rad"].as<double>();
        axis.lower_limit_rad = item["lower_limit_rad"].as<double>();
        axis.upper_limit_rad = item["upper_limit_rad"].as<double>();
      }
      if (axis.name != (i == 0 ? "left" : "right") ||
          !std::isfinite(axis.position_rad) || !std::isfinite(axis.velocity_rad_s) ||
          !std::isfinite(axis.torque_nm) || !std::isfinite(axis.target_rad) ||
          !std::isfinite(axis.lower_limit_rad) || !std::isfinite(axis.upper_limit_rad) ||
          axis.lower_limit_rad > axis.upper_limit_rad)
        throw std::invalid_argument("invalid EE axis state");
    }
    for (std::size_t i = 0; i < state.tactile.size(); ++i) {
      const auto item = node["tactile"][i];
      detail::requireOnlyKeys(item, {"name", "valid", "baseline_ready", "age_ms", "sequence"});
      auto& tactile = state.tactile[i];
      tactile.name = item["name"].as<std::string>();
      tactile.valid = item["valid"].as<bool>();
      tactile.baseline_ready = item["baseline_ready"].as<bool>();
      tactile.age_ms = item["age_ms"].as<double>();
      tactile.sequence = item["sequence"].as<std::uint64_t>();
      if (tactile.name != (i == 0 ? "left" : "right") ||
          !std::isfinite(tactile.age_ms) || tactile.age_ms < -1.)
        throw std::invalid_argument("invalid EE tactile state");
    }
    return state;
  } catch (const YAML::Exception& e) {
    throw std::invalid_argument(std::string("invalid EE state YAML: ") + e.what());
  }
}

}  // namespace raisin::raipal_ee
