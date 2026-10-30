#pragma once

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace raisin::raipal_ee {

struct HardwareConfiguration {
  std::uint32_t schema_version{1};
  std::string hardware_profile;
  std::string selection_source;
  std::size_t arm_dof{};
  std::size_t expected_slaves{};
  std::vector<std::string> modules;
  std::vector<std::string> gui_arm_modules;
  std::string raisim_config;
  bool ee_present{};
  bool ee_communication{};
  bool ee_actuation_qualified{};
  std::string ee_mount_mode;
  std::string ee_visual_resource;
  std::string ee_mount_resource;
  std::string initial_motor_state;
};

inline HardwareConfiguration parseHardwareConfiguration(const std::string& yaml) {
  const auto node = YAML::Load(yaml);
  if (!node.IsMap()) throw std::invalid_argument("hardware configuration must be a YAML mapping");
  std::set<std::string> seen;
  const std::set<std::string> allowed{
      "schema_version", "hardware_profile", "selection_source", "arm_dof",
      "expected_slaves", "modules", "gui_arm_modules", "raisim_config", "ee_present",
      "ee_communication", "ee_actuation_qualified", "ee_mount_mode",
      "ee_visual_resource", "ee_mount_resource", "initial_motor_state"};
  for (const auto& item : node) {
    const auto key = item.first.as<std::string>();
    if (!seen.insert(key).second) throw std::invalid_argument("duplicate hardware configuration field: " + key);
    if (!allowed.contains(key)) throw std::invalid_argument("unknown hardware configuration field: " + key);
  }

  HardwareConfiguration config;
  config.schema_version = node["schema_version"].as<std::uint32_t>();
  if (config.schema_version != 1) throw std::invalid_argument("unsupported hardware configuration schema_version");
  config.hardware_profile = node["hardware_profile"].as<std::string>();
  config.selection_source = node["selection_source"].as<std::string>();
  config.arm_dof = node["arm_dof"].as<std::size_t>();
  config.expected_slaves = node["expected_slaves"].as<std::size_t>();
  config.modules = node["modules"].as<std::vector<std::string>>();
  config.gui_arm_modules = node["gui_arm_modules"].as<std::vector<std::string>>();
  config.raisim_config = node["raisim_config"].as<std::string>();
  config.ee_present = node["ee_present"].as<bool>();
  config.ee_communication = node["ee_communication"].as<bool>();
  config.ee_actuation_qualified = node["ee_actuation_qualified"].as<bool>();
  config.ee_mount_mode = node["ee_mount_mode"].as<std::string>();
  config.ee_visual_resource = node["ee_visual_resource"].as<std::string>();
  config.ee_mount_resource = node["ee_mount_resource"].as<std::string>();
  config.initial_motor_state = node["initial_motor_state"].as<std::string>();

  if ((config.selection_source != "default" && config.selection_source != "yaml" &&
       config.selection_source != "cli") ||
      config.raisim_config.empty() || config.initial_motor_state != "COMM_DISABLED")
    throw std::invalid_argument("incomplete hardware configuration snapshot");
  const std::vector<std::string> armModules{
      "raipal_R", "stub-0_R", "raipal_L", "tool-empty_L"};
  const bool singleLeft = config.hardware_profile == "single_L_ee" ||
                          config.hardware_profile == "single_L_ee_nonreal";
  const bool nonreal = config.hardware_profile == "single_L_ee_nonreal";
  const bool leftVisual =
      config.ee_visual_resource == "resource/raisin_raipal_ee/resource/ee_visual.urdf" ||
      config.ee_visual_resource == "resource/raisin_raipal_ee/resource/ee_visual_d405.urdf";
  if (config.arm_dof != (singleLeft ? 7u : 14u))
    throw std::invalid_argument("arm DOF contradicts hardware profile");
  if (config.hardware_profile == "dual_arm") {
    if (config.expected_slaves != 16 || config.ee_present || config.ee_communication ||
        config.ee_actuation_qualified || config.ee_mount_mode != "none" ||
        !config.ee_visual_resource.empty() || !config.ee_mount_resource.empty() ||
        config.modules != armModules || config.gui_arm_modules != armModules ||
        config.raisim_config != "/raisim_config/flat_terrain.xml")
      throw std::invalid_argument("dual_arm snapshot contradicts its fixed hardware profile");
  } else if (config.hardware_profile == "dual_arm_right_ee") {
    const std::vector<std::string> rightEeModules{
        "raipal_R", "raipal-ee-visual_R", "raipal_L", "tool-empty_L"};
    if (config.expected_slaves != 18 || !config.ee_present || !config.ee_communication ||
        config.ee_mount_mode != "single_R" ||
        config.ee_visual_resource != "resource/raisin_raipal_ee/resource/ee_visual.urdf" ||
        config.ee_mount_resource != "resource/raisin_raipal_ee/resource/ee_mounts.yaml" ||
        config.modules != rightEeModules || config.gui_arm_modules != armModules ||
        config.raisim_config != "/raisim_config/flat_terrain_ee_visual_R.xml")
      throw std::invalid_argument("dual_arm_right_ee snapshot contradicts its fixed hardware profile");
  } else if (config.hardware_profile == "dual_arm_left_ee") {
    const std::vector<std::string> leftEeModules{
        "raipal_R", "stub-0_R", "raipal_L", "raipal-ee-visual_L"};
    if (config.expected_slaves != 18 || !config.ee_present || !config.ee_communication ||
        config.ee_mount_mode != "single_L" ||
        !leftVisual ||
        config.ee_mount_resource != "resource/raisin_raipal_ee/resource/ee_mounts.yaml" ||
        config.modules != leftEeModules || config.gui_arm_modules != armModules ||
        config.raisim_config != "/raisim_config/flat_terrain_ee_visual_L.xml")
      throw std::invalid_argument("dual_arm_left_ee snapshot contradicts its fixed hardware profile");
  } else if (singleLeft) {
    const std::vector<std::string> singleLeftModules{"raipal_L", "raipal-ee-visual_L"};
    const std::vector<std::string> singleLeftGuiModules{"raipal_L", "tool-empty_L"};
    if (config.expected_slaves != 11 || !config.ee_present ||
        config.ee_communication != !nonreal ||
        (nonreal && !config.ee_actuation_qualified) || config.ee_mount_mode != "single_L" ||
        !leftVisual ||
        config.ee_mount_resource != "resource/raisin_raipal_ee/resource/ee_mounts.yaml" ||
        config.modules != singleLeftModules || config.gui_arm_modules != singleLeftGuiModules ||
        config.raisim_config != "/raisim_config/flat_terrain_single_L_ee.xml")
      throw std::invalid_argument("single left EE snapshot contradicts its fixed hardware profile");
  } else {
    throw std::invalid_argument("unknown hardware profile: " + config.hardware_profile);
  }
  if (config.modules.empty() || config.gui_arm_modules.empty())
    throw std::invalid_argument("hardware configuration modules cannot be empty");
  if (config.ee_actuation_qualified && (!config.ee_present || (!config.ee_communication && !nonreal)))
    throw std::invalid_argument("EE actuation cannot be qualified without EE communications");
  if (config.ee_present) {
    if ((!config.ee_communication && !nonreal) ||
        (config.ee_mount_mode != "single_R" && config.ee_mount_mode != "single_L") ||
        config.ee_visual_resource.empty() || config.ee_mount_resource.empty())
      throw std::invalid_argument("EE hardware configuration is incomplete");
  } else if (config.ee_communication || config.ee_actuation_qualified ||
             config.ee_mount_mode != "none" || !config.ee_visual_resource.empty() ||
             !config.ee_mount_resource.empty()) {
    throw std::invalid_argument("EE-absent profile contains EE settings");
  }
  return config;
}

inline std::string encodeHardwareConfiguration(const HardwareConfiguration& config) {
  YAML::Node node;
  node["schema_version"] = config.schema_version;
  node["hardware_profile"] = config.hardware_profile;
  node["selection_source"] = config.selection_source;
  node["arm_dof"] = config.arm_dof;
  node["expected_slaves"] = config.expected_slaves;
  node["modules"] = config.modules;
  node["gui_arm_modules"] = config.gui_arm_modules;
  node["raisim_config"] = config.raisim_config;
  node["ee_present"] = config.ee_present;
  node["ee_communication"] = config.ee_communication;
  node["ee_actuation_qualified"] = config.ee_actuation_qualified;
  node["ee_mount_mode"] = config.ee_mount_mode;
  node["ee_visual_resource"] = config.ee_visual_resource;
  node["ee_mount_resource"] = config.ee_mount_resource;
  node["initial_motor_state"] = config.initial_motor_state;
  return YAML::Dump(node);
}

}  // namespace raisin::raipal_ee
