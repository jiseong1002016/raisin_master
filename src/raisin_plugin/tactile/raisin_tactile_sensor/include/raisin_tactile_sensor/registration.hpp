#pragma once

#include "receiver.hpp"
#include "receiver_v2.hpp"
#include <map>

namespace raisin::tactile {

inline Layout readLayout(const parameter::ParameterContainer& params) {
  Layout layout{params.as_string("frame"), params.as_int("rows"), params.as_int("cols"),
                params.as_double("size_u_m"), params.as_double("size_v_m"),
                params.as_bool("flip_rows"), params.as_bool("flip_cols")};
  layout.validate();
  return layout;
}

// Called by core before Raipal::createCommunication starts the existing sensor
// lifecycle. Config comes from the plugin package once, not a second geometry copy.
inline void registerReceivers(
    std::map<std::string, std::shared_ptr<robot::sensor::Sensor>>& sensors,
    raisim::World& hub, raisim::RaisimServer& server,
    parameter::ParameterContainer& config, const std::shared_ptr<Network>& network) {
  if (!config.as_bool("enabled"))
    throw std::invalid_argument("tactile bridge requires enabled tactile plugin config");
  if (config.as_string("backend") == "usb") {
    const auto prefix = config["usb"].as_string("topic_prefix");
    std::map<std::string, std::shared_ptr<robot::sensor::Sensor>> added;
    for (const auto& name : {std::string("left"), std::string("right")}) {
      const auto key = "tactile/" + name;
      if (sensors.contains(key)) throw std::invalid_argument("duplicate tactile sensor: " + key);
      added.emplace(key, std::make_shared<ReceiverV2>(hub, server, key, config, network,
                                                     name, prefix + "/" + name));
    }
    sensors.merge(added);
    return;
  }
  const auto prefix = config.as_string("topic_prefix");
  std::map<std::string, std::shared_ptr<robot::sensor::Sensor>> added;
  for (const auto& [name, params] : config["sensors"].getParamContainerList()) {
    const auto key = "tactile/" + name;
    if (sensors.contains(key)) throw std::invalid_argument("duplicate tactile sensor: " + key);
    added.emplace(key, std::make_shared<Receiver>(hub, server, key, *params, network,
                                                readLayout(*params), prefix + "/" + name));
  }
  if (added.empty()) throw std::invalid_argument("no tactile receivers configured");
  sensors.merge(added);
}

}  // namespace raisin::tactile
