#include "raisin_raipal_ee/contract.hpp"

#include <yaml-cpp/yaml.h>

#include <cassert>
#include <stdexcept>
#include <string>

using raisin::raipal_ee::Action;
using raisin::raipal_ee::Backend;
using raisin::raipal_ee::Command;
using raisin::raipal_ee::State;

namespace {

constexpr char kValidState[] = R"yaml(
schema_version: 1
sequence: 12
last_command_sequence: 7
backend: mock
enabled: false
homed: false
tactile_ready: false
error: ""
gate_reasons: [left_tactile_missing, right_tactile_missing]
axes:
  - {name: left, position_rad: 0, velocity_rad_s: 0, torque_nm: 0, target_rad: 0,
     lower_limit_rad: -1, upper_limit_rad: 1, status_word: 0, error_code: 0}
  - {name: right, position_rad: 0, velocity_rad_s: 0, torque_nm: 0, target_rad: 0,
     lower_limit_rad: -1, upper_limit_rad: 1, status_word: 0, error_code: 0}
tactile:
  - {name: left, valid: false, baseline_ready: false, age_ms: -1, sequence: 0}
  - {name: right, valid: false, baseline_ready: false, age_ms: -1, sequence: 0}
)yaml";

template <typename F>
void expectInvalid(F&& fn) {
  bool rejected = false;
  try {
    fn();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  assert(rejected);
}

}  // namespace

int main() {
  Command publicTarget;
  publicTarget.action = Action::TARGET;
  publicTarget.request_id = "gui-42";
  publicTarget.target_rad = std::array<double, 2>{0.1, -0.1};
  const auto decodedPublic = raisin::raipal_ee::parsePublicCommand(
      raisin::raipal_ee::encodePublicCommand(publicTarget));
  assert(decodedPublic.action == Action::TARGET);
  assert(decodedPublic.request_id == "gui-42");
  assert(decodedPublic.target_rad && (*decodedPublic.target_rad)[1] == -0.1);
  assert(!decodedPublic.sequence);

  const auto decodedInternal = raisin::raipal_ee::parseInternalCommand(
      raisin::raipal_ee::encodeInternalCommand(publicTarget, 19));
  assert(decodedInternal.sequence && *decodedInternal.sequence == 19);
  assert(decodedInternal.action == Action::TARGET);

  expectInvalid([] {
    raisin::raipal_ee::parsePublicCommand(
        "schema_version: 1\nschema_version: 1\naction: stop\n");
  });
  expectInvalid([] {
    raisin::raipal_ee::parseInternalCommand(
        "schema_version: 1\nsequence: 1\naction: target\ntarget_rad: [.nan, 0]\n");
  });

  const auto parsedState = raisin::raipal_ee::parseState(kValidState);
  assert(parsedState.backend == Backend::MOCK);
  assert(parsedState.sequence == 12);
  assert(parsedState.last_command_sequence == 7);
  assert(parsedState.axes[0].name == "left");
  assert(parsedState.tactile[0].age_ms == -1.0);
  assert(raisin::raipal_ee::parseState(raisin::raipal_ee::encodeState(parsedState)).axes[1].name == "right");

  State emptyState;
  emptyState.axes[0] = {"left", 0, 0, 0, 0, -1, 1, 0, 0};
  emptyState.axes[1] = {"right", 0, 0, 0, 0, -1, 1, 0, 0};
  emptyState.tactile[0].name = "left";
  emptyState.tactile[1].name = "right";
  const auto emptyStateYaml = raisin::raipal_ee::encodeState(emptyState);
  const auto emptyStateNode = YAML::Load(emptyStateYaml);
  assert(emptyStateNode["gate_reasons"].IsSequence());
  assert(emptyStateNode["gate_reasons"].size() == 0);
  const auto emptyStateRoundTrip = raisin::raipal_ee::parseState(emptyStateYaml);
  assert(emptyStateRoundTrip.gate_reasons.empty());

  State rawState;
  rawState.backend = Backend::ETHERCAT;
  rawState.axes[0].raw_encoder_count = -123456;
  rawState.axes[0].raw_valid = true;
  rawState.axes[0].calibrated_valid = false;
  const auto rawYaml = raisin::raipal_ee::encodeState(rawState);
  const auto rawNode = YAML::Load(rawYaml);
  assert(rawNode["schema_version"].as<std::uint32_t>() ==
         raisin::raipal_ee::kStateSchemaVersion);
  assert(rawNode["axes"][0]["raw_encoder_count"].as<std::int32_t>() == -123456);
  assert(rawNode["axes"][0]["position_rad"].IsNull());
  assert(rawNode["axes"][0]["torque_nm"].IsNull());
  const auto rawRoundTrip = raisin::raipal_ee::parseState(rawYaml);
  assert(rawRoundTrip.axes[0].raw_valid &&
         rawRoundTrip.axes[0].raw_encoder_count == -123456 &&
         !rawRoundTrip.axes[0].calibrated_valid && rawRoundTrip.axes[0].position_rad == 0.);
  assert(!rawRoundTrip.actuation_qualified);

  auto version2 = YAML::Clone(rawNode);
  version2["schema_version"] = 2;
  for (auto axis : version2["axes"]) axis.remove("position_valid");
  const auto legacyRaw = raisin::raipal_ee::parseState(YAML::Dump(version2));
  assert(legacyRaw.axes[0].raw_valid && !legacyRaw.axes[0].position_valid);
  assert(!parsedState.axes[0].position_valid && !parsedState.axes[0].calibrated_valid);
  for (auto axis : version2["axes"]) {
    axis["calibrated_valid"] = true;
    for (const auto* key : {"position_rad", "velocity_rad_s", "torque_nm", "target_rad",
                            "lower_limit_rad", "upper_limit_rad"}) axis[key] = 0.;
  }
  const auto legacyCalibrated = raisin::raipal_ee::parseState(YAML::Dump(version2));
  assert(legacyCalibrated.axes[0].calibrated_valid && !legacyCalibrated.axes[0].position_valid);
  assert(raisin::raipal_ee::parseState(raisin::raipal_ee::encodeState(legacyCalibrated))
             .axes[0].calibrated_valid);

  rawState.axes[0].position_valid = true;
  rawState.axes[0].position_rad = -0.25;
  const auto positionNode = YAML::Load(raisin::raipal_ee::encodeState(rawState));
  assert(positionNode["schema_version"].as<int>() == 3);
  for (const auto* key : {"velocity_rad_s", "torque_nm", "target_rad",
                          "lower_limit_rad", "upper_limit_rad"})
    assert(positionNode["axes"][0][key].IsNull());
  const auto positionState = raisin::raipal_ee::parseState(YAML::Dump(positionNode));
  assert(positionState.axes[0].position_valid && positionState.axes[0].position_rad == -0.25);
  assert(!positionState.axes[0].calibrated_valid && !positionState.actuation_qualified &&
         !positionState.homed && !positionState.enabled);
  auto invalidPosition = YAML::Clone(positionNode);
  invalidPosition["axes"][0]["position_rad"] = ".nan";
  expectInvalid([&] { raisin::raipal_ee::parseState(YAML::Dump(invalidPosition)); });
  invalidPosition = YAML::Clone(positionNode);
  invalidPosition["axes"][0]["torque_nm"] = 1.;
  expectInvalid([&] { raisin::raipal_ee::parseState(YAML::Dump(invalidPosition)); });
  invalidPosition = YAML::Clone(positionNode);
  invalidPosition["axes"][0]["position_valid"] = false;
  expectInvalid([&] { raisin::raipal_ee::parseState(YAML::Dump(invalidPosition)); });

  std::string nanAxis = kValidState;
  const auto position = nanAxis.find("position_rad: 0");
  assert(position != std::string::npos);
  nanAxis.replace(position, std::string("position_rad: 0").size(), "position_rad: .nan");
  expectInvalid([&] { raisin::raipal_ee::parseState(nanAxis); });

  std::string infAxis = kValidState;
  const auto velocity = infAxis.find("velocity_rad_s: 0");
  assert(velocity != std::string::npos);
  infAxis.replace(velocity, std::string("velocity_rad_s: 0").size(), "velocity_rad_s: .inf");
  expectInvalid([&] { raisin::raipal_ee::parseState(infAxis); });

  std::string reversedLimits = kValidState;
  const auto lower = reversedLimits.find("lower_limit_rad: -1");
  assert(lower != std::string::npos);
  reversedLimits.replace(lower, std::string("lower_limit_rad: -1").size(), "lower_limit_rad: 2");
  expectInvalid([&] { raisin::raipal_ee::parseState(reversedLimits); });
}
