#include "ee_drive_bridge.hpp"
#include "ee_drive_pdo.hpp"
#include "ethercat_scan.hpp"
#include "ethercat_start_sequence.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "raisin_ethercat/raisin_ethercat_interface.hpp"

namespace {

template <typename Tag, typename Tag::Type Member>
struct BusFixtureAccess {
  friend typename Tag::Type fixtureMember(Tag) { return Member; }
};

struct BusContext {
  using Type = ecx_contextt raisin::ethercat::EthercatInterface::*;
  friend Type fixtureMember(BusContext);
};
struct BusDevices {
  using Type = std::vector<std::shared_ptr<raisin::ethercat::EthercatDevice>>
      raisin::ethercat::EthercatInterface::*;
  friend Type fixtureMember(BusDevices);
};
struct BusExpectedWkc {
  using Type = int raisin::ethercat::EthercatInterface::*;
  friend Type fixtureMember(BusExpectedWkc);
};
struct BusRealWkc {
  using Type = int raisin::ethercat::EthercatInterface::*;
  friend Type fixtureMember(BusRealWkc);
};
template struct BusFixtureAccess<BusContext, &raisin::ethercat::EthercatInterface::context_>;
template struct BusFixtureAccess<BusDevices, &raisin::ethercat::EthercatInterface::devices_>;
template struct BusFixtureAccess<BusExpectedWkc, &raisin::ethercat::EthercatInterface::expectedWKC_>;
template struct BusFixtureAccess<BusRealWkc, &raisin::ethercat::EthercatInterface::realWKC_>;

using raisin::raipal_ee::Action;
using raisin::raipal_ee::Backend;
using raisin::raipal_ee::Command;
using raisin::robot::EeAxisConfig;
using raisin::robot::EeDriveBridge;
using raisin::robot::EeDriveConfig;
using raisin::robot::EePdoProfile;
using raisin::robot::ee_pdo::RxPdo;
using raisin::robot::ee_pdo::TxPdo;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Callable>
void requireThrows(Callable&& callable, const std::string& contains,
                   const std::string& scenario) {
  try {
    callable();
  } catch (const std::exception& error) {
    if (error.what() == nullptr || std::string(error.what()).find(contains) == std::string::npos)
      throw std::runtime_error(scenario + ": wrong exception: " + error.what());
    return;
  }
  throw std::runtime_error(scenario + ": expected an exception");
}

struct TemporaryDirectory {
  std::filesystem::path path = std::filesystem::temp_directory_path() /
      ("ee_core_pdo_test_" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  TemporaryDirectory() { std::filesystem::create_directories(path); }
  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

EeAxisConfig axisConfig(const std::string& name) {
  EeAxisConfig axis;
  axis.name = name;
  axis.lower_limit_rad = name == "left" ? -0.96 : 0.0;
  axis.upper_limit_rad = name == "left" ? 0.132 : 0.96;
  axis.home_position_rad = 0.0;
  axis.max_velocity_rad_s = 1.0;
  axis.motor_peak_torque_nm = 1.0;
  axis.kp_nm_per_rad = 2.0;
  axis.kd_nm_s_per_rad = 0.1;
  return axis;
}

EeDriveConfig mockConfig() {
  EeDriveConfig config;
  config.backend = Backend::MOCK;
  config.enabled = true;
  config.actuation_qualified = true;
  config.axes = {axisConfig("left"), axisConfig("right")};
  config.command_timeout = std::chrono::milliseconds(500);
  config.tactile_stale_timeout = std::chrono::milliseconds(300);
  config.tactile_max_skew = std::chrono::milliseconds(50);
  config.mock_velocity_rad_s = 1.0;
  return config;
}

EeDriveConfig qualifiedEthercatConfig(const std::string& homing_file = {}) {
  auto config = mockConfig();
  config.backend = Backend::ETHERCAT;
  config.communication_enabled = true;
  config.actuation_qualified = true;
  config.homing_file = homing_file;
  config.axes[0].slave_id = 9;
  config.axes[1].slave_id = 10;
  config.axes[0].vendor_id = 0x0e01;
  config.axes[1].vendor_id = 0x0e01;
  config.axes[0].product_code = 0x0e50;
  config.axes[1].product_code = 0x0e50;
  config.axes[0].revision = 7;
  config.axes[1].revision = 7;
  config.axes[0].firmware_version = 9001;
  config.axes[1].firmware_version = 9001;
  config.axes[0].pdo_profile = EePdoProfile::RX16_TX13;
  config.axes[1].pdo_profile = EePdoProfile::RX16_TX13;
  config.axes[0].encoder_counts_per_motor_revolution = 12345;
  config.axes[1].encoder_counts_per_motor_revolution = 12345;
  config.axes[0].encoder_cpr_sdo_index = 0x8011;
  config.axes[1].encoder_cpr_sdo_index = 0x8011;
  for (auto& axis : config.axes) {
    axis.kp_nm_per_rad = 100.;
    axis.kd_nm_s_per_rad = 5.;
  }
  config.axes[0].gear_ratio = 68.0;
  config.axes[1].gear_ratio = 68.0;
  config.axes[0].direction = 1;
  config.axes[1].direction = -1;
  config.axes[0].motor_rated_torque_nm = 1.0;
  config.axes[1].motor_rated_torque_nm = 1.0;
  config.axes[0].gear_efficiency = 0.8;
  config.axes[1].gear_efficiency = 0.8;
  return config;
}

EeDriveConfig observedRaipalEeConfig(const std::string& homing_file) {
  auto config = qualifiedEthercatConfig(homing_file);
  config.axes[0].slave_id = 10;
  config.axes[1].slave_id = 11;
  for (auto& axis : config.axes) {
    axis.vendor_id = 0x4150;
    axis.product_code = 0x9245;
    axis.revision = 1;
    axis.firmware_version = 142;
    axis.pdo_profile = EePdoProfile::RX16_TX13;
    axis.encoder_cpr_sdo_index = 0x8012;
    axis.encoder_counts_per_motor_revolution = 65536;
  }
  return config;
}

YAML::Node eeParameterNode(const EeDriveConfig& config, bool includeCprSdoIndex = true) {
  YAML::Node root;
  root["ee"]["enabled"] = config.enabled;
  root["ee"]["communication_enabled"] = config.communication_enabled;
  root["ee"]["actuation_qualified"] = config.actuation_qualified;
  root["ee"]["backend"] = raisin::raipal_ee::toString(config.backend);
  root["ee"]["homing_file"] = config.homing_file;
  root["ee"]["persistent_multiturn_home"] = config.persistent_multiturn_home;
  for (const auto& axis : config.axes) {
    auto node = root["ee"]["axes"][axis.name];
    node["slave_id"] = static_cast<int>(axis.slave_id);
    node["vendor_id"] = static_cast<int>(axis.vendor_id);
    node["product_code"] = static_cast<int>(axis.product_code);
    node["revision"] = static_cast<int>(axis.revision);
    node["firmware_version"] = axis.firmware_version;
    node["pdo_profile"] = axis.pdo_profile == EePdoProfile::RX12_TX13
        ? "rx12_tx13" : "rx16_tx13";
    node["encoder_counts_per_motor_revolution"] =
        axis.encoder_counts_per_motor_revolution;
    if (includeCprSdoIndex)
      node["encoder_cpr_sdo_index"] = static_cast<int>(axis.encoder_cpr_sdo_index);
    node["gear_ratio"] = axis.gear_ratio;
    node["direction"] = axis.direction;
    node["motor_rated_torque_nm"] = axis.motor_rated_torque_nm;
    node["gear_efficiency"] = axis.gear_efficiency;
    node["lower_limit_rad"] = axis.lower_limit_rad;
    node["upper_limit_rad"] = axis.upper_limit_rad;
    node["home_position_rad"] = axis.home_position_rad;
    node["max_velocity_rad_s"] = axis.max_velocity_rad_s;
    if (axis.command_velocity_rad_s)
      node["command_velocity_rad_s"] = *axis.command_velocity_rad_s;
    node["motor_peak_torque_nm"] = axis.motor_peak_torque_nm;
    node["kp_nm_per_rad"] = axis.kp_nm_per_rad;
    node["kd_nm_s_per_rad"] = axis.kd_nm_s_per_rad;
  }
  return root;
}

EeDriveConfig parseEeConfig(const EeDriveConfig& config, bool includeCprSdoIndex = true) {
  raisin::parameter::ParameterContainer parameters;
  auto node = eeParameterNode(config, includeCprSdoIndex);
  parameters.yamlIterator(node);
  return raisin::robot::readEeDriveConfig(parameters);
}

std::uint64_t stampNs(EeDriveBridge::TimePoint time) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count());
}

raisin::tactile::FrameV2 tactileFrame(const std::string& side, std::uint64_t sequence,
                                     std::uint64_t stamp_ns, bool baseline = true,
                                     bool valid = true, std::uint64_t session = 1) {
  raisin::tactile::FrameV2 frame;
  frame.sensor_name = side;
  frame.frame_id = "tactile_" + side + "_frame";
  frame.clock_domain = "CLOCK_MONOTONIC";
  frame.calibration_id = "test-fixture";
  frame.calibration_status = "provisional";
  frame.baseline_status = baseline ? "operator_confirmed_unloaded"
                                   : "awaiting_unloaded_baseline";
  frame.force_unit = "N_estimate";
  frame.stamp_monotonic_ns = stamp_ns;
  frame.session_id = session;
  frame.sequence = sequence;
  frame.rows = raisin::tactile::kUsbRows;
  frame.cols = raisin::tactile::kUsbCols;
  frame.code_to_force_n_per_code = 1.f;
  frame.polarity = 1;
  frame.baseline_valid = baseline;
  frame.raw_code.assign(raisin::tactile::kUsbCells, 0);
  frame.baseline_code.assign(raisin::tactile::kUsbCells, 0);
  frame.normal_force_n.assign(raisin::tactile::kUsbCells, 0.f);
  frame.valid.assign(raisin::tactile::kUsbCells, 0);
  if (baseline && valid) frame.valid[0] = 1;
  return frame;
}

void ingestPair(EeDriveBridge& bridge, std::uint64_t left_sequence,
                std::uint64_t right_sequence, std::uint64_t left_stamp,
                std::uint64_t right_stamp, bool baseline = true, bool valid = true) {
  bridge.ingestTactileFrame(
      tactileFrame("left", left_sequence, left_stamp, baseline, valid));
  bridge.ingestTactileFrame(
      tactileFrame("right", right_sequence, right_stamp, baseline, valid));
}

Command command(Action action, std::uint64_t sequence) {
  Command result;
  result.action = action;
  result.sequence = sequence;
  return result;
}

void testSafeOpBeforeValidationAndOperationalTransition() {
  std::vector<std::uint16_t> stateRequests;
  std::vector<std::string> sequence;
  const auto result = raisin::robot::startSafeValidateOperational(
      [&stateRequests, &sequence](std::uint16_t target) {
        stateRequests.push_back(target);
        sequence.emplace_back(target == EC_STATE_SAFE_OP ? "safe_op" : "operational");
        return true;
      },
      [&sequence] { sequence.emplace_back("validate_sdo_pdo"); });
  require(result == raisin::robot::EthercatStartupResult::OPERATIONAL &&
              stateRequests == std::vector<std::uint16_t>{EC_STATE_SAFE_OP, EC_STATE_OPERATIONAL} &&
              sequence == std::vector<std::string>{"safe_op", "validate_sdo_pdo", "operational"},
          "the retained EtherCAT instance must enter SAFE_OP before SDO/PDO checks and OP");

  sequence.clear();
  const auto failedSafeOp = raisin::robot::startSafeValidateOperational(
      [&sequence](std::uint16_t) { sequence.emplace_back("safe_op_failed"); return false; },
      [&sequence] { sequence.emplace_back("validate_must_not_run"); });
  require(failedSafeOp == raisin::robot::EthercatStartupResult::SAFE_OP_FAILED &&
              sequence == std::vector<std::string>{"safe_op_failed"},
          "validation must not run when the same instance cannot enter SAFE_OP");

  sequence.clear();
  const auto failedOperational = raisin::robot::startSafeValidateOperational(
      [&sequence](std::uint16_t target) {
        sequence.emplace_back(target == EC_STATE_SAFE_OP ? "safe_op" : "operational_failed");
        return target == EC_STATE_SAFE_OP;
      },
      [&sequence] { sequence.emplace_back("validate_sdo_pdo"); });
  require(failedOperational == raisin::robot::EthercatStartupResult::OPERATIONAL_FAILED &&
              sequence == std::vector<std::string>{"safe_op", "validate_sdo_pdo", "operational_failed"},
          "OP transition must follow successful SAFE_OP validation on the same interface");
}

void testPdoLayouts() {
  RxPdo output;
  output.target_position = -0x01020304;
  output.target_velocity = 0x1020304;
  output.target_torque = -0x1234;
  output.control_word = 0x0f;
  output.mode = 10;
  output.p_gain = 0x1234;
  output.d_gain = 0xabcd;
  std::array<std::uint8_t, raisin::robot::ee_pdo::kRx16Bytes> bytes{};

  const auto rx12_size = raisin::robot::ee_pdo::encodeRx(EePdoProfile::RX12_TX13, output, bytes);
  require(rx12_size == 12, "ESI sw_1_22 host RxPDO must be 12 bytes");
  const std::array<std::uint8_t, 12> expected_rx12{
      0xfc, 0xfc, 0xfd, 0xfe, 0x04, 0x03, 0x02, 0x01, 0xcc, 0xed, 0x0f, 0x0a};
  require(std::equal(expected_rx12.begin(), expected_rx12.end(), bytes.begin()),
          "RxPDO 12-byte field order/endianness differs from the ESI layout");

  const auto rx16_size = raisin::robot::ee_pdo::encodeRx(EePdoProfile::RX16_TX13, output, bytes);
  require(rx16_size == 16, "ESI sw_1_23 host RxPDO must be 16 bytes");
  const std::array<std::uint8_t, 16> expected_rx16{
      0xfc, 0xfc, 0xfd, 0xfe, 0x04, 0x03, 0x02, 0x01,
      0xcc, 0xed, 0x0f, 0x0a, 0x34, 0x12, 0xcd, 0xab};
  require(std::equal(expected_rx16.begin(), expected_rx16.end(), bytes.begin()),
          "RxPDO 16-byte field order/endianness differs from the ESI layout");
  require(raisin::robot::ee_pdo::matchesProfile(EePdoProfile::RX12_TX13, 96, 104) &&
              raisin::robot::ee_pdo::matchesProfile(EePdoProfile::RX16_TX13, 128, 104) &&
              !raisin::robot::ee_pdo::matchesProfile(EePdoProfile::RX12_TX13, 128, 104),
          "ESI Rx/Tx bit counts must select exactly one supported profile");

  const std::array<std::uint8_t, 13> tx_bytes{
      0xfc, 0xfd, 0xfe, 0xff, 0x34, 0x12, 0x04, 0x03, 0x02, 0x01, 0x27, 0x5a, 0x09};
  const TxPdo input = raisin::robot::ee_pdo::decodeTx(tx_bytes.data(), tx_bytes.size());
  require(input.position == -0x00010204 && input.torque == 0x1234 &&
              input.velocity == 0x01020304 && input.status_word == 0x27 &&
              input.temperature == 0x5a && input.error_code == 0x09,
          "13-byte TxPDO field offsets/endianness differ from the ESI layout");
  requireThrows([&] { (void)raisin::robot::ee_pdo::decodeTx(tx_bytes.data(), 12); },
                "exactly 13 bytes", "short TxPDO");
}

void testInvalidConfigurations() {
  {
    auto invalid = qualifiedEthercatConfig("fixture-home.yaml");
    invalid.axes[0].kd_nm_s_per_rad = 0.2;
    requireThrows([&] { EeDriveBridge bridge(invalid); }, "rounding to zero",
                  "positive damping must not silently disappear at gear ratio 68");
    invalid.axes[0].kd_nm_s_per_rad = 5.;
    invalid.axes[0].pdo_profile = EePdoProfile::RX12_TX13;
    requireThrows([&] { EeDriveBridge bridge(invalid); }, "rx16_tx13",
                  "firmware impedance needs gain fields in RxPDO");
  }
  for (const double speed : {0., -0.1, 1.1,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
    auto invalid = mockConfig();
    invalid.axes[0].command_velocity_rad_s = speed;
    requireThrows([&] { EeDriveBridge bridge(invalid); }, "invalid EE axis limits",
                  "invalid command slew speed");
  }
  auto config = mockConfig();
  config.axes[0].max_velocity_rad_s = std::numeric_limits<double>::quiet_NaN();
  requireThrows([&] { EeDriveBridge bridge(config); }, "invalid EE axis limits",
                "non-finite mock velocity limit");

  config = mockConfig();
  config.axes[0].lower_limit_rad = config.axes[0].upper_limit_rad;
  requireThrows([&] { EeDriveBridge bridge(config); }, "invalid EE axis limits",
                "reversed/equal mock limits");

  config = qualifiedEthercatConfig();
  config.axes[1].slave_id = config.axes[0].slave_id;
  requireThrows([&] { EeDriveBridge bridge(config); }, "unique one-based slave IDs",
                "duplicate EtherCAT slave IDs");

  config = observedRaipalEeConfig("fixture-home.yaml");
  config.axes[0].encoder_cpr_sdo_index = 0x18012;
  requireThrows([&] { EeDriveBridge bridge(config); }, "encoder_cpr_sdo_index",
                "oversized CPR SDO index must not truncate to 0x8012");
}

void testCprSdoAddressConfigurationAndQualification() {
  auto legacy = qualifiedEthercatConfig("legacy-home.yaml");
  const auto parsedLegacy = parseEeConfig(legacy);
  require(parsedLegacy.axes[0].encoder_cpr_sdo_index == 0x8011 &&
              parsedLegacy.axes[1].encoder_cpr_sdo_index == 0x8011,
          "legacy per-axis CPR SDO index 0x8011 must parse explicitly");

  auto actual = observedRaipalEeConfig("observed-home.yaml");
  const auto parsedActual = parseEeConfig(actual);
  require(parsedActual.axes[0].encoder_cpr_sdo_index == 0x8012 &&
              parsedActual.axes[1].encoder_cpr_sdo_index == 0x8012 &&
              parsedActual.axes[0].encoder_counts_per_motor_revolution == 65536 &&
              parsedActual.axes[1].encoder_counts_per_motor_revolution == 65536,
          "observed firmware 142 profile must parse CPR from SDO 0x8012 as 65536");

  requireThrows([&] { (void)parseEeConfig(actual, false); }, "encoder_cpr_sdo_index",
                "enabled real axis requires an explicit CPR SDO index");
  actual.axes[0].encoder_cpr_sdo_index = 0x8010;
  requireThrows([&] { (void)parseEeConfig(actual); }, "encoder_cpr_sdo_index",
                "unsupported CPR SDO index must be rejected by the config parser");

  const auto makeReader = [](const EeAxisConfig& axis, std::int32_t legacyCpr,
                             std::int32_t currentCpr,
                             std::vector<std::uint16_t>& requested) {
    const int expectedSlave = static_cast<int>(axis.slave_id);
    const auto firmware = axis.firmware_version;
    return [&requested, expectedSlave, firmware, legacyCpr, currentCpr](
               int slave, std::uint16_t index, std::uint8_t subindex,
               void* buffer, int& size) {
      if (slave != expectedSlave || subindex != 0 || size < static_cast<int>(sizeof(std::int32_t)))
        return false;
      requested.push_back(index);
      std::int32_t value{};
      if (index == 0x8007) value = firmware;
      else if (index == 0x8011) value = legacyCpr;
      else if (index == 0x8012) value = currentCpr;
      else return false;
      std::memcpy(buffer, &value, sizeof(value));
      size = sizeof(value);
      return true;
    };
  };

  for (const auto& axis : parsedActual.axes) {
    std::vector<std::uint16_t> requested;
    raisin::robot::ee_drive_bridge_detail::validateDriveSdoIdentity(
        axis, makeReader(axis, 1500, 65536, requested));
    require(requested == std::vector<std::uint16_t>{0x8007, 0x8012},
            "actual profile qualification must read CPR from 0x8012, not 0x8011");
  }

  std::vector<std::uint16_t> legacyRequested;
  raisin::robot::ee_drive_bridge_detail::validateDriveSdoIdentity(
      parsedLegacy.axes[0], makeReader(parsedLegacy.axes[0], 12345, 65536, legacyRequested));
  require(legacyRequested == std::vector<std::uint16_t>{0x8007, 0x8011},
          "legacy profile qualification must read CPR from explicit 0x8011");

  auto wrongValue = parsedActual.axes[0];
  std::vector<std::uint16_t> wrongValueReads;
  requireThrows([&] {
    raisin::robot::ee_drive_bridge_detail::validateDriveSdoIdentity(
        wrongValue, makeReader(wrongValue, 1500, 1500, wrongValueReads));
  }, "SDO 0x8012=1500", "actual CPR value mismatch must reject qualification");

  auto wrongIndex = parsedActual.axes[0];
  wrongIndex.encoder_cpr_sdo_index = 0x8011;
  std::vector<std::uint16_t> wrongIndexReads;
  requireThrows([&] {
    raisin::robot::ee_drive_bridge_detail::validateDriveSdoIdentity(
        wrongIndex, makeReader(wrongIndex, 1500, 65536, wrongIndexReads));
  }, "SDO 0x8011=1500", "mismatched CPR address/value pair must reject qualification");
}

std::vector<raisin::ethercat::SlaveDescription> scanRecords(
    const EeDriveConfig& config) {
  std::vector<raisin::ethercat::SlaveDescription> records(11);
  records[0].name = "dummy";
  records[1].name = "hub";
  for (std::size_t index = 2; index <= 8; ++index)
    records[index].name = "arm_axis_" + std::to_string(index - 1);
  for (std::size_t i = 0; i < config.axes.size(); ++i) {
    const auto& axis = config.axes[i];
    auto& record = records[axis.slave_id];
    record.name = "raidrive_app_1_30";
    record.eep_man = axis.vendor_id;
    record.eep_id = axis.product_code;
    record.eep_rev = axis.revision;
    record.Obits = axis.pdo_profile == EePdoProfile::RX12_TX13 ? 96 : 128;
    record.Ibits = 104;
  }
  return records;
}

void testScanTopologyValidation(const TemporaryDirectory& temporary) {
  const auto config = qualifiedEthercatConfig(
      (temporary.path / "unused-scan-home.yaml").string());
  {
    EeDriveBridge bridge(config);
    raisin::ethercat::EthercatInterface bus({});
    auto records = scanRecords(config);  // dummy 0, hub 1, seven arm axes 2..8, EE 9..10
    bridge.attachEthercat(bus, records); // Validation and registration only; no bus start.
  }
  {
    EeDriveBridge bridge(config);
    raisin::ethercat::EthercatInterface bus({});
    auto records = scanRecords(config);
    records.resize(10);
    requireThrows([&] { bridge.attachEthercat(bus, records); },
                  "outside scan", "missing configured EE scan records");
  }
  {
    EeDriveBridge bridge(config);
    raisin::ethercat::EthercatInterface bus({});
    auto records = scanRecords(config);
    records[9].eep_man += 1;
    requireThrows([&] { bridge.attachEthercat(bus, records); }, "identity/PDO mismatch",
                  "wrong EE EEPROM identity");
  }
  {
    EeDriveBridge bridge(config);
    raisin::ethercat::EthercatInterface bus({});
    auto records = scanRecords(config);
    records[9].Obits = 96;
    requireThrows([&] { bridge.attachEthercat(bus, records); }, "identity/PDO mismatch",
                  "wrong EE RxPDO profile");
  }
}

std::vector<raisin::ethercat::SlaveDescription> observedRaipalScan() {
  // Hardware-observed 2026-10-03 SOEM enumeration associated with source
  // commit 39e643881fc7b538581bd73dc62b90ce5628aee0. This records topology only;
  // it is not evidence of physical motion.
  std::vector<raisin::ethercat::SlaveDescription> records(19);
  records[0].name = "dummy";
  records[1].name = "GX-JC06(IN,X2,X3) Main device";
  records[1].eep_man = 0x83;
  records[1].eep_id = 0x64;
  records[2].name = "GX-JC06(X4,X5,X6) Sub-device";
  records[2].eep_man = 0x83;
  records[2].eep_id = 0x65;
  for (std::size_t address = 3; address <= 9; ++address) {
    records[address].name = "dongdrive_app";
    records[address].eep_man = 0x4150;
    records[address].eep_id = 0x9255;
    records[address].eep_rev = 1;
    records[address].Obits = 128;
    records[address].Ibits = 112;
  }
  for (std::size_t address = 12; address <= 18; ++address) {
    records[address].name = "dongdrive_app";
    records[address].eep_man = 0x4150;
    records[address].eep_id = 0x9255;
    records[address].eep_rev = 1;
    records[address].Obits = 128;
    records[address].Ibits = 112;
  }
  for (std::size_t address = 10; address <= 11; ++address) {
    records[address].name = "raidrive_app_1_30";
    records[address].eep_man = 0x4150;
    records[address].eep_id = 0x9245;
    records[address].eep_rev = 1;
    records[address].Obits = 128;
    records[address].Ibits = 104;
  }
  return records;
}

std::vector<std::size_t> armDriveAddresses(
    const raisin::robot::RaipalEthercatScanClassification& scan) {
  std::vector<std::size_t> addresses;
  for (const auto& device : scan.devices) {
    if (device.kind == raisin::robot::RaipalEthercatDeviceKind::ARM_DRIVE)
      addresses.push_back(device.physicalAddress);
  }
  return addresses;
}

std::string hardwareProfilesYaml() {
  return YAML::Dump(raisin::raipal::loadHardwareProfiles(RAISIN_HARDWARE_PROFILE_FILE));
}

void testObservedRaipalScanClassification(const TemporaryDirectory& temporary) {
  const auto yaml = hardwareProfilesYaml();
  const auto rightEeProfile = raisin::raipal::resolveHardwareProfile(
      yaml, "dual_arm_right_ee", "yaml");
  const auto armOnlyProfile = raisin::raipal::resolveHardwareProfile(
      yaml, "dual_arm", "default");
  const auto records = observedRaipalScan();
  const auto config = observedRaipalEeConfig(
      (temporary.path / "observed-topology-home.yaml").string());
  const auto scan = raisin::robot::classifyRaipalEthercatScan(records, rightEeProfile);
  const std::vector<std::size_t> expectedArmAddresses{
      3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18};
  require(armDriveAddresses(scan) == expectedArmAddresses,
          "recorded Dongdrive scan must preserve R7 then L7 at slaves 3..9 and 12..18");
  require(std::none_of(scan.devices.begin(), scan.devices.end(), [](const auto& device) {
            return device.kind == raisin::robot::RaipalEthercatDeviceKind::ARM_DRIVE &&
                   (device.physicalAddress == 10 || device.physicalAddress == 11);
          }), "EE slaves 10 and 11 must not enter the arm drive vector");
  require(scan.devices.size() == 16, "right-EE profile must classify 14 arms and two EE drives");
  const auto leftEeProfile = raisin::raipal::resolveHardwareProfile(
      yaml, "dual_arm_left_ee", "yaml");
  raisin::parameter::ParameterContainer installedParameters;
  auto installedOverrides = leftEeProfile.parameter_overrides;
  installedParameters.yamlIterator(installedOverrides);
  const auto installedConfig = raisin::robot::readEeDriveConfig(installedParameters);
  for (const auto& axis : installedConfig.axes) {
    const double capturedMotorTorque = raisin::robot::ee_pdo::motorTorque(
        145, axis.motor_rated_torque_nm);
    require(std::abs(capturedMotorTorque - 0.069368) < 1e-12 &&
                std::abs(capturedMotorTorque * axis.gear_ratio - 4.717024) < 1e-12 &&
                capturedMotorTorque < axis.motor_peak_torque_nm,
            "recorded 4.717024 output Nm must remain 0.069368 motor Nm for peak protection");
    require(raisin::robot::ee_pdo::impedanceGain(axis.kp_nm_per_rad, axis.gear_ratio) == 1 &&
                raisin::robot::ee_pdo::impedanceGain(axis.kd_nm_s_per_rad, axis.gear_ratio) == 1,
            "installed EE profile must serialize both impedance gains without losing damping");
    require(raisin::robot::ee_pdo::motorTorque(2048, axis.motor_rated_torque_nm) <
                axis.motor_peak_torque_nm &&
                raisin::robot::ee_pdo::motorTorque(2049, axis.motor_rated_torque_nm) >
                axis.motor_peak_torque_nm,
            "installed firmware normalization must use the motor peak boundary before gearing");
  }
  const auto singleLeftProfile = raisin::raipal::resolveHardwareProfile(
      yaml, "single_L_ee", "yaml");
  auto leftRecords = records;
  for (std::size_t address = 10; address <= 16; ++address)
    leftRecords[address] = records[address + 2];
  leftRecords[17] = records[10];
  leftRecords[18] = records[11];
  const auto leftScan = raisin::robot::classifyRaipalEthercatScan(leftRecords, leftEeProfile);
  std::vector<std::size_t> expectedLeftArmAddresses;
  for (std::size_t address = 3; address <= 16; ++address)
    expectedLeftArmAddresses.push_back(address);
  require(armDriveAddresses(leftScan) == expectedLeftArmAddresses && leftScan.devices.size() == 16,
          "2026-10-22 left-EE scan must map 14 arm drives at 3..16 and EE at 17..18");
  auto leftConfig = config;
  leftConfig.axes[0].slave_id = 17;
  leftConfig.axes[1].slave_id = 18;
  {
    EeDriveBridge bridge(leftConfig);
    raisin::ethercat::EthercatInterface bus({});
    bridge.attachEthercat(bus, leftRecords);
  }
  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(leftRecords, singleLeftProfile);
  }, "18 physical slaves; profile single_L_ee expects 11",
     "single-arm profile must report all 18 slaves instead of truncating the scan");
  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(leftRecords, rightEeProfile);
  }, "EE drive identity/PDO mismatch", "right-EE mapping must reject left-EE topology");
  {
    EeDriveBridge bridge(config);
    raisin::ethercat::EthercatInterface bus({});
    bridge.attachEthercat(bus, records);
  }

  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(records, armOnlyProfile);
  }, "expects 16", "EE-disabled profile must reject the observed 18-slave scan before connect");

  std::vector<raisin::ethercat::SlaveDescription> armOnlyRecords(17);
  armOnlyRecords[0] = records[0];
  for (std::size_t address = 1; address <= 9; ++address)
    armOnlyRecords[address] = records[address];
  for (std::size_t address = 10; address <= 16; ++address)
    armOnlyRecords[address] = records[address + 2];
  const auto armOnlyScan = raisin::robot::classifyRaipalEthercatScan(
      armOnlyRecords, armOnlyProfile);
  std::vector<std::size_t> expectedArmOnlyAddresses;
  for (std::size_t address = 3; address <= 16; ++address)
    expectedArmOnlyAddresses.push_back(address);
  require(armDriveAddresses(armOnlyScan) == expectedArmOnlyAddresses &&
              armOnlyScan.devices.size() == 14,
          "16-slave profile must map R7 then L7 across slaves 3..16 after EE removal");

  auto wrongEeIdentity = records;
  wrongEeIdentity[10].eep_id += 1;
  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(wrongEeIdentity, rightEeProfile);
  }, "EE drive identity/PDO mismatch", "configured EE EEPROM must be validated before connect");

  auto staleRaidriveArm = armOnlyRecords;
  staleRaidriveArm[5].name = "raidrive_app_1_30";
  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(staleRaidriveArm, armOnlyProfile);
  }, "arm drive identity/PDO mismatch", "unconfigured Raidrive must not enter Dongdrive arms");

  auto extraSlave = records;
  extraSlave.push_back(records.back());
  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(extraSlave, rightEeProfile);
  }, "expects 18", "unexpected additional driver must be rejected before connect");

  auto wrongHub = records;
  wrongHub[1].eep_id += 1;
  requireThrows([&] {
    (void)raisin::robot::classifyRaipalEthercatScan(wrongHub, rightEeProfile);
  }, "hub identity mismatch", "wrong hub EEPROM must be rejected before connect");
}

void testFeedbackFreshnessAndHeartbeat(const TemporaryDirectory& temporary) {
  using namespace std::chrono_literals;
  auto feedbackTime = EeDriveBridge::TimePoint{} + 5s;
  const auto timeout = 150ms;
  require(!raisin::robot::ee_pdo::feedbackIsFresh(feedbackTime, feedbackTime - 1ns, timeout),
          "an explicit query time before feedback must remain stale");
  raisin::raipal_ee::State state;
  TxPdo feedback;
  feedback.position = -123456;
  raisin::robot::ee_pdo::recordFeedback(state, 0, feedback, true);
  raisin::robot::ee_pdo::applyFeedbackFreshness(
      state.axes[0], true, feedbackTime, feedbackTime + timeout, timeout, true, true);
  require(state.axes[0].raw_valid && state.axes[0].calibrated_valid &&
              state.axes[0].raw_encoder_count == -123456,
          "feedback remains valid at the configured freshness boundary");
  const auto staleTime = feedbackTime + timeout + 1ms;
  raisin::robot::ee_pdo::applyFeedbackFreshness(
      state.axes[0], true, feedbackTime, staleTime, timeout, true, true);
  require(!state.axes[0].raw_valid && !state.axes[0].calibrated_valid &&
              state.axes[0].raw_encoder_count == 0,
          "stale feedback invalidates the published encoder count and calibrated state");
  require(raisin::robot::ee_pdo::feedbackCycleIsValid(true, 12, 12) &&
              !raisin::robot::ee_pdo::feedbackCycleIsValid(true, 11, 12) &&
              !raisin::robot::ee_pdo::feedbackCycleIsValid(true, 0, 12) &&
              !raisin::robot::ee_pdo::feedbackCycleIsValid(true, 12, 0) &&
              !raisin::robot::ee_pdo::feedbackCycleIsValid(false, 12, 12),
          "only a complete WKC cycle on an attached interface can carry fresh PDO feedback");

  raisin::raipal_ee::State failedCycleState;
  auto lastGoodFeedback = feedbackTime;
  raisin::robot::ee_pdo::recordFeedback(failedCycleState, 0, feedback, true);
  const auto failedCycle = [&](bool interfaceAvailable, int realWkc, int expectedWkc,
                               EeDriveBridge::TimePoint cycleTime) {
    ++failedCycleState.sequence;
    if (raisin::robot::ee_pdo::feedbackCycleIsValid(
            interfaceAvailable, realWkc, expectedWkc)) {
      raisin::robot::ee_pdo::recordFeedback(failedCycleState, 0, feedback, true);
      lastGoodFeedback = cycleTime;
    }
    raisin::robot::ee_pdo::applyFeedbackFreshness(
        failedCycleState.axes[0], true, lastGoodFeedback, cycleTime, timeout, true, true);
  };
  failedCycle(true, 11, 12, feedbackTime + 100ms);
  failedCycle(true, 0, 12, feedbackTime + 151ms);
  failedCycle(false, 12, 12, feedbackTime + 160ms);
  require(failedCycleState.sequence == 3 && !failedCycleState.axes[0].raw_valid &&
              !failedCycleState.axes[0].calibrated_valid,
          "partial/zero WKC and missing-interface cycles cannot refresh feedback through publication heartbeats");

  const bool staleOutputAllowed = raisin::robot::ee_pdo::motorOutputPermitted(
      true, true, true, false, true,
      raisin::robot::ee_pdo::feedbackIsFresh(feedbackTime, staleTime, timeout));
  RxPdo requested;
  requested.target_position = 4567;
  requested.target_torque = 23;
  requested.control_word = 0x0f;
  requested.mode = 10;
  const auto gated = raisin::robot::ee_pdo::gateMotorOutput(staleOutputAllowed, requested);
  require(!staleOutputAllowed && gated.target_position == 0 && gated.target_torque == 0 &&
              gated.control_word == 0 && gated.mode == 0,
          "stale feedback must force every outgoing RxPDO field to its safe zero value");

  auto raw = observedRaipalEeConfig({});
  raw.enabled = false;
  raw.actuation_qualified = false;
  raw.homing_file.clear();
  EeDriveBridge bridge(raw);
  raisin::ethercat::EthercatInterface bus({});
  bridge.attachEthercat(bus, observedRaipalScan());
  bridge.tick(feedbackTime);
  bridge.tick(feedbackTime + 20ms);
  const auto heartbeat = bridge.state(staleTime);
  require(heartbeat.sequence == 2 && !heartbeat.axes[0].raw_valid &&
              !heartbeat.axes[0].calibrated_valid,
          "publishing ticks may advance state sequence but cannot refresh missing PDO feedback");
  const auto encoded = YAML::Load(raisin::raipal_ee::encodeState(heartbeat));
  require(encoded["axes"][0]["raw_encoder_count"].IsNull() &&
              encoded["axes"][0]["position_rad"].IsNull(),
          "a stale state heartbeat must serialize untrusted encoder and angle values as null");
}

void testRawCommunicationOnlyAndSafeOutput(const TemporaryDirectory& temporary) {
  auto rawYaml = YAML::Load(R"yaml(
ee:
  enabled: false
  communication_enabled: true
  actuation_qualified: false
  backend: ethercat
  axes:
    left: {slave_id: 10, vendor_id: 16720, product_code: 37445, revision: 1,
           firmware_version: 142, pdo_profile: rx16_tx13,
           encoder_counts_per_motor_revolution: 65536, encoder_cpr_sdo_index: 32786}
    right: {slave_id: 11, vendor_id: 16720, product_code: 37445, revision: 1,
            firmware_version: 142, pdo_profile: rx16_tx13,
            encoder_counts_per_motor_revolution: 65536, encoder_cpr_sdo_index: 32786}
)yaml");
  raisin::parameter::ParameterContainer parameters;
  parameters.yamlIterator(rawYaml);
  const auto rawConfig = raisin::robot::readEeDriveConfig(parameters);
  require(rawConfig.communication_enabled && !rawConfig.enabled &&
              !rawConfig.actuation_qualified && rawConfig.axes[0].gear_ratio == 0. &&
              rawConfig.axes[0].direction == 0,
          "raw-only EtherCAT config must parse without invented calibration fields");
  require(!rawConfig.position_feedback_calibrated,
          "position feedback calibration must default to false");
  auto positionYaml = YAML::Clone(rawYaml);
  positionYaml["ee"]["position_feedback_calibrated"] = true;
  for (const auto* name : {"left", "right"}) {
    auto axis = positionYaml["ee"]["axes"][name];
    axis["gear_ratio"] = 68.;
    axis["direction"] = std::string(name) == "left" ? 1 : -1;
    axis["position_reference_count"] = 1000.;
  }
  const auto readPositionConfig = [](YAML::Node node) {
    raisin::parameter::ParameterContainer positionParameters;
    positionParameters.yamlIterator(node);
    return raisin::robot::readEeDriveConfig(positionParameters);
  };
  const auto positionConfig = readPositionConfig(positionYaml);
  TemporaryDirectory homeDirectory;
  auto homeYaml = YAML::Clone(positionYaml);
  homeYaml["ee"]["position_feedback_calibrated"] = false;
  homeYaml["ee"]["position_homing_enabled"] = true;
  homeYaml["ee"]["homing_file"] = (homeDirectory.path / "ee_home.conf").string();
  for (const auto* name : {"left", "right"}) {
    homeYaml["ee"]["axes"][name].remove("position_reference_count");
    homeYaml["ee"]["axes"][name]["home_position_rad"] = 0.;
  }
  const auto homeConfig = readPositionConfig(homeYaml);
  require(homeConfig.position_homing_enabled && !homeConfig.enabled &&
              !homeConfig.actuation_qualified && homeConfig.axes[0].gear_ratio == 68. &&
              homeConfig.axes[1].direction == -1,
          "position-only Set Home must parse without enabling motors or inventing an encoder zero");
  EeDriveBridge unreadyHomeBridge(homeConfig);
  require(!unreadyHomeBridge.setHomeFromCore().success &&
              !std::filesystem::exists(homeConfig.homing_file),
          "Set Home must not save a zero without fresh validated device feedback");
  for (const auto* field : {"homing_file", "communication_enabled"}) {
    auto invalid = YAML::Clone(homeYaml);
    if (std::string(field) == "homing_file") invalid["ee"][field] = "";
    else invalid["ee"][field] = false;
    requireThrows([&] { (void)readPositionConfig(invalid); }, "position",
                  "position-only Set Home requires a file and EtherCAT communication");
  }
  auto ambiguous = YAML::Clone(homeYaml);
  ambiguous["ee"]["position_feedback_calibrated"] = true;
  requireThrows([&] { (void)readPositionConfig(ambiguous); }, "",
                "Set Home and fixed encoder reference modes must not be mixed");
  for (const auto* name : {"left", "right"}) {
    for (const auto* field : {"gear_ratio", "direction", "position_reference_count"}) {
      auto missing = YAML::Clone(positionYaml);
      missing["ee"]["axes"][name].remove(field);
      requireThrows([&] { (void)readPositionConfig(missing); }, field,
                    "position calibration must be explicit on both axes");
    }
  }
  for (const auto* field : {"gear_ratio", "direction", "position_reference_count"}) {
    auto invalid = YAML::Clone(positionYaml);
    invalid["ee"]["axes"]["left"][field] = std::string(field) == "position_reference_count"
        ? 2147483648. : 0.;
    requireThrows([&] { (void)readPositionConfig(invalid); }, "position feedback",
                  "invalid position calibration must be rejected");
  }
  for (const double invalidValue : {std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::quiet_NaN()}) {
    for (const auto* field : {"gear_ratio", "position_reference_count"}) {
      auto invalid = YAML::Clone(positionYaml);
      invalid["ee"]["axes"]["right"][field] = invalidValue;
      requireThrows([&] { (void)readPositionConfig(invalid); }, "",
                    "nonfinite calibration must be rejected");
    }
  }
  {
    using namespace std::chrono_literals;
    EeDriveBridge positionBridge(positionConfig);
    raisin::ethercat::EthercatInterface bus({});
    positionBridge.attachEthercat(bus, observedRaipalScan());
    auto& context = bus.*fixtureMember(BusContext{});
    auto& slaves = context.slavelist;
    context.slavecount = 11;
    bus.*fixtureMember(BusExpectedWkc{}) = 6;
    bus.*fixtureMember(BusRealWkc{}) = 6;
    std::array<std::array<std::uint8_t, 13>, 2> inputs{};
    std::array<std::array<std::uint8_t, 16>, 2> outputs{};
    auto& devices = bus.*fixtureMember(BusDevices{});
    for (std::size_t index = 0; index < 2; ++index) {
      auto& slave = slaves[positionConfig.axes[index].slave_id];
      slave.state = EC_STATE_SAFE_OP;
      slave.inputs = inputs[index].data();
      slave.outputs = outputs[index].data();
      slave.Ibytes = 13;
      slave.Ibits = 104;
      slave.Obytes = 16;
      slave.Obits = 128;
      raisin::robot::ee_pdo::writeLe32(inputs[index].data(), 558056);
      inputs[index][10] = 0x27;
      outputs[index].fill(0xff);
      devices[index]->read();
      devices[index]->write();
      require(std::all_of(outputs[index].begin(), outputs[index].end(),
                          [](auto value) { return value == 0; }),
              "position-only bridge must write zero motor outputs");
    }
    const auto now = EeDriveBridge::Clock::now();
    const auto positionState = positionBridge.state(now);
    require(!positionState.actuation_qualified && !positionState.homed && !positionState.enabled,
            "position-only feedback cannot promote actuation or homing");
    for (std::size_t index = 0; index < 2; ++index) {
      const auto& axis = positionState.axes[index];
      require(axis.raw_valid && axis.position_valid && !axis.calibrated_valid &&
                  std::abs(axis.position_rad - positionConfig.axes[index].direction *
                           0.7853981633974483) < 1e-12,
              "bridge must divide motor counts by the 68:1 reducer exactly once");
    }
    for (const auto action : {Action::ENABLE, Action::TARGET, Action::SET_HOME})
      require(!positionBridge.handleCommand(command(action, 1), now).success,
              "position-only calibration must not authorize commands");
    const auto stale = positionBridge.state(now + 151ms);
    require(!stale.axes[0].position_valid && !stale.axes[1].position_valid,
            "snapshot freshness must invalidate position-only feedback");
    bus.*fixtureMember(BusRealWkc{}) = 0;
    devices[0]->read();
    require(!positionBridge.state(now + 151ms).axes[0].position_valid,
            "failed WKC cannot refresh position-only feedback");
    bus.*fixtureMember(BusRealWkc{}) = 6;
    raisin::robot::ee_pdo::writeLe32(inputs[0].data(), static_cast<std::uint32_t>(-556056));
    devices[0]->read();
    require(positionBridge.state().axes[0].position_valid &&
                positionBridge.state().axes[0].position_rad < 0.,
            "new signed PDO feedback must restore position validity");
    positionBridge.invalidateFeedback("fixture invalidation");
    require(!positionBridge.state().axes[0].position_valid,
            "explicit feedback invalidation must clear position validity");
  }
  {
    EeDriveBridge bridge(rawConfig);
    raisin::ethercat::EthercatInterface bus({});
    bridge.attachEthercat(bus, observedRaipalScan());
    auto now = EeDriveBridge::Clock::now();
    const auto stamp = stampNs(now);
    bridge.ingestTactileFrame(tactileFrame("left", 1, stamp));
    bridge.tick(now);
    const auto state = bridge.state(now);
    require(state.backend == Backend::ETHERCAT && !state.enabled && !state.homed &&
                !state.actuation_qualified && !state.tactile_ready,
            "raw EE state must remain available without homing or bilateral tactile readiness");

    auto feedbackState = state;
    TxPdo feedback;
    feedback.position = -123456;
    feedback.status_word = 0x27;
    raisin::robot::ee_pdo::recordFeedback(feedbackState, 0, feedback, false);
    const auto encoded = raisin::raipal_ee::encodeState(feedbackState);
    const auto encodedNode = YAML::Load(encoded);
    require(encodedNode["axes"][0]["raw_encoder_count"].as<std::int32_t>() == -123456 &&
                encodedNode["axes"][0]["raw_valid"].as<bool>() &&
                encodedNode["axes"][0]["position_rad"].IsNull() &&
                encodedNode["axes"][0]["torque_nm"].IsNull(),
            "signed raw feedback must publish while uncalibrated numeric fields stay null");
    const auto decoded = raisin::raipal_ee::parseState(encoded);
    require(decoded.axes[0].raw_valid && decoded.axes[0].raw_encoder_count == -123456 &&
                !decoded.axes[0].calibrated_valid && decoded.axes[0].position_rad == 0.,
            "state v2 round-trip must retain signed raw feedback without making angle valid");
  }

  auto qualified = observedRaipalEeConfig(
      (temporary.path / "initial-output-home.yaml").string());
  qualified.enabled = true;
  EeDriveBridge bridge(qualified);
  require(!bridge.state().enabled,
          "a qualified real EE must not auto-enable during bridge construction");
  RxPdo requested;
  requested.target_position = 4567;
  requested.target_torque = 23;
  requested.control_word = 0x0f;
  requested.mode = 10;
  const bool mayDrive = raisin::robot::ee_pdo::motorOutputPermitted(
      qualified.enabled, qualified.actuation_qualified, false, false, false, false);
  const auto safe = raisin::robot::ee_pdo::gateMotorOutput(mayDrive, requested);
  std::array<std::uint8_t, raisin::robot::ee_pdo::kRx16Bytes> outputBytes{};
  raisin::robot::ee_pdo::encodeRx(EePdoProfile::RX16_TX13, safe, outputBytes);
  require(!mayDrive && safe.target_position == 0 && safe.target_torque == 0 &&
              safe.control_word == 0 && safe.mode == 0 && outputBytes[10] == 0,
          "actual bridge output policy must hold a zero RxPDO until explicit enable");
}

void testTactileGateAndRecovery() {
  const auto now = EeDriveBridge::Clock::now();
  const auto now_ns = stampNs(now);
  {
    auto config = mockConfig();
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    bridge.ingestTactileFrame(tactileFrame("left", 1, now_ns));
    require(!bridge.state().tactile_ready, "unilateral tactile sample must keep gate closed");
    const auto result = bridge.handleCommand(command(Action::SET_HOME, 1));
    require(!result.success, "set_home must reject a unilateral tactile pair");
  }
  {
    auto config = mockConfig();
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    ingestPair(bridge, 1, 1, now_ns - 1000000ULL, now_ns, false, false);
    require(!bridge.state().tactile_ready, "unbaselined pair must keep gate closed");
    const auto result = bridge.handleCommand(command(Action::SET_HOME, 1));
    require(!result.success && result.message.find("baseline") != std::string::npos,
            "set_home must reject an unbaselined pair");
  }
  {
    auto config = mockConfig();
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    const auto skewed_left = now_ns - 100000000ULL;
    ingestPair(bridge, 1, 1, skewed_left, now_ns);
    const auto state = bridge.state();
    require(!state.tactile_ready &&
                std::find(state.gate_reasons.begin(), state.gate_reasons.end(),
                          "tactile_pair_skew") != state.gate_reasons.end(),
            "pair outside maximum timestamp skew must keep gate closed");
  }
  {
    auto config = mockConfig();
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    auto invalid = tactileFrame("left", 1, now_ns);
    invalid.normal_force_n[0] = std::numeric_limits<float>::quiet_NaN();
    bridge.ingestTactileFrame(invalid);
    require(!bridge.state().tactile_ready &&
                bridge.state().error.find("invalid tactile frame") != std::string::npos,
            "malformed tactile frame must invalidate the gate");
  }

  auto config = mockConfig();
  config.command_timeout = std::chrono::seconds(2);
  config.tactile_stale_timeout = std::chrono::milliseconds(150);
  EeDriveBridge bridge(config);
  bridge.setArmReady(true);
  ingestPair(bridge, 1, 1, now_ns - 1000000ULL, now_ns);
  require(bridge.state().tactile_ready, "fresh bilateral pair should open tactile gate");
  require(bridge.handleCommand(command(Action::SET_HOME, 1)).success,
          "fresh bilateral pair should permit set_home");
  require(bridge.handleCommand(command(Action::ENABLE, 2)).success && bridge.state().enabled,
          "fresh bilateral pair and home should permit enable");

  bridge.reportTactileError("test invalid-frame event");
  require(!bridge.state().enabled, "invalid tactile loss while enabled must safe-stop");
  auto recovery_time = EeDriveBridge::Clock::now();
  ingestPair(bridge, 2, 2, stampNs(recovery_time) - 1000000ULL, stampNs(recovery_time));
  require(bridge.state().tactile_ready && !bridge.state().enabled,
          "fresh bilateral recovery must not implicitly re-enable the drives");
  require(bridge.handleCommand(command(Action::ENABLE, 3)).success && bridge.state().enabled,
          "recovery requires a new explicit enable command");

  const auto later = recovery_time + std::chrono::milliseconds(200);
  bridge.tick(later);
  const auto stale_state = bridge.state(later);
  require(!stale_state.enabled && !stale_state.tactile_ready &&
              stale_state.error.find("tactile") != std::string::npos,
          "stale bilateral frames must safe-stop an enabled mock drive");
  recovery_time = EeDriveBridge::Clock::now();
  ingestPair(bridge, 3, 3, stampNs(recovery_time) - 1000000ULL, stampNs(recovery_time));
  require(bridge.state().tactile_ready && !bridge.state().enabled,
          "fresh frames after stale stop must remain disabled until explicit enable");
  require(bridge.handleCommand(command(Action::ENABLE, 4)).success && bridge.state().enabled,
          "fresh frames after stale stop require an explicit new enable");
}

void testSeparateCommandVelocity() {
  auto config = mockConfig();
  config.maintenance_requires_tactile = false;
  for (auto& axis : config.axes) axis.command_velocity_rad_s = 0.02;
  config = parseEeConfig(config);
  config.maintenance_requires_tactile = false;
  require(config.axes[0].command_velocity_rad_s == 0.02 &&
              config.axes[0].max_velocity_rad_s == 1.,
          "command slew must parse independently from the overspeed limit");
  EeDriveBridge bridge(config);
  const auto now = EeDriveBridge::Clock::now();
  bridge.setArmReady(true);
  bridge.setMaintenanceMode(true);
  bridge.tick(now);
  require(bridge.setHomeFromCore(now).success, "mock commissioning home");
  require(bridge.handleCommand(command(Action::ENABLE, 1), now).success,
          "mock commissioning enable");
  auto target = command(Action::TARGET, 2);
  target.target_rad = std::array<double, 2>{-0.02, 0.02};
  require(bridge.handleCommand(target, now).success, "mock commissioning target");
  const auto next = now + std::chrono::milliseconds(50);
  bridge.tick(next);
  const auto state = bridge.state(next);
  require(state.enabled && std::abs(state.axes[0].position_rad + 0.001) < 1e-9 &&
              std::abs(state.axes[1].position_rad - 0.001) < 1e-9,
          "mock motion must obey the lower command slew instead of the fault ceiling");
  require(std::abs(state.axes[0].velocity_rad_s + 0.02) < 1e-9 &&
              std::abs(state.axes[1].velocity_rad_s - 0.02) < 1e-9,
          "mock velocity must match the configured command slew");
}

void testMaintenanceTactilePolicyAndStops() {
  const auto now = EeDriveBridge::Clock::now();
  require(EeDriveConfig{}.maintenance_requires_tactile,
          "maintenance must require tactile by default");
  {
    auto config = mockConfig();
    config.maintenance_requires_tactile = true;
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    bridge.tick(now);
    bridge.setMaintenanceMode(true);
    require(!bridge.setHomeFromCore(now).success,
            "sensor-required maintenance must reject home without tactile");
    require(!bridge.handleCommand(command(Action::ENABLE, 1), now).success,
            "sensor-required maintenance must reject enable without tactile");
    require(!bridge.state(now).enabled && !bridge.state(now).tactile_ready,
            "sensor-required maintenance must remain disabled without tactile");
  }

  auto config = mockConfig();
  config.maintenance_requires_tactile = false;
  config.command_timeout = std::chrono::milliseconds(40);
  EeDriveBridge bridge(config);
  bridge.setArmReady(true);
  bridge.tick(now);
  require(!bridge.setHomeFromCore(now).success &&
              !bridge.handleCommand(command(Action::ENABLE, 1), now).success,
          "optional maintenance tactile must not bypass ordinary-mode tactile gates");
  bridge.setMaintenanceMode(true);
  require(!bridge.state(now).enabled,
          "entering maintenance must leave actuation disabled");
  bridge.setArmReady(false);
  require(!bridge.setHomeFromCore(now).success,
          "sensor-optional maintenance must still require arm readiness");
  bridge.setArmReady(true);
  require(bridge.setHomeFromCore(now).success,
          "sensor-optional maintenance must allow core-owned home without tactile");
  auto state = bridge.state(now);
  require(state.homed && state.axes[0].calibrated_valid && state.axes[1].calibrated_valid,
          "maintenance home must retain calibrated encoder readback");
  require(!state.tactile_ready && !state.tactile[0].valid && !state.tactile[1].valid,
          "sensor-optional maintenance must not fabricate tactile readiness");
  require(bridge.handleCommand(command(Action::ENABLE, 1), now).success,
          "sensor-optional maintenance must allow explicit enable");
  auto target = command(Action::TARGET, 2);
  target.target_rad = std::array<double, 2>{-0.2, 0.2};
  require(bridge.handleCommand(target, now).success,
          "sensor-optional maintenance must accept bounded targets");
  const auto moving = now + std::chrono::milliseconds(10);
  bridge.tick(moving);
  state = bridge.state(moving);
  require(state.enabled && state.axes[0].position_rad < 0. &&
              state.axes[1].position_rad > 0. && !state.tactile_ready,
          "maintenance must move both mock axes without claiming tactile readiness");
  bridge.setMaintenanceMode(true);
  require(bridge.state(moving).enabled,
          "repeating the current owner must not stop maintenance");
  bridge.setMaintenanceMode(false);
  state = bridge.state(moving);
  require(!state.enabled && !state.tactile_ready &&
              state.axes[0].target_rad == state.axes[0].position_rad &&
              state.axes[1].target_rad == state.axes[1].position_rad,
          "leaving maintenance must safe-stop and discard outstanding motion");
  require(std::find(state.gate_reasons.begin(), state.gate_reasons.end(),
                    "left_tactile_missing") != state.gate_reasons.end() &&
              std::find(state.gate_reasons.begin(), state.gate_reasons.end(),
                        "right_tactile_missing") != state.gate_reasons.end(),
          "ordinary owner must restore both missing tactile gate reasons");
  require(!bridge.handleCommand(command(Action::ENABLE, 3), moving).success &&
              !bridge.setHomeFromCore(moving).success,
          "ordinary owner must remain tactile-blocked after maintenance home");

  bridge.setMaintenanceMode(true);
  require(bridge.handleCommand(command(Action::ENABLE, 3), moving).success,
          "maintenance restart requires a fresh explicit enable");
  target.sequence = 4;
  require(bridge.handleCommand(target, moving).success,
          "maintenance restart target must be accepted");
  require(bridge.handleCommand(command(Action::STOP, 5), moving).success,
          "STOP must be accepted during sensor-optional maintenance");
  const auto stopped = moving + std::chrono::milliseconds(10);
  bridge.tick(stopped);
  state = bridge.state(stopped);
  require(!state.enabled && state.axes[0].target_rad == state.axes[0].position_rad &&
              state.axes[1].target_rad == state.axes[1].position_rad,
          "STOP must abort maintenance motion and keep it disabled");
  target.sequence = 6;
  require(!bridge.handleCommand(target, stopped).success,
          "a target must not resume stopped maintenance without explicit enable");

  require(bridge.handleCommand(command(Action::ENABLE, 6), stopped).success,
          "explicit enable must allow a new maintenance session after STOP");
  const auto heartbeat = stopped + std::chrono::milliseconds(30);
  require(bridge.handleCommand(command(Action::HEARTBEAT, 7), heartbeat).success,
          "maintenance heartbeat must be accepted without tactile");
  bridge.tick(heartbeat + std::chrono::milliseconds(30));
  require(bridge.state(heartbeat + std::chrono::milliseconds(30)).enabled,
          "fresh heartbeat must extend maintenance command liveness");
  const auto expired = heartbeat + std::chrono::milliseconds(41);
  bridge.tick(expired);
  state = bridge.state(expired);
  require(!state.enabled && !state.tactile_ready &&
              state.error.find("heartbeat expired") != std::string::npos,
          "heartbeat expiry must safe-stop maintenance despite optional tactile");
}

void testHeartbeatTimeoutAndHomeFiles(const TemporaryDirectory& temporary) {
  const auto now = EeDriveBridge::Clock::now();
  const auto now_ns = stampNs(now);
  {
    auto config = mockConfig();
    config.command_timeout = std::chrono::milliseconds(40);
    config.tactile_stale_timeout = std::chrono::seconds(5);
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    ingestPair(bridge, 1, 1, now_ns - 1000000ULL, now_ns);
    require(bridge.handleCommand(command(Action::SET_HOME, 1), now).success,
            "heartbeat fixture set_home");
    require(bridge.handleCommand(command(Action::ENABLE, 2), now).success,
            "heartbeat fixture enable");
    const auto later = now + std::chrono::milliseconds(50);
    bridge.tick(later);
    const auto state = bridge.state(later);
    require(!state.enabled && state.error.find("heartbeat expired") != std::string::npos,
            "command heartbeat timeout must safe-stop the enabled drive");
  }
  {
    auto config = mockConfig();
    config.homing_file = (temporary.path / "missing-parent" / "ee-home.yaml").string();
    EeDriveBridge bridge(config);
    bridge.setArmReady(true);
    ingestPair(bridge, 1, 1, now_ns - 1000000ULL, now_ns);
    const auto result = bridge.handleCommand(command(Action::SET_HOME, 1));
    require(!result.success && result.message.find("cannot open temporary") != std::string::npos &&
                !bridge.state().homed && bridge.state().last_command_sequence == 0,
            "failed home-file write must not report home saved");
  }

  const auto home_file = (temporary.path / "mock-home.yaml").string();
  auto mock = qualifiedEthercatConfig(home_file);
  mock.backend = Backend::MOCK;
  EeDriveBridge first_mock(mock);
  first_mock.setArmReady(true);
  auto current = EeDriveBridge::Clock::now();
  ingestPair(first_mock, 1, 1, stampNs(current) - 1000000ULL, stampNs(current));
  require(first_mock.handleCommand(command(Action::SET_HOME, 1)).success &&
              std::filesystem::exists(home_file),
          "mock set_home should atomically persist its software-only home fixture");
  EeDriveBridge restarted_mock(mock);
  require(restarted_mock.state().homed,
          "matching mock restart should reload its own persisted home");
  auto real = qualifiedEthercatConfig(home_file);
  EeDriveBridge real_from_mock_file(real);
  require(!real_from_mock_file.state().homed &&
              real_from_mock_file.state().error.find("re-home required") != std::string::npos,
          "mock-origin homing file must never qualify a real EtherCAT configuration");
}

std::string testFingerprint(const EeDriveConfig& config) {
  const auto axisFingerprint = [](const EeAxisConfig& axis) {
    std::ostringstream out;
    out << std::setprecision(17) << axis.name << ':' << axis.slave_id << ':' << axis.vendor_id
        << ':' << axis.product_code << ':' << axis.revision << ':' << axis.firmware_version << ':'
        << (axis.pdo_profile == EePdoProfile::RX12_TX13 ? "rx12_tx13" : "rx16_tx13") << ':'
        << axis.encoder_cpr_sdo_index << ':' << axis.encoder_counts_per_motor_revolution << ':'
        << axis.gear_ratio << ':' << axis.direction << ':' << axis.home_position_rad;
    return out.str();
  };
  return raisin::raipal_ee::toString(config.backend) + "|" +
      axisFingerprint(config.axes[0]) + "|" + axisFingerprint(config.axes[1]);
}

std::string legacyTestFingerprint(const EeDriveConfig& config) {
  auto legacy = config;
  for (auto& axis : legacy.axes) {
    axis.motor_rated_torque_nm = 0.;
    axis.gear_efficiency = 0.;
    axis.lower_limit_rad = 0.;
    axis.upper_limit_rad = 0.;
    axis.max_velocity_rad_s = 0.;
    axis.motor_peak_torque_nm = 0.;
  }
  const auto fingerprint = [](const EeAxisConfig& axis) {
    std::ostringstream out;
    out << std::setprecision(17) << axis.name << ':' << axis.slave_id << ':' << axis.vendor_id
        << ':' << axis.product_code << ':' << axis.revision << ':' << axis.firmware_version << ':'
        << (axis.pdo_profile == EePdoProfile::RX12_TX13 ? "rx12_tx13" : "rx16_tx13") << ':'
        << axis.encoder_cpr_sdo_index << ':' << axis.encoder_counts_per_motor_revolution << ':'
        << axis.gear_ratio << ':' << axis.direction << ':' << axis.motor_rated_torque_nm << ':'
        << axis.gear_efficiency << ':' << axis.lower_limit_rad << ':' << axis.upper_limit_rad << ':'
        << axis.home_position_rad << ':' << axis.max_velocity_rad_s << ':' << axis.motor_peak_torque_nm;
    return out.str();
  };
  return raisin::raipal_ee::toString(legacy.backend) + "|" + fingerprint(legacy.axes[0]) + "|" +
         fingerprint(legacy.axes[1]);
}

void writeRealHomeFixture(const EeDriveConfig& config, bool legacy = false) {
  YAML::Node root;
  root["schema_version"] = 1;
  root["fingerprint"] = legacy ? legacyTestFingerprint(config) : testFingerprint(config);
  for (const auto& name : {"left", "right"}) {
    YAML::Node axis;
    axis["name"] = name;
    axis["encoder_offset_counts"] = 0;
    root["axes"].push_back(axis);
  }
  std::ofstream file(config.homing_file);
  file << YAML::Dump(root) << '\n';
  require(static_cast<bool>(file), "could not write synthetic real-origin home fixture");
}

void testCalibrationFingerprint(const TemporaryDirectory& temporary) {
  const auto path = (temporary.path / "real-origin-home.yaml").string();
  auto real = qualifiedEthercatConfig(path);
  writeRealHomeFixture(real);
  EeDriveBridge matching(real);  // Qualified config parser only; no EtherCAT attach/start.
  const auto matching_state = matching.state();
  require(!matching_state.homed && matching_state.error.empty() &&
              std::find(matching_state.gate_reasons.begin(), matching_state.gate_reasons.end(),
                        "ee_not_homed") != matching_state.gate_reasons.end(),
          "matching real home file may seed offsets but cannot home a new communication session");
  matching.setArmReady(true);
  const auto now = EeDriveBridge::Clock::now();
  ingestPair(matching, 1, 1, stampNs(now) - 1000000ULL, stampNs(now));
  const auto enable = matching.handleCommand(command(Action::ENABLE, 1), now);
  require(!enable.success && !matching.state().enabled && !matching.state().homed,
          "a persisted real offset file alone must not authorize enable before explicit set_home");

  auto changed = real;
  ++changed.axes[0].encoder_counts_per_motor_revolution;
  EeDriveBridge changed_calibration(changed);
  require(!changed_calibration.state().homed &&
              changed_calibration.state().error.find("re-home required") != std::string::npos,
          "encoder calibration change must invalidate persisted real home");

  auto changed_sdo = real;
  changed_sdo.axes[0].encoder_cpr_sdo_index = 0x8012;
  EeDriveBridge changed_sdo_address(changed_sdo);
  require(!changed_sdo_address.state().homed &&
              changed_sdo_address.state().error.find("re-home required") != std::string::npos,
          "changing the CPR SDO address must invalidate persisted real home");

  real.persistent_multiturn_home = true;
  real = parseEeConfig(real);
  require(real.persistent_multiturn_home, "persistent multiturn option must parse");
  writeRealHomeFixture(real, true);
  EeDriveBridge persistent(real);
  require(persistent.state().homed && !persistent.state().enabled &&
              !persistent.state().axes[0].position_valid,
          "matching multiturn home must persist without enabling or fabricating feedback");
  persistent.invalidateFeedback("fixture power interruption");
  require(persistent.state().homed && !persistent.state().enabled &&
              !persistent.state().axes[0].position_valid,
          "feedback loss must preserve multiturn calibration but invalidate position and output");
  persistent.setArmReady(true);
  require(!persistent.handleCommand(command(Action::ENABLE, 1)).success,
          "a restored multiturn home must not bypass attachment and fresh feedback");
  auto incompatible = real;
  ++incompatible.axes[0].encoder_counts_per_motor_revolution;
  EeDriveBridge persistent_changed(incompatible);
  require(!persistent_changed.state().homed,
          "persistent multiturn must not bypass a mismatched calibration fingerprint");
  auto missing = real;
  missing.homing_file = (temporary.path / "missing-multiturn-home.yaml").string();
  EeDriveBridge persistent_missing(missing);
  require(!persistent_missing.state().homed,
          "persistent multiturn must not fabricate missing offsets");
}

void testMultiturnFeedbackRecovery(const TemporaryDirectory& temporary) {
  auto config = observedRaipalEeConfig((temporary.path / "multiturn-home.yaml").string());
  config.enabled = false;
  config.actuation_qualified = false;
  config.position_homing_enabled = true;
  config.persistent_multiturn_home = true;
  config.axes[0].lower_limit_rad = 0.;
  config.axes[0].upper_limit_rad = 0.96;
  config.axes[1].lower_limit_rad = -0.96;
  config.axes[1].upper_limit_rad = 0.;
  writeRealHomeFixture(config);
  EeDriveBridge bridge(config);
  raisin::ethercat::EthercatInterface bus({});
  bridge.attachEthercat(bus, observedRaipalScan());
  auto& context = bus.*fixtureMember(BusContext{});
  auto& slaves = context.slavelist;
  context.slavecount = 11;
  bus.*fixtureMember(BusExpectedWkc{}) = 6;
  bus.*fixtureMember(BusRealWkc{}) = 6;
  std::array<std::array<std::uint8_t, 13>, 2> inputs{};
  std::array<std::array<std::uint8_t, 16>, 2> outputs{};
  auto& devices = bus.*fixtureMember(BusDevices{});
  for (std::size_t index = 0; index < 2; ++index) {
    auto& slave = slaves[config.axes[index].slave_id];
    slave.state = EC_STATE_SAFE_OP;
    slave.inputs = inputs[index].data();
    slave.outputs = outputs[index].data();
    slave.Ibytes = 13;
    slave.Ibits = 104;
    slave.Obytes = 16;
    slave.Obits = 128;
    const auto motorCounts = static_cast<std::int32_t>(std::llround(
        0.96 * 65536. * 68. / (2. * std::acos(-1.))));
    raisin::robot::ee_pdo::writeLe32(inputs[index].data(), motorCounts);
    inputs[index][10] = 0x21;
    devices[index]->read();
  }
  const auto fresh = bridge.state();
  require(fresh.homed && !fresh.enabled && fresh.axes[0].position_valid &&
              std::abs(fresh.axes[0].position_rad - 0.96) < 1.e-6 &&
              std::abs(fresh.axes[1].position_rad + 0.96) < 1.e-6,
          "motor counts must convert once through 68:1 to signed 0.96-rad outputs");
  const auto stale = bridge.state(EeDriveBridge::Clock::now() + std::chrono::seconds(1));
  require(stale.homed && !stale.enabled && !stale.axes[0].position_valid,
          "stale feedback must hide positions without discarding saved multiturn home");
  bridge.invalidateFeedback("fixture communication interruption");
  for (std::size_t index = 0; index < 2; ++index) devices[index]->read();
  const auto recovered = bridge.state();
  require(recovered.homed && !recovered.enabled && recovered.axes[0].position_valid &&
              recovered.axes[0].position_rad == fresh.axes[0].position_rad,
          "feedback recovery must reuse the original offset without auto-enable");
}

void testLatchedCoreFaultRestart(const TemporaryDirectory& temporary) {
  auto config = mockConfig();
  config.homing_file = (temporary.path / "latched-fault-home.yaml").string();
  EeDriveBridge bridge(config);
  bridge.setArmReady(true);
  auto now = EeDriveBridge::Clock::now();
  ingestPair(bridge, 1, 1, stampNs(now) - 1000000ULL, stampNs(now));
  require(bridge.handleCommand(command(Action::SET_HOME, 1), now).success,
          "fault fixture home should persist before fault injection");
  require(bridge.handleCommand(command(Action::ENABLE, 2), now).success &&
              bridge.state().enabled,
          "fault fixture should enable before a core fault");

  bridge.setCoreFault("fixture core fault");
  require(!bridge.state().enabled && bridge.state().error == "fixture core fault",
          "core fault should immediately safe-stop and latch its reason");
  now = EeDriveBridge::Clock::now();
  ingestPair(bridge, 2, 2, stampNs(now) - 1000000ULL, stampNs(now));
  const auto blocked = bridge.handleCommand(command(Action::ENABLE, 3), now);
  require(!blocked.success && !bridge.state().enabled &&
              blocked.message.find("fixture core fault") != std::string::npos,
          "fresh tactile data must not clear a latched core fault in-process");

  EeDriveBridge restarted(config);
  require(restarted.state().homed && !restarted.state().enabled,
          "restart should reload the matching home file without auto-enabling");
  restarted.setArmReady(true);
  now = EeDriveBridge::Clock::now();
  ingestPair(restarted, 1, 1, stampNs(now) - 1000000ULL, stampNs(now));
  require(restarted.state().tactile_ready && !restarted.state().enabled,
          "restart requires fresh bilateral tactile frames and remains disabled");
  const auto enabled = restarted.handleCommand(command(Action::ENABLE, 1), now);
  require(enabled.success && restarted.state().enabled,
          "restart should permit enable only after a new explicit command");
}

void testIdentificationLeaseAndEnvelope() {
  auto config = mockConfig();
  config.identification_enabled = true;
  config.maintenance_requires_tactile = false;
  for (auto& axis : config.axes) {
    axis.gear_ratio = 1.;
    axis.gear_efficiency = 1.;
    axis.motor_rated_torque_nm = 1.;
    axis.kp_nm_per_rad = 5.;
    axis.kd_nm_s_per_rad = 2.5;
  }
  EeDriveBridge bridge(config);
  bridge.setArmReady(true);
  bridge.setMaintenanceMode(true);
  bridge.tick();
  require(bridge.setHomeFromCore().success, "identification fixture home");
  bridge.setMaintenanceMode(false);
  const std::string begin = R"({action: begin, token: fixture, duration_s: 1, axis: 0, max_displacement_rad: 0.02,
    axes: [{kp: 5, kd: 2.5, lower_rad: -0.05, upper_rad: 0, velocity_rad_s: 0.2,
            command_velocity_rad_s: 0.02, output_torque_nm: 0.5},
           {kp: 5, kd: 2.5, lower_rad: 0, upper_rad: 0.05, velocity_rad_s: 0.2,
            command_velocity_rad_s: 0.02, output_torque_nm: 0.5}]})";
  require(YAML::Load(bridge.identify(begin))["active"].as<bool>(), "identification begin");
  require(!bridge.state().enabled, "begin never enables motors");
  requireThrows([&] { bridge.identify("{action: enable, token: stranger}"); },
                "matching", "wrong session cannot enable");
  bridge.identify("{action: enable, token: fixture}");
  require(bridge.state().enabled, "explicit identification enable");
  requireThrows([&] { bridge.identify("{action: target, token: fixture, target_rad: [-0.2, 0]}"); },
                "outside", "target wrapper is enforced inside node");
  requireThrows([&] { bridge.identify("{action: target, token: fixture, target_rad: [0, 0.01]}"); },
                "outside", "stationary axis target cannot change");
  require(!bridge.setHomeFromCore().success, "identification never changes home");
  require(!bridge.handleCommand(command(Action::HEARTBEAT, 10000)).success,
          "plugin heartbeat cannot renew identification lease");
  bridge.tick(EeDriveBridge::Clock::now() + std::chrono::milliseconds(501));
  require(!bridge.identificationActive() && !bridge.state().enabled,
          "lost wrapper disables and releases identification");
  require(std::abs(bridge.state().axes[0].lower_limit_rad - config.axes[0].lower_limit_rad) < 1e-12,
          "timeout restores production bounds");
  bridge.tick();
  auto invalid = YAML::Load(begin);
  invalid["axes"][0]["lower_rad"] = -2.;
  requireThrows([&] { bridge.identify(YAML::Dump(invalid)); }, "node limits", "cannot enlarge hard limits");
  invalid = YAML::Load(begin);
  invalid["axes"][0]["kp"] = 101.;
  requireThrows([&] { bridge.identify(YAML::Dump(invalid)); }, "wrist reference", "gain cap");
  invalid["axes"][0]["kp"] = std::numeric_limits<double>::quiet_NaN();
  requireThrows([&] { bridge.identify(YAML::Dump(invalid)); }, "nonfinite", "NaN gain");
  bridge.identify(begin);
  require(bridge.handleCommand(command(Action::STOP, 10001)).success &&
          !bridge.identificationActive(), "regular STOP aborts identification");
  config.identification_enabled = false;
  EeDriveBridge disabled(config);
  requireThrows([&] { disabled.identify(begin); }, "false", "default identification disabled");
}

}  // namespace

int main() {
  try {
    using raisin::robot::ee_pdo::impedanceHoldOutput;
    for (const auto status : {0x40, 0x21, 0x23, 0x27}) {
      const auto hold = impedanceHoldOutput(status, -154906, 1, 1);
      require(hold.mode == 7 && hold.target_position == -154906 &&
                  hold.target_velocity == 0 && hold.target_torque == 0 &&
                  hold.p_gain == 1 && hold.d_gain == 1,
              "every enable stage must hold measured position with configured damping");
      require(hold.control_word == (status == 0x40 ? 6 : status == 0x21 ? 7 : 15),
              "impedance hold must retain the existing CiA402 transition words");
    }
    const auto invalidHold = impedanceHoldOutput(0x08, 123, 1, 1);
    require(invalidHold.control_word == 0 && invalidHold.p_gain == 0 && invalidHold.d_gain == 0,
            "faulted drive must not receive an enable command or gains");
    using raisin::robot::ee_pdo::measuredPositionWithinLimits;
    require(measuredPositionWithinLimits(0.00173418784, -0.2, 0.) &&
            measuredPositionWithinLimits(-0.00173418784, 0., 0.2),
            "closed EE feedback must tolerate the recorded home offset within 0.005 rad");
    require(!measuredPositionWithinLimits(0.0051, -0.2, 0.) &&
            !measuredPositionWithinLimits(-0.0051, 0., 0.2) &&
            !measuredPositionWithinLimits(-0.2051, -0.2, 0.) &&
            !measuredPositionWithinLimits(0.2051, 0., 0.2) &&
            !measuredPositionWithinLimits(std::numeric_limits<double>::quiet_NaN(), -0.2, 0.),
            "feedback outside the bounded tolerance must still be rejected");
    using raisin::robot::ee_pdo::impedanceGain;
    using raisin::robot::ee_pdo::motorTorque;
    require(impedanceGain(100., 68.) == 22 && impedanceGain(5., 68.) == 1,
            "gain conversion must match Dongdrive joint gain / N squared * 1000");
    require(impedanceGain(0., 68.) == 0 && impedanceGain(5., 1.) == 5000,
            "zero gains and unit gearing must retain Dongdrive units");
    requireThrows([&] { (void)impedanceGain(0.2, 68.); }, "rounding to zero",
                  "nonzero requested damping cannot serialize as zero");
    requireThrows([&] { (void)impedanceGain(100000., 1.); }, "overflow",
                  "wire gains must not wrap uint16");
    require(std::abs(motorTorque(-41, .3) + .0123) < 1e-12 &&
                std::abs(motorTorque(145, .3) - .0435) < 1e-12,
            "captured feedback uses the E50x08 rated motor torque, not a joint limit");
    require(std::abs(motorTorque(3266, .3) - .9798) < 1e-12 &&
                motorTorque(3267, .3) > .9798,
            "E50x08 peak boundary must be checked before reducer multiplication");
    EeAxisConfig diagnosticAxis;
    diagnosticAxis.name = "left";
    diagnosticAxis.slave_id = 17;
    int reads = 0;
    const auto diagnostic = raisin::robot::ee_drive_bridge_detail::describeDriveCalibrationSdo(
        diagnosticAxis, [&](int slave, std::uint16_t index, std::uint8_t subindex,
                            void* value, int& size) {
          assert(slave == 17 && subindex == 0 && size == sizeof(std::int32_t));
          ++reads;
          *static_cast<std::int32_t*>(value) = 123;
          if (index == 0x800e) { size = 2; return true; }
          return index != 0x8014;
        });
    assert(reads == 8);
    assert(diagnostic.find("0x800d=123") != std::string::npos);
    assert(diagnostic.find("0x800e=unreadable") != std::string::npos);
    assert(diagnostic.find("0x8014=unreadable") != std::string::npos);
    TemporaryDirectory temporary;
    testPdoLayouts();
    testSafeOpBeforeValidationAndOperationalTransition();
    testInvalidConfigurations();
    testCprSdoAddressConfigurationAndQualification();
    testScanTopologyValidation(temporary);
    testObservedRaipalScanClassification(temporary);
    testFeedbackFreshnessAndHeartbeat(temporary);
    testRawCommunicationOnlyAndSafeOutput(temporary);
    testTactileGateAndRecovery();
    testSeparateCommandVelocity();
    testMaintenanceTactilePolicyAndStops();
    testHeartbeatTimeoutAndHomeFiles(temporary);
    testCalibrationFingerprint(temporary);
    testMultiturnFeedbackRecovery(temporary);
    testLatchedCoreFaultRestart(temporary);
    testIdentificationLeaseAndEnvelope();
    std::cout << "EE PDO and bridge safety tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "EE PDO/bridge test failed: " << error.what() << '\n';
    return 1;
  }
}
