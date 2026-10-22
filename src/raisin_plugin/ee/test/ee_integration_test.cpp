#include "ee_drive_bridge.hpp"
#include "raisin_raipal_ee/contract.hpp"
#include "raisin_raipal_ee/hardware_configuration.hpp"
#include "raisin_raipal_ee/mechanics.hpp"

#include <raisim/RaisimServer.hpp>
#include <raisim/World.hpp>
#include <raisin_interfaces/srv/string.hpp>
#include <raisin_network/network.hpp>
#include <raisin_network/node.hpp>
#include <raisin_network/raisin.hpp>
#include <raisin_plugin/plugin_manager.hpp>
#include <raisin_robot/robot.hpp>
#include <raisin_util/raisin_directories.hpp>
#include <raisin_tactile_sensor/frame_v2.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <sstream>
#include <system_error>
#include <unistd.h>

using namespace std::chrono_literals;

namespace {

using Clock = std::chrono::steady_clock;
using StringService = raisin::raisin_interfaces::srv::String;
using StateMessage = raisin::std_msgs::msg::String;
using TactileMessage = raisin::raisin_tactile_sensor::msg::TactileFrame;
using State = raisin::raipal_ee::State;

struct RemoveTemporaryTree {
  std::filesystem::path path;
  ~RemoveTemporaryTree() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::uint64_t monotonicNs(Clock::time_point now) {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count());
}

raisin::tactile::FrameV2 tactileFrame(const std::string& sensor, std::uint64_t sequence,
                                     std::uint64_t stamp, std::uint64_t sessionId = 1) {
  raisin::tactile::FrameV2 frame;
  frame.sensor_name = sensor;
  frame.frame_id = "tactile_" + sensor + "_frame";
  frame.clock_domain = "CLOCK_MONOTONIC";
  frame.calibration_id = "integration_provisional";
  frame.calibration_status = "provisional";
  frame.baseline_status = "operator_confirmed_unloaded";
  frame.force_unit = "N_estimate";
  frame.stamp_monotonic_ns = stamp;
  frame.session_id = sessionId;
  frame.sequence = sequence;
  frame.rows = raisin::tactile::kUsbRows;
  frame.cols = raisin::tactile::kUsbCols;
  frame.code_to_force_n_per_code = 0.001F;
  frame.polarity = 1;
  frame.baseline_valid = true;
  frame.raw_code.assign(raisin::tactile::kUsbCells, 0);
  frame.baseline_code.assign(raisin::tactile::kUsbCells, 0);
  frame.normal_force_n.assign(raisin::tactile::kUsbCells, 0.0F);
  frame.valid.assign(raisin::tactile::kUsbCells, 0);
  constexpr auto firstRoiCell = raisin::tactile::kUsbRoiRxFirst * raisin::tactile::kUsbCols +
                                raisin::tactile::kUsbRoiTxFirst;
  frame.valid[firstRoiCell] = 1;
  return frame;
}

std::filesystem::path makePrivateInstall(const std::filesystem::path& stageInstall,
                                         const std::filesystem::path& encryptionLibrary,
                                         const std::filesystem::path& temporaryRoot) {
  const auto prefix = temporaryRoot / "runtime";
  std::filesystem::create_directories(prefix / "lib");
  std::filesystem::create_directories(prefix / "resource/raisin_raipal_ee");
  std::filesystem::create_directories(prefix / "config");

  const auto stagedLib = stageInstall / "lib";
  require(std::filesystem::is_directory(stagedLib), "staged install lib directory is missing");
  for (const auto& entry : std::filesystem::recursive_directory_iterator(stagedLib)) {
    if (!entry.is_regular_file() ||
        entry.path().filename() == "libraisin_empty_encryption.so")
      continue;
    const auto relativePath = entry.path().lexically_relative(stagedLib);
    const auto destination = prefix / "lib" / relativePath;
    std::filesystem::create_directories(destination.parent_path());
    std::filesystem::create_symlink(entry.path(), destination);
  }
  const auto encryptionDestination = prefix / "lib/libraisin_empty_encryption.so";
  if (!std::filesystem::exists(encryptionDestination)) {
    require(std::filesystem::is_regular_file(encryptionLibrary),
            "SDK empty-encryption module is missing");
    std::filesystem::create_symlink(encryptionLibrary, encryptionDestination);
  }

  const auto stagedResource = stageInstall / "resource/raisin_raipal_ee/resource";
  require(std::filesystem::is_directory(stagedResource), "staged EE model resources are missing");
  std::filesystem::create_directory_symlink(
      stagedResource, prefix / "resource/raisin_raipal_ee/resource");

  const auto stagedPluginConfig = stageInstall / "config/raisin_raipal_ee_plugin";
  require(std::filesystem::is_directory(stagedPluginConfig), "staged EE plugin config is missing");
  std::filesystem::create_directory_symlink(stagedPluginConfig,
      prefix / "config/raisin_raipal_ee_plugin");
  const auto stagedTactileConfig = stageInstall / "config/raisin_tactile_plugin";
  require(std::filesystem::is_directory(stagedTactileConfig),
          "staged tactile plugin config is missing");
  std::filesystem::create_directory_symlink(stagedTactileConfig,
      prefix / "config/raisin_tactile_plugin");

  const auto networkConfig = prefix / "config/raisin_network/config";
  std::filesystem::create_directories(networkConfig);
  std::ofstream params(networkConfig / "params.yaml");
  params << "websocket_interface:\n"
         << "  value: 127.0.0.1\n"
         << "  dtype: string\n"
         << "encryption: empty\n";
  params.close();
  require(params.good(), "cannot write private loopback network config");
  return prefix;
}

raisin::raipal_ee::HardwareConfiguration rightEeHardwareConfiguration() {
  raisin::raipal_ee::HardwareConfiguration config;
  config.hardware_profile = "dual_arm_right_ee";
  config.selection_source = "yaml";
  config.arm_dof = 14;
  config.expected_slaves = 18;
  config.modules = {"raipal_R", "raipal-ee-visual_R", "raipal_L", "tool-empty_L"};
  config.gui_arm_modules = {"raipal_R", "stub-0_R", "raipal_L", "tool-empty_L"};
  config.raisim_config = "/raisim_config/flat_terrain_ee_visual_R.xml";
  config.ee_present = true;
  config.ee_communication = true;
  config.ee_actuation_qualified = true;
  config.ee_mount_mode = "single_R";
  config.ee_visual_resource = "resource/raisin_raipal_ee/resource/ee_visual.urdf";
  config.ee_mount_resource = "resource/raisin_raipal_ee/resource/ee_mounts.yaml";
  config.initial_motor_state = "COMM_DISABLED";
  return config;
}

raisin::robot::EeDriveConfig mockConfig(const std::filesystem::path& homePath) {
  raisin::robot::EeDriveConfig config;
  config.backend = raisin::raipal_ee::Backend::MOCK;
  config.enabled = true;
  config.actuation_qualified = true;
  config.homing_file = homePath.string();
  config.command_timeout = 800ms;
  config.tactile_stale_timeout = 150ms;
  config.tactile_max_skew = 50ms;
  config.mock_velocity_rad_s = 0.8;
  config.state_publish_rate_hz = 50.0;
  config.axes[0] = {.name = "left", .lower_limit_rad = -0.96, .upper_limit_rad = 0.132,
                    .home_position_rad = 0.0, .max_velocity_rad_s = 0.8,
                    .motor_peak_torque_nm = 1.0, .kp_nm_per_rad = 0.0, .kd_nm_s_per_rad = 0.0};
  config.axes[1] = {.name = "right", .lower_limit_rad = 0.0, .upper_limit_rad = 0.96,
                    .home_position_rad = 0.0, .max_velocity_rad_s = 0.8,
                    .motor_peak_torque_nm = 1.0, .kp_nm_per_rad = 0.0, .kd_nm_s_per_rad = 0.0};
  return config;
}

class CoreServiceNode final : public raisin::Node {
 public:
  CoreServiceNode(const std::shared_ptr<raisin::Network>& network,
                  const std::string& nickname, const std::filesystem::path& homePath,
                  const std::string& tactileTopicPrefix)
      : Node("ee_test_core_" + std::to_string(::getpid()), network),
        bridge_(mockConfig(homePath)) {
    bridge_.setArmReady(true);
    statePublisher_ = createPublisher<StateMessage>(nickname + "/ee/state", 5);
    driveService_ = createService<StringService>(nickname + "/ee/drive_command",
        [this](StringService::Request::SharedPtr request,
               StringService::Response::SharedPtr response) {
          handleDriveCommand(request, response);
        }, "drive_service");
    leftTactileSubscriber_ = createSubscriber<TactileMessage>(
        tactileTopicPrefix + "/left", nullptr,
        [this](const TactileMessage::SharedPtr& message) {
          ingestTactileMessage("left", message);
        }, "core_sensor");
    rightTactileSubscriber_ = createSubscriber<TactileMessage>(
        tactileTopicPrefix + "/right", nullptr,
        [this](const TactileMessage::SharedPtr& message) {
          ingestTactileMessage("right", message);
        }, "core_sensor");
    require(statePublisher_ != nullptr, "failed to create core EE state publisher");
    require(driveService_ != nullptr, "failed to create core EE drive service");
    require(leftTactileSubscriber_ != nullptr && rightTactileSubscriber_ != nullptr,
            "failed to create standalone tactile subscribers");
  }

  ~CoreServiceNode() override {
    stop();
    cleanupResources();
  }

  void start() {
    running_ = true;
    worker_ = std::thread([this] { publishLoop(); });
  }

  void stop() {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

  void stopFeedingTactile() { feedTactile_ = false; }
  void resumeFeedingTactile() { feedTactile_ = true; }

  void holdNextEnableAck() {
    std::lock_guard lock(ackMutex_);
    holdNextEnableAck_ = true;
    enableAckHeld_ = false;
    releaseEnableAck_ = false;
    heldEnableCommand_.reset();
  }

  void waitForHeldEnableAck(std::chrono::milliseconds timeout) {
    std::unique_lock lock(ackMutex_);
    if (!ackCondition_.wait_for(lock, timeout, [this] { return enableAckHeld_; }))
      throw std::runtime_error("core did not hold the requested ENABLE acknowledgement");
  }

  std::uint64_t heldEnableSequence() const {
    std::lock_guard lock(ackMutex_);
    require(heldEnableCommand_ && heldEnableCommand_->sequence,
            "held ENABLE command has no internal sequence");
    return *heldEnableCommand_->sequence;
  }

  void releaseEnableAck() {
    {
      std::lock_guard lock(ackMutex_);
      releaseEnableAck_ = true;
    }
    ackCondition_.notify_all();
  }

  raisin::robot::EeCommandResult replayHeldEnable() {
    raisin::raipal_ee::Command saved;
    {
      std::lock_guard lock(ackMutex_);
      require(heldEnableCommand_.has_value(), "no held ENABLE command to replay");
      saved = *heldEnableCommand_;
    }
    return bridge_.handleCommand(saved, Clock::now());
  }

  State state() const { return bridge_.state(); }

 private:
  void ingestTactileMessage(const std::string& sensor,
                            const TactileMessage::SharedPtr& message) {
    if (!message) return;
    try {
      bridge_.ingestTactileFrame(raisin::tactile::fromMessage(*message, sensor));
    } catch (const std::exception& error) {
      bridge_.reportTactileError(error.what());
    }
  }

  void handleDriveCommand(const StringService::Request::SharedPtr& request,
                          StringService::Response::SharedPtr response) {
    try {
      const auto command = raisin::raipal_ee::parseInternalCommand(request->data);
      const auto result = bridge_.handleCommand(command, Clock::now());
      bool holdAck = false;
      if (command.action == raisin::raipal_ee::Action::ENABLE) {
        std::lock_guard lock(ackMutex_);
        if (holdNextEnableAck_) {
          holdNextEnableAck_ = false;
          enableAckHeld_ = true;
          heldEnableCommand_ = command;
          holdAck = true;
          ackCondition_.notify_all();
        }
      }
      if (holdAck) {
        std::unique_lock lock(ackMutex_);
        ackCondition_.wait_for(lock, 3s, [this] { return releaseEnableAck_; });
      }
      response->success = result.success;
      response->message = result.message;
    } catch (const std::exception& error) {
      response->success = false;
      response->message = error.what();
    }
    publishState();
  }

  void publishLoop() {
    auto next = Clock::now();
    while (running_) {
      next += 20ms;
      const auto now = Clock::now();
      if (feedTactile_) {
        const auto stamp = monotonicNs(now);
        const auto sequence = ++frameSequence_;
        bridge_.ingestTactileFrame(tactileFrame("left", sequence, stamp));
        bridge_.ingestTactileFrame(tactileFrame("right", sequence, stamp));
      }
      bridge_.tick(now);
      publishState();
      std::this_thread::sleep_until(next);
    }
  }

  void publishState() {
    StateMessage message;
    message.data = raisin::raipal_ee::encodeState(bridge_.state());
    statePublisher_->publish(message);
  }

  raisin::robot::EeDriveBridge bridge_;
  std::shared_ptr<raisin::Publisher<StateMessage>> statePublisher_;
  std::shared_ptr<raisin::Service<StringService>> driveService_;
  std::shared_ptr<raisin::Subscriber<TactileMessage>> leftTactileSubscriber_;
  std::shared_ptr<raisin::Subscriber<TactileMessage>> rightTactileSubscriber_;
  mutable std::mutex ackMutex_;
  std::condition_variable ackCondition_;
  bool holdNextEnableAck_{};
  bool enableAckHeld_{};
  bool releaseEnableAck_{};
  std::optional<raisin::raipal_ee::Command> heldEnableCommand_;
  std::atomic<bool> running_{};
  std::atomic<bool> feedTactile_{false};
  std::thread worker_;
  std::uint64_t frameSequence_{};
};

class EnableAckReleaseGuard {
 public:
  explicit EnableAckReleaseGuard(CoreServiceNode& core) : core_(core) {}
  ~EnableAckReleaseGuard() { core_.releaseEnableAck(); }
  EnableAckReleaseGuard(const EnableAckReleaseGuard&) = delete;
  EnableAckReleaseGuard& operator=(const EnableAckReleaseGuard&) = delete;

 private:
  CoreServiceNode& core_;
};

class IntegrationClient final : public raisin::Node {
 public:
  IntegrationClient(const std::shared_ptr<raisin::Network>& network,
                    const std::string& nickname, const std::string& tactileTopicPrefix)
      : Node("ee_test_client_" + std::to_string(::getpid()), network) {
    commandTopic_ = nickname + "/ee/command";
    commandClient_ = createClient<StringService>(commandTopic_, nullptr, "main");
    stateSubscriber_ = createSubscriber<StateMessage>(nickname + "/ee/state", nullptr,
        [this](const StateMessage::SharedPtr& message) { receiveState(message); }, "main");
    leftTactilePublisher_ = createPublisher<TactileMessage>(tactileTopicPrefix + "/left", 5);
    rightTactilePublisher_ = createPublisher<TactileMessage>(tactileTopicPrefix + "/right", 5);
    require(commandClient_ != nullptr, "failed to create public EE command client");
    require(stateSubscriber_ != nullptr, "failed to subscribe to EE state");
    require(leftTactilePublisher_ != nullptr && rightTactilePublisher_ != nullptr,
            "failed to create tactile fixture publishers");
  }

  ~IntegrationClient() override { cleanupResources(); }

  void publishTactileFrame(const std::string& sensor, std::uint64_t sequence) {
    auto message = raisin::tactile::toMessage(
        tactileFrame(sensor, sequence, monotonicNs(Clock::now()), 2));
    auto& publisher = sensor == "left" ? leftTactilePublisher_ : rightTactilePublisher_;
    publisher->publish(message);
  }

  State waitForState(const std::function<bool(const State&)>& predicate,
                     std::chrono::milliseconds timeout = 2s) {
    std::unique_lock lock(mutex_);
    const auto deadline = Clock::now() + timeout;
    while (!state_ || !predicate(*state_)) {
      if (condition_.wait_until(lock, deadline) == std::cv_status::timeout) {
        std::ostringstream detail;
        detail << "timed out waiting for EE state";
        if (state_) {
          detail << " (sequence=" << state_->sequence
                 << ", command_sequence=" << state_->last_command_sequence
                 << ", enabled=" << state_->enabled
                 << ", tactile_ready=" << state_->tactile_ready
                 << ", error='" << state_->error << "', gates=";
          for (const auto& reason : state_->gate_reasons) detail << reason << ',';
          detail << ')';
        }
        if (!parseError_.empty()) detail << "; latest state parse error: " << parseError_;
        throw std::runtime_error(detail.str());
      }
    }
    return *state_;
  }

  State latestState() const {
    std::lock_guard lock(mutex_);
    require(state_.has_value(), "no EE state has arrived");
    return *state_;
  }

  void recreateCommandClient() {
    if (commandClient_) {
      removeClient(commandClient_);
      commandClient_.reset();
    }
    commandClient_ = createClient<StringService>(commandTopic_, nullptr, "main");
    require(commandClient_ != nullptr, "failed to recreate public EE command client");
  }

  auto asyncCommand(raisin::raipal_ee::Action action,
                    std::optional<std::array<double, 2>> target = std::nullopt) {
    raisin::raipal_ee::Command command;
    command.action = action;
    command.request_id = "integration-" + std::to_string(++requestSequence_);
    command.target_rad = target;
    auto request = std::make_shared<StringService::Request>();
    request->data = raisin::raipal_ee::encodePublicCommand(command);
    return commandClient_->asyncSendRequest(request);
  }

  StringService::Response::SharedPtr command(
      raisin::raipal_ee::Action action,
      std::optional<std::array<double, 2>> target = std::nullopt) {
    auto future = asyncCommand(action, target);
    if (future.wait_for(2s) != std::future_status::ready)
      throw std::runtime_error("public EE command service timed out");
    auto response = future.get();
    require(response != nullptr, "public EE command returned no response");
    return response;
  }

 private:
  void receiveState(const StateMessage::SharedPtr& message) {
    try {
      auto incoming = raisin::raipal_ee::parseState(message->data);
      std::lock_guard lock(mutex_);
      if (!state_ || incoming.sequence > state_->sequence) state_ = std::move(incoming);
      condition_.notify_all();
    } catch (const std::exception& error) {
      std::lock_guard lock(mutex_);
      parseError_ = error.what();
      condition_.notify_all();
    }
  }

  std::shared_ptr<raisin::Client<StringService>> commandClient_;
  std::shared_ptr<raisin::Subscriber<StateMessage>> stateSubscriber_;
  std::shared_ptr<raisin::Publisher<TactileMessage>> leftTactilePublisher_;
  std::shared_ptr<raisin::Publisher<TactileMessage>> rightTactilePublisher_;
  std::string commandTopic_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::optional<State> state_;
  std::string parseError_;
  std::uint64_t requestSequence_{};
};

void requirePublicSuccess(IntegrationClient& client, raisin::raipal_ee::Action action,
                          const std::string& phase,
                          std::optional<std::array<double, 2>> target = std::nullopt) {
  const auto response = client.command(action, target);
  if (response->success) return;
  const auto state = client.latestState();
  throw std::runtime_error(phase + " rejected: " + response->message +
      "; core sequence=" + std::to_string(state.last_command_sequence) +
      ", enabled=" + (state.enabled ? "true" : "false") +
      ", tactile_ready=" + (state.tactile_ready ? "true" : "false") +
      ", core_error='" + state.error + "'");
}

std::array<double, 8> visualJointState(raisim::ArticulatedSystemVisual* visual) {
  require(visual != nullptr, "EE preview visual is missing");
  std::lock_guard<raisim::ArticulatedSystemVisual> lock(*visual);
  const Eigen::VectorXd coordinates = visual->obj.getGeneralizedCoordinate().e();
  const auto coordinateOffset = coordinates.size() > visual->obj.getDOF() ? 1U : 0U;
  std::array<double, 8> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto index = visual->obj.getGeneralizedVelocityIndex(
        std::string(raisin::raipal_ee::kPreviewJointNames[i])) + coordinateOffset;
    require(index < static_cast<std::size_t>(coordinates.size()),
            "EE visual joint index exceeds generalized coordinates");
    result[i] = coordinates[static_cast<Eigen::Index>(index)];
  }
  return result;
}

void requirePreviewMatches(const std::array<double, 8>& actual,
                           const std::array<double, 2>& drive) {
  const auto expected = raisin::raipal_ee::mapDriveToPreview(drive);
  for (std::size_t i = 0; i < actual.size(); ++i)
    require(std::abs(actual[i] - expected[i]) < 0.01,
            "plugin visual joints do not match the drive-state preview mapping");
}

}  // namespace

namespace raisin {

std::string getInstallDirectory() {
  const char* prefix = std::getenv("RAISIN_TEST_INSTALL_PREFIX");
  if (prefix == nullptr) return {};
  std::string result(prefix);
  if (!result.empty() && result.back() != '/') result.push_back('/');
  return result;
}

}  // namespace raisin

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: ee_integration_test <staged-install-prefix> <sdk-encryption-library>\n";
    return 2;
  }
  const auto process = std::to_string(::getpid());
  const auto nickname = "ee_test_" + process;
  const auto pluginTitle = "ee_integration_" + process;
  const auto temporaryRoot = std::filesystem::temp_directory_path() /
      ("raipal_ee_test_" + process);
  RemoveTemporaryTree cleanup{temporaryRoot};
  try {
    std::filesystem::remove_all(temporaryRoot);
    std::filesystem::create_directories(temporaryRoot);
    const auto privateInstall = makePrivateInstall(
        std::filesystem::path(argv[1]), std::filesystem::path(argv[2]), temporaryRoot);
    ::setenv("RAISIN_TEST_INSTALL_PREFIX", privateInstall.c_str(), 1);
    raisin::raisinInit();

    raisim::World hub, sim;
    const std::filesystem::path armUrdf(RAISIN_TEST_ARM_URDF);
    require(std::filesystem::is_regular_file(armUrdf), "RAIPAL arm URDF test fixture is missing");
    hub.addArticulatedSystem(armUrdf.string())->setName("robot");
    sim.addArticulatedSystem(armUrdf.string())->setName("robot");
    raisim::RaisimServer hubServer(&hub), simServer(&sim);

    auto& resources = raisin::GlobalResource::getInstance();
    auto& root = raisin::parameter::ParameterContainer::getRoot();
    auto& raipal = root["Raipal"];
    raipal("robot_nickname") = nickname;
    raipal("hardware_configuration") =
        raisin::raipal_ee::encodeHardwareConfiguration(rightEeHardwareConfiguration());
    const auto tactileTopicPrefix = "tactile/v2/" + nickname;
    auto& tactilePluginConfig = root["raisin_tactile_plugin"];
    tactilePluginConfig.loadFromPackageParameterFile("raisin_tactile_plugin");
    tactilePluginConfig["usb"]("topic_prefix") = tactileTopicPrefix;
    resources.network = std::make_shared<raisin::Network>(
        nickname, "test",
        std::vector<std::vector<std::string>>{
            {"main"}, {"plugin"}, {"drive_service"}, {"core_sensor"}},
        std::vector<std::string>{"lo"});

    const auto homingPath = temporaryRoot / "ee.homing.yaml";
    CoreServiceNode core(resources.network, nickname, homingPath, tactileTopicPrefix);
    IntegrationClient client(resources.network, nickname, tactileTopicPrefix);
    core.start();
    client.waitForState([](const State& state) {
      return state.backend == raisin::raipal_ee::Backend::MOCK && !state.enabled;
    });

    raisin::plugin::PluginManager pluginManager(
        hub, hubServer, sim, simServer, "raisin_raipal", false, &raipal);
    require(pluginManager.load("raipal_ee", 50.0, "ee_integration_test", pluginTitle),
            "PluginManager failed to load the staged RAIPAL EE plugin");
    auto* plugin = hubServer.getVisualArticulatedSystem(pluginTitle + "_ee_right");
    require(plugin != nullptr, "loaded EE plugin did not attach its visual model");

    client.waitForState([](const State& state) {
      return state.backend == raisin::raipal_ee::Backend::MOCK &&
             state.last_command_sequence > 0;
    });
    for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
      client.publishTactileFrame("right", sequence);
      std::this_thread::sleep_for(20ms);
    }
    const auto rightOnly = client.waitForState([](const State& state) {
      return state.tactile[1].valid && state.tactile[1].baseline_ready &&
             !state.tactile[0].valid && !state.tactile_ready;
    });
    require(!rightOnly.enabled,
            "one valid right tactile pad must leave the EE motor disabled");
    const auto oneSidedEnable = client.command(raisin::raipal_ee::Action::ENABLE);
    require(!oneSidedEnable->success && !client.latestState().enabled,
            "one-sided tactile input must not enable the EE motors");
    core.resumeFeedingTactile();
    const auto tactileReadyState = client.waitForState([](const State& state) {
      return state.tactile_ready && state.tactile[0].valid && state.tactile[1].valid;
    });
    // The observer and plugin share the main executor. One newer state ensures
    // the plugin has consumed the prerequisite state before the next request.
    client.waitForState([&](const State& state) {
      return state.tactile_ready && state.tactile[0].valid && state.tactile[1].valid &&
             state.sequence > tactileReadyState.sequence;
    });
    requirePublicSuccess(client, raisin::raipal_ee::Action::SET_HOME,
                         "initial public set_home");
    const auto homedState = client.waitForState([](const State& state) {
      return state.homed;
    });
    client.waitForState([&](const State& state) {
      return state.homed && state.sequence > homedState.sequence;
    });
    requirePublicSuccess(client, raisin::raipal_ee::Action::ENABLE,
                         "initial public enable");
    const auto enabledState = client.waitForState([](const State& state) {
      return state.homed && state.enabled;
    });
    client.waitForState([&](const State& state) {
      return state.homed && state.enabled && state.sequence > enabledState.sequence;
    });
    const std::array<double, 2> target{0.10, 0.25};
    requirePublicSuccess(client, raisin::raipal_ee::Action::TARGET,
                         "public target", target);
    const auto moved = client.waitForState([&](const State& state) {
      return state.enabled && std::abs(state.axes[0].position_rad - target[0]) < 0.005 &&
             std::abs(state.axes[1].position_rad - target[1]) < 0.005;
    }, 2s);
    std::array<double, 8> visual{};
    const auto visualDeadline = Clock::now() + 1s;
    while (true) {
      visual = visualJointState(plugin);
      const auto expected = raisin::raipal_ee::mapDriveToPreview(
          {moved.axes[0].position_rad, moved.axes[1].position_rad});
      const bool matches = std::equal(visual.begin(), visual.end(), expected.begin(),
          [](double actual, double targetValue) { return std::abs(actual - targetValue) < 0.01; });
      if (matches) break;
      require(Clock::now() < visualDeadline,
              "plugin preview visual did not catch up to the settled core state");
      std::this_thread::sleep_for(10ms);
    }
    requirePreviewMatches(visual, {moved.axes[0].position_rad, moved.axes[1].position_rad});

    requirePublicSuccess(client, raisin::raipal_ee::Action::STOP, "public stop");
    client.waitForState([](const State& state) { return !state.enabled; });

    core.resumeFeedingTactile();
    client.waitForState([](const State& state) { return state.tactile_ready; });
    EnableAckReleaseGuard releaseHeldAck(core);
    core.holdNextEnableAck();
    auto delayedEnableResponse = client.asyncCommand(raisin::raipal_ee::Action::ENABLE);
    core.waitForHeldEnableAck(1s);
    const auto delayedEnableSequence = core.heldEnableSequence();
    client.waitForState([&](const State& state) {
      return state.enabled && state.last_command_sequence >= delayedEnableSequence;
    });
    require(delayedEnableResponse.wait_for(1s) == std::future_status::ready,
            "plugin did not return its bounded public response while core ACK was held");
    const auto delayedPublicResponse = delayedEnableResponse.get();
    require(delayedPublicResponse && !delayedPublicResponse->success &&
                delayedPublicResponse->message.find("timed out") != std::string::npos,
            "held core ACK should time out at the public plugin boundary");
    require(pluginManager.unload(pluginTitle), "PluginManager failed to unload the EE plugin");
    core.releaseEnableAck();
    const auto unloadedState = client.waitForState([&](const State& state) {
      return !state.enabled && state.error.empty() &&
             state.last_command_sequence > delayedEnableSequence;
    }, 500ms);
    const auto staleEnable = core.replayHeldEnable();
    require(!staleEnable.success && staleEnable.message.find("sequence") != std::string::npos &&
                !core.state().enabled,
            "delayed duplicate ENABLE must be rejected after unload Stop advanced the sequence");

    require(pluginManager.load("raipal_ee", 50.0, "ee_integration_test", pluginTitle),
            "PluginManager failed to reload the EE plugin");
    // Rebind to the new service header. The previous local client may still be
    // waiting on the removed service's shared-memory futex.
    client.recreateCommandClient();
    plugin = hubServer.getVisualArticulatedSystem(pluginTitle + "_ee_right");
    require(plugin != nullptr, "reloaded EE plugin did not recreate its visual model");

    const auto commandSequenceBeforeReload = client.latestState().last_command_sequence;
    client.waitForState([&](const State& state) {
      return state.last_command_sequence > commandSequenceBeforeReload;
    });

    requirePublicSuccess(client, raisin::raipal_ee::Action::ENABLE,
                         "reloaded plugin enable");
    client.waitForState([](const State& state) { return state.enabled; });
    core.stopFeedingTactile();
    const auto stale = client.waitForState([](const State& state) {
      return !state.enabled && !state.tactile_ready && !state.gate_reasons.empty();
    }, 2s);
    require(!stale.enabled, "stale bilateral tactile did not disable the EE bridge");
    const auto staleCommand = client.command(raisin::raipal_ee::Action::ENABLE);
    require(!staleCommand->success && staleCommand->message.find("tactile") != std::string::npos,
            "reloaded plugin did not reject a command while bilateral tactile was stale");

    core.resumeFeedingTactile();
    client.waitForState([](const State& state) { return state.tactile_ready; });
    require(pluginManager.unload(pluginTitle), "PluginManager failed to unload reloaded EE plugin");
    core.stop();
    std::cout << "PASS: PluginManager EE command, state, visual preview and safe lifecycle\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
