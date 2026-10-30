#include "raisin_raipal_ee/end_effector_plugin.hpp"

#include "raisin_raipal_ee/mechanics.hpp"
#include "raisin_raipal_ee/hardware_configuration.hpp"
#include "raisin_util/raisin_directories.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace raisin::raipal_ee {
namespace {

using Clock = std::chrono::steady_clock;

class WorldMutexGuard {
 public:
  explicit WorldMutexGuard(raisim::World& world) : world_(world) { world_.lockMutex(); }
  ~WorldMutexGuard() { world_.unlockMutex(); }

  WorldMutexGuard(const WorldMutexGuard&) = delete;
  WorldMutexGuard& operator=(const WorldMutexGuard&) = delete;

 private:
  raisim::World& world_;
};

class VisualMutexGuard {
 public:
  explicit VisualMutexGuard(raisim::ArticulatedSystemVisual& visual) : visual_(visual) {
    visual_.lockMutex();
  }
  ~VisualMutexGuard() { visual_.unlockMutex(); }

  VisualMutexGuard(const VisualMutexGuard&) = delete;
  VisualMutexGuard& operator=(const VisualMutexGuard&) = delete;

 private:
  raisim::ArticulatedSystemVisual& visual_;
};

Eigen::Vector3d readVector3(const YAML::Node& node, const char* name) {
  const auto values = node[name];
  if (!values.IsSequence() || values.size() != 3)
    throw std::invalid_argument(std::string("mount ") + name + " must have three values");
  Eigen::Vector3d result(values[0].as<double>(), values[1].as<double>(), values[2].as<double>());
  if (!result.allFinite()) throw std::invalid_argument(std::string("mount ") + name + " is not finite");
  return result;
}

Eigen::Matrix3d rpyRotation(const Eigen::Vector3d& rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

std::string reasonsText(const State& state) {
  std::string result;
  for (const auto& reason : state.gate_reasons) {
    if (!result.empty()) result += ", ";
    result += reason;
  }
  return result;
}

void setVisualState(raisim::ArticulatedSystemVisual* visual,
                    const Eigen::Vector3d& mountPosition,
                    const Eigen::Matrix3d& mountRotation,
                    const Eigen::Vector3d& flangePosition,
                    const Eigen::Matrix3d& flangeRotation,
                    const std::array<double, 8>& preview, bool stale) {
  if (!visual) return;
  VisualMutexGuard visualLock(*visual);
  const auto& articulated = visual->obj;
  Eigen::VectorXd coordinates = articulated.getGeneralizedCoordinate().e();
  const auto dof = static_cast<std::size_t>(articulated.getDOF());
  for (std::size_t i = 0; i < kPreviewJointNames.size(); ++i) {
    const auto velocityIndex = static_cast<std::size_t>(articulated.getGeneralizedVelocityIndex(
        std::string(kPreviewJointNames[i])));
    const auto coordinateIndex = generalizedCoordinateIndex(
        velocityIndex, static_cast<std::size_t>(coordinates.size()), dof);
    coordinates[static_cast<Eigen::Index>(coordinateIndex)] = preview[i];
  }
  visual->obj.setGeneralizedCoordinate(coordinates);
  visual->obj.setBasePos_e(flangePosition + flangeRotation * mountPosition);
  visual->obj.setBaseOrientation_e(flangeRotation * mountRotation);
  visual->setColor(stale ? 1. : 0., stale ? 0.3 : 0., stale ? 0.1 : 0., stale ? 0.8 : 0.);
}

}  // namespace

EndEffectorPlugin::EndEffectorPlugin(raisim::World& hub, raisim::RaisimServer& server,
                                     raisim::World& sim, raisim::RaisimServer& simServer,
                                     GlobalResource& resource, const std::string& owner,
                                     const std::string& title, bool real)
    : plugin::Plugin(hub, server, sim, simServer, resource, owner, title, real),
      Node("raisin_raipal_ee_plugin", resource.paramRoot, resource.network),
      robotNickname_(paramRoot_["Raipal"].as_string("robot_nickname")),
      visualNamePrefix_(title.empty() ? "raipal_ee" : title) {}

EndEffectorPlugin::~EndEffectorPlugin() {
  sendUnloadStop();
  cleanupResources();
  publicService_.reset();
  stateSubscriber_.reset();
  coreClient_.reset();
  stopClient_.reset();
  if (leftVisual_) serverHub_.removeVisualArticulatedSystem(leftVisual_);
  if (rightVisual_) serverHub_.removeVisualArticulatedSystem(rightVisual_);
}

bool EndEffectorPlugin::init() {
  try {
    if (robotNickname_.empty()) throw std::invalid_argument("robot_nickname is required");
    const auto timeoutMs = param_->as_int("response_timeout_ms");
    const auto stateTimeoutMs = param_->as_int("state_timeout_ms");
    const auto heartbeatMs = param_->as_int("heartbeat_period_ms");
    auto& raipalParameters = paramRoot_["Raipal"];
    std::optional<HardwareConfiguration> hardware;
    if (raipalParameters.getParamList().contains("hardware_configuration")) {
      hardware = parseHardwareConfiguration(
          raipalParameters.as_string("hardware_configuration"));
    }
    // The live RAIPAL snapshot owns the mount choice. The package parameter is
    // used only by standalone legacy tests that have no node snapshot.
    mountMode_ = resolveVisualMountMode(
        hardware, hardware ? std::string{} : param_->as_string("mount_mode"));
    if (timeoutMs <= 0 || timeoutMs > 1000 || stateTimeoutMs <= 0 ||
        heartbeatMs <= 0 || heartbeatMs >= 500)
      throw std::invalid_argument("invalid EE bridge timeout parameters");
    if (mountMode_ != "single_L" && mountMode_ != "single_R")
      throw std::invalid_argument("mount_mode must be single_L or single_R");
    responseTimeout_ = std::chrono::milliseconds(timeoutMs);
    stateTimeout_ = std::chrono::milliseconds(stateTimeoutMs);
    heartbeatPeriod_ = std::chrono::milliseconds(heartbeatMs);
    topicPrefix_ = robotNickname_ + "/ee";

    coreClient_ = createClient<StringService>(topicPrefix_ + "/drive_command", nullptr, "main");
    stopClient_ = createClient<StringService>(topicPrefix_ + "/drive_command", nullptr, "main");
    stateSubscriber_ = createSubscriber<StateMessage>(topicPrefix_ + "/state", nullptr,
        [this](const StateMessage::SharedPtr message) { receiveState(message); }, "main");
    publicService_ = createService<StringService>(topicPrefix_ + "/command",
        [this](const StringService::Request::SharedPtr request,
               StringService::Response::SharedPtr response) {
          handlePublicCommand(request, response);
        }, "plugin");
    if (!coreClient_ || !stopClient_ || !stateSubscriber_ || !publicService_)
      throw std::runtime_error("failed to register EE bridge endpoints");

    resourceDirectory_ = getResourceDirectory("raisin_raipal_ee");
    const auto modelPath = resourceDirectory_ + "ee_visual.urdf";
    const auto mountPath = resourceDirectory_ + "ee_mounts.yaml";
    if (!std::filesystem::is_regular_file(modelPath) || !std::filesystem::is_regular_file(mountPath))
      throw std::runtime_error("EE visual model or mount metadata is missing from package resources");
    const auto mounts = YAML::LoadFile(mountPath)["mounts"];
    const auto leftMount = mounts["left"];
    const auto rightMount = mounts["right"];
    if (leftMount["parent_link"].as<std::string>() != "L_ee" ||
        rightMount["parent_link"].as<std::string>() != "R_ee")
      throw std::invalid_argument("EE visual mounts must attach to RAIPAL L_ee and R_ee");
    leftMountPosition_ = readVector3(leftMount, "xyz_m");
    rightMountPosition_ = readVector3(rightMount, "xyz_m");
    leftMountRotation_ = rpyRotation(readVector3(leftMount, "rpy_rad"));
    rightMountRotation_ = rpyRotation(readVector3(rightMount, "rpy_rad"));

    if (!robotHub_) throw std::runtime_error("RAIPAL arm model is unavailable for EE preview");
    const auto hasLink = [this](const std::string& name) {
      return std::any_of(robotHub_->getFrames().begin(), robotHub_->getFrames().end(),
          [&name](const raisim::CoordinateFrame& frame) { return frame.bodyName == name; });
    };
    const auto selectedLink = mountMode_ == "single_L" ? "L_ee" : "R_ee";
    if (!hasLink(selectedLink))
      throw std::runtime_error(std::string("RAIPAL arm model does not expose selected flange ") +
                               selectedLink);

    if (mountMode_ == "single_L")
      leftVisual_ = serverHub_.addVisualArticulatedSystem(
          visualNamePrefix_ + "_ee_left", modelPath);
    if (mountMode_ == "single_R")
      rightVisual_ = serverHub_.addVisualArticulatedSystem(
          visualNamePrefix_ + "_ee_right", modelPath);
    for (const auto* visual : {leftVisual_, rightVisual_}) {
      if (!visual) continue;
      for (const auto jointName : kPreviewJointNames)
        (void)visual->obj.getGeneralizedVelocityIndex(std::string(jointName));
    }
    State initialState;
    {
      std::lock_guard lock(stateMutex_);
      initialState = state_;
    }
    updateVisuals(initialState, true);
    return true;
  } catch (const std::exception& error) {
    RSWARN("RAIPAL EE bridge init: " << error.what());
    return false;
  }
}

bool EndEffectorPlugin::advance() {
  State current;
  bool haveVisualState = false;
  bool haveState = false;
  Clock::time_point visualReceived{};
  Clock::time_point received{};
  {
    std::lock_guard lock(stateMutex_);
    haveState = hasState_;
    if (haveState) received = stateReceived_;
    haveVisualState = hasVisualState_;
    if (haveVisualState) {
      current = visualState_;
      visualReceived = visualStateReceived_;
    }
  }
  try {
    updateVisuals(current, !haveVisualState || !stateIsFresh(visualReceived));
  } catch (const std::exception& error) {
    RSWARN("RAIPAL EE visual update: " << error.what());
    return false;
  }

  const auto now = Clock::now();
  std::lock_guard commandLock(commandMutex_);
  if (pendingStopResponse_) {
    if (pendingStopResponse_->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
      (void)pollStopResponse();
    else
      return true;
  }
  if (pendingCoreResponse_) {
    if (pendingCoreResponse_->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
      (void)pollCoreResponse();
    else
      return true;
  }
  if (!haveState || !stateIsFresh(received) || !sequenceSynchronized_.load() ||
      !coreClient_ || !coreClient_->isServiceAvailable() ||
      (lastHeartbeat_ != Clock::time_point{} && now - lastHeartbeat_ < heartbeatPeriod_))
    return true;

  internalSequence_ = std::max(internalSequence_, latestCoreSequence_.load());
  Command heartbeat;
  heartbeat.action = Action::HEARTBEAT;
  auto request = std::make_shared<StringService::Request>();
  request->data = encodeInternalCommand(heartbeat, ++internalSequence_);
  pendingCoreResponse_ = coreClient_->asyncSendRequest(request);
  lastHeartbeat_ = now;
  return true;
}

bool EndEffectorPlugin::reset() { return true; }

void EndEffectorPlugin::handlePublicCommand(const StringService::Request::SharedPtr request,
                                            StringService::Response::SharedPtr response) {
  try {
    if (!request || !response) return;
    const Command command = parsePublicCommand(request->data);
    if (command.action == Action::HEARTBEAT) {
      response->success = false;
      response->message = "heartbeat is reserved for the EE bridge";
      return;
    }

    State current;
    std::chrono::steady_clock::time_point received;
    bool hasState = false;
    std::string stateError;
    {
      std::lock_guard lock(stateMutex_);
      current = state_;
      received = stateReceived_;
      hasState = hasState_;
      stateError = stateError_;
    }
    const bool stop = command.action == Action::STOP;
    if (!stop && (!hasState || !stateIsFresh(received))) {
      response->success = false;
      response->message = stateError.empty() ? "EE state is missing or stale" : stateError;
      return;
    }
    if (!stop && current.backend == Backend::DISABLED) {
      response->success = false;
      response->message = "EE drive backend is disabled";
      return;
    }
    if (!stop && (!current.actuation_qualified ||
                  !current.axes[0].calibrated_valid ||
                  !current.axes[1].calibrated_valid)) {
      response->success = false;
      response->message = "EE feedback calibration is not qualified";
      return;
    }
    if (!stop && !tactilePairReady(current)) {
      response->success = false;
      response->message = "both tactile sensors must be valid, baselined and fresh";
      const auto reasons = reasonsText(current);
      if (!reasons.empty()) response->message += ": " + reasons;
      return;
    }
    if (command.action == Action::TARGET) {
      if (!current.enabled || !current.homed) {
        response->success = false;
        response->message = "EE must be enabled and homed before targeting";
        return;
      }
      for (std::size_t i = 0; i < 2; ++i) {
        const double target = (*command.target_rad)[i];
        if (target < current.axes[i].lower_limit_rad || target > current.axes[i].upper_limit_rad) {
          response->success = false;
          response->message = "target is outside the current EE axis limits";
          return;
        }
      }
    }

    if (!sendCoreCommand(command, *response) && response->message.empty())
      response->message = "EE core command was not accepted";
  } catch (const std::exception& error) {
    response->success = false;
    response->message = error.what();
  }
}

void EndEffectorPlugin::receiveState(const StateMessage::SharedPtr& message) {
  try {
    if (!message) return;
    State incoming = parseState(message->data);
    {
      std::lock_guard lock(stateMutex_);
      if (hasState_ && incoming.sequence <= state_.sequence) return;
      const auto received = Clock::now();
      state_ = incoming;
      stateReceived_ = received;
      stateError_.clear();
      hasState_ = true;
      if (hasQualifiedVisualFeedback(incoming)) {
        visualState_ = incoming;
        visualStateReceived_ = received;
        hasVisualState_ = true;
      }
    }
    auto sequence = latestCoreSequence_.load();
    while (sequence < incoming.last_command_sequence &&
           !latestCoreSequence_.compare_exchange_weak(sequence, incoming.last_command_sequence)) {}
    sequenceSynchronized_.store(true);
  } catch (const std::exception& error) {
    std::lock_guard lock(stateMutex_);
    hasState_ = false;
    stateError_ = std::string("invalid EE state: ") + error.what();
  }
}

bool EndEffectorPlugin::sendCoreCommand(const Command& command,
                                         StringService::Response& response) {
  std::lock_guard commandLock(commandMutex_);
  const bool isStop = command.action == Action::STOP;
  const auto deadline = Clock::now() + responseTimeout_;
  auto& client = isStop ? stopClient_ : coreClient_;
  auto& pending = isStop ? pendingStopResponse_ : pendingCoreResponse_;

  if (pendingStopResponse_) {
    if (pendingStopResponse_->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
      (void)pollStopResponse();
    else {
      response.success = false;
      response.message = "previous EE stop response is still pending";
      return false;
    }
  }
  if (!client || !client->isServiceAvailable()) {
    response.success = false;
    response.message = "EE core drive service is unavailable";
    return false;
  }
  if (!isStop && pendingCoreResponse_) {
    if (pendingCoreResponse_->wait_for(responseTimeout_) != std::future_status::ready) {
      response.success = false;
      response.message = "previous EE core response is still pending";
      return false;
    }
    (void)pollCoreResponse();
  }
  if (!sequenceSynchronized_.load() && !isStop) {
    response.success = false;
    response.message = "waiting for EE core sequence state before forwarding commands";
    return false;
  }

  internalSequence_ = std::max(internalSequence_, latestCoreSequence_.load());
  const auto sequence = ++internalSequence_;
  auto request = std::make_shared<StringService::Request>();
  request->data = encodeInternalCommand(command, sequence);
  auto future = client->asyncSendRequest(request);
  const auto remaining = deadline > Clock::now()
      ? deadline - Clock::now() : Clock::duration::zero();
  if (future.wait_for(remaining) != std::future_status::ready) {
    pending = future;
    response.success = false;
    response.message = "EE core response timed out; sequence=" + std::to_string(sequence);
    return false;
  }
  const auto coreResponse = future.get();
  if (!coreResponse) {
    response.success = false;
    response.message = "EE core returned no command response";
    return false;
  }
  response.success = coreResponse->success;
  response.message = (coreResponse->success ? "accepted" : "rejected") +
      std::string(" sequence=") + std::to_string(sequence) + "; " + coreResponse->message;
  return response.success;
}

void EndEffectorPlugin::sendUnloadStop() {
  std::lock_guard lock(commandMutex_);
  if (pendingStopResponse_) {
    if (pendingStopResponse_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
      return;
    (void)pollStopResponse();
  }
  if (!stopClient_ || !stopClient_->isServiceAvailable())
    return;
  internalSequence_ = std::max(internalSequence_, latestCoreSequence_.load());
  Command stop;
  stop.action = Action::STOP;
  auto request = std::make_shared<StringService::Request>();
  request->data = encodeInternalCommand(stop, ++internalSequence_);
  pendingStopResponse_ = stopClient_->asyncSendRequest(request);
}

bool EndEffectorPlugin::pollCoreResponse() {
  if (!pendingCoreResponse_ ||
      pendingCoreResponse_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return false;
  try {
    (void)pendingCoreResponse_->get();
  } catch (...) {
  }
  pendingCoreResponse_.reset();
  return true;
}

bool EndEffectorPlugin::pollStopResponse() {
  if (!pendingStopResponse_ ||
      pendingStopResponse_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    return false;
  try {
    (void)pendingStopResponse_->get();
  } catch (...) {
  }
  pendingStopResponse_.reset();
  return true;
}

void EndEffectorPlugin::updateVisuals(const State& state, bool stale) {
  const auto preview = mapDriveToPreviewForMount(
      {state.axes[0].position_rad, state.axes[1].position_rad}, mountMode_);
  Eigen::Vector3d flangePosition;
  Eigen::Matrix3d flangeRotation;
  {
    WorldMutexGuard worldLock(worldHub_);
    const std::string linkName = mountMode_ == "single_L" ? "L_ee" : "R_ee";
    const auto frameIndex = robotHub_->getFrameIdxByLinkName(linkName);
    const auto& frames = robotHub_->getFrames();
    if (frameIndex >= frames.size() || frames[frameIndex].bodyName != linkName)
      throw std::runtime_error("RAIPAL flange frame index is missing or out of range");
    raisim::Vec<3> position;
    raisim::Mat<3, 3> orientation;
    robotHub_->getFramePosition(frameIndex, position);
    robotHub_->getFrameOrientation(frameIndex, orientation);
    flangePosition = position.e();
    flangeRotation = orientation.e();
  }

  if (leftVisual_) {
    setVisualState(leftVisual_, leftMountPosition_, leftMountRotation_, flangePosition,
                   flangeRotation, preview, stale);
  }
  if (rightVisual_) {
    setVisualState(rightVisual_, rightMountPosition_, rightMountRotation_, flangePosition,
                   flangeRotation, preview, stale);
  }
}

bool EndEffectorPlugin::stateIsFresh(Clock::time_point received) const {
  return visualStateIsFresh(received, Clock::now(), stateTimeout_);
}

bool EndEffectorPlugin::tactilePairReady(const State& state) {
  if (!state.tactile_ready) return false;
  return std::all_of(state.tactile.begin(), state.tactile.end(), [](const TactileState& tactile) {
    return tactile.valid && tactile.baseline_ready && tactile.age_ms >= 0. &&
           std::isfinite(tactile.age_ms);
  });
}

}  // namespace raisin::raipal_ee

extern "C" raisin::plugin::Plugin* create_with_owner_title_robot(
    raisim::World& hub, raisim::RaisimServer& server, raisim::World& sim,
    raisim::RaisimServer& simServer, raisin::GlobalResource& resource,
    const std::string& owner, const std::string& title, bool real) {
  return new raisin::raipal_ee::EndEffectorPlugin(
      hub, server, sim, simServer, resource, owner, title, real);
}

extern "C" raisin::plugin::Plugin* create(
    raisim::World& hub, raisim::RaisimServer& server, raisim::World& sim,
    raisim::RaisimServer& simServer, raisin::GlobalResource& resource) {
  return new raisin::raipal_ee::EndEffectorPlugin(
      hub, server, sim, simServer, resource, "", "raipal_ee", false);
}

extern "C" void destroy(raisin::plugin::Plugin* plugin) { delete plugin; }
