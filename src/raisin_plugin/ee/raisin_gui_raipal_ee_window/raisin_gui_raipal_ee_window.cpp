#include "raisin_gui_raipal_ee_window/raisin_gui_raipal_ee_window.hpp"
#include "raisin_raipal_ee/mechanics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace raisin {

RaipalEeWindow::RaipalEeWindow(const std::string& title,
                               std::shared_ptr<GuiResource> resource)
    : GuiWindow(title.empty() ? "RAIPAL EE" : title, resource),
      Node("raisin_gui_raipal_ee_window", resource->paramRoot, resource->network) {}

RaipalEeWindow::~RaipalEeWindow() { shutDown(); }

bool RaipalEeWindow::init() {
  try {
    robotNickname_ = param_->as_string("robot_nickname");
    const int timeoutMs = param_->as_int("state_timeout_ms");
    const int responseTimeoutMs = param_->as_int("response_timeout_ms");
    if (robotNickname_.empty() || timeoutMs <= 0 || responseTimeoutMs <= 0)
      throw std::invalid_argument("robot_nickname and positive state/response timeouts are required");
    stateTimeout_ = std::chrono::milliseconds(timeoutMs);
    responseTimeout_ = std::chrono::milliseconds(responseTimeoutMs);

    hardwareConfigurationClient_ = createClient<StringService>(
        robotNickname_ + "/hardware_configuration", resource_->connection, "main");
    if (!hardwareConfigurationClient_)
      throw std::runtime_error("failed to register hardware configuration client");
    initialized_ = true;
    hardwareConfigurationResolved_ = false;
    hardwareConfigurationError_.clear();
    return true;
  } catch (const std::exception& error) {
    initializationError_ = error.what();
    return false;
  }
}
bool RaipalEeWindow::update() {
  pollHardwareConfiguration();
  const auto collect = [this](
      std::optional<Client<StringService>::SharedFuture>& pending,
      Clock::time_point sentAt, bool& timedOut, const std::string& label) {
    if (!pending) return;
    if (pending->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      try {
        const auto response = pending->get();
        commandFeedback_ = label + (response ? response->message : "no EE command response");
        if (!response || !response->success) commandFeedback_ = "Error: " + commandFeedback_;
      } catch (const std::exception& error) {
        commandFeedback_ = "Error: " + std::string(error.what());
      }
      pending.reset();
      timedOut = false;
      return;
    }
    if (!timedOut && Clock::now() - sentAt >= responseTimeout_) {
      commandFeedback_ = label + "response timed out; request remains pending. Reload the EE plugin before retrying.";
      timedOut = true;
    }
  };
  collect(pendingResponse_, commandSentAt_, commandTimedOut_, "");
  collect(pendingStopResponse_, stopSentAt_, stopTimedOut_, "Stop: ");
  return true;
}

void RaipalEeWindow::pollHardwareConfiguration() {
  if (hardwareConfigurationResolved_ || !hardwareConfigurationClient_) return;
  if (!hardwareConfigurationFuture_.valid()) {
    if (!hardwareConfigurationClient_->isServiceAvailable()) return;
    auto request = std::make_shared<StringService::Request>();
    request->data.clear();
    try {
      hardwareConfigurationFuture_ = hardwareConfigurationClient_->asyncSendRequest(request);
      hardwareConfigurationRequestAt_ = Clock::now();
    } catch (const std::exception& error) {
      std::lock_guard lock(stateMutex_);
      hardwareConfigurationError_ = error.what();
      return;
    }
  }
  if (hardwareConfigurationFuture_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready) {
    if (Clock::now() - hardwareConfigurationRequestAt_ >= responseTimeout_) {
      std::lock_guard lock(stateMutex_);
      hardwareConfigurationError_ = "Waiting for hardware configuration snapshot.";
    }
    return;
  }

  try {
    const auto response = hardwareConfigurationFuture_.get();
    hardwareConfigurationFuture_ = {};
    if (!response || !response->success)
      throw std::runtime_error(response ? response->message : "no configuration response");
    auto configuration = raisin::raipal_ee::parseHardwareConfiguration(response->message);
    if (configuration.schema_version != 1)
      throw std::runtime_error("unsupported hardware configuration schema_version");
    const bool eePresent = configuration.ee_present;
    {
      std::lock_guard lock(stateMutex_);
      hardwareConfiguration_ = std::move(configuration);
      hardwareConfigurationResolved_ = true;
      hardwareConfigurationError_.clear();
    }
    open = eePresent;

    if (eePresent) {
      std::optional<raisin::raipal_ee::HardwareConfiguration> config;
      {
        std::lock_guard lock(stateMutex_);
        config = hardwareConfiguration_;
      }
      if (!config || !config->ee_communication) return;
      const auto prefix = robotNickname_ + "/ee";
      stateSubscriber_ = createSubscriber<StateMessage>(prefix + "/state", resource_->connection,
          [this](const StateMessage::SharedPtr& message) { receiveState(message); }, "main");
      commandClient_ = createClient<StringService>(prefix + "/command", resource_->connection, "main");
      stopClient_ = createClient<StringService>(prefix + "/command", resource_->connection, "main");
      if (!stateSubscriber_ || !commandClient_ || !stopClient_)
        throw std::runtime_error("failed to register RAIPAL EE endpoints");
    }
  } catch (const std::exception& error) {
    hardwareConfigurationFuture_ = {};
    std::lock_guard lock(stateMutex_);
    hardwareConfigurationResolved_ = true;
    hardwareConfigurationError_ = std::string("Invalid hardware configuration: ") + error.what();
  }
}
bool RaipalEeWindow::draw() {
  if (!open) return true;

  std::optional<raisin::raipal_ee::State> state;
  std::optional<raisin::raipal_ee::HardwareConfiguration> configuration;
  Clock::time_point received;
  std::string stateError;
  std::string configurationError;
  {
    std::lock_guard lock(stateMutex_);
    state = state_;
    received = stateReceived_;
    stateError = stateError_;
    configuration = hardwareConfiguration_;
    configurationError = hardwareConfigurationError_;
  }
  const bool hasState = state.has_value();
  const auto now = Clock::now();
  const bool fresh = hasState && received != Clock::time_point{} && now - received <= stateTimeout_;
  const bool tactileReady = fresh && state->tactile_ready &&
      state->tactile[0].valid && state->tactile[0].baseline_ready && state->tactile[0].age_ms >= 0. &&
      state->tactile[1].valid && state->tactile[1].baseline_ready && state->tactile[1].age_ms >= 0.;
  const bool axesQualified = fresh && state->homed && state->actuation_qualified &&
      std::all_of(state->axes.begin(), state->axes.end(), [](const auto& axis) {
        return axis.calibrated_valid && std::isfinite(axis.position_rad) &&
               std::isfinite(axis.lower_limit_rad) && std::isfinite(axis.upper_limit_rad);
      });
  const bool commandReady = tactileReady &&
      state->backend != raisin::raipal_ee::Backend::DISABLED;
  const bool targetReady = commandReady && axesQualified;
  const bool commandPending = pendingResponse_.has_value() || pendingStopResponse_.has_value();

  if (ImGui::Begin(title.c_str(), &open)) {
    if (!initialized_) {
      ImGui::TextWrapped("RAIPAL EE GUI setup failed: %s", initializationError_.c_str());
    } else if (!hardwareConfigurationResolved_) {
      ImGui::Text("Robot: %s | loading hardware configuration", robotNickname_.c_str());
      if (!configurationError.empty()) ImGui::TextWrapped("%s", configurationError.c_str());
    } else if (!configuration) {
      ImGui::TextWrapped("Hardware configuration unavailable: %s", configurationError.c_str());
    } else if (!configuration->ee_present) {
      ImGui::Text("Profile %s has no EE; this window is hidden.",
                  configuration->hardware_profile.c_str());
      open = false;
    } else {
      ImGui::Text("Robot: %s | profile: %s | EE: %s | mount: %s",
          robotNickname_.c_str(), configuration->hardware_profile.c_str(),
          configuration->ee_communication ? "communication configured" : "communication disabled",
          configuration->ee_mount_mode.c_str());
      if (configuration->ee_communication) {
        ImGui::BeginDisabled(!stopClient_ || pendingStopResponse_.has_value());
        if (ImGui::Button("Stop EE", ImVec2(90.F, 0.F)))
          sendCommand(raisin::raipal_ee::Command{.action = raisin::raipal_ee::Action::STOP}, true);
        ImGui::EndDisabled();
      }
      if (!configuration->ee_communication)
        ImGui::TextUnformatted("EE is present; communication is disabled. No controls are available.");
      if (!hasState) {
        ImGui::TextUnformatted(stateError.empty() ? "Waiting for EE state." : stateError.c_str());
      } else {
        ImGui::Text("State: %s | age %.0f ms | stream seq %llu | command seq %llu",
            fresh ? "fresh" : "STALE",
            std::chrono::duration<double, std::milli>(now - received).count(),
            static_cast<unsigned long long>(state->sequence),
            static_cast<unsigned long long>(state->last_command_sequence));
        ImGui::Text("Backend: %s | enabled: %s | homed: %s | actuation qualified: %s | tactile pair: %s",
            raisin::raipal_ee::toString(state->backend).c_str(),
            state->enabled ? "yes" : "no", state->homed ? "yes" : "no",
            state->actuation_qualified ? "yes" : "no", tactileReady ? "ready" : "not ready");
        if (!state->error.empty()) ImGui::TextWrapped("Core error: %s", state->error.c_str());
        if (!stateError.empty()) ImGui::TextWrapped("State error: %s", stateError.c_str());
        for (const auto& reason : state->gate_reasons)
          ImGui::BulletText("Gate: %s", reason.c_str());

        const bool canInitializeTargets = axesQualified;
        if (!targetInitialized_ && canInitializeTargets) {
          targetRad_[0] = state->axes[0].position_rad;
          targetRad_[1] = state->axes[1].position_rad;
          targetInitialized_ = true;
        }

        ImGui::SeparatorText("Two EE drive axes");
        for (std::size_t i = 0; i < state->axes.size(); ++i) {
          const auto& axis = state->axes[i];
          if (axis.raw_valid)
            ImGui::Text("Axis %zu | raw encoder count %d | calibrated %s", i,
                axis.raw_encoder_count, axis.calibrated_valid ? "yes" : "no");
          else
            ImGui::Text("Axis %zu | raw encoder count unavailable | calibrated %s", i,
                axis.calibrated_valid ? "yes" : "no");
          if (axis.position_valid || (state->homed && axis.calibrated_valid))
            ImGui::Text("  position %.5f rad", axis.position_rad);
          else
            ImGui::TextUnformatted("  Position calibration missing; displaying raw encoder counts.");
          if (axis.calibrated_valid)
            ImGui::Text("  vel %.5f rad/s | torque %.4f Nm", axis.velocity_rad_s, axis.torque_nm);
          if (axis.calibrated_valid)
            ImGui::Text("  target %.5f rad", axis.target_rad);
          ImGui::Text("  status 0x%04x | error 0x%04x", axis.status_word, axis.error_code);
        }

        ImGui::SeparatorText("Tactile interlock");
        for (const auto& tactile : state->tactile) {
          ImGui::Text("%s: valid %s | baseline %s | core age %.1f ms | seq %llu",
              tactile.name.c_str(), tactile.valid ? "yes" : "no",
              tactile.baseline_ready ? "ready" : "missing", tactile.age_ms,
              static_cast<unsigned long long>(tactile.sequence));
        }

        if (configuration->ee_communication) {
          ImGui::SeparatorText("EE controls");
          ImGui::BeginDisabled(!commandClient_ || !commandReady || !axesQualified ||
                               commandPending || state->enabled);
          if (ImGui::Button("Enable EE"))
            sendCommand(raisin::raipal_ee::Command{.action = raisin::raipal_ee::Action::ENABLE});
          ImGui::EndDisabled();
          ImGui::BeginDisabled(!commandClient_ || !commandReady || commandPending || state->enabled);
          if (ImGui::Button("Set EE home"))
            sendCommand(raisin::raipal_ee::Command{.action = raisin::raipal_ee::Action::SET_HOME});
          ImGui::EndDisabled();
          bool targetInLimits = targetReady && targetInitialized_;
          for (std::size_t i = 0; i < state->axes.size(); ++i) {
            const auto& axis = state->axes[i];
            ImGui::BeginDisabled(!targetReady || !targetInitialized_ || commandPending);
            const auto label = "Axis " + std::to_string(i) + " target (rad)";
            ImGui::InputDouble(label.c_str(), &targetRad_[i], 0.01, 0.05, "%.5f");
            const bool valid = axis.calibrated_valid && std::isfinite(targetRad_[i]) &&
                targetRad_[i] >= axis.lower_limit_rad && targetRad_[i] <= axis.upper_limit_rad;
            targetInLimits = targetInLimits && valid;
            ImGui::EndDisabled();
            if (!valid) ImGui::TextColored(ImVec4(1.F, 0.35F, 0.25F, 1.F),
                                          "Target exceeds current axis limits.");
          }
          ImGui::BeginDisabled(!targetReady || commandPending || !state->enabled ||
                               !state->homed || !targetInLimits);
          if (ImGui::Button("Send target")) {
            raisin::raipal_ee::Command command;
            command.action = raisin::raipal_ee::Action::TARGET;
            command.target_rad = std::array<double, 2>{targetRad_[0], targetRad_[1]};
            sendCommand(std::move(command));
          }
          ImGui::EndDisabled();
        }
        ImGui::TextUnformatted(fresh && raisin::raipal_ee::hasQualifiedVisualFeedback(*state)
            ? "Display: calibrated positions drive the integrated EE mimic joints."
            : "Display waits for fresh position calibration; motor authorization is separate.");
      }
      if ((commandPending && !commandTimedOut_) ||
          (pendingStopResponse_ && !stopTimedOut_))
        ImGui::TextUnformatted("Command pending...");
      if (commandTimedOut_ || stopTimedOut_)
        ImGui::TextUnformatted("Timed-out request is retained; unload/reload the EE plugin before retrying.");
      if (!commandFeedback_.empty()) ImGui::TextWrapped("%s", commandFeedback_.c_str());
    }
  }
  ImGui::End();
  return open;
}
bool RaipalEeWindow::shutDown() {
  cleanupResources();
  stateSubscriber_.reset();
  commandClient_.reset();
  stopClient_.reset();
  hardwareConfigurationClient_.reset();
  hardwareConfigurationFuture_ = {};
  pendingResponse_.reset();
  pendingStopResponse_.reset();
  initialized_ = false;
  return true;
}

bool RaipalEeWindow::reset() {
  std::lock_guard lock(stateMutex_);
  state_.reset();
  stateReceived_ = {};
  stateError_.clear();
  hardwareConfiguration_.reset();
  hardwareConfigurationResolved_ = false;
  hardwareConfigurationError_.clear();
  hardwareConfigurationRequestAt_ = {};
  targetInitialized_ = false;
  return true;
}

void RaipalEeWindow::updateConnection() {
  shutDown();
  reset();
  (void)init();
}

void RaipalEeWindow::receiveState(const StateMessage::SharedPtr& message) {
  try {
    if (!message) return;
    auto incoming = raisin::raipal_ee::parseState(message->data);
    std::lock_guard lock(stateMutex_);
    if (state_ && incoming.sequence <= state_->sequence) return;
    state_ = std::move(incoming);
    stateReceived_ = Clock::now();
    stateError_.clear();
  } catch (const std::exception& error) {
    std::lock_guard lock(stateMutex_);
    state_.reset();
    stateReceived_ = {};
    stateError_ = std::string("Invalid EE state: ") + error.what();
  }
}

void RaipalEeWindow::sendCommand(raisin::raipal_ee::Command command, bool stop) {
  if (!initialized_) return;
  auto& client = stop ? stopClient_ : commandClient_;
  auto& pending = stop ? pendingStopResponse_ : pendingResponse_;
  if (!client || pending) return;
  if (!client->isServiceAvailable()) {
    commandFeedback_ = "Error: EE command service is unavailable";
    return;
  }
  command.request_id = "gui-" + std::to_string(++requestCounter_);
  auto request = std::make_shared<StringService::Request>();
  request->data = raisin::raipal_ee::encodePublicCommand(command);
  try {
    pending = client->asyncSendRequest(request);
    if (stop) {
      stopSentAt_ = Clock::now();
      stopTimedOut_ = false;
    } else {
      commandSentAt_ = Clock::now();
      commandTimedOut_ = false;
    }
  } catch (const std::exception& error) {
    commandFeedback_ = "Error: " + std::string(error.what());
  }
}

}  // namespace raisin

extern "C" raisin::GuiWindow* create(const std::string& title,
                                      std::shared_ptr<raisin::GuiResource> resource) {
  return new raisin::RaipalEeWindow(title, std::move(resource));
}

extern "C" void destroy(raisin::GuiWindow* window) { delete window; }
