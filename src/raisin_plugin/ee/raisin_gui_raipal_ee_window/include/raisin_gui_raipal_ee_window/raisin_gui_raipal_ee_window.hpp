#pragma once

#include "imgui/imgui.h"
#include "raisin_gui_base/raisin_gui_window.hpp"
#include "raisin_network/node.hpp"
#include "raisin_raipal_ee/contract.hpp"
#include "raisin_raipal_ee/hardware_configuration.hpp"
#include "raisin_interfaces/srv/string.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace raisin {

class RaipalEeWindow final : public GuiWindow, public Node {
 public:
  RaipalEeWindow(const std::string& title, std::shared_ptr<GuiResource> resource);
  ~RaipalEeWindow() override;

  bool init() override;
  bool update() override;
  bool draw() override;
  bool shutDown() override;
  bool reset() override;
  void updateConnection() override;

 private:
  using StringService = raisin_interfaces::srv::String;
  using StateMessage = std_msgs::msg::String;
  using Clock = std::chrono::steady_clock;

  void receiveState(const StateMessage::SharedPtr& message);
  void pollHardwareConfiguration();
  void sendCommand(raisin::raipal_ee::Command command, bool stop = false);

  std::string robotNickname_;
  std::chrono::milliseconds stateTimeout_{500};
  std::chrono::milliseconds responseTimeout_{500};
  std::shared_ptr<Client<StringService>> hardwareConfigurationClient_;
  Client<StringService>::SharedFuture hardwareConfigurationFuture_;
  Clock::time_point hardwareConfigurationRequestAt_{};
  std::optional<raisin::raipal_ee::HardwareConfiguration> hardwareConfiguration_;
  bool hardwareConfigurationResolved_{};
  std::string hardwareConfigurationError_;
  std::shared_ptr<Subscriber<StateMessage>> stateSubscriber_;
  std::shared_ptr<Client<StringService>> commandClient_;
  std::shared_ptr<Client<StringService>> stopClient_;
  std::optional<Client<StringService>::SharedFuture> pendingResponse_;
  std::optional<Client<StringService>::SharedFuture> pendingStopResponse_;
  Clock::time_point commandSentAt_{};
  Clock::time_point stopSentAt_{};
  bool commandTimedOut_{};
  bool stopTimedOut_{};

  mutable std::mutex stateMutex_;
  std::optional<raisin::raipal_ee::State> state_;
  Clock::time_point stateReceived_{};
  std::string stateError_;

  double targetRad_[2]{};
  bool targetInitialized_{};
  std::uint64_t requestCounter_{};
  std::string commandFeedback_;
  std::string initializationError_;
  bool initialized_{};
};

}  // namespace raisin
