#pragma once

#include "raisin_network/node.hpp"
#include "raisin_plugin/plugin.hpp"
#include "raisin_raipal_ee/contract.hpp"

#include "raisin_interfaces/srv/string.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <string>

namespace raisin::raipal_ee {

class EndEffectorPlugin final : public plugin::Plugin, public Node {
 public:
  EndEffectorPlugin(raisim::World& hub, raisim::RaisimServer& server,
                    raisim::World& sim, raisim::RaisimServer& simServer,
                    GlobalResource& resource, const std::string& owner,
                    const std::string& title, bool real);
  ~EndEffectorPlugin() override;

  bool init() override;
  bool advance() override;
  bool reset() override;

 private:
  using StringService = raisin_interfaces::srv::String;
  using StateMessage = std_msgs::msg::String;

  void handlePublicCommand(const StringService::Request::SharedPtr request,
                           StringService::Response::SharedPtr response);
  void receiveState(const StateMessage::SharedPtr& message);
  bool sendCoreCommand(const Command& command, StringService::Response& response);
  void sendUnloadStop();
  bool pollCoreResponse();
  bool pollStopResponse();
  void updateVisuals(const State& state, bool stale);
  bool stateIsFresh(std::chrono::steady_clock::time_point received) const;
  static bool tactilePairReady(const State& state);

  std::string robotNickname_;
  std::string topicPrefix_;
  std::chrono::milliseconds responseTimeout_{250};
  std::chrono::milliseconds stateTimeout_{500};
  std::chrono::milliseconds heartbeatPeriod_{100};
  std::string mountMode_{"single_L"};

  std::shared_ptr<Service<StringService>> publicService_;
  std::shared_ptr<Client<StringService>> coreClient_;
  std::shared_ptr<Client<StringService>> stopClient_;
  std::shared_ptr<Subscriber<StateMessage>> stateSubscriber_;

  mutable std::mutex stateMutex_;
  State state_;
  State visualState_;
  bool hasState_{};
  bool hasVisualState_{};
  std::chrono::steady_clock::time_point stateReceived_{};
  std::chrono::steady_clock::time_point visualStateReceived_{};
  std::string stateError_;

  std::mutex commandMutex_;
  std::uint64_t internalSequence_{};
  std::atomic<std::uint64_t> latestCoreSequence_{};
  std::atomic_bool sequenceSynchronized_{};
  std::optional<Client<StringService>::SharedFuture> pendingCoreResponse_;
  std::optional<Client<StringService>::SharedFuture> pendingStopResponse_;
  std::chrono::steady_clock::time_point lastHeartbeat_{};

  raisim::ArticulatedSystemVisual* leftVisual_{};
  raisim::ArticulatedSystemVisual* rightVisual_{};
  std::string visualNamePrefix_;
  std::string resourceDirectory_;
  Eigen::Vector3d leftMountPosition_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rightMountPosition_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d leftMountRotation_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d rightMountRotation_{Eigen::Matrix3d::Identity()};
};

}  // namespace raisin::raipal_ee
