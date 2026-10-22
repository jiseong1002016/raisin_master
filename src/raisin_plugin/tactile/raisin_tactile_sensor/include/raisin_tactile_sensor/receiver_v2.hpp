#pragma once

#include "frame_v2.hpp"
#include "raisin_robot/sensor.hpp"

#include <atomic>
#include <memory>
#include <string>

namespace raisin::tactile {

class ReceiverV2 final : public robot::sensor::Sensor {
 public:
  ReceiverV2(raisim::World& hub, raisim::RaisimServer& server,
             const std::string& name, parameter::ParameterContainer& params,
             const std::shared_ptr<Network>& network, std::string sensor_name,
             std::string topic)
      : robot::sensor::Sensor(hub, server, name, params, network),
        sensor_name_(std::move(sensor_name)), topic_(std::move(topic)),
        snapshot_(sensor_name_) {}
  ~ReceiverV2() override { cleanupResources(); }

  void createCommunication(std::shared_ptr<Network>) final {
    if (subscription_) return;
    subscription_ = createSubscriber<raisin_tactile_sensor::msg::TactileFrame>(topic_, nullptr,
        [this](const raisin_tactile_sensor::msg::TactileFrame::SharedPtr message) {
          try {
            if (!snapshot_.set(fromMessage(*message, sensor_name_))) ++rejected_;
          } catch (const std::invalid_argument&) {
            ++rejected_;
          }
        }, "core_sensor");
  }

  bool read() final { return false; }
  double getCommunicationRate() const final { return 0.; }
  std::optional<FrameV2> latest() const { return snapshot_.latest(); }
  std::uint64_t rejected() const { return rejected_.load(); }
  void reset() { snapshot_.reset(); }

  void shutDown() final {
    cleanupResources();
    subscription_.reset();
    snapshot_.reset();
  }

 private:
  std::string sensor_name_;
  std::string topic_;
  SnapshotV2 snapshot_;
  std::atomic<std::uint64_t> rejected_{};
  std::shared_ptr<Subscriber<raisin_tactile_sensor::msg::TactileFrame>> subscription_;
};

}  // namespace raisin::tactile
