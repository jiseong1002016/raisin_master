#pragma once

#include "image.hpp"
#include "raisin_robot/sensor.hpp"
#include <atomic>

namespace raisin::tactile {

// RAIPAL owns this in Raipal::sensors_. RaipalServer already exposes that map
// through GlobalResource.sensors before controller loading. This is a Raisin
// sensor, not raisim::Sensor: no visualizer protocol changes or UNKNOWN type.
class Receiver final : public robot::sensor::Sensor {
 public:
  Receiver(raisim::World& hub, raisim::RaisimServer& server,
           const std::string& name, parameter::ParameterContainer& params,
           const std::shared_ptr<Network>& network, Layout layout, std::string topic)
      : robot::sensor::Sensor(hub, server, name, params, network),
        layout_(std::move(layout)), topic_(std::move(topic)), snapshot_(layout_) {}
  ~Receiver() override { cleanupResources(); }

  void createCommunication(std::shared_ptr<Network>) final {
    if (subscription_) return;
    subscription_ = createSubscriber<sensor_msgs::msg::Image>(topic_, nullptr,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
          try {
            if (!snapshot_.set(fromImage(*msg, layout_))) ++rejected_;
          } catch (const std::invalid_argument&) { ++rejected_; }
        }, "core_sensor");
  }
  bool read() final { return false; }  // push-driven; no extra USB/read thread
  double getCommunicationRate() const final { return 0.; }  // no periodic read loop
  std::optional<Frame> latest(double now_s, double stale_s) const {
    return snapshot_.latest(now_s, stale_s);
  }
  void reset() { snapshot_.reset(); }
  std::uint64_t rejected() const { return rejected_.load(); }
  void shutDown() final { cleanupResources(); subscription_.reset(); snapshot_.reset(); }

 private:
  Layout layout_;
  std::string topic_;
  Snapshot snapshot_;
  std::atomic<std::uint64_t> rejected_{0};
  std::shared_ptr<Subscriber<sensor_msgs::msg::Image>> subscription_;
};

}  // namespace raisin::tactile
