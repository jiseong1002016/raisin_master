#pragma once

#include "raisin_plugin/plugin.hpp"
#include "raisin_plugin/process.hpp"
#include "raisin_network/node.hpp"
#include "raisin_tactile_plugin/backend.hpp"
#include "raisin_tactile_sensor/image.hpp"
#include "raisin_tactile_sensor/receiver_v2.hpp"
#include "raisin_tactile_sensor/shadow_consumer.hpp"
#include "std_msgs/msg/string.hpp"

namespace raisin::tactile {

class TactilePlugin final : public plugin::Plugin, public Node {
 public:
  TactilePlugin(raisim::World&, raisim::RaisimServer&, raisim::World&,
                raisim::RaisimServer&, GlobalResource&, const std::string&,
                const std::string&, bool);
  ~TactilePlugin() override;
  bool init() final;
  bool advance() final;
  bool reset() final;
  bool shouldTerminate() final { return failed_ || !usbAlive(); }
 private:
  bool enabled_{false}, failed_{false};
  GlobalResource& global_resource_;
  std::vector<SensorConfig> configs_;
  std::vector<Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
  std::vector<double> previous_;
  std::unique_ptr<SimBackend> sim_;
  std::unique_ptr<SpringBackend> spring_;
  std::unique_ptr<ReplayBackend> replay_;
  std::vector<std::unique_ptr<plugin::Process>> usb_processes_;
  bool usbAlive() const {
    for (const auto& process : usb_processes_) if (!process->isAlive()) return false;
    return true;
  }
  std::shared_ptr<ReceiverV2> shadow_left_;
  std::shared_ptr<ReceiverV2> shadow_right_;
  std::shared_ptr<Publisher<std_msgs::msg::String>> shadow_event_publisher_;
  ShadowPairConsumer shadow_consumer_;
  std::uint64_t shadow_event_count_{};
  std::uint64_t shadow_stale_timeout_ns_{150000000ULL};
  std::uint64_t shadow_max_skew_ns_{50000000ULL};
  bool shadow_enabled_{};
};

}  // namespace raisin::tactile
