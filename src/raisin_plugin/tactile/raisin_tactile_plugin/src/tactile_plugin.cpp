#include "raisin_tactile_plugin/tactile_plugin.hpp"
#include "raisin_tactile_plugin/usb_backend.hpp"
#include "raisin_tactile_sensor/registration.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <sstream>

namespace raisin::tactile {

TactilePlugin::TactilePlugin(raisim::World& hub, raisim::RaisimServer& server,
    raisim::World& sim, raisim::RaisimServer& simServer, GlobalResource& resource,
    const std::string& owner, const std::string& title, bool real)
    : plugin::Plugin(hub, server, sim, simServer, resource, owner, title, real),
      Node("raisin_tactile_plugin", resource.paramRoot, resource.network),
      global_resource_(resource) {}

TactilePlugin::~TactilePlugin() { cleanupResources(); }

bool TactilePlugin::init() {
  try {
    enabled_ = param_->as_bool("enabled");
    if (!enabled_) return true;
    const std::string backend = param_->as_string("backend");
    shadow_enabled_ = (*param_)["shadow"].as_bool("enabled");
    if (shadow_enabled_) {
      if (backend != "usb")
        throw std::invalid_argument("shadow consumer requires backend: usb");
      const double stale_s = (*param_)["bilateral"].as_double("stale_timeout_s");
      const double skew_s = (*param_)["bilateral"].as_double("max_skew_s");
      if (!std::isfinite(stale_s) || stale_s <= 0 || !std::isfinite(skew_s) || skew_s < 0)
        throw std::invalid_argument("invalid shadow stale/skew limits");
      shadow_stale_timeout_ns_ = static_cast<std::uint64_t>(stale_s * 1e9);
      shadow_max_skew_ns_ = static_cast<std::uint64_t>(skew_s * 1e9);
    }
    if (backend == "usb") {
      auto& usb = (*param_)["usb"];
      const auto port = usb.as_string("port");
      const auto left_port = std::string(usb("left_port", std::string{}));
      const auto right_port = std::string(usb("right_port", std::string{}));
      const bool separate = !left_port.empty() || !right_port.empty();
      if (separate && (left_port.empty() || right_port.empty() || left_port == right_port || !port.empty()))
        throw std::invalid_argument("two USB sensors require distinct left_port/right_port and empty port");
      const auto sensor_count = separate ? 1 : usb.as_int("stream_sensor_count");
      const auto record_dir = usb.as_string("record_dir");
      if ((!separate && port.empty()) || record_dir.empty())
        throw std::invalid_argument("USB backend requires one explicit port and record_dir");
      if (sensor_count != 1 && sensor_count != 2)
        throw std::invalid_argument("USB stream_sensor_count must be 1 or 2");
      const auto prefix = usb.as_string("topic_prefix");
      if (shadow_enabled_)
        shadow_event_publisher_ = createPublisher<std_msgs::msg::String>(
            prefix + "/shadow/event", 5);
      auto& conversion = (*param_)["force_conversion"];
      for (const auto& side : separate ? std::vector<std::string>{"left", "right"}
                                        : std::vector<std::string>{""}) {
        std::vector<std::string> args{side == "left" ? left_port : side == "right" ? right_port : port,
              std::to_string(sensor_count), side.empty() ? record_dir : record_dir + "/" + side,
              std::to_string(usb.as_int("rate_hz")), std::to_string(usb.as_int("baud_rate")),
              std::to_string(usb.as_double("frame_timeout_s")),
              std::to_string(usb.as_double("retry_s")), prefix,
              std::to_string(usb.as_double("poll_rate_hz")),
              std::to_string(conversion.as_double("code_to_force_n_per_code")),
              conversion.as_string("calibration_id"),
              std::to_string(conversion.as_int("left_polarity")),
              std::to_string(conversion.as_int("right_polarity"))};
        if (!side.empty()) args.push_back(side);
        usb_processes_.push_back(std::make_unique<plugin::Process>(
            "raisin_tactile_plugin", "raisin_tactile_usb_process", args));
      }

      return true;
    }
    // Never silently replace a missing USB driver with simulated/recorded data.
    if (isRealRobot_) throw std::runtime_error("real robot requires backend: usb");
    if (backend != "sim" && backend != "replay" && backend != "spring")
      throw std::invalid_argument("backend must be sim, spring, replay or usb");
    for (const auto& [name, ptr] : (*param_)["sensors"].getParamContainerList()) {
      const auto& p = *ptr;
      SensorConfig c;
      c.name = name;
      c.layout = readLayout(p);
      c.parent_link = p.as_string("parent_link");
      c.urdf_frame = p.as_string("urdf_frame");
      const auto offset = p.as_vector_double("offset_m");
      const auto rpy = p.as_vector_double("rpy_rad");
      if (offset.size() != 3 || rpy.size() != 3) throw std::invalid_argument("expected xyz/rpy triples");
      c.offset_m = Eigen::Vector3d(offset[0], offset[1], offset[2]);
      c.rotation = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
      c.surface_tolerance_m = p.as_double("surface_tolerance_m");
      c.max_force_n = p.as_double("max_force_n");
      c.replay_csv = p.as_string("replay_csv");
      if (backend == "spring") {
        c.taxel_prefix = p.as_string("taxel_prefix");
        c.spring_stiffness = p.as_double("spring_stiffness_n_per_m");
        c.spring_damping = p.as_double("spring_damping_n_s_per_m");
        c.spring_travel = p.as_double("spring_travel_m");
        c.spring_deflection_gate = p.as_double("spring_defl_eps_m");
      }
      configs_.push_back(std::move(c));
    }
    if (configs_.empty()) throw std::invalid_argument("no tactile sensors configured");
    // Map iteration order is unspecified; wire topic identity is always the name.
    std::sort(configs_.begin(), configs_.end(), [](const auto& a, const auto& b) { return a.name < b.name; });
    if (backend == "sim" || backend == "spring") {
      if (!robotSim_) throw std::runtime_error("sim robot unavailable");
      if (backend == "sim") sim_ = std::make_unique<SimBackend>(worldSim_, *robotSim_, configs_);
      else spring_ = std::make_unique<SpringBackend>(worldSim_, *robotSim_, configs_);
    } else replay_ = std::make_unique<ReplayBackend>(configs_);
    const auto prefix = param_->as_string("topic_prefix");
    for (const auto& config : configs_)
      publishers_.push_back(createPublisher<sensor_msgs::msg::Image>(prefix + "/" + config.name, 1));
    previous_.assign(configs_.size(), -1.);
    return true;
  } catch (const std::exception& e) {
    RSWARN("tactile init: " << e.what());
    failed_ = true;
    return false;
  }
}

bool TactilePlugin::advance() {
  if (!enabled_) return true;
  if (failed_) return false;
  if (!usb_processes_.empty()) {
    if (!usbAlive()) return false;
    if (!shadow_enabled_) return true;
    try {
      if (!shadow_left_ || !shadow_right_) {
        if (!global_resource_.sensors) return true;
        const auto left = global_resource_.sensors->find("tactile/left");
        const auto right = global_resource_.sensors->find("tactile/right");
        if (left == global_resource_.sensors->end() || right == global_resource_.sensors->end())
          throw std::runtime_error("core tactile/left and tactile/right Receivers are required");
        shadow_left_ = std::dynamic_pointer_cast<ReceiverV2>(left->second);
        shadow_right_ = std::dynamic_pointer_cast<ReceiverV2>(right->second);
        if (!shadow_left_ || !shadow_right_)
          throw std::runtime_error("core tactile entries must be ReceiverV2 instances");
      }
      const auto pair = shadow_consumer_.tick(
          shadow_left_->latest(), shadow_right_->latest(), usb::monotonicNowNs(),
          shadow_stale_timeout_ns_, shadow_max_skew_ns_);
      if (pair) {
        ++shadow_event_count_;
        std_msgs::msg::String event;
        std::ostringstream message;
        message << "event=" << shadow_event_count_
                << " left_session=" << pair->left.session_id
                << " left_sequence=" << pair->left.sequence
                << " right_session=" << pair->right.session_id
                << " right_sequence=" << pair->right.sequence
                << " skew_ms=" << static_cast<double>(pair->skew_ns) / 1e6;
        event.data = message.str();
        shadow_event_publisher_->publish(event);
      }
      return true;
    } catch (const std::exception& error) {
      RSWARN("tactile shadow consumer: " << error.what());
      failed_ = true;
      return false;
    }
  }
  try {
    const auto frames = sim_ ? sim_->sample() : spring_ ? spring_->sample() : replay_->sample();
    for (std::size_t i = 0; i < frames.size(); ++i) {
      if (frames[i].stamp_s < previous_[i])
        throw std::runtime_error("simulation clock reset requires explicit plugin/consumer reset");
      if (frames[i].stamp_s == previous_[i]) continue;  // paused sim is not new data
      publishers_[i]->publish(toImage(frames[i], configs_[i].layout));
      previous_[i] = frames[i].stamp_s;
    }
    return true;
  } catch (const std::exception& e) {
    RSWARN("tactile sample: " << e.what());
    failed_ = true;
    return false;
  }
}

bool TactilePlugin::reset() {
  // Explicit reset also requires consumer history reset; no silent stamp rebasing.
  if (!usb_processes_.empty()) return usbAlive();
  if (enabled_ && !sim_ && !spring_ && !replay_) return false;  // failed init is not recoverable by reset
  try {
    previous_.assign(configs_.size(), -1.);
    if (replay_) replay_ = std::make_unique<ReplayBackend>(configs_);
    failed_ = false;
    return true;
  } catch (const std::exception& e) {
    RSWARN("tactile reset: " << e.what());
    failed_ = true;
    return false;
  }
}

}  // namespace raisin::tactile

extern "C" raisin::plugin::Plugin* create_with_owner_title_robot(
    raisim::World& hub, raisim::RaisimServer& server, raisim::World& sim,
    raisim::RaisimServer& simServer, raisin::GlobalResource& resource,
    const std::string& owner, const std::string& title, bool real) {
  return new raisin::tactile::TactilePlugin(hub, server, sim, simServer, resource, owner, title, real);
}
extern "C" void destroy(raisin::plugin::Plugin* plugin) { delete plugin; }
