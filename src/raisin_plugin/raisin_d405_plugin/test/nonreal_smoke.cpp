// Loads only the NONREAL factory path; this never opens a USB camera or motor bus.
#include "raisin_plugin/plugin.hpp"
#include "raisin_network/raisin.hpp"
#include "raisin_interfaces/srv/get_camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "raisin_interfaces/msg/ffmpeg_packet.hpp"
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <dlfcn.h>
#include <iostream>
#include <thread>
#include <unistd.h>

namespace raisin {
std::string getInstallDirectory() {
  const char* prefix = std::getenv("D405_SMOKE_PREFIX");
  return prefix ? std::string(prefix) + "/" : std::string{};
}
}
int main(int argc, char** argv) {
  if (argc != 4) return 2;  // plugin .so, existing harmless fixture URDF, private install
  ::setenv("D405_SMOKE_PREFIX", argv[3], 1);
  using namespace raisin;
  using namespace std::chrono_literals;
  raisinInit();
  auto& resources = GlobalResource::getInstance();
  resources.network = std::make_shared<Network>("d405_nonreal_smoke_" + std::to_string(::getpid()),
      "test", std::vector<std::vector<std::string>>{{"main"}, {"core_sensor"}});
  Node probe("d405_probe", resources.network);
  const std::string prefix = "d405_nonreal_smoke_" + std::to_string(::getpid());
  std::atomic<bool> raw{}, snapshot{}, compressed{};
  auto raw_sub = probe.createSubscriber<sensor_msgs::msg::Image>(prefix + "/depth", nullptr,
      [&](const sensor_msgs::msg::Image::SharedPtr message) {
        raw = message->width == 640 && message->height == 480 && message->encoding == "32FC1";
      });
  auto snapshot_sub = probe.createSubscriber<std_msgs::msg::String>(prefix + "/grasp_input", nullptr,
      [&](const std_msgs::msg::String::SharedPtr message) {
        const auto value = nlohmann::json::parse(message->data);
        snapshot = value.at("is_simulation") == true && value.at("valid") == true
                   && !value.at("points_camera_m").empty()
                   && std::abs(value.at("point_camera_m")[2].get<double>() - .22) < 1e-6;
      });
  auto compressed_sub = probe.createSubscriber<raisin_interfaces::msg::FfmpegPacket>(
      prefix + "/depth_compressed", nullptr,
      [&](const raisin_interfaces::msg::FfmpegPacket::SharedPtr message) {
        compressed = !message->data.empty() && message->encoding == "libaom-av1";
      });
  auto client = probe.createClient<raisin_interfaces::srv::GetCameraInfo>(
      prefix + "/depth/get_camera_info", nullptr);
  const char* home = std::getenv("HOME");
  if (home) raisim::World::setActivationKey(std::string(home) + "/.raisim/activation.raisim");
  raisim::World hub, sim;
  hub.addArticulatedSystem(argv[2])->setName("robot");
  sim.addArticulatedSystem(argv[2])->setName("robot");
  raisim::RaisimServer hub_server(&hub), sim_server(&sim);
  void* library = ::dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (!library) { std::cerr << ::dlerror() << '\n'; return 1; }
  auto create = reinterpret_cast<plugin::create_with_owner_title_robot_t*>(::dlsym(library, "create_with_owner_title_robot"));
  auto destroy = reinterpret_cast<plugin::destroy_t*>(::dlsym(library, "destroy"));
  auto* camera = create(hub, hub_server, sim, sim_server, resources, "test", "d405", false);
  resources.paramRoot["raisin_d405_plugin"]("topic_prefix") = prefix;
  resources.paramRoot["raisin_d405_plugin"]("simulation_source") = std::string("plane");
  bool passed = camera->init();
  for (int i = 0; i < 200 && passed && !(raw && snapshot && compressed && client->isServiceAvailable()); ++i) {
    passed = camera->advance(); std::this_thread::sleep_for(25ms);
  }
  passed = passed && raw && snapshot && compressed && client->isServiceAvailable();
  std::cout << "raw=" << raw << " synthetic_snapshot=" << snapshot << " av1=" << compressed
            << " camera_info_service=" << client->isServiceAvailable() << '\n';
  if (passed) {
    auto result = client->asyncSendRequest(std::make_shared<raisin_interfaces::srv::GetCameraInfo::Request>());
    passed = result.wait_for(3s) == std::future_status::ready;
    if (passed) { const auto info = result.get(); passed = info && info->success && info->camera_info.width == 640; }
  }
  destroy(camera);
  probe.cleanupResources();
  resources.network->shutdown(); resources.network.reset();
  ::dlclose(library);
  std::cout << (passed ? "PASS" : "FAIL") << ": NONREAL D405 raw depth, explicit synthetic ROI point, AV1 GUI transport, CameraInfo service\n";
  return passed ? 0 : 1;
}
