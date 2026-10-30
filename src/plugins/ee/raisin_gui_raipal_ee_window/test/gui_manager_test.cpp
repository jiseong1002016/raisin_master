#include "raisin_gui_base/raisin_gui_window_manager.hpp"
#include "raisin_network/raisin.hpp"
#include "raisin_network/node.hpp"
#include "raisin_raipal_ee/contract.hpp"
#include "raisin_raipal_ee/hardware_configuration.hpp"
#include "raisin_interfaces/srv/string.hpp"
#include "std_msgs/msg/string.hpp"

#include "imgui/imgui.h"
#include "imgui/imgui_internal.h"

#include <chrono>
#include <atomic>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;
using StringService = raisin::raisin_interfaces::srv::String;

class GuiFixtureNode final : public raisin::Node {
 public:
  GuiFixtureNode(const std::string& name, std::shared_ptr<raisin::Network> network)
      : raisin::Node(name, std::move(network)) {}
  ~GuiFixtureNode() override { cleanupResources(); }
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::size_t openWindowCount(raisin::GuiWindowManager& manager) {
  std::size_t count = 0;
  for (const auto& entry : manager.getWindows()) count += entry.second.instances.size();
  return count;
}

std::filesystem::path makePrivateInstall(const std::filesystem::path& stage,
                                         const std::filesystem::path& emptyEncryptionLibrary,
                                         const std::string& nickname,
                                         const std::filesystem::path& artifactRoot) {
  const auto prefix = artifactRoot / ("ee_gui_install_" + std::to_string(::getpid()));
  std::filesystem::remove_all(prefix);
  std::filesystem::create_directories(prefix / "config/raisin_gui_raipal_ee_window/config");
  std::filesystem::create_directories(prefix / "config/raisin_network/config");
  std::filesystem::create_directories(prefix / "lib");
  for (const auto& entry : std::filesystem::directory_iterator(stage / "config")) {
    const auto package = entry.path().filename();
    if (package == "raisin_network" || package == "raisin_gui_raipal_ee_window") continue;
    std::filesystem::create_symlink(entry.path(), prefix / "config" / package);
  }
  require(std::filesystem::is_regular_file(emptyEncryptionLibrary),
          "SDK empty-encryption library is missing");
  const auto stageLib = std::filesystem::absolute(stage / "lib");
  for (const auto& entry : std::filesystem::recursive_directory_iterator(stageLib)) {
    if (!entry.is_regular_file() || entry.path().filename() == "libraisin_empty_encryption.so")
      continue;
    const auto relativePath = entry.path().lexically_relative(stageLib);
    const auto destination = prefix / "lib" / relativePath;
    std::filesystem::create_directories(destination.parent_path());
    std::filesystem::create_symlink(entry.path(), destination);
  }
  std::filesystem::create_symlink(emptyEncryptionLibrary,
                                  prefix / "lib/libraisin_empty_encryption.so");
  std::ofstream networkConfig(prefix / "config/raisin_network/config/params.yaml");
  networkConfig << "websocket_interface:\n"
                << "  value: 127.0.0.1\n"
                << "  dtype: string\n"
                << "encryption: empty\n";
  networkConfig.close();
  require(networkConfig.good(), "cannot write private network test config");
  std::ofstream config(prefix / "config/raisin_gui_raipal_ee_window/config/params.yaml");
  config << "robot_nickname: " << nickname << "\n"
         << "state_timeout_ms: 1000\n"
         << "response_timeout_ms: 500\n"
         << "open_when_start: false\n"
         << "only_single_instance_allowed: true\n"
         << "menu_tab: tools\n";
  config.close();
  require(config.good(), "cannot write private GUI test config");
  return prefix;
}

int drawFrame(raisin::GuiWindowManager& manager, ImVec2 mousePosition = ImVec2(-10.F, -10.F),
              bool mouseDown = false, ImGuiID* hoveredId = nullptr,
              ImGuiID* activeId = nullptr) {
  auto& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(1280.F, 800.F);
  io.DeltaTime = 1.F / 60.F;
  io.MousePos = mousePosition;
  io.MouseDown[0] = mouseDown;
  ImGui::NewFrame();
  manager.update();
  ImGui::SetNextWindowPos(ImVec2(20.F, 20.F), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(760.F, 700.F), ImGuiCond_Always);
  manager.draw();
  if (hoveredId || activeId) {
    const auto* context = ImGui::GetCurrentContext();
    if (hoveredId) *hoveredId = context->HoveredId;
    if (activeId) *activeId = context->ActiveId;
  }
  ImGui::Render();
  const auto* drawData = ImGui::GetDrawData();
  return drawData ? drawData->TotalVtxCount : 0;
}

int drawVisibleFrame(raisin::GuiWindowManager& manager, const char* failureMessage) {
  constexpr int kWarmupFrames = 6;
  for (int frame = 0; frame < kWarmupFrames; ++frame) {
    const auto vertices = drawFrame(manager);
    if (vertices > 0) return vertices;
  }
  throw std::runtime_error(failureMessage);
}

ImVec2 findStopButton(raisin::GuiWindowManager& manager, const std::string& windowTitle,
                      ImGuiID& stopId) {
  auto* window = ImGui::FindWindowByName(windowTitle.c_str());
  require(window != nullptr, "ImGui window for the loaded RAIPAL EE factory is missing");
  stopId = window->GetID("Stop EE");
  const auto contentStart = window->DC.CursorStartPos;
  const auto textHeight = window->CalcFontSize();

  // Probe the first content row with hover-only frames. This uses ImGui's actual
  // content origin and verifies the target item's ID before sending a click.
  for (float y = contentStart.y; y < contentStart.y + textHeight * 12.F; y += 4.F) {
    for (float x = contentStart.x + 2.F; x < contentStart.x + 88.F; x += 4.F) {
      const ImVec2 position(x, y);
      ImGuiID hoveredId = 0;
      drawFrame(manager, position, false, &hoveredId);
      if (hoveredId == stopId) return position;
    }
  }
  throw std::runtime_error("could not locate the Stop EE ImGui item in its first content row");
}

}  // namespace

namespace raisin {

std::string getInstallDirectory() {
  const char* prefix = std::getenv("RAISIN_TEST_INSTALL_PREFIX");
  if (!prefix) return {};
  std::string path(prefix);
  if (!path.empty() && path.back() != '/') path.push_back('/');
  return path;
}

}  // namespace raisin

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  const std::string nickname = "ee_test_" + std::to_string(::getpid());
  const std::filesystem::path artifactRoot(argv[3]);
  std::filesystem::create_directories(artifactRoot);
  const auto privateInstall = makePrivateInstall(argv[1], argv[2], nickname, artifactRoot);
  const auto privateHome = artifactRoot / "home";
  std::filesystem::create_directories(privateHome);
  ::setenv("HOME", privateHome.c_str(), 1);
  ::setenv("RAISIN_TEST_INSTALL_PREFIX", privateInstall.c_str(), 1);

  try {
    raisin::raisinInit();
    ImGui::CreateContext();
    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

    auto network = std::make_shared<raisin::Network>(nickname, "ee_gui_test",
        std::vector<std::vector<std::string>>{{"main"}, {"plugin"}, {"core_sensor"}},
        std::vector<std::string>{"lo"}, "", 60, false);
    auto resource = std::make_shared<raisin::GuiResource>();
    resource->network = network;
    auto manager = std::make_shared<raisin::GuiWindowManager>(resource);
    resource->guiWindowManager = manager;

    GuiFixtureNode fixture("ee_gui_fixture", network);
    auto statePublisher = fixture.createPublisher<raisin::std_msgs::msg::String>(
        nickname + "/ee/state", 5);
    raisin::raipal_ee::HardwareConfiguration configuration;
    configuration.hardware_profile = "dual_arm_right_ee";
    configuration.selection_source = "default";
    configuration.arm_dof = 14;
    configuration.expected_slaves = 18;
    configuration.modules = {"raipal_R", "raipal-ee-visual_R", "raipal_L", "tool-empty_L"};
    configuration.gui_arm_modules = {"raipal_R", "stub-0_R", "raipal_L", "tool-empty_L"};
    configuration.raisim_config = "/raisim_config/flat_terrain_ee_visual_R.xml";
    configuration.ee_present = true;
    configuration.ee_communication = true;
    configuration.ee_mount_mode = "single_R";
    configuration.ee_visual_resource = "resource/raisin_raipal_ee/resource/ee_visual.urdf";
    configuration.ee_mount_resource = "resource/raisin_raipal_ee/resource/ee_mounts.yaml";
    configuration.initial_motor_state = "COMM_DISABLED";
    auto configurationService = fixture.createService<StringService>(
        nickname + "/hardware_configuration",
        [configuration](const StringService::Request::SharedPtr&,
                        StringService::Response::SharedPtr response) {
          response->success = true;
          response->message = raisin::raipal_ee::encodeHardwareConfiguration(configuration);
        }, "plugin");
    std::atomic_uint stopRequests{};
    auto commandService = fixture.createService<StringService>(nickname + "/ee/command",
        [&stopRequests](const StringService::Request::SharedPtr& request,
                        StringService::Response::SharedPtr response) {
          try {
            const auto command = raisin::raipal_ee::parsePublicCommand(request->data);
            if (command.action == raisin::raipal_ee::Action::STOP) ++stopRequests;
            response->success = true;
            response->message = "fixture accepted";
          } catch (const std::exception& error) {
            response->success = false;
            response->message = error.what();
          }
        }, "plugin");
    require(statePublisher != nullptr && commandService != nullptr && configurationService != nullptr,
            "cannot create EE GUI fixture endpoints");
    std::this_thread::sleep_for(100ms);

    const auto countBefore = openWindowCount(*manager);
    manager->addWindow("raipal_ee");
    require(openWindowCount(*manager) == countBefore + 1,
            "GuiWindowManager did not load the RAIPAL EE window factory");
    const auto waitingVertices = drawVisibleFrame(
        *manager, "RAIPAL EE window stayed hidden after ImGui warmup frames");

    raisin::raipal_ee::State state;
    state.backend = raisin::raipal_ee::Backend::MOCK;
    state.tactile_ready = true;
    state.actuation_qualified = true;
    state.homed = true;
    state.axes[0].position_rad = -0.1;
    state.axes[0].calibrated_valid = true;
    state.axes[0].lower_limit_rad = -1.0;
    state.axes[0].upper_limit_rad = 1.0;
    state.axes[1].position_rad = 0.2;
    state.axes[1].calibrated_valid = true;
    state.axes[1].lower_limit_rad = -1.0;
    state.axes[1].upper_limit_rad = 1.0;
    for (auto& tactile : state.tactile) {
      tactile.valid = true;
      tactile.baseline_ready = true;
      tactile.age_ms = 5.0;
      tactile.sequence = 1;
    }
    for (std::uint64_t seq = 1; seq <= 3; ++seq) {
      state.sequence = seq;
      raisin::std_msgs::msg::String message;
      message.data = raisin::raipal_ee::encodeState(state);
      statePublisher->publish(message);
      std::this_thread::sleep_for(50ms);
      const auto stateVertices = drawFrame(*manager);
      if (seq == 3)
        require(stateVertices > waitingVertices + 100,
                "RAIPAL EE GUI did not render the received state and controls");
    }

    const auto windowType = manager->getWindows().find("raipal_ee");
    require(windowType != manager->getWindows().end() && !windowType->second.instances.empty(),
            "loaded RAIPAL EE window is missing from the manager");
    const auto instance = windowType->second.instances.begin();
    require(instance->second != nullptr, "RAIPAL EE manager instance is null");
    const auto windowTitle = instance->second->title;
    require(!windowTitle.empty(), "RAIPAL EE GUI window has no display title");
    const auto closeReference = windowType->first + "<&>" + instance->first;
    ImGuiID stopButtonId = 0;
    const auto stopButtonPosition = findStopButton(*manager, windowTitle, stopButtonId);
    drawFrame(*manager, stopButtonPosition, false);
    ImGuiID activeId = 0;
    drawFrame(*manager, stopButtonPosition, true, nullptr, &activeId);
    require(activeId == stopButtonId,
            "mouse press did not activate the located Stop EE ImGui item");
    drawFrame(*manager, stopButtonPosition, false);
    const auto stopDeadline = std::chrono::steady_clock::now() + 1s;
    while (stopRequests.load() == 0 && std::chrono::steady_clock::now() < stopDeadline)
      std::this_thread::sleep_for(10ms);
    require(stopRequests.load() == 1,
            "clicking Stop EE did not send a stop action through the GUI service client");

    manager->shutDownWindow(closeReference);
    require(openWindowCount(*manager) == countBefore,
            "GuiWindowManager did not close the RAIPAL EE window");
    manager->addWindow("raipal_ee");
    require(openWindowCount(*manager) == countBefore + 1,
            "GuiWindowManager did not reload the RAIPAL EE window");
    drawVisibleFrame(*manager, "reloaded RAIPAL EE window stayed hidden after ImGui warmup frames");

    manager->shutDown();
    manager.reset();
    resource->guiWindowManager.reset();
    resource.reset();
    fixture.cleanupResources();
    network->shutdown();
    network.reset();
    ImGui::DestroyContext();
    std::filesystem::remove_all(privateInstall);
    std::cout << "PASS: GuiWindowManager loaded, drew, closed and reloaded RAIPAL EE GUI\n";
    return 0;
  } catch (const std::exception& error) {
    std::filesystem::remove_all(privateInstall);
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
