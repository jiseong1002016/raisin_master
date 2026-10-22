#include "imgui/imgui.h"
#include "raisin_gui_base/raisin_gui_window_manager.hpp"
#include "raisin_network/raisin.hpp"
#include "raisin_tactile_sensor/frame_v2.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

using namespace std::chrono_literals;
using TactileMessage = raisin::raisin_tactile_sensor::msg::TactileFrame;
using StatusMessage = raisin::std_msgs::msg::String;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::size_t openWindowCount(raisin::GuiWindowManager& manager) {
  std::size_t count = 0;
  for (const auto& entry : manager.getWindows())
    count += entry.second.instances.size();
  return count;
}

std::uint64_t monotonicNowNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

raisin::tactile::FrameV2 frame(const std::string& sensor, std::uint64_t sequence) {
  raisin::tactile::FrameV2 result;
  result.sensor_name = sensor;
  result.frame_id = "tactile_" + sensor + "_frame";
  result.clock_domain = "CLOCK_MONOTONIC";
  result.calibration_id = "provisional_linear_0p001";
  result.calibration_status = "provisional";
  result.baseline_status = "operator_confirmed_unloaded";
  result.force_unit = "N_estimate";
  result.stamp_monotonic_ns = monotonicNowNs();
  result.session_id = sensor == "left" ? 101 : 202;
  result.sequence = sequence;
  result.rows = raisin::tactile::kUsbRows;
  result.cols = raisin::tactile::kUsbCols;
  result.code_to_force_n_per_code = 0.001F;
  result.polarity = 1;
  result.baseline_valid = true;
  result.raw_code.assign(raisin::tactile::kUsbCells, 100);
  result.baseline_code.assign(raisin::tactile::kUsbCells, 0);
  result.normal_force_n.assign(raisin::tactile::kUsbCells, 0.F);
  result.valid.assign(raisin::tactile::kUsbCells, 0);
  for (std::size_t row = raisin::tactile::kUsbRoiRxFirst;
       row <= raisin::tactile::kUsbRoiRxLast; ++row) {
    for (std::size_t col = raisin::tactile::kUsbRoiTxFirst;
         col <= raisin::tactile::kUsbRoiTxLast; ++col) {
      const auto index = row * result.cols + col;
      result.normal_force_n[index] = 0.1F;
      result.valid[index] = 1;
    }
  }
  return result;
}

void beginDrawFrame() {
  auto& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(1280.F, 800.F);
  io.DeltaTime = 1.F / 60.F;
  ImGui::NewFrame();
}

std::filesystem::path makePrivateInstall(const std::filesystem::path& stage,
                                         const std::filesystem::path& sdk,
                                         const std::string& topic_prefix) {
  const auto prefix = std::filesystem::temp_directory_path() /
      ("tactile_gui_test_install_" + std::to_string(::getpid()));
  std::filesystem::remove_all(prefix);
  std::filesystem::create_directories(prefix / "config");
  std::filesystem::create_directories(prefix / "lib");
  const auto sdk_config = sdk / "config";
  for (const auto& entry : std::filesystem::directory_iterator(sdk_config)) {
    if (entry.path().filename() == "raisin_gui_tactile_window") continue;
    const auto destination = prefix / "config" / entry.path().filename();
    if (entry.is_directory())
      std::filesystem::create_directory_symlink(entry.path(), destination);
    else
      std::filesystem::create_symlink(entry.path(), destination);
  }
  for (const auto* directory : {"resource", "messages", "scripts"}) {
    const auto source = sdk / directory;
    if (std::filesystem::is_directory(source))
      std::filesystem::create_directory_symlink(source, prefix / directory);
  }
  const auto empty_encryption = sdk / "lib/libraisin_empty_encryption.so";
  if (std::filesystem::is_regular_file(empty_encryption))
    std::filesystem::create_symlink(empty_encryption,
                                    prefix / "lib/libraisin_empty_encryption.so");
  std::filesystem::create_directories(prefix / "config/raisin_gui_tactile_window/config");
  const auto stage_lib = std::filesystem::absolute(stage / "lib");
  for (const auto& entry : std::filesystem::recursive_directory_iterator(stage_lib)) {
    if (!entry.is_regular_file() ||
        entry.path().filename() == "libraisin_empty_encryption.so") continue;
    const auto relative_path = entry.path().lexically_relative(stage_lib);
    const auto destination = prefix / "lib" / relative_path;
    std::filesystem::create_directories(destination.parent_path());
    std::filesystem::create_symlink(entry.path(), destination);
  }
  std::ofstream config(prefix / "config/raisin_gui_tactile_window/config/params.yaml");
  config << "topic_prefix: " << topic_prefix << "\n"
         << "command_suffix: command\n"
         << "open_when_start: false\n"
         << "only_single_instance_allowed: true\n"
         << "menu_tab: tools\n"
         << "record_root: \"\"\n"
         << "display_max_n_estimate: 0.5\n"
         << "stale_timeout_ms: 150.0\n"
         << "max_skew_ms: 50.0\n";
  config.close();
  require(config.good(), "cannot write private tactile GUI test config");
  return prefix;
}

int drawVertexCount(raisin::GuiWindowManager& manager) {
  beginDrawFrame();
  manager.update();
  manager.draw();
  ImGui::Render();
  const auto* draw_data = ImGui::GetDrawData();
  return draw_data ? draw_data->TotalVtxCount : 0;
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
  if (argc != 3) return 2;
  const std::string topic_prefix = "tactile/test/roi_manager_" +
                                   std::to_string(::getpid());
  const auto private_install = makePrivateInstall(argv[1], argv[2], topic_prefix);
  ::setenv("RAISIN_TEST_INSTALL_PREFIX", private_install.c_str(), 1);
  std::filesystem::create_directories(private_install / "home");
  ::setenv("HOME", (private_install / "home").c_str(), 1);
  raisin::raisinInit();
  ImGui::CreateContext();
  unsigned char* font_pixels = nullptr;
  int font_width = 0;
  int font_height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&font_pixels, &font_width, &font_height);

  auto network = std::make_shared<raisin::Network>(
      "tactile_gui_manager_load_" + std::to_string(::getpid()), "test",
      std::vector<std::vector<std::string>>{{"main"}, {"plugin"}, {"core_sensor"}});
  auto resource = std::make_shared<raisin::GuiResource>();
  resource->network = network;
  auto manager = std::make_shared<raisin::GuiWindowManager>(resource);
  resource->guiWindowManager = manager;

  const auto count_before = openWindowCount(*manager);
  manager->addWindow("tactile");
  require(openWindowCount(*manager) > count_before,
          "Raisin GUI window manager did not instantiate the tactile window");

  raisin::Node producer("main", network);
  auto left_publisher = producer.createPublisher<TactileMessage>(topic_prefix + "/left");
  auto right_publisher = producer.createPublisher<TactileMessage>(topic_prefix + "/right");
  auto left_status = producer.createPublisher<StatusMessage>(topic_prefix + "/status/left");
  auto right_status = producer.createPublisher<StatusMessage>(topic_prefix + "/status/right");
  require(left_publisher && right_publisher && left_status && right_status,
          "cannot create tactile GUI fixture publishers");
  std::this_thread::sleep_for(100ms);

  const auto empty_vertices = drawVertexCount(*manager);
  auto left_frame = frame("left", 1);
  left_publisher->publish(raisin::tactile::toMessage(left_frame));
  StatusMessage left_status_message;
  left_status_message.data = "connected=1 frames=1 age_ms=1 port=fixture-left";
  left_status->publish(left_status_message);
  std::this_thread::sleep_for(100ms);
  const auto left_only_vertices = drawVertexCount(*manager);

  auto right_frame = frame("right", 1);
  right_publisher->publish(raisin::tactile::toMessage(right_frame));
  StatusMessage right_status_message;
  right_status_message.data = "connected=1 frames=1 age_ms=1 port=fixture-right";
  right_status->publish(right_status_message);
  std::this_thread::sleep_for(100ms);
  const auto bilateral_vertices = drawVertexCount(*manager);
  std::cout << "draw vertices: empty=" << empty_vertices
            << " left_only=" << left_only_vertices
            << " bilateral=" << bilateral_vertices << '\n';
  const auto left_heatmap_vertices = left_only_vertices - empty_vertices;
  const auto right_heatmap_vertices = bilateral_vertices - left_only_vertices;
  require(left_heatmap_vertices > raisin::tactile::kUsbRoiCells * 4 &&
              left_heatmap_vertices < raisin::tactile::kUsbRoiCells * 12,
          "Raisin GUI did not draw only the left 35x22 tactile ROI");
  require(right_heatmap_vertices > raisin::tactile::kUsbRoiCells * 4 &&
              right_heatmap_vertices < raisin::tactile::kUsbRoiCells * 12,
          "Raisin GUI did not draw only the right 35x22 tactile ROI beside the left map");

  manager->shutDown();
  manager.reset();
  resource->guiWindowManager.reset();
  producer.cleanupResources();
  resource.reset();
  network->shutdown();
  network.reset();
  ImGui::DestroyContext();
  std::filesystem::remove_all(private_install);
  std::cout << "PASS: Raisin GuiWindowManager loaded and initialized the staged tactile window\n";
}
