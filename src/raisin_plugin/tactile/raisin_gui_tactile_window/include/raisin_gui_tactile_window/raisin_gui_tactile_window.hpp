#pragma once

#include "imgui/imgui.h"
#include "raisin_gui_base/raisin_gui_window.hpp"
#include "raisin_network/node.hpp"
#include "raisin_tactile_sensor/frame_v2.hpp"
#include "raisin_tactile_sensor/recording.hpp"
#include "std_msgs/msg/string.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace raisin {

class TactileWindow final : public GuiWindow, public Node {
 public:
  TactileWindow(const std::string& title, std::shared_ptr<GuiResource> resource);
  ~TactileWindow() override;

  bool update() final;
  bool init() final;
  bool draw() final;
  bool shutDown() final;
  bool reset() final;

 private:
  using WireFrame = raisin_tactile_sensor::msg::TactileFrame;
  using StatusMessage = std_msgs::msg::String;

  void receiveFrame(const WireFrame::SharedPtr& message, std::size_t sensor_index);
  void receiveStatus(const StatusMessage::SharedPtr& message, std::size_t sensor_index);
  void requestBaseline(const std::string& sensor);
  void startRecording();
  void stopRecording();
  void drawSensor(const char* label, const std::optional<tactile::FrameV2>& frame,
                  const std::string& transport_status, double age_ms,
                  double fps, std::uint64_t rejected);
  static double statusAgeMs(const std::string& status);
  static std::uint64_t statusFrameCount(const std::string& status);
  static std::uint64_t monotonicNowNs();

  std::array<std::optional<tactile::FrameV2>, 2> frames_;
  std::array<std::string, 2> transport_status_;
  std::array<double, 2> age_ms_at_status_{-1., -1.};
  std::array<double, 2> fps_{};
  std::array<std::uint64_t, 2> previous_status_frames_{};
  std::array<bool, 2> has_previous_status_{};
  std::array<std::chrono::steady_clock::time_point, 2> status_received_{};
  std::array<std::uint64_t, 2> rejected_{};
  std::array<std::shared_ptr<Subscriber<WireFrame>>, 2> frame_subscribers_;
  std::array<std::shared_ptr<Subscriber<StatusMessage>>, 2> status_subscribers_;
  std::shared_ptr<Publisher<StatusMessage>> command_publisher_;
  tactile::FrameRecorder recorder_;
  std::filesystem::path record_root_;
  std::filesystem::path record_directory_;
  double display_max_n_estimate_{0.5};
  double stale_timeout_ms_{150.};
  double max_skew_ms_{50.};
  std::string topic_prefix_;
  std::string error_;
  mutable std::mutex mutex_;
  bool initialized_{};
};

}  // namespace raisin
