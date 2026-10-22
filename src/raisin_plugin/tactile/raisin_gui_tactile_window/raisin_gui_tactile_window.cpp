#include "raisin_gui_tactile_window/raisin_gui_tactile_window.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <time.h>

namespace raisin {

TactileWindow::TactileWindow(const std::string& title, std::shared_ptr<GuiResource> resource)
    : GuiWindow(title, resource),
      Node("raisin_gui_tactile_window", resource->paramRoot, resource->network) {}

TactileWindow::~TactileWindow() {
  shutDown();
}

bool TactileWindow::init() {
  try {
    topic_prefix_ = param_->as_string("topic_prefix");
    record_root_ = param_->as_string("record_root");
    display_max_n_estimate_ = param_->as_double("display_max_n_estimate");
    stale_timeout_ms_ = param_->as_double("stale_timeout_ms");
    max_skew_ms_ = param_->as_double("max_skew_ms");
    if (topic_prefix_.empty() || display_max_n_estimate_ <= 0 ||
        stale_timeout_ms_ <= 0 || max_skew_ms_ < 0)
      throw std::invalid_argument("invalid tactile GUI parameters");
    for (std::size_t i = 0; i < 2; ++i) {
      const auto sensor = i == 0 ? "left" : "right";
      frame_subscribers_[i] = createSubscriber<WireFrame>(topic_prefix_ + "/" + sensor, nullptr,
          [this, i](const WireFrame::SharedPtr message) { receiveFrame(message, i); }, "main");
      status_subscribers_[i] = createSubscriber<StatusMessage>(
          topic_prefix_ + "/status/" + sensor, nullptr,
          [this, i](const StatusMessage::SharedPtr message) { receiveStatus(message, i); }, "main");
    }
    command_publisher_ = createPublisher<StatusMessage>(
        topic_prefix_ + "/" + param_->as_string("command_suffix"), 5);
    initialized_ = true;
    return true;
  } catch (const std::exception& error) {
    error_ = error.what();
    return false;
  }
}

bool TactileWindow::update() { return true; }

bool TactileWindow::draw() {
  if (!open) return true;
  if (!initialized_) {
    ImGui::Begin(title.c_str(), &open);
    ImGui::TextWrapped("Tactile window setup failed: %s", error_.c_str());
    ImGui::End();
    return open;
  }

  std::array<std::optional<tactile::FrameV2>, 2> frames;
  std::array<std::string, 2> statuses;
  std::array<double, 2> ages{};
  std::array<double, 2> fps{};
  std::array<std::uint64_t, 2> rejected{};
  bool recording = false;
  std::string record_path;
  std::string error;
  {
    std::lock_guard lock(mutex_);
    frames = frames_;
    statuses = transport_status_;
    fps = fps_;
    rejected = rejected_;
    recording = recorder_.active();
    record_path = record_directory_.string();
    error = error_;
    for (std::size_t i = 0; i < 2; ++i) {
      ages[i] = age_ms_at_status_[i];
      if (ages[i] >= 0.) {
        ages[i] += std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - status_received_[i]).count();
      }
    }
  }

  if (ImGui::Begin(title.c_str(), &open)) {
    ImGui::Text("Provisional force display: N_estimate; measured calibration is required.");
    if (ImGui::Button("Capture unloaded baseline: left")) requestBaseline("left");
    ImGui::SameLine();
    if (ImGui::Button("Capture unloaded baseline: right")) requestBaseline("right");
    ImGui::SameLine();
    if (ImGui::Button("Capture both")) requestBaseline("both");
    ImGui::SameLine();
    if (!recording) {
      if (ImGui::Button("Start frame recording")) startRecording();
    } else if (ImGui::Button("Stop frame recording")) {
      stopRecording();
    }
    if (recording) ImGui::Text("Recording: %s", record_path.c_str());
    if (!error.empty()) ImGui::TextWrapped("Status: %s", error.c_str());

    bool pair_ready = false;
    double skew_ms = -1.;
    if (frames[0] && frames[1]) {
      const auto left_ns = frames[0]->stamp_monotonic_ns;
      const auto right_ns = frames[1]->stamp_monotonic_ns;
      const auto skew_ns = left_ns > right_ns ? left_ns - right_ns : right_ns - left_ns;
      skew_ms = static_cast<double>(skew_ns) / 1e6;
      pair_ready = frames[0]->baseline_valid && frames[1]->baseline_valid &&
                   ages[0] >= 0. && ages[1] >= 0. &&
                   ages[0] <= stale_timeout_ms_ && ages[1] <= stale_timeout_ms_ &&
                   std::any_of(frames[0]->valid.begin(), frames[0]->valid.end(),
                               [](std::uint8_t value) { return value == 1; }) &&
                   std::any_of(frames[1]->valid.begin(), frames[1]->valid.end(),
                               [](std::uint8_t value) { return value == 1; }) &&
                   skew_ms <= max_skew_ms_;
    }
    ImGui::Text("Latest pair: %s | skew %.1f / %.1f ms",
                pair_ready ? "ready" : "waiting", skew_ms, max_skew_ms_);

    if (ImGui::BeginTable("tactile_sensors", 2, ImGuiTableFlags_BordersInnerV)) {
      ImGui::TableNextColumn();
      drawSensor("left", frames[0], statuses[0], ages[0], fps[0], rejected[0]);
      ImGui::TableNextColumn();
      drawSensor("right", frames[1], statuses[1], ages[1], fps[1], rejected[1]);
      ImGui::EndTable();
    }
  }
  ImGui::End();
  return open;
}

bool TactileWindow::shutDown() {
  stopRecording();
  cleanupResources();
  frame_subscribers_ = {};
  status_subscribers_ = {};
  command_publisher_.reset();
  initialized_ = false;
  return true;
}

bool TactileWindow::reset() {
  std::lock_guard lock(mutex_);
  frames_ = {};
  transport_status_ = {};
  age_ms_at_status_ = {-1., -1.};
  fps_ = {};
  previous_status_frames_ = {};
  has_previous_status_ = {};
  rejected_ = {};
  return true;
}

void TactileWindow::receiveFrame(const WireFrame::SharedPtr& message,
                                 std::size_t sensor_index) {
  try {
    const auto sensor = sensor_index == 0 ? "left" : "right";
    auto frame = tactile::fromMessage(*message, sensor);
    std::lock_guard lock(mutex_);
    const auto& previous = frames_[sensor_index];
    if (previous && (frame.stamp_monotonic_ns <= previous->stamp_monotonic_ns ||
        (frame.session_id == previous->session_id && frame.sequence <= previous->sequence))) {
      ++rejected_[sensor_index];
      return;
    }
    frames_[sensor_index] = frame;
    if (recorder_.active()) {
      try {
        recorder_.write(frame);
      } catch (const std::exception& error) {
        recorder_.stop();
        error_ = error.what();
      }
    }
  } catch (const std::invalid_argument&) {
    std::lock_guard lock(mutex_);
    ++rejected_[sensor_index];
  }
}

void TactileWindow::receiveStatus(const StatusMessage::SharedPtr& message,
                                  std::size_t sensor_index) {
  std::lock_guard lock(mutex_);
  const auto received = std::chrono::steady_clock::now();
  const auto frames = statusFrameCount(message->data);
  if (has_previous_status_[sensor_index] && frames >= previous_status_frames_[sensor_index]) {
    const double elapsed = std::chrono::duration<double>(
        received - status_received_[sensor_index]).count();
    if (elapsed > 0.)
      fps_[sensor_index] = static_cast<double>(frames - previous_status_frames_[sensor_index]) /
                           elapsed;
  } else {
    fps_[sensor_index] = 0.;
  }
  previous_status_frames_[sensor_index] = frames;
  has_previous_status_[sensor_index] = true;
  transport_status_[sensor_index] = message->data;
  age_ms_at_status_[sensor_index] = statusAgeMs(message->data);
  status_received_[sensor_index] = received;
}

void TactileWindow::requestBaseline(const std::string& sensor) {
  if (!command_publisher_) return;
  StatusMessage command;
  command.data = "capture_baseline:" + sensor;
  command_publisher_->publish(command);
}

void TactileWindow::startRecording() {
  std::lock_guard lock(mutex_);
  if (record_root_.empty()) {
    error_ = "Set record_root to the same run directory as the USB plugin.";
    return;
  }
  try {
    record_directory_ = record_root_ / ("frames_" + std::to_string(monotonicNowNs()));
    recorder_.start(record_directory_);
    error_.clear();
  } catch (const std::exception& error) {
    error_ = error.what();
  }
}

void TactileWindow::stopRecording() {
  std::lock_guard lock(mutex_);
  recorder_.stop();
}

void TactileWindow::drawSensor(const char* label,
                               const std::optional<tactile::FrameV2>& frame,
                               const std::string& transport_status, double age_ms,
                               double fps, std::uint64_t rejected) {
  ImGui::Text("%s", label);
  ImGui::Separator();
  ImGui::TextWrapped("USB: %s", transport_status.empty() ? "waiting for status" :
                                                            transport_status.c_str());
  ImGui::Text("Age: %.1f ms | rejected frames: %llu", age_ms,
              static_cast<unsigned long long>(rejected));
  ImGui::Text("Receive rate: %.1f FPS", fps);
  if (!frame) {
    ImGui::Text("No tactile frame received.");
    return;
  }
  ImGui::Text("Session %llu | seq %llu | %s | baseline %s",
      static_cast<unsigned long long>(frame->session_id),
      static_cast<unsigned long long>(frame->sequence), frame->calibration_status.c_str(),
      frame->baseline_valid ? "ready" : "pending");
  ImGui::Text("%s | %.3f %s/code | polarity %+d",
      frame->calibration_id.c_str(), frame->code_to_force_n_per_code,
      frame->force_unit.c_str(), frame->polarity);

  const bool physical_usb_grid = frame->rows == tactile::kUsbRows &&
                                 frame->cols == tactile::kUsbCols;
  const std::size_t row_first = physical_usb_grid ? tactile::kUsbRoiRxFirst : 0;
  const std::size_t row_last = physical_usb_grid ? tactile::kUsbRoiRxLast : frame->rows - 1;
  const std::size_t col_first = physical_usb_grid ? tactile::kUsbRoiTxFirst : 0;
  const std::size_t col_last = physical_usb_grid ? tactile::kUsbRoiTxLast : frame->cols - 1;
  const std::size_t display_rows = row_last - row_first + 1;
  const std::size_t display_cols = col_last - col_first + 1;
  if (physical_usb_grid)
    ImGui::Text("Display ROI: RX rows %zu-%zu / TX columns %zu-%zu (%zux%zu)",
                row_first, row_last, col_first, col_last, display_rows, display_cols);

  const float width = ImGui::GetContentRegionAvail().x;
  const float cell = std::max(1.F, std::min(width / static_cast<float>(display_cols), 4.F));
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  auto* draw_list = ImGui::GetWindowDrawList();
  for (std::size_t row = row_first; row <= row_last; ++row) {
    for (std::size_t col = col_first; col <= col_last; ++col) {
      const auto index = row * frame->cols + col;
      ImU32 color = IM_COL32(48, 48, 52, 255);
      if (frame->valid[index]) {
        const float level = std::clamp(frame->normal_force_n[index] /
                                           static_cast<float>(display_max_n_estimate_),
                                       0.F, 1.F);
        color = IM_COL32(static_cast<int>(255.F * level),
                         static_cast<int>(80.F * (1.F - level)),
                         static_cast<int>(255.F * (1.F - level)), 255);
      }
      const ImVec2 a(origin.x + static_cast<float>(col - col_first) * cell,
                     origin.y + static_cast<float>(row - row_first) * cell);
      const ImVec2 b(a.x + cell - 0.5F, a.y + cell - 0.5F);
      draw_list->AddRectFilled(a, b, color);
    }
  }
  ImGui::Dummy(ImVec2(cell * static_cast<float>(display_cols),
                      cell * static_cast<float>(display_rows)));
}

double TactileWindow::statusAgeMs(const std::string& status) {
  const auto key = status.find("age_ms=");
  if (key == std::string::npos) return -1.;
  try {
    const auto begin = key + 7;
    return std::stod(status.substr(begin));
  } catch (...) {
    return -1.;
  }
}

std::uint64_t TactileWindow::statusFrameCount(const std::string& status) {
  const auto key = status.find("frames=");
  if (key == std::string::npos) return 0;
  try {
    return std::stoull(status.substr(key + 7));
  } catch (...) {
    return 0;
  }
}

std::uint64_t TactileWindow::monotonicNowNs() {
  timespec now{};
  if (::clock_gettime(CLOCK_MONOTONIC, &now) != 0)
    throw std::runtime_error("cannot read CLOCK_MONOTONIC for recording name");
  return static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}

}  // namespace raisin

extern "C" raisin::GuiWindow* create(const std::string& title,
                                      std::shared_ptr<raisin::GuiResource> resource) {
  return new raisin::TactileWindow(title, resource);
}

extern "C" void destroy(raisin::GuiWindow* window) { delete window; }
