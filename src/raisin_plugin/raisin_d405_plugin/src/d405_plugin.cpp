// Both acquisition backends publish the same aligned RGBD grasp input.
#include "raisin_plugin/plugin.hpp"
#include "raisin_network/node.hpp"
#include "raisin_util/raisin_directories.hpp"
#include "raisin_av1_encoder/raisin_av1_encoder.hpp"
#include "raisin_interfaces/srv/get_camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "raisin_interfaces/msg/ffmpeg_packet.hpp"
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <mutex>
#include <thread>
#include <time.h>
#include "rayrai_rgbd.hpp"

namespace raisin::plugin {
namespace {
using Json = nlohmann::json;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using InfoService = raisin_interfaces::srv::GetCameraInfo;
using Packet = raisin_interfaces::msg::FfmpegPacket;
std::uint64_t monotonicNs() {
  timespec t{};
  if (::clock_gettime(CLOCK_MONOTONIC, &t)) throw std::runtime_error("CLOCK_MONOTONIC failed");
  return std::uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
}

class D405Plugin final : public Plugin, public Node {
 public:
  D405Plugin(raisim::World& h, raisim::RaisimServer& hs, raisim::World& s,
             raisim::RaisimServer& ss, GlobalResource& r, const std::string& owner,
             const std::string& title, bool real)
      : Plugin(h, hs, s, ss, r, owner, title, real),
        Node("raisin_d405_plugin", r.paramRoot, r.network) {}
  ~D405Plugin() override {
    running_ = false;
    if (capture_.joinable()) capture_.join();
    cleanupResources();
  }
  bool init() override {
    try {
      prefix_ = param_->as_string("topic_prefix"); frame_ = param_->as_string("frame_id");
      serial_ = param_->as_string("serial");
      width_ = param_->as_int("width"); height_ = param_->as_int("height"); fps_ = param_->as_int("fps");
      roi_ = param_->as_vector_int("roi_depth_xywh");
      near_ = param_->as_double("min_depth_m"); far_ = param_->as_double("max_depth_m");
      minimum_ = param_->as_int("min_valid_pixels");
      compressed_ = param_->as_bool("publish_compressed");
      if (!isRealRobot_) {
        simulation_source_ = std::string((*param_)("simulation_source", std::string("rayrai")));
        if (simulation_source_ != "plane" && simulation_source_ != "world" && simulation_source_ != "rayrai")
          throw std::invalid_argument("simulation_source must be rayrai, plane or world");
        if (simulation_source_ != "plane") initializeWorldCamera();
      }
      if (prefix_.empty() || frame_.empty() || width_ <= 0 || height_ <= 0 || fps_ <= 0 || fps_ > 90
          || roi_.size() != 4 || roi_[0] < 0 || roi_[1] < 0 || roi_[2] <= 0 || roi_[3] <= 0
          || roi_[0] + roi_[2] > width_ || roi_[1] + roi_[3] > height_
          || !std::isfinite(near_) || !std::isfinite(far_) || near_ <= 0 || near_ >= far_
          || minimum_ <= 0 || minimum_ > roi_[2] * roi_[3])
        throw std::invalid_argument("invalid D405 stream/ROI configuration");
      depth_ = createPublisher<Image>(prefix_ + "/depth", 2);
      rgb_pub_ = createPublisher<Image>(prefix_ + "/rgb", 2);
      info_pub_ = createPublisher<Info>(prefix_ + "/depth/camera_info", 2);
      snapshot_ = createPublisher<std_msgs::msg::String>(prefix_ + "/grasp_input", 2);
      status_ = createPublisher<std_msgs::msg::String>(prefix_ + "/status", 2);
      if (compressed_) compressed_pub_ = createPublisher<Packet>(prefix_ + "/depth_compressed", 2);
      info_service_ = createService<InfoService>(prefix_ + "/depth/get_camera_info",
          [this](const InfoService::Request::SharedPtr, InfoService::Response::SharedPtr response) {
            std::lock_guard lock(info_mutex_);
            response->camera_info = info_; response->success = info_.width > 0;
            response->message = response->success ? (isRealRobot_ ? "D405 depth optical" : "synthetic NONREAL") : "awaiting frame";
          }, "core_sensor");
      running_ = true;
      capture_ = std::thread([this] { run(); });
      return true;
    } catch (const std::exception& e) { RSWARN("D405 init: " << e.what()); failed_ = true; return false; }
  }
  bool advance() override { return !failed_; }
  bool reset() override { return !failed_; }
  bool shouldTerminate() override { return failed_; }

 private:
  void initializeWorldCamera() {
    ray_stride_ = int((*param_)("simulation_ray_stride", 16));
    const std::string ee_frame = (*param_)("simulation_ee_frame", std::string("L_ee"));
    if (ray_stride_ < 1 || ray_stride_ > 64 || ee_frame.empty())
      throw std::invalid_argument("invalid simulation ray stride or EE frame");
    const std::string mount_path = (*param_)("simulation_mount_path",
        getResourceDirectory("raisin_d405_plugin") + "/mount.nominal.json");
    std::ifstream file(mount_path);
    const auto mount = Json::parse(file).at("camera_to_ee_base");
    Eigen::Matrix3d base_from_camera;
    Eigen::Vector3d camera_in_base;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) base_from_camera(row, col) = mount.at(row).at(col);
      camera_in_base[row] = mount.at(row).at(3);
    }
    const auto eeMounts = YAML::LoadFile(
        getResourceDirectory("raisin_raipal_ee") + "/ee_mounts.yaml")["mounts"];
    const auto side = ee_frame == "L_ee" ? "left" : ee_frame == "R_ee" ? "right" : "";
    if (!side[0]) throw std::invalid_argument("simulation EE frame must be L_ee or R_ee");
    const auto flangeMount = eeMounts[side];
    if (!flangeMount || flangeMount["parent_link"].as<std::string>() != ee_frame)
      throw std::invalid_argument("canonical EE mount does not match simulation EE frame");
    const auto xyz = flangeMount["xyz_m"].as<std::vector<double>>();
    const auto rpy = flangeMount["rpy_rad"].as<std::vector<double>>();
    if (xyz.size() != 3 || rpy.size() != 3)
      throw std::invalid_argument("canonical EE mount must provide xyz and rpy triples");
    Eigen::Matrix3d ee_from_base;
    ee_from_base = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    camera_in_ee_ = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) + ee_from_base * camera_in_base;
    ee_from_camera_ = ee_from_base * base_from_camera;
    const std::string extrinsic = (*param_)("simulation_camera_to_ee_json", std::string{});
    if (!extrinsic.empty()) {
      const auto transform = Json::parse(extrinsic);
      if (!transform.is_array() || transform.size() != 4)
        throw std::invalid_argument("simulation_camera_to_ee_json requires a 4x4 rigid transform");
      for (int row = 0; row < 4; ++row) {
        if (!transform[row].is_array() || transform[row].size() != 4)
          throw std::invalid_argument("simulation_camera_to_ee_json requires four columns");
        for (int col = 0; col < 4; ++col) {
          const double value = transform[row][col];
          if (!std::isfinite(value) || (row == 3 && std::abs(value - (col == 3 ? 1. : 0.)) > 1e-8))
            throw std::invalid_argument("simulation_camera_to_ee_json is not a finite rigid transform");
          if (row < 3 && col < 3) ee_from_camera_(row, col) = value;
          if (row < 3 && col == 3) camera_in_ee_[row] = value;
        }
      }
      if (!(ee_from_camera_.transpose() * ee_from_camera_).isApprox(Eigen::Matrix3d::Identity(), 1e-6)
          || std::abs(ee_from_camera_.determinant() - 1.) > 1e-6)
        throw std::invalid_argument("simulation_camera_to_ee_json rotation must be right-handed/orthonormal");
    }
    // Body names/indices are immutable for this robot. Controller dependency
    // loading holds its manager mutex while the control loop owns worldSim_.
    // Locking the world here would reverse that order and deadlock the load.
    const auto& bodies = robotSim_->getBodyNames();
    if (std::find(bodies.begin(), bodies.end(), ee_frame) == bodies.end())
      throw std::invalid_argument("simulation EE body is absent: " + ee_frame);
    ee_body_index_ = robotSim_->getBodyIdx(ee_frame);
  }

  void captureWorldDepth(std::vector<float>& depth, const rs2_intrinsics& intr) {
    Eigen::Vector3d origin;
    Eigen::Matrix3d world_from_camera;
    {
      std::lock_guard lock(worldSim_);
      raisim::Vec<3> position;
      raisim::Mat<3, 3> rotation;
      robotSim_->getPosition(ee_body_index_, position);
      robotSim_->getOrientation(ee_body_index_, rotation);
      origin = position.e() + rotation.e() * camera_in_ee_;
      world_from_camera = rotation.e() * ee_from_camera_;
    }
    std::vector<float> row_depth((width_ + ray_stride_ - 1) / ray_stride_);
    // A coarse ray's depth is expanded over its whole block, so only the ray's
    // own pixel is a measurement.  The other pixels of the block are
    // interpolated and must not be reported as measured points.
    for (int y = 0; y < height_ && running_; y += ray_stride_) {
      {
        // Bound each lock to one coarse row so the 2 kHz plant can advance.
        std::lock_guard lock(worldSim_);
        for (int x = 0; x < width_; x += ray_stride_) {
          const double u = x + .5 * (std::min(ray_stride_, width_ - x) - 1);
          const double v = y + .5 * (std::min(ray_stride_, height_ - y) - 1);
          const Eigen::Vector3d ray((u - intr.ppx) / intr.fx, (v - intr.ppy) / intr.fy, 1.);
          const double norm = ray.norm();
          const auto& hits = worldSim_.rayTest(origin, world_from_camera * (ray / norm), far_ * norm);
          float z = 0.F;
          if (hits.size()) {
            const double value = world_from_camera.col(2).dot(hits[0].getPosition() - origin);
            if (std::isfinite(value) && value >= near_ && value <= far_) z = value;
          }
          row_depth[x / ray_stride_] = z;
        }
      }
      // ponytail: nearest-neighbor expansion keeps ray cost bounded; this is not a D405 optics model.
      for (int block_y = y; block_y < std::min(height_, y + ray_stride_); ++block_y)
        for (int x = 0; x < width_; ++x) depth[block_y * width_ + x] = row_depth[x / ray_stride_];
    }
  }

  void publish(const std::vector<float>& meters, const rs2_intrinsics& intrinsics,
               const std::vector<std::uint8_t>& rgb = {}, std::uint64_t stamp = 0) {
    if (!stamp) stamp = monotonicNs();
    const bool rgbd = rgb.size() == meters.size() * 3;
    Image image{};
    image.header.frame_id = frame_;
    image.header.stamp.sec = stamp / 1000000000ULL;
    image.header.stamp.nanosec = stamp % 1000000000ULL;
    image.width = width_; image.height = height_; image.step = width_ * sizeof(float);
    image.encoding = "32FC1"; image.is_bigendian = std::endian::native == std::endian::big;
    image.data.resize(meters.size() * sizeof(float));
    std::memcpy(image.data.data(), meters.data(), image.data.size());
    Info info{};
    info.header = image.header; info.width = width_; info.height = height_;
    info.distortion_model = "plumb_bob";
    info.d.assign(intrinsics.coeffs, intrinsics.coeffs + 5);
    info.k = {intrinsics.fx, 0., intrinsics.ppx, 0., intrinsics.fy, intrinsics.ppy, 0., 0., 1.};
    info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    info.p = {intrinsics.fx, 0., intrinsics.ppx, 0., 0., intrinsics.fy, intrinsics.ppy, 0., 0., 0., 1., 0.};
    { std::lock_guard lock(info_mutex_); info_ = info; }
    depth_->publish(image); info_pub_->publish(info);
    if (rgbd) {
      Image color{}; color.header = image.header;
      color.width = width_; color.height = height_; color.step = width_ * 3;
      color.encoding = "rgb8"; color.is_bigendian = false; color.data = rgb;
      rgb_pub_->publish(color);
    }
    std::vector<float> valid;
    for (int y = roi_[1]; y < roi_[1] + roi_[3]; ++y)
      for (int x = roi_[0]; x < roi_[0] + roi_[2]; ++x) {
        const auto z = meters[y * width_ + x];
        if (std::isfinite(z) && z >= near_ && z <= far_) valid.push_back(z);
      }
    // Bound the grasp snapshot independently of stream/ROI resolution.
    Json points = Json::array(), colors = Json::array();
    const std::size_t point_limit = rgbd ? 12000 : 1200;
    const int sample_stride = std::max(1, int(std::ceil(std::sqrt(
      double(roi_[2]) * roi_[3] / (rgbd ? 12000. : 1000.)))));
    for (int y = roi_[1]; y < roi_[1] + roi_[3] && points.size() < point_limit; y += sample_stride)
      for (int x = roi_[0]; x < roi_[0] + roi_[2] && points.size() < point_limit; x += sample_stride) {
        // A block-expanded pixel carries a neighbour's depth, and deprojecting
        // it at its own pixel centre places the point off the surface it
        // belongs to.  Report the block's sampled pixel instead.
        int sample_x = x, sample_y = y;
        const bool world_depth = !isRealRobot_ && simulation_source_ == "world";
        if (world_depth) {
          sample_x -= sample_x % ray_stride_;
          sample_y -= sample_y % ray_stride_;
        }
        const float z = meters[sample_y * width_ + sample_x];
        if (!std::isfinite(z) || z < near_ || z > far_) continue;
        const float pixel[2]{
          sample_x + (world_depth ? .5F * (std::min(ray_stride_, width_ - sample_x) - 1) : 0.F),
          sample_y + (world_depth ? .5F * (std::min(ray_stride_, height_ - sample_y) - 1) : 0.F)};
        float xyz[3]; rs2_deproject_pixel_to_point(xyz, &intrinsics, pixel, z);
        points.push_back({xyz[0], xyz[1], xyz[2]});
        if (rgbd) {
          const auto offset = size_t(sample_y * width_ + sample_x) * 3;
          colors.push_back({rgb[offset], rgb[offset + 1], rgb[offset + 2]});
        }
      }
    Json point = nullptr, median = nullptr;
    if (valid.size() >= static_cast<std::size_t>(minimum_)) {
      auto middle = valid.begin() + valid.size() / 2;
      std::nth_element(valid.begin(), middle, valid.end());
      median = *middle;
      const float pixel[2]{roi_[0] + .5F * (roi_[2] - 1), roi_[1] + .5F * (roi_[3] - 1)};
      float xyz[3]; rs2_deproject_pixel_to_point(xyz, &intrinsics, pixel, *middle);
      point = {xyz[0], xyz[1], xyz[2]};
    }
    std_msgs::msg::String message;
    message.data = Json{{"schema", rgbd ? "raisin_d405_grasp_input_v2" : "raisin_d405_grasp_input_v1"}, {"valid", !point.is_null()},
      {"acquisition_source", isRealRobot_ ? "d405" : simulation_source_}, {"colors_rgb_u8", colors},
      {"is_simulation", !isRealRobot_}, {"point_camera_m", point}, {"depth_median_m", median},
      {"points_camera_m", points}, {"points_pixel_stride", sample_stride},
      {"simulation_source", isRealRobot_ ? Json(nullptr) : Json(simulation_source_)},
      {"valid_pixels", valid.size()}, {"roi_depth_xywh", roi_}, {"sequence", sequence_++},
      {"stamp_monotonic_ns", stamp}, {"serial", serial_}, {"frame_id", frame_},
      {"depth_intrinsics", {{"width", width_}, {"height", height_}, {"fx", intrinsics.fx},
        {"fy", intrinsics.fy}, {"ppx", intrinsics.ppx}, {"ppy", intrinsics.ppy},
        {"model", int(intrinsics.model)}, {"coeffs", info.d}}}}.dump();
    snapshot_->publish(message);
    if (compressed_pub_ && compressed_pub_->getNumberOfSubscribers() > 0) {
      if (!encoder_) encoder_ = std::make_unique<Av1Encoder>(width_, height_, fps_, 100000, 35);
      cv::Mat depth(height_, width_, CV_32F, const_cast<float*>(meters.data())), gray, rgb;
      depth.convertTo(gray, CV_8U, 255. / (far_ - near_), -near_ * 255. / (far_ - near_));
      cv::cvtColor(gray, rgb, cv::COLOR_GRAY2RGB);
      Packet packet{}; packet.header = image.header; packet.encoding = "libaom-av1";
      packet.width = width_; packet.height = height_; packet.is_bigendian = false;
      packet.pts = encoder_->encode_frame(rgb, packet.data); compressed_pub_->publish(packet);
    }
  }
  void run() {
    try {
      if (!isRealRobot_) {
        serial_ = "simulation_" + simulation_source_;
        rs2_intrinsics intr{}; intr.width = width_; intr.height = height_;
        intr.fx = intr.fy = 400.F; intr.ppx = .5F * (width_ - 1); intr.ppy = .5F * (height_ - 1);
        intr.model = RS2_DISTORTION_NONE;
        std::vector<float> depth(width_ * height_, .32F);
        for (int y = roi_[1]; y < roi_[1] + roi_[3]; ++y)
          for (int x = roi_[0]; x < roi_[0] + roi_[2]; ++x) depth[y * width_ + x] = .22F;
        auto next = std::chrono::steady_clock::now();
        const auto period = std::chrono::nanoseconds(1000000000ULL / fps_);
        std::unique_ptr<RayraiRgbd> renderer;
        std::vector<std::uint8_t> rgb;
        if (simulation_source_ == "rayrai")
          renderer = std::make_unique<RayraiRgbd>(worldSim_, width_, height_, intr.fx, near_, far_);
        while (running_) {
          std::uint64_t stamp = 0;
          if (simulation_source_ == "world") captureWorldDepth(depth, intr);
          if (renderer) {
            {
              std::lock_guard lock(worldSim_);
              raisim::Vec<3> position; raisim::Mat<3, 3> rotation;
              robotSim_->getPosition(ee_body_index_, position);
              robotSim_->getOrientation(ee_body_index_, rotation);
              stamp = monotonicNs();
              renderer->render(position.e() + rotation.e() * camera_in_ee_, rotation.e() * ee_from_camera_);
            }
            renderer->read(depth, rgb);
          }
          if (!running_) break;
          publish(depth, intr, rgb, stamp);
          next = std::max(next + period, std::chrono::steady_clock::now());
          std::this_thread::sleep_until(next);
        }
        return;
      }
      rs2::context context;
      std::vector<std::string> matches;
      for (auto&& device : context.query_devices()) {
        const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
        const std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (name.ends_with("D405") && (serial_.empty() || serial_ == serial)) matches.push_back(serial);
      }
      if (matches.size() != 1) throw std::runtime_error("exactly one matching USB D405 is required; set serial");
      serial_ = matches[0];
      rs2::pipeline pipeline(context); rs2::config config;
      config.enable_device(serial_); config.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
      config.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);
      pipeline.start(config);
      try {
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        auto last = std::chrono::steady_clock::now();
        while (running_) {
          rs2::frameset frames;
          if (!pipeline.try_wait_for_frames(&frames, 200)) {
            if (std::chrono::steady_clock::now() - last > std::chrono::seconds(2)) throw std::runtime_error("D405 depth timeout");
            continue;
          }
          last = std::chrono::steady_clock::now();
          const auto stamp = monotonicNs();
          frames = align_to_depth.process(frames);
          auto depth = frames.get_depth_frame();
          auto color = frames.get_color_frame();
          if (!depth || !color || color.get_width() != width_ || color.get_height() != height_)
            throw std::runtime_error("D405 did not provide depth-aligned RGBD");
          const auto intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
          const float units = depth.get_units();
          std::vector<float> meters(width_ * height_);
          std::vector<std::uint8_t> rgb(size_t(width_) * height_ * 3);
          for (int y = 0; y < height_; ++y) {
            const auto* row = reinterpret_cast<const std::uint16_t*>(
                static_cast<const std::uint8_t*>(depth.get_data()) + y * depth.get_stride_in_bytes());
            for (int x = 0; x < width_; ++x) meters[y * width_ + x] = row[x] * units;
            std::memcpy(rgb.data() + size_t(y) * width_ * 3,
                        static_cast<const std::uint8_t*>(color.get_data()) + y * color.get_stride_in_bytes(),
                        size_t(width_) * 3);
          }
          publish(meters, intr, rgb, stamp);
        }
      } catch (...) { pipeline.stop(); throw; }
      pipeline.stop();
    } catch (const std::exception& e) {
      std_msgs::msg::String message; message.data = e.what(); status_->publish(message);
      RSWARN("D405 capture: " << e.what()); failed_ = true;
    }
  }
  std::atomic<bool> running_{false}, failed_{false};
  std::thread capture_; std::mutex info_mutex_; Info info_{};
  std::string prefix_, frame_, serial_; std::vector<int> roi_;
  std::string simulation_source_{"rayrai"};
  int ray_stride_{16}; std::size_t ee_body_index_{};
  Eigen::Vector3d camera_in_ee_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d ee_from_camera_ = Eigen::Matrix3d::Identity();
  int width_{}, height_{}, fps_{}, minimum_{}; double near_{}, far_{}; bool compressed_{};
  std::uint64_t sequence_{}; std::unique_ptr<Av1Encoder> encoder_;
  std::shared_ptr<Publisher<Image>> depth_, rgb_pub_; std::shared_ptr<Publisher<Info>> info_pub_;
  std::shared_ptr<Publisher<std_msgs::msg::String>> snapshot_, status_;
  std::shared_ptr<Publisher<Packet>> compressed_pub_; std::shared_ptr<Service<InfoService>> info_service_;
};
}

extern "C" raisin::plugin::Plugin* create_with_owner_title_robot(
    raisim::World& h, raisim::RaisimServer& hs, raisim::World& s, raisim::RaisimServer& ss,
    raisin::GlobalResource& r, const std::string& owner, const std::string& title, bool real) {
  return new raisin::plugin::D405Plugin(h, hs, s, ss, r, owner, title, real);
}
extern "C" void destroy(raisin::plugin::Plugin* instance) { delete instance; }
