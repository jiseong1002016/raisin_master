// Offscreen RGBD only; callers hold the world mutex across both render passes.
#pragma once
#include "rayrai/Camera.hpp"
#include "rayrai/RayraiWindow.hpp"
#include "rayrai/sdl_shared.hpp"
#include "raisim/sensors/DepthSensor.hpp"
#include "raisim/sensors/RGBSensor.hpp"
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

namespace raisin::plugin {
class RayraiRgbd {
 public:
  RayraiRgbd(raisim::World& world, int width, int height, float focal, double near, double far)
      : width_(width), height_(height) {
    rayrai::sdl::ensureSdlVideoInit();
    RayraiWindow::createOffscreenGlContext(window_, context_, "d405_rayrai_rgbd");
    try {
      std::lock_guard lock(world);
      renderer_ = std::make_unique<RayraiWindow>(world, width, height);
      renderer_->setAsyncMeshLoadingEnabled(false);
      camera_ = std::make_unique<Camera>();
      camera_->resizeRenderTargets(width, height);
      camera_->aspect = float(width) / height;
      camera_->zoom = 2.F * std::atan(.5F * height / focal) * 180.F / float(M_PI);
      camera_->nearPlane = camera_->zNear = near;
      camera_->farPlane = camera_->zFar = far;
      raisim::Vec<3> position; position.setZero();
      raisim::Mat<3, 3> rotation; rotation.setIdentity();
      // These detached sensor objects provide only readback dimensions. Their
      // poses are never updated: the measured EE transform drives camera_.
      raisim::DepthCamera::DepthCameraProperties depth{};
      depth.name = depth.full_name = "d405_depth";
      depth.width = width; depth.height = height; depth.clipNear = near; depth.clipFar = far;
      depth.hFOV = 2. * std::atan(.5 * width / focal);
      depth_ = std::make_unique<raisim::DepthCamera>(depth, nullptr, position, rotation);
      raisim::RGBCamera::RGBCameraProperties rgb{};
      rgb.name = rgb.full_name = "d405_rgb";
      rgb.width = width; rgb.height = height; rgb.clipNear = near; rgb.clipFar = far;
      rgb.hFOV = depth.hFOV;
      rgb_ = std::make_unique<raisim::RGBCamera>(rgb, nullptr, position, rotation);
      bgra_.resize(size_t(width) * height * 4);
    } catch (...) { cleanup(); throw; }
  }
  ~RayraiRgbd() { cleanup(); }

  void render(const Eigen::Vector3d& origin, const Eigen::Matrix3d& optical) {
    camera_->position = {origin.x(), origin.y(), origin.z()};
    camera_->front = {optical(0, 2), optical(1, 2), optical(2, 2)};
    camera_->up = {-optical(0, 1), -optical(1, 1), -optical(2, 1)};
    camera_->right = {optical(0, 0), optical(1, 0), optical(2, 0)};
    renderer_->renderWithExternalCamera(*camera_, {});
    renderer_->renderDepthPlaneDistance(*camera_);
  }
  void read(std::vector<float>& meters, std::vector<std::uint8_t>& rgb) {
    meters.resize(size_t(width_) * height_);
    rgb.resize(meters.size() * 3);
    camera_->getRawImage(*depth_, Camera::SensorStorageMode::CUSTOM_BUFFER,
                         meters.data(), meters.size(), true);
    camera_->getRawImage(*rgb_, Camera::SensorStorageMode::CUSTOM_BUFFER,
                         bgra_.data(), bgra_.size(), true);
    // Rayrai readback is BGRA, while AnyGrasp expects RGB in [0,1].
    for (size_t i = 0; i < meters.size(); ++i) {
      rgb[3*i] = std::uint8_t(bgra_[4*i + 2]);
      rgb[3*i + 1] = std::uint8_t(bgra_[4*i + 1]);
      rgb[3*i + 2] = std::uint8_t(bgra_[4*i]);
    }
  }

 private:
  void cleanup() {
    if (window_ && context_) RayraiWindow::makeOffscreenContextCurrent(window_, context_);
    camera_.reset(); renderer_.reset();
    if (context_) { SDL_GL_DeleteContext(context_); context_ = nullptr; }
    if (window_) { SDL_DestroyWindow(window_); window_ = nullptr; }
  }
  int width_, height_;
  SDL_Window* window_{};
  SDL_GLContext context_{};
  std::unique_ptr<RayraiWindow> renderer_;
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<raisim::DepthCamera> depth_;
  std::unique_ptr<raisim::RGBCamera> rgb_;
  std::vector<char> bgra_;
};
}
