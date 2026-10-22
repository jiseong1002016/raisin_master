#pragma once

#include "raisin_tactile_sensor/frame.hpp"
#include "raisim/World.hpp"
#include <fstream>

namespace raisin::tactile {

struct SensorConfig {
  std::string name;
  Layout layout;
  std::string parent_link;
  // Empty urdf_frame explicitly selects the calibrated parent/offset transform.
  std::string urdf_frame;
  Eigen::Vector3d offset_m = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  double surface_tolerance_m{};
  double max_force_n{};  // zero: no clamp; saturation marks that cell invalid
  std::string replay_csv;
  std::string taxel_prefix;
  double spring_stiffness{}, spring_damping{}, spring_travel{}, spring_deflection_gate{};
};

class SimBackend {
 public:
  SimBackend(raisim::World& world, raisim::ArticulatedSystem& robot,
             std::vector<SensorConfig> configs);
  std::vector<Frame> sample();
 private:
  raisim::World& world_;
  raisim::ArticulatedSystem& robot_;
  std::vector<SensorConfig> configs_;
  std::vector<std::size_t> bodies_, frames_;
};

// Replay input is already calibrated. CSV rows: stamp_s, then N forces, N validity
// values. No header. Timestamps are acquisition times, never replaced at publish.
class ReplayBackend {
 public:
  explicit ReplayBackend(const std::vector<SensorConfig>& configs);
  std::vector<Frame> sample();
 private:
  std::vector<SensorConfig> configs_;
  std::vector<std::ifstream> files_;
  std::vector<double> previous_;
};

// Reads only robot-side passive joint states; no object/contact-ID qualifier.
class SpringBackend {
 public:
  SpringBackend(raisim::World&, raisim::ArticulatedSystem&, std::vector<SensorConfig>);
  std::vector<Frame> sample();
 private:
  raisim::World& world_;
  raisim::ArticulatedSystem& robot_;
  std::vector<SensorConfig> configs_;
  std::vector<std::vector<size_t>> gc_indices_, gv_indices_;
};

}  // namespace raisin::tactile
