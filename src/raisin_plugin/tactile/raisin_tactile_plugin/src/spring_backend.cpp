#include "raisin_tactile_plugin/backend.hpp"
#include <set>

namespace raisin::tactile {

SpringBackend::SpringBackend(raisim::World& world, raisim::ArticulatedSystem& robot,
                             std::vector<SensorConfig> configs)
    : world_(world), robot_(robot), configs_(std::move(configs)) {
  std::lock_guard guard(world_);
  std::set<std::string> used;
  for (const auto& config : configs_) {
    config.layout.validate();
    if (config.taxel_prefix.empty() || !std::isfinite(config.spring_stiffness) ||
        !std::isfinite(config.spring_damping) || !std::isfinite(config.spring_travel) ||
        !std::isfinite(config.spring_deflection_gate) ||
        !std::isfinite(config.max_force_n) || config.max_force_n < 0 ||
        config.spring_stiffness <= 0 || config.spring_damping < 0 ||
        config.spring_travel <= 0 || config.spring_deflection_gate < 0 ||
        config.spring_deflection_gate >= config.spring_travel)
      throw std::invalid_argument("invalid tactile spring configuration");
    std::vector<size_t> gc, gv;
    for (int row = 0; row < config.layout.rows; ++row)
      for (int col = 0; col < config.layout.cols; ++col) {
        const auto name = config.taxel_prefix + std::to_string(row) + "_" + std::to_string(col);
        if (!used.insert(name).second) throw std::invalid_argument("duplicate tactile taxel");
        // getBodyIdx alone on an unknown name can be fatal in this SDK.
        const auto& names = robot_.getMovableJointNames();
        if (std::find(names.begin(), names.end(), name) == names.end())
          throw std::invalid_argument("missing tactile taxel: " + name);
        const auto body = robot_.getBodyIdx(name);
        // SDK 1.1.8 JointRef::getType uses the velocity index, whereas
        // getJointType expects a body/joint index (different for a fixed base).
        if (robot_.getJointType(body) != raisim::Joint::Type::PRISMATIC)
          throw std::invalid_argument("tactile taxel must be prismatic: " + name);
        gc.push_back(robot_.getMappingFromBodyIndexToGeneralizedCoordinateIndex().at(body));
        gv.push_back(robot_.getMappingFromBodyIndexToGeneralizedVelocityIndex().at(body));
      }
    gc_indices_.push_back(std::move(gc));
    gv_indices_.push_back(std::move(gv));
  }
}

std::vector<Frame> SpringBackend::sample() {
  // No pointers/references to live physics state survive the world lock.
  Eigen::VectorXd q, v;
  double stamp;
  {
    std::lock_guard guard(world_);
    stamp = world_.getWorldTime();
    q = robot_.getGeneralizedCoordinate().e();
    v = robot_.getGeneralizedVelocity().e();
  }
  std::vector<Frame> output;
  for (size_t side = 0; side < configs_.size(); ++side) {
    const auto& c = configs_[side];
    Frame f{stamp, std::vector<float>(c.layout.cells(), 0.f),
            std::vector<std::uint8_t>(c.layout.cells(), 1)};
    for (int row = 0; row < c.layout.rows; ++row)
      for (int col = 0; col < c.layout.cols; ++col) {
        const size_t source = row * c.layout.cols + col;
        const size_t cell = (c.layout.flip_rows ? c.layout.rows - 1 - row : row) * c.layout.cols +
                            (c.layout.flip_cols ? c.layout.cols - 1 - col : col);
        const double position = q[gc_indices_[side][source]], velocity = v[gv_indices_[side][source]];
        if (!std::isfinite(position) || !std::isfinite(velocity)) {
          f.valid[cell] = 0;
          continue;
        }
        const double deflection = std::clamp(-position, 0., c.spring_travel);
        if (deflection <= c.spring_deflection_gate) continue;
        double force = std::max(0., c.spring_stiffness * deflection - c.spring_damping * velocity);
        if (c.max_force_n > 0 && force >= c.max_force_n) {
          force = c.max_force_n;
          f.valid[cell] = 0;
        }
        f.normal_force_n[cell] = static_cast<float>(force);
      }
    f.validate(c.layout);
    output.push_back(std::move(f));
  }
  return output;
}

}  // namespace raisin::tactile
