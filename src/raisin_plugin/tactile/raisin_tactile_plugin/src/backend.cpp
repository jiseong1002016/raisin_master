#include "raisin_tactile_plugin/backend.hpp"
#include <sstream>

namespace raisin::tactile {

SimBackend::SimBackend(raisim::World& world, raisim::ArticulatedSystem& robot,
                       std::vector<SensorConfig> configs)
    : world_(world), robot_(robot), configs_(std::move(configs)) {
  std::lock_guard guard(world_);
  for (const auto& config : configs_) {
    config.layout.validate();
    if (!std::isfinite(config.surface_tolerance_m) || config.surface_tolerance_m < 0 ||
        !std::isfinite(config.max_force_n) || config.max_force_n < 0 ||
        !config.offset_m.allFinite() || !config.rotation.allFinite())
      throw std::invalid_argument("invalid tactile simulation configuration");
    bodies_.push_back(robot_.getBodyIdx(config.parent_link));
    frames_.push_back(config.urdf_frame.empty() ? 0 : robot_.getFrameIdxByName(config.urdf_frame));
  }
}

std::vector<Frame> SimBackend::sample() {
  struct ContactSample { Eigen::Vector3d point; std::size_t body; double force; };
  std::vector<ContactSample> contacts;
  std::vector<Eigen::Vector3d> origins;
  std::vector<Eigen::Matrix3d> rotations;
  double stamp;
  {
    // Copy scalars/vectors, never Contact objects (their impulses are pointers).
    // Binning/publishing happen after releasing the 2 kHz simulation mutex.
    std::lock_guard guard(world_);
    stamp = world_.getWorldTime();
    const double dt = world_.getTimeStep();
    if (!std::isfinite(dt) || dt <= 0) throw std::runtime_error("invalid simulation dt");
    for (std::size_t i = 0; i < configs_.size(); ++i) {
      raisim::Vec<3> p;
      raisim::Mat<3, 3> r;
      if (configs_[i].urdf_frame.empty()) {
        robot_.getPosition(bodies_[i], p);
        robot_.getOrientation(bodies_[i], r);
        origins.push_back(p.e() + r.e() * configs_[i].offset_m);
        rotations.push_back(r.e() * configs_[i].rotation);
      } else {
        robot_.getFramePosition(frames_[i], p);
        robot_.getFrameOrientation(frames_[i], r);
        origins.push_back(p.e());
        rotations.push_back(r.e());
      }
    }
    for (const auto& contact : robot_.getContacts()) {
      if (contact.skip() || contact.isSelfCollision()) continue;
      if (std::find(bodies_.begin(), bodies_.end(), contact.getlocalBodyIndex()) == bodies_.end())
        continue;
      contacts.push_back({contact.getPosition().e(), contact.getlocalBodyIndex(),
                         std::abs(contact.getImpulse()[2]) / dt});
    }
  }
  std::vector<Frame> output;
  for (std::size_t i = 0; i < configs_.size(); ++i) {
    const auto& config = configs_[i];
    Frame frame{stamp, std::vector<float>(config.layout.cells(), 0.f),
                std::vector<std::uint8_t>(config.layout.cells(), 1)};
    for (const auto& contact : contacts) {
      if (contact.body != bodies_[i]) continue;
      const Eigen::Vector3d local = rotations[i].transpose() * (contact.point - origins[i]);
      if (std::abs(local.z()) > config.surface_tolerance_m) continue;
      addNormalForce(frame, config.layout, local.x(), local.y(), contact.force);
    }
    for (std::size_t cell = 0; cell < frame.valid.size(); ++cell)
      if (config.max_force_n > 0 && frame.normal_force_n[cell] >= config.max_force_n) {
        frame.normal_force_n[cell] = static_cast<float>(config.max_force_n);
        frame.valid[cell] = 0;
      }
    frame.validate(config.layout);
    output.push_back(std::move(frame));
  }
  return output;
}

ReplayBackend::ReplayBackend(const std::vector<SensorConfig>& configs) : configs_(configs) {
  for (const auto& config : configs_) {
    config.layout.validate();
    files_.emplace_back(config.replay_csv);
    if (!files_.back()) throw std::runtime_error("cannot open tactile replay: " + config.replay_csv);
    previous_.push_back(-1.);
  }
}

std::vector<Frame> ReplayBackend::sample() {
  std::vector<Frame> output;
  for (std::size_t i = 0; i < configs_.size(); ++i) {
    std::string line, token;
    if (!std::getline(files_[i], line)) throw std::runtime_error("tactile replay ended");
    std::istringstream row(line);
    std::vector<double> values;
    while (std::getline(row, token, ',')) {
      std::size_t end{};
      double value = std::stod(token, &end);
      if (end != token.size() || !std::isfinite(value))
        throw std::runtime_error("invalid tactile CSV field");
      values.push_back(value);
    }
    const auto cells = configs_[i].layout.cells();
    if (values.size() != 1 + 2 * cells || values[0] <= previous_[i])
      throw std::runtime_error("invalid tactile CSV shape or non-increasing timestamp");
    Frame frame;
    frame.stamp_s = values[0];
    for (std::size_t cell = 0; cell < cells; ++cell) {
      frame.normal_force_n.push_back(static_cast<float>(values[1 + cell]));
      const auto validity = values[1 + cells + cell];
      if (validity != 0 && validity != 1) throw std::runtime_error("invalid replay validity");
      frame.valid.push_back(static_cast<std::uint8_t>(validity));
    }
    frame.validate(configs_[i].layout);
    previous_[i] = frame.stamp_s;
    output.push_back(std::move(frame));
  }
  return output;
}

}  // namespace raisin::tactile
