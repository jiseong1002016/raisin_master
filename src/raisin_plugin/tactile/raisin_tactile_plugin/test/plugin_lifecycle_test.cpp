#include "raisin_plugin/plugin.hpp"
#include "raisin_network/raisin.hpp"
#include "raisin_util/raisin_directories.hpp"
#include "raisin_tactile_sensor/registration.hpp"
#include <chrono>
#include <thread>
#include <numeric>
#include <dlfcn.h>
#include <filesystem>
#include <iostream>
#include <unistd.h>

using namespace raisin;
void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

int main(int argc, char** argv) {
  if (argc != 4) return 2;  // URDF, exact staged .so, exact staged install
  require(std::filesystem::weakly_canonical(getInstallDirectory()) ==
          std::filesystem::weakly_canonical(argv[3]), "refusing non-staged parameter lookup");
  raisinInit();
  raisim::World hub, sim;
  hub.addArticulatedSystem(argv[1])->setName("robot");
  auto* simulatedRobot = sim.addArticulatedSystem(argv[1]);
  simulatedRobot->setName("robot");
  sim.setTimeStep(.001);
  sim.addGround();
  simulatedRobot->setGeneralizedCoordinate(Eigen::VectorXd::Zero(simulatedRobot->getGeneralizedCoordinateDim()));
  simulatedRobot->setGeneralizedVelocity(Eigen::VectorXd::Zero(simulatedRobot->getDOF()));
  simulatedRobot->setGeneralizedForce(Eigen::VectorXd::Zero(simulatedRobot->getDOF()));
  for (int i = 0; i < 500; ++i) sim.integrate();
  raisim::RaisimServer hubServer(&hub), simServer(&sim);
  auto& resources = GlobalResource::getInstance();
  resources.network = std::make_shared<Network>("tactile_lifecycle_" + std::to_string(getpid()),
      "test", std::vector<std::vector<std::string>>{{"main"}, {"plugin"}, {"core_sensor"}});
  using Create = plugin::Plugin* (*)(raisim::World&, raisim::RaisimServer&,
      raisim::World&, raisim::RaisimServer&, GlobalResource&,
      const std::string&, const std::string&, bool);
  using Destroy = void (*)(plugin::Plugin*);
  for (int round = 0; round < 2; ++round) {
    void* library = dlopen(argv[2], RTLD_NOW | RTLD_LOCAL);
    require(library, "dlopen failed");
    auto create = reinterpret_cast<Create>(dlsym(library, "create_with_owner_title_robot"));
    auto destroy = reinterpret_cast<Destroy>(dlsym(library, "destroy"));
    require(create && destroy, "factory ABI missing");
    auto* instance = create(hub, hubServer, sim, simServer, resources, "test", "tactile:0", false);
    require(instance->init() && instance->advance() && instance->reset() &&
            !instance->shouldTerminate(), "disabled lifecycle failed");
    destroy(instance);
    instance = create(hub, hubServer, sim, simServer, resources, "test", "tactile:0", false);
    auto& config = resources.paramRoot["raisin_tactile_plugin"];
    config("enabled") = true;
    config("backend") = std::string("sim");
    config("topic_prefix") = "tactile_lifecycle_" + std::to_string(getpid()) + "/" + std::to_string(round);
    auto& pad = config["sensors"]["pad"];
    pad("frame", std::string("pad_tactile"));
    pad("rows", 3); pad("cols", 5);
    pad("size_u_m", .10); pad("size_v_m", .06);
    pad("flip_rows", false); pad("flip_cols", false);
    pad("parent_link", std::string("pad"));
    pad("urdf_frame", std::string(""));
    pad("offset_m", std::vector<double>{0., 0., -.01});
    pad("rpy_rad", std::vector<double>{std::acos(-1.), 0., 0.});
    pad("surface_tolerance_m", .003); pad("max_force_n", 0.);
    pad("replay_csv", std::string(""));
    std::map<std::string, std::shared_ptr<robot::sensor::Sensor>> registry;
    tactile::registerReceivers(registry, hub, hubServer, config, resources.network);
    auto receiver = std::dynamic_pointer_cast<tactile::Receiver>(registry.at("tactile/pad"));
    receiver->createCommunication(resources.network);
    require(instance->init(), "enabled sim plugin init failed");
    std::optional<tactile::Frame> frame;
    for (int attempt = 0; attempt < 200 && !frame; ++attempt) {
      sim.integrate();
      require(instance->advance(), "enabled sim plugin advance failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      frame = receiver->latest(sim.getWorldTime(), .05);
    }
    require(frame.has_value(), "enabled plugin did not reach core Receiver");
    const float force = std::accumulate(frame->normal_force_n.begin(), frame->normal_force_n.end(), 0.f);
    require(force > 8 && force < 12, "enabled plugin lost known load");
    require(instance->reset(), "enabled sim reset failed");
    destroy(instance);
    // A controller may still own Receiver after plugin unload. Freshness expires.
    require(!receiver->latest(sim.getWorldTime() + 1., .05), "unloaded plugin appears fresh");
    receiver->shutDown();
    registry.clear();
    receiver.reset();
    // Same constructor ABI must carry real=true. Missing USB must fail explicitly.
    instance = create(hub, hubServer, sim, simServer, resources, "test", "tactile:0", true);
    resources.paramRoot["raisin_tactile_plugin"]("enabled") = true;
    require(!instance->init() && instance->shouldTerminate() && !instance->reset(),
            "real mode silently accepted unavailable USB");
    destroy(instance);
    dlclose(library);
  }
  resources.network->shutdown();
  resources.network.reset();
  std::cout << "PASS: repeated factory lifecycle, enabled sim -> core Receiver/load/stale, real mode rejects non-USB backend\n";
}
