#include "raisin_plugin/plugin.hpp"
#include "raisin_network/raisin.hpp"
#include "raisin_tactile_plugin/usb_protocol.hpp"
#include "raisin_util/raisin_directories.hpp"

#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <dlfcn.h>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <stdlib.h>

using namespace std::chrono_literals;
using namespace raisin;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class Pty {
 public:
  Pty() {
    master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (master < 0 || ::grantpt(master) != 0 || ::unlockpt(master) != 0)
      throw std::runtime_error("cannot create tactile lifecycle PTY");
    std::array<char, 256> path{};
    if (::ptsname_r(master, path.data(), path.size()) != 0)
      throw std::runtime_error("cannot resolve tactile lifecycle PTY");
    slave_path = path.data();
  }
  ~Pty() { if (master >= 0) ::close(master); }
  int master{-1};
  std::string slave_path;
};

void expectCommand(int fd, const std::array<std::uint8_t, 7>& expected) {
  std::vector<std::uint8_t> received;
  for (int attempt = 0; attempt < 60 && received.size() < expected.size(); ++attempt) {
    pollfd descriptor{fd, POLLIN, 0};
    ::poll(&descriptor, 1, 20);
    std::array<std::uint8_t, 32> buffer{};
    for (;;) {
      const auto count = ::read(fd, buffer.data(), buffer.size());
      if (count > 0) received.insert(received.end(), buffer.begin(), buffer.begin() + count);
      else break;
    }
  }
  if (received.size() != expected.size() ||
      !std::equal(received.begin(), received.end(), expected.begin())) {
    std::ostringstream detail;
    detail << "Raisin plugin child startup command mismatch (received " << received.size()
           << " bytes):";
    for (const auto byte : received) detail << ' ' << static_cast<unsigned>(byte);
    throw std::runtime_error(detail.str());
  }
}

void expectStarted(int stream_fd) {
  std::this_thread::sleep_for(220ms);
  expectCommand(stream_fd, tactile::usb::buildCommand(0xF2, 0x01));
  std::this_thread::sleep_for(310ms);
  expectCommand(stream_fd, tactile::usb::buildCommand(0xF3, 0x02));
  std::this_thread::sleep_for(310ms);
  expectCommand(stream_fd, tactile::usb::buildCommand(0xF1, 0x01));
}

void configureUsb(parameter::ParameterContainer& config, const Pty& stream,
                  const std::filesystem::path& record_dir, int round) {
  config("enabled") = true;
  config("backend") = std::string("usb");
  config("rate") = 100;
  config["shadow"]("enabled") = false;
  auto& usb = config["usb"];
  usb("port") = stream.slave_path;
  usb("stream_sensor_count") = 1;
  usb("record_dir") = record_dir.string();
  usb("topic_prefix") = "tactile/v2/plugin_lifecycle/" + std::to_string(round);
  usb("rate_hz") = 20;
  usb("baud_rate") = 115200;
  usb("frame_timeout_s") = 0.5;
  usb("retry_s") = 0.05;
  usb("poll_rate_hz") = 100.0;
  auto& conversion = config["force_conversion"];
  conversion("code_to_force_n_per_code") = 0.001;
  conversion("calibration_id") = std::string("provisional_linear_0p001");
  conversion("left_polarity") = 1;
  conversion("right_polarity") = 1;
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
  if (argc != 5) return 2;  // URDF, staged plugin library, evidence directory, staged install
  ::setenv("RAISIN_TEST_INSTALL_PREFIX", argv[4], 1);
  raisinInit();
  raisim::World hub, sim;
  hub.addArticulatedSystem(argv[1])->setName("robot");
  sim.addArticulatedSystem(argv[1])->setName("robot");
  raisim::RaisimServer hub_server(&hub), sim_server(&sim);

  auto& resources = GlobalResource::getInstance();
  resources.network = std::make_shared<Network>(
      "tactile_usb_plugin_lifecycle_" + std::to_string(::getpid()), "test",
      std::vector<std::vector<std::string>>{{"main"}, {"plugin"}, {"core_sensor"}});
  const std::filesystem::path evidence_root(argv[3]);
  std::filesystem::create_directories(evidence_root);
  Pty stream;
  using Create = plugin::Plugin* (*)(raisim::World&, raisim::RaisimServer&,
      raisim::World&, raisim::RaisimServer&, GlobalResource&,
      const std::string&, const std::string&, bool);
  using Destroy = void (*)(plugin::Plugin*);

  try {
    for (int round = 0; round < 2; ++round) {
      const auto record_dir = evidence_root / ("round_" + std::to_string(round));
      void* library = ::dlopen(argv[2], RTLD_NOW | RTLD_LOCAL);
      require(library != nullptr, "cannot load staged tactile plugin library");
      auto create = reinterpret_cast<Create>(::dlsym(library, "create_with_owner_title_robot"));
      auto destroy = reinterpret_cast<Destroy>(::dlsym(library, "destroy"));
      require(create && destroy, "staged tactile plugin factory ABI is missing");
      auto* instance = create(hub, hub_server, sim, sim_server, resources,
                              "test", "tactile:usb", true);
      auto& config = resources.paramRoot["raisin_tactile_plugin"];
      configureUsb(config, stream, record_dir, round);
      require(instance->init(), "real-mode USB plugin initialization failed");

      std::atomic<bool> running{true};
      std::atomic<bool> healthy{true};
      std::thread supervisor([&] {
        while (running.load()) {
          if (!instance->advance()) {
            healthy.store(false);
            break;
          }
          std::this_thread::sleep_for(10ms);
        }
      });
      try {
        expectStarted(stream.master);
      } catch (...) {
        running.store(false);
        supervisor.join();
        destroy(instance);
        ::dlclose(library);
        throw;
      }
      running.store(false);
      supervisor.join();
      require(healthy.load(), "USB plugin advance failed while its child process was alive");

      destroy(instance);  // unload must terminate the child and release the PTY
      require(::dlclose(library) == 0, "cannot unload staged tactile plugin library");
      bool found_run = false;
      for (const auto& entry : std::filesystem::directory_iterator(record_dir)) {
        if (entry.is_directory() && entry.path().filename().string().starts_with("usb_") &&
            std::filesystem::exists(entry.path() / "run_manifest.json"))
          found_run = true;
      }
      require(found_run, "USB child did not create its run directory and manifest");
    }
    std::filesystem::remove_all(evidence_root);
    resources.network->shutdown();
    resources.network.reset();
    std::cout << "PASS: Raisin USB plugin factory init/advance/unload/reload starts one PTY twice\n";
  } catch (const std::exception& error) {
    std::filesystem::remove_all(evidence_root);
    if (resources.network) {
      resources.network->shutdown();
      resources.network.reset();
    }
    std::cerr << error.what() << '\n';
    return 1;
  }
}
