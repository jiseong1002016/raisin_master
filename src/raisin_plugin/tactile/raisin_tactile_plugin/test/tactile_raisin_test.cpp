#include "raisin_tactile_plugin/backend.hpp"
#include "raisin_tactile_sensor/frame_v2.hpp"
#include "raisin_tactile_sensor/registration.hpp"
#include "raisin_network/raisin.hpp"
#include "raisim/RaisimServer.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <time.h>
#include <thread>
#include <unistd.h>

using namespace raisin;
using namespace raisin::tactile;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  raisinInit();
  // Replay data has independent expected values and preserves invalid cells.
  const std::string replayPath = "/tmp/tactile_replay_" + std::to_string(getpid()) + ".csv";
  {
    std::ofstream replay(replayPath);
    replay << "1,3.5,1\n2,7,0\n2,9,1\n";
  }
  SensorConfig replayConfig;
  replayConfig.layout = {"replay", 1, 1, .01, .01, false, false};
  replayConfig.replay_csv = replayPath;
  ReplayBackend replay({replayConfig});
  std::remove(replayPath.c_str());
  const auto first = replay.sample().front();
  const auto second = replay.sample().front();
  require(first.stamp_s == 1 && first.normal_force_n[0] == 3.5f && first.valid[0] == 1,
          "replay changed measured data");
  require(second.stamp_s == 2 && second.normal_force_n[0] == 7.f && second.valid[0] == 0,
          "replay lost invalid measurement");
  bool rejected = false;
  try { replay.sample(); } catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "replay accepted duplicate timestamp");
  raisim::World world;
  world.setTimeStep(.001);
  world.addGround();
  auto* robot = world.addArticulatedSystem(argv[1]);
  robot->setName("robot");
  robot->setGeneralizedCoordinate(Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim()));
  robot->setGeneralizedVelocity(Eigen::VectorXd::Zero(robot->getDOF()));
  robot->setGeneralizedForce(Eigen::VectorXd::Zero(robot->getDOF()));
  require(robot->getDOF() == 1, "fixture must have a fixed base and one slider");
  raisim::RaisimServer server(&world);  // no GUI socket, no real robot or drivers
  SensorConfig config;
  config.name = "test_pad";
  config.layout = {"test_tactile", 3, 5, .10, .06, false, false};
  config.parent_link = "pad";
  config.offset_m = {0., 0., -.01};
  config.rotation = Eigen::Vector3d(1., -1., -1.).asDiagonal();
  config.surface_tolerance_m = .003;
  SimBackend backend(world, *robot, {config});
  auto before = backend.sample().front();
  require(std::accumulate(before.normal_force_n.begin(), before.normal_force_n.end(), 0.f) == 0.f,
          "unloaded sensor must be valid zero");
  for (int step = 0; step < 500; ++step) world.integrate();
  auto pressed = backend.sample().front();
  const float total = std::accumulate(pressed.normal_force_n.begin(), pressed.normal_force_n.end(), 0.f);
  require(total > 8.f && total < 12.f, "1 kg static load should be near independently specified mg");

  const std::string id = "tactile_test_" + std::to_string(getpid());
  auto network = std::make_shared<Network>(id, "test",
      std::vector<std::vector<std::string>>{{"main"}, {"core_sensor"}});
  auto& parameters = parameter::ParameterContainer::getRoot()[id];
  parameters("enabled", true);
  parameters("backend", std::string("sim"));
  parameters("topic_prefix", id);
  auto& layoutParams = parameters["sensors"]["pad"];
  layoutParams("frame", config.layout.frame);
  layoutParams("rows", config.layout.rows);
  layoutParams("cols", config.layout.cols);
  layoutParams("size_u_m", config.layout.size_u_m);
  layoutParams("size_v_m", config.layout.size_v_m);
  layoutParams("flip_rows", false);
  layoutParams("flip_cols", false);
  const auto topic = id + "/pad";
  Node publisherNode(network);
  auto publisher = publisherNode.createPublisher<sensor_msgs::msg::Image>(topic, 1);
  std::map<std::string, std::shared_ptr<robot::sensor::Sensor>> registry;
  registerReceivers(registry, world, server, parameters, network);
  auto receiverPtr = std::dynamic_pointer_cast<Receiver>(registry.at("tactile/pad"));
  require(receiverPtr != nullptr, "core registry entry has wrong type");
  auto& receiver = *receiverPtr;
  receiver.createCommunication(network);
  std::optional<Frame> received;
  // Bounded wall time accommodates discovery of the shared-memory publisher.
  for (int attempt = 0; attempt < 200 && !received; ++attempt) {
    publisher->publish(toImage(pressed, config.layout));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    received = receiver.latest(pressed.stamp_s, .05);
  }
  require(received.has_value(), "Raisin shared-memory receiver did not receive frame");
  require(received->normal_force_n == pressed.normal_force_n && received->valid == pressed.valid,
          "core-visible frame differs from published frame");
  require(!receiver.latest(pressed.stamp_s + 1., .05), "stale frame remained fresh");

  auto& v2Parameters = parameter::ParameterContainer::getRoot()[id + "_v2"];
  v2Parameters("enabled", true);
  v2Parameters("backend", std::string("usb"));
  const auto v2Prefix = id + "/v2";
  v2Parameters["usb"]("topic_prefix", v2Prefix);
  registerReceivers(registry, world, server, v2Parameters, network);
  auto leftV2 = std::dynamic_pointer_cast<ReceiverV2>(registry.at("tactile/left"));
  auto rightV2 = std::dynamic_pointer_cast<ReceiverV2>(registry.at("tactile/right"));
  require(leftV2 && rightV2, "USB core registry did not create both ReceiverV2 instances");
  leftV2->createCommunication(network);
  rightV2->createCommunication(network);
  auto leftV2Publisher = publisherNode.createPublisher<raisin_tactile_sensor::msg::TactileFrame>(
      v2Prefix + "/left", 1);
  auto rightV2Publisher = publisherNode.createPublisher<raisin_tactile_sensor::msg::TactileFrame>(
      v2Prefix + "/right", 1);
  const auto makeV2Frame = [](const std::string& side, std::uint64_t session,
                              std::int16_t raw, float force) {
    FrameV2 frame;
    frame.sensor_name = side;
    frame.frame_id = "tactile_" + side + "_frame";
    frame.clock_domain = "CLOCK_MONOTONIC";
    frame.calibration_id = "provisional_linear_0p001";
    frame.calibration_status = "provisional";
    frame.baseline_status = "operator_confirmed_unloaded";
    frame.force_unit = "N_estimate";
    frame.session_id = session;
    frame.rows = kUsbRows;
    frame.cols = kUsbCols;
    frame.code_to_force_n_per_code = 0.001F;
    frame.polarity = 1;
    frame.baseline_valid = true;
    frame.raw_code.assign(kUsbCells, raw);
    frame.baseline_code.assign(kUsbCells, 0);
    frame.normal_force_n.assign(kUsbCells, force);
    frame.valid.assign(kUsbCells, 1);
    for (std::size_t row = 0; row < kUsbRows; ++row)
      frame.valid[row * kUsbCols + kUsbCols - 1] = 0;
    return frame;
  };
  auto leftFrame = makeV2Frame("left", 101, 12, 0.25F);
  auto rightFrame = makeV2Frame("right", 202, 34, 0.5F);
  for (int attempt = 0; attempt < 200 && (!leftV2->latest() || !rightV2->latest()); ++attempt) {
    timespec now{};
    require(::clock_gettime(CLOCK_MONOTONIC, &now) == 0,
            "cannot read monotonic clock for v2 receiver test");
    const auto stamp = static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
    leftFrame.sequence = rightFrame.sequence = static_cast<std::uint64_t>(attempt);
    leftFrame.stamp_monotonic_ns = stamp;
    rightFrame.stamp_monotonic_ns = stamp + 1;
    leftV2Publisher->publish(toMessage(leftFrame));
    rightV2Publisher->publish(toMessage(rightFrame));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto leftReceived = leftV2->latest();
  const auto rightReceived = rightV2->latest();
  require(leftReceived && rightReceived && leftReceived->sensor_name == "left" &&
              rightReceived->sensor_name == "right" && leftReceived->raw_code[0] == 12 &&
              rightReceived->raw_code[0] == 34 && leftReceived->normal_force_n[0] == 0.25F &&
              rightReceived->normal_force_n[0] == 0.5F && leftV2->rejected() == 0 &&
              rightV2->rejected() == 0,
          "registered core ReceiverV2 did not accept its matching side topic and identity");
  leftV2->shutDown();
  rightV2->shutDown();
  receiver.shutDown();
  publisherNode.cleanupResources();
  network->shutdown();
  std::cout << "PASS: sim no-contact/load " << total
            << " N -> Raisin shared memory -> core-owned Receiver -> stale/shutdown\n";
}
