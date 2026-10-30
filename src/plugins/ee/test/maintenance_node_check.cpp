#include <raisin_network/node.hpp>
#include <raisin_network/raisin.hpp>
#include <raisin_thread_pool/raisin_thread_pool.hpp>
#include <raisin_interfaces/msg/joint_states.hpp>
#include <raisin_interfaces/srv/robot_description.hpp>
#include <raisin_interfaces/srv/string.hpp>
#include <raisin_raipal_ee/contract.hpp>
#include <raisin_raipal_ee/hardware_configuration.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Json = nlohmann::json;
using StringService = raisin::raisin_interfaces::srv::String;

template <class Service>
std::shared_ptr<typename Service::Response> request(
    const std::shared_ptr<raisin::Client<Service>>& client,
    const std::shared_ptr<typename Service::Request>& message,
    std::chrono::milliseconds timeout) {
  if (!client || !client->isServiceAvailable())
    throw std::runtime_error("required Raisin service is unavailable");
  auto future = client->asyncSendRequest(message);
  if (future.wait_for(timeout) != std::future_status::ready)
    throw std::runtime_error("Raisin service response timed out");
  auto response = future.get();
  if (!response) throw std::runtime_error("Raisin service returned no response");
  return response;
}

struct JointSample {
  std::array<double, 7> position{};
  Clock::time_point received{};
};

struct Status {
  std::string operation;
  std::string phase;
  std::string error;
  bool tested{};
  std::optional<std::array<double, 7>> arm_target;
};

Status parseStatus(const std::string& yaml) {
  const auto node = YAML::Load(yaml);
  if (!node.IsMap()) throw std::invalid_argument("maintenance_status must return YAML mapping");
  Status result;
  result.operation = node["operation"].as<std::string>();
  result.phase = node["phase"].as<std::string>();
  result.error = node["error"].as<std::string>();
  result.tested = node["tested"].as<bool>();
  if (node["arm_target_rad"] && node["arm_target_rad"].IsSequence()) {
    if (node["arm_target_rad"].size() != 7)
      throw std::invalid_argument("maintenance_status arm_target_rad must contain seven values");
    std::array<double, 7> target{};
    for (std::size_t i = 0; i < target.size(); ++i) {
      target[i] = node["arm_target_rad"][i].as<double>();
      if (!std::isfinite(target[i]))
        throw std::invalid_argument("maintenance_status arm_target_rad contains non-finite value");
    }
    result.arm_target = target;
  }
  return result;
}

class MaintenanceClient final : public raisin::Node {
 public:
  MaintenanceClient(const std::shared_ptr<raisin::ThreadPool>& pool, const std::string& nickname)
      : Node("raipal_maintenance_node_check", pool),
        description_(createClient<raisin::raisin_interfaces::srv::RobotDescription>("describe_robot", nullptr)),
        hardware_(createClient<StringService>(nickname + "/hardware_configuration", nullptr)),
        turn_on_(createClient<raisin::std_srvs::srv::Trigger>("turn_on", nullptr)),
        init_motor_(createClient<raisin::std_srvs::srv::Trigger>("init_motor", nullptr)),
        joint_test_(createClient<raisin::std_srvs::srv::Trigger>("joint_test", nullptr)),
        get_ready_(createClient<raisin::std_srvs::srv::Trigger>("get_ready", nullptr)),
        stop_motor_(createClient<raisin::std_srvs::srv::Trigger>("stop_motor", nullptr)),
        ee_command_(createClient<StringService>(nickname + "/ee/drive_command", nullptr)),
        status_(createClient<StringService>("maintenance_status", nullptr)) {
    joints_ = createSubscriber<raisin::raisin_interfaces::msg::JointStates>("joint_states", nullptr,
        [this](const auto& message) {
          if (!message || message->num_joints != 7 || message->joint_states.size() != 7) return;
          JointSample sample;
          sample.received = Clock::now();
          for (std::size_t i = 0; i < sample.position.size(); ++i) {
            sample.position[i] = message->joint_states[i].joint_position;
            if (!std::isfinite(sample.position[i])) return;
          }
          std::lock_guard lock(mutex_);
          joint_sample_ = sample;
        });
    ee_state_ = createSubscriber<raisin::std_msgs::msg::String>(nickname + "/ee/state", nullptr,
        [this](const auto& message) {
          if (!message) return;
          try {
            auto sample = raisin::raipal_ee::parseState(message->data);
            std::lock_guard lock(mutex_);
            ee_sample_ = std::move(sample);
            ee_received_ = Clock::now();
            ee_error_.clear();
          } catch (const std::exception& error) {
            std::lock_guard lock(mutex_);
            ee_error_ = error.what();
          }
        });
    if (!description_ || !hardware_ || !turn_on_ || !init_motor_ || !joint_test_ || !get_ready_ ||
        !stop_motor_ || !ee_command_ || !status_ ||
        !joints_ || !ee_state_)
      throw std::runtime_error("failed to register maintenance clients/subscribers");
  }

  ~MaintenanceClient() override { cleanupResources(); }

  auto description() const { return description_; }
  auto hardware() const { return hardware_; }
  auto turnOn() const { return turn_on_; }
  auto initMotor() const { return init_motor_; }
  auto jointTest() const { return joint_test_; }
  auto getReady() const { return get_ready_; }
  auto stopMotor() const { return stop_motor_; }
  auto eeCommand() const { return ee_command_; }
  auto statusClient() const { return status_; }

  Status status() const {
    auto request_message = std::make_shared<StringService::Request>();
    request_message->data = "maintenance_status";
    const auto response = request<StringService>(status_, request_message, 2s);
    if (!response->success) throw std::runtime_error("maintenance_status failed: " + response->message);
    return parseStatus(response->message);
  }

  std::optional<JointSample> jointSample() const {
    std::lock_guard lock(mutex_);
    if (!joint_sample_ || Clock::now() - joint_sample_->received > 200ms) return std::nullopt;
    return joint_sample_;
  }

  std::optional<raisin::raipal_ee::State> eeSample() const {
    std::lock_guard lock(mutex_);
    if (!ee_error_.empty()) throw std::runtime_error("invalid EE state: " + ee_error_);
    if (!ee_sample_ || Clock::now() - ee_received_ > 200ms) return std::nullopt;
    return ee_sample_;
  }

 private:
  std::shared_ptr<raisin::Client<raisin::raisin_interfaces::srv::RobotDescription>> description_;
  std::shared_ptr<raisin::Client<StringService>> hardware_, status_, ee_command_;
  std::shared_ptr<raisin::Client<raisin::std_srvs::srv::Trigger>> turn_on_, init_motor_, joint_test_, get_ready_, stop_motor_;
  std::shared_ptr<raisin::Subscriber<raisin::raisin_interfaces::msg::JointStates>> joints_;
  std::shared_ptr<raisin::Subscriber<raisin::std_msgs::msg::String>> ee_state_;
  mutable std::mutex mutex_;
  std::optional<JointSample> joint_sample_;
  std::optional<raisin::raipal_ee::State> ee_sample_;
  Clock::time_point ee_received_{};
  std::string ee_error_;
};

template <class Service>
void trigger(const std::shared_ptr<raisin::Client<Service>>& client, const std::string& name) {
  const auto response = request<Service>(client, std::make_shared<typename Service::Request>(), 5s);
  if (!response->success) throw std::runtime_error(name + " failed: " + response->message);
}

Json run(const std::string& nickname, const std::filesystem::path& output_path) {
  auto pool = std::make_shared<raisin::ThreadPool>(std::vector<std::vector<std::string>>{{"main"}});
  MaintenanceClient client(pool, nickname);
  if (!client.description()->waitForService(5s))
    throw std::runtime_error("describe_robot service unavailable");
  const auto description = request<raisin::raisin_interfaces::srv::RobotDescription>(
      client.description(), std::make_shared<raisin::raisin_interfaces::srv::RobotDescription::Request>(), 2s);
  if (!description->success) throw std::runtime_error("describe_robot failed");
  if (description->is_real_robot)
    throw std::runtime_error("real robot detected; no maintenance action was sent");
  if (description->robot_nickname != nickname)
    throw std::runtime_error("connected robot nickname does not match --nickname");
  if (!client.initMotor()->isServiceAvailable()) {
    if (!client.turnOn()->waitForService(5s))
      throw std::runtime_error("nonreal turn_on service unavailable");
    trigger(client.turnOn(), "turn_on");
  }
  if (!client.hardware()->waitForService(30s))
    throw std::runtime_error("hardware_configuration service unavailable");
  const auto hardware_response = request<StringService>(client.hardware(),
      std::make_shared<StringService::Request>(), 2s);
  if (!hardware_response->success) throw std::runtime_error("hardware_configuration failed");
  const auto hardware = raisin::raipal_ee::parseHardwareConfiguration(hardware_response->message);
  if (hardware.hardware_profile != "single_L_ee_nonreal")
    throw std::runtime_error("hardware profile must be single_L_ee_nonreal");

  if (!client.statusClient()->waitForService(5s))
    throw std::runtime_error("maintenance_status service unavailable");
  for (const auto& entry : std::array<std::pair<std::string, decltype(client.initMotor())>, 3>{{
           {"init_motor", client.initMotor()}, {"joint_test", client.jointTest()},
           {"get_ready", client.getReady()}}}) {
    if (!entry.second->waitForService(5s)) throw std::runtime_error(entry.first + " service unavailable");
  }
  trigger(client.initMotor(), "init_motor");
  const auto premature_ready = request<raisin::std_srvs::srv::Trigger>(client.getReady(),
      std::make_shared<raisin::std_srvs::srv::Trigger::Request>(), 2s);
  if (premature_ready->success) throw std::runtime_error("get_ready bypassed the fresh joint test");
  const auto deadline = Clock::now() + 180s;
  const auto eeInitialDeadline = Clock::now() + 5s;
  while ((!client.eeSample() || !client.jointSample()) && Clock::now() < eeInitialDeadline)
    std::this_thread::sleep_for(10ms);
  auto ee = client.eeSample();
  auto joints = client.jointSample();
  if (!ee || !joints) throw std::runtime_error("fresh initial arm/EE telemetry unavailable");

  std::array<double, 2> ee_min{}, ee_max{};
  std::array<double, 2> lower{}, upper{};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    ee_min[axis] = ee_max[axis] = ee->axes[axis].position_rad;
    lower[axis] = ee->axes[axis].lower_limit_rad;
    upper[axis] = ee->axes[axis].upper_limit_rad;
    if (!std::isfinite(lower[axis]) || !std::isfinite(upper[axis]) || upper[axis] <= lower[axis])
      throw std::runtime_error("EE state has invalid axis limits");
  }
  trigger(client.jointTest(), "joint_test");
  const auto concurrent_test = request<raisin::std_srvs::srv::Trigger>(client.jointTest(),
      std::make_shared<raisin::std_srvs::srv::Trigger::Request>(), 2s);
  if (concurrent_test->success) throw std::runtime_error("concurrent joint test was accepted");
  if (!client.eeCommand()->waitForService(2s))
    throw std::runtime_error("EE command service unavailable");
  raisin::raipal_ee::Command competing_command;
  competing_command.action = raisin::raipal_ee::Action::TARGET;
  competing_command.target_rad = std::array<double, 2>{0.0, 0.0};
  auto competing_request = std::make_shared<StringService::Request>();
  competing_request->data = raisin::raipal_ee::encodeInternalCommand(competing_command, 900000);
  const auto competing_response = request<StringService>(client.eeCommand(), competing_request, 2s);
  if (competing_response->success ||
      competing_response->message != "EE commands are owned by joint_test/get_ready")
    throw std::runtime_error("maintenance did not reject the competing EE command");
  bool saw_joint_complete = false;
  bool saw_closing = false;
  bool saw_ee_test = false;
  bool saw_joint_settling = false;
  bool saw_arm_phase = false;
  bool saw_ready_holding = false;
  bool ready_sent = false;
  double arm_ee_deviation = 0.0;
  Status current;
  std::optional<Status> completed_ready_status;
  while (Clock::now() < deadline) {
    current = client.status();
    if (!current.error.empty() || current.phase == "failed")
      throw std::runtime_error("maintenance operation failed: " + current.error);
    if (current.operation == "joint_test" && current.phase == "complete" && current.tested)
      saw_joint_complete = true;
    if (current.operation == "joint_test" && current.phase == "closing") saw_closing = true;
    if (current.operation == "joint_test" && current.phase == "settling") saw_joint_settling = true;
    if (current.operation == "joint_test" && current.phase == "ee_test") saw_ee_test = true;
    if (current.operation == "get_ready" && current.phase == "holding") {
      saw_ready_holding = true;
      completed_ready_status = current;
    }
    if (current.phase == "arm" || current.phase == "settling") saw_arm_phase = true;
    if (auto sample = client.eeSample()) {
      ee = sample;
      for (std::size_t axis = 0; axis < 2; ++axis) {
        const double position = ee->axes[axis].position_rad;
        ee_min[axis] = std::min(ee_min[axis], position);
        ee_max[axis] = std::max(ee_max[axis], position);
        if (current.phase == "arm" || current.phase == "settling")
          arm_ee_deviation = std::max(arm_ee_deviation, std::abs(position));
      }
    }
    if (saw_joint_complete && !ready_sent) {
      trigger(client.getReady(), "get_ready");
      ready_sent = true;
    }
    if (saw_ready_holding) break;
    std::this_thread::sleep_for(50ms);
  }
  if (Clock::now() >= deadline) throw std::runtime_error("maintenance test timed out");
  if (!saw_joint_complete || !saw_closing || !saw_joint_settling || !saw_ee_test ||
      !saw_arm_phase || !saw_ready_holding)
    throw std::runtime_error("maintenance status did not report all expected phases");
  if (!completed_ready_status || !completed_ready_status->arm_target)
    throw std::runtime_error("completed get_ready status omitted arm_target_rad");
  const auto hold_deadline = Clock::now() + 1s;
  while (Clock::now() < hold_deadline) {
    const auto held = client.status();
    const auto held_ee = client.eeSample();
    if (held.phase != "holding" || !held.error.empty() || !held_ee ||
        std::abs(held_ee->axes[0].position_rad) > 0.02 ||
        std::abs(held_ee->axes[1].position_rad) > 0.02)
      throw std::runtime_error("get_ready did not retain its closed EE hold");
    std::this_thread::sleep_for(20ms);
  }
  joints = client.jointSample();
  ee = client.eeSample();
  if (!joints || !ee) throw std::runtime_error("final arm/EE telemetry is stale");
  Json range = Json::array();
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const double lower_test = axis == 0 ? std::max(lower[axis], -0.2) : 0.;
    const double upper_test = axis == 1 ? std::min(upper[axis], 0.2) : 0.;
    const double tolerance = 0.01;
    if (ee_min[axis] > lower_test + tolerance || ee_max[axis] < upper_test - tolerance ||
        ee_min[axis] < lower_test - tolerance || ee_max[axis] > upper_test + tolerance)
      throw std::runtime_error("EE axis did not follow its bounded partial-angle test");
    const double final_position = ee->axes[axis].position_rad;
    if (std::abs(final_position - ee->axes[axis].target_rad) > tolerance ||
        std::abs(ee->axes[axis].target_rad) > 0.02)
      throw std::runtime_error("final EE position did not settle at its reported target");
    range.push_back({{"axis", axis}, {"lower_limit_rad", lower[axis]}, {"upper_limit_rad", upper[axis]},
                     {"observed_min_rad", ee_min[axis]}, {"observed_max_rad", ee_max[axis]},
                     {"final_position_rad", final_position}, {"final_target_rad", ee->axes[axis].target_rad}});
  }
  if (arm_ee_deviation > 0.02)
    throw std::runtime_error("EE moved during arm phase; max absolute position was " +
                             std::to_string(arm_ee_deviation));
  double arm_error = 0.0;
  for (std::size_t joint = 0; joint < 7; ++joint)
    arm_error = std::max(arm_error,
        std::abs(joints->position[joint] - (*completed_ready_status->arm_target)[joint]));
  if (arm_error > 0.05) throw std::runtime_error("final arm readback differs from reported target");
  if (!client.stopMotor()->waitForService(2s))
    throw std::runtime_error("stop_motor service unavailable");
  trigger(client.stopMotor(), "stop_motor");
  if (client.status().phase != "idle")
    throw std::runtime_error("stop_motor did not cancel maintenance ownership");

  return {{"robot_nickname", nickname}, {"robot_model", description->robot_model},
          {"hardware_profile", hardware.hardware_profile}, {"joint_test_complete", saw_joint_complete},
          {"ee_test_phase_observed", saw_ee_test}, {"arm_phase_observed", saw_arm_phase},
          {"premature_ready_rejected", true}, {"concurrent_joint_test_rejected", true},
          {"competing_ee_command_rejected", true}, {"closed_hold_observed_seconds", 1},
          {"stop_motor_cancelled_ownership", true},
          {"ee_max_abs_position_during_arm_rad", arm_ee_deviation}, {"ee_axes", range},
          {"arm_target_rad", *completed_ready_status->arm_target},
          {"arm_final_position_rad", joints->position},
          {"arm_max_target_error_rad", arm_error}, {"final_status", completed_ready_status->phase}};
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string nickname;
    std::filesystem::path output_path;
    for (int index = 1; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--nickname" && index + 1 < argc) nickname = argv[++index];
      else if (argument == "--output" && index + 1 < argc) output_path = argv[++index];
      else throw std::invalid_argument("usage: maintenance_node_check --nickname NAME --output PATH");
    }
    if (nickname.empty() || output_path.empty())
      throw std::invalid_argument("usage: maintenance_node_check --nickname NAME --output PATH");
    raisin::raisinInit();
    const Json evidence = run(nickname, output_path);
    if (output_path.has_parent_path()) std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("cannot open evidence output: " + output_path.string());
    output << evidence.dump(2) << '\n';
    if (!output) throw std::runtime_error("failed writing evidence output");
    std::cout << evidence.dump(2) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "maintenance_node_check: " << error.what() << '\n';
    return 1;
  }
}
