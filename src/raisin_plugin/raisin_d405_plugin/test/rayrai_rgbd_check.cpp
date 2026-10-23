// Hardware-free render check: known red box, independently predicted plane depth.
#include "raisim/World.hpp"
#include "../src/rayrai_rgbd.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

int main() {
  if (const char* home = std::getenv("HOME"))
    raisim::World::setActivationKey(std::string(home) + "/.raisim/activation.raisim");
  raisim::World world;
  auto* box = world.addBox(.1, .1, .1, 1.);
  box->setPosition(0., 0., .25);
  box->setAppearance("1,0,0,1");
  raisin::plugin::RayraiRgbd renderer(world, 64, 48, 40., .07, .5);
  std::vector<float> depth;
  std::vector<std::uint8_t> rgb;
  const auto capture = [&] {
    std::lock_guard lock(world);
    renderer.render(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
    renderer.read(depth, rgb);
  };
  capture();
  const size_t center = 24 * 64 + 32;
  const bool passed = depth.size() == 64 * 48 && rgb.size() == depth.size() * 3
      && std::abs(depth[center] - .2F) < .002F
      && rgb[3*center] > rgb[3*center + 1] && rgb[3*center] > rgb[3*center + 2];
  std::cout << (passed ? "PASS" : "FAIL") << ": rayrai paired RGBD red box center_depth_m="
            << depth[center] << " RGB=" << int(rgb[3*center]) << ','
            << int(rgb[3*center + 1]) << ',' << int(rgb[3*center + 2]) << '\n';
  if (!passed) return 1;

  const auto path = std::filesystem::temp_directory_path() / ("d405_visuals_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".urdf");
  std::ofstream(path) << R"(<robot name="preview"><link name="world"/><link name="boxes">
    <inertial><mass value="1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial>
    <visual><origin xyz="-.1 0 .3"/><geometry><box size=".08 .08 .08"/></geometry>
      <material name="green"><color rgba="0 1 0 1"/></material></visual>
    <visual><origin xyz=".1 0 .3"/><geometry><box size=".08 .08 .08"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material></visual></link>
    <joint name="pose" type="revolute"><parent link="world"/><child link="boxes"/>
      <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>)";
  auto* preview = world.addArticulatedSystem(path.string());
  std::filesystem::remove(path);
  world.removeObject(box);
  const auto visible = [&](size_t x, size_t channel) {
    const auto pixel = 24 * 64 + x;
    return std::abs(depth[pixel] - .26F) < .002F &&
        rgb[3*pixel+channel] > rgb[3*pixel+(channel+1)%3] &&
        rgb[3*pixel+channel] > rgb[3*pixel+(channel+2)%3];
  };
  capture();
  const bool initial_pair = visible(16, 1) && visible(47, 2);
  auto original = preview->getVisOb();
  std::erase_if(preview->getVisOb(), [](const auto& visual) { return visual.offset[0] < 0.; });
  auto* episode = world.addBox(.01, .01, .01, 1.);
  episode->setPosition(5., 5., 5.);
  capture();
  const bool after_add = preview->getVisOb().size() == 1 && !visible(16, 1) && visible(47, 2);
  preview->getVisOb().swap(original);
  world.removeObject(episode);
  capture();
  const bool after_remove = visible(16, 1) && visible(47, 2);
  const bool refreshed = initial_pair && after_add && after_remove;
  std::cout << (refreshed ? "PASS" : "FAIL") << ": visual-count/topology refresh initial_pair="
            << initial_pair << " survivor_after_add=" << after_add
            << " restored_after_remove=" << after_remove << '\n';
  return refreshed ? 0 : 1;
}
