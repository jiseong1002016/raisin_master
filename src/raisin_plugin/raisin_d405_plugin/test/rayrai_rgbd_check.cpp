// Hardware-free render check: known red box, independently predicted plane depth.
#include "raisim/World.hpp"
#include "../src/rayrai_rgbd.hpp"
#include <cstdlib>
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
  {
    std::lock_guard lock(world);
    renderer.render(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
  }
  renderer.read(depth, rgb);
  const size_t center = 24 * 64 + 32;
  const bool passed = depth.size() == 64 * 48 && rgb.size() == depth.size() * 3
      && std::abs(depth[center] - .2F) < .002F
      && rgb[3*center] > rgb[3*center + 1] && rgb[3*center] > rgb[3*center + 2];
  std::cout << (passed ? "PASS" : "FAIL") << ": rayrai paired RGBD red box center_depth_m="
            << depth[center] << " RGB=" << int(rgb[3*center]) << ','
            << int(rgb[3*center + 1]) << ',' << int(rgb[3*center + 2]) << '\n';
  return passed ? 0 : 1;
}
