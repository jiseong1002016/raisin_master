#include "raisin_tactile_sensor/image.hpp"
#include <iostream>

using namespace raisin::tactile;
void require(bool condition) { if (!condition) throw std::runtime_error("test failed"); }
template<class F> void rejects(F f) {
  bool rejected = false;
  try { f(); } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected);
}

int main() {
  for (const auto& [rows, cols] : {std::pair{3, 5}, {5, 7}, {10, 10}}) {
    Layout layout{"pad", rows, cols, .10, .06, false, false};
    Frame frame{1.25, std::vector<float>(layout.cells(), 0.f),
                std::vector<std::uint8_t>(layout.cells(), 1)};
    require(addNormalForce(frame, layout, -.05, -.03, 2.));
    require(addNormalForce(frame, layout, -.05, -.03, 3.));
    require(frame.normal_force_n[0] == 5.f);
    require(!addNormalForce(frame, layout, .05, 0., 1.));
    require(!addNormalForce(frame, layout, 0., .03, 1.));
    layout.flip_rows = layout.flip_cols = true;
    require(addNormalForce(frame, layout, -.05, -.03, 7.));
    require(frame.normal_force_n.back() == 7.f);
    frame.valid.back() = 0;
    auto wire = toImage(frame, layout);
    auto decoded = fromImage(wire, layout);
    require(decoded.stamp_s == 1.25 && decoded.normal_force_n == frame.normal_force_n &&
            decoded.valid == frame.valid);
    for (std::size_t i = 0; i < wire.data.size(); i += sizeof(float))
      std::reverse(wire.data.begin() + i, wire.data.begin() + i + sizeof(float));
    wire.is_bigendian = !wire.is_bigendian;
    require(fromImage(wire, layout).normal_force_n == frame.normal_force_n);
    wire.data.pop_back();
    rejects([&] { fromImage(wire, layout); });
    wire = toImage(frame, layout);
    wire.encoding = "32FC1";
    rejects([&] { fromImage(wire, layout); });
    Snapshot store(layout);
    require(store.set(frame));
    require(!store.set(frame));
    require(store.latest(1.26, .05).has_value());
    require(!store.latest(1.31, .05));
    require(!store.latest(1., .05));
    frame.stamp_s = 1.27;
    frame.normal_force_n[0] = std::numeric_limits<float>::quiet_NaN();
    rejects([&] { store.set(frame); });
    require(store.latest(1.28, .05)->stamp_s == 1.25);
    store.reset();
    require(!store.latest(1.28, .05));
  }
  std::cout << "PASS: grid geometry, mirrors, atomic validity, endian, malformed wire, freshness/reset\n";
}
