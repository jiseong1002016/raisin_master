#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace raisin::tactile {

// V1: one calibrated compressive normal force [N] per taxel. No solver labels,
// tangential force, object identity, or synthetic contact-point count.
struct Layout {
  std::string frame;
  int rows{}, cols{};
  double size_u_m{}, size_v_m{};
  bool flip_rows{}, flip_cols{};

  void validate() const {
    if (frame.empty() || rows <= 0 || cols <= 0 ||
        !std::isfinite(size_u_m) || !std::isfinite(size_v_m) ||
        size_u_m <= 0 || size_v_m <= 0 ||
        static_cast<std::uint64_t>(rows) * cols >
            std::numeric_limits<std::uint32_t>::max() / (2 * sizeof(float)))
      throw std::invalid_argument("invalid tactile layout");
  }
  std::size_t cells() const { return static_cast<std::size_t>(rows) * cols; }
};

struct Frame {
  double stamp_s{};  // acquisition time in the consumer's configured clock domain
  std::vector<float> normal_force_n;
  std::vector<std::uint8_t> valid;

  void validate(const Layout& layout) const {
    layout.validate();
    if (!std::isfinite(stamp_s) || stamp_s < 0 ||
        normal_force_n.size() != layout.cells() || valid.size() != layout.cells())
      throw std::invalid_argument("invalid tactile frame shape or time");
    for (std::size_t i = 0; i < valid.size(); ++i)
      if (valid[i] > 1 || !std::isfinite(normal_force_n[i]) || normal_force_n[i] < 0)
        throw std::invalid_argument("invalid tactile force or validity");
  }
};

// Consumers own this store. A plugin never lends an object/vtable to a controller.
// A reset is an explicit episode/clock-epoch transition, never inferred from an
// out-of-order packet. Invalid frames do not refresh the last good timestamp.
class Snapshot {
 public:
  explicit Snapshot(Layout layout) : layout_(std::move(layout)) { layout_.validate(); }
  bool set(Frame frame) {
    frame.validate(layout_);
    std::lock_guard guard(mutex_);
    if (latest_ && frame.stamp_s <= latest_->stamp_s) return false;
    latest_ = std::move(frame);
    return true;
  }
  std::optional<Frame> latest(double now_s, double stale_s) const {
    if (!std::isfinite(now_s) || !std::isfinite(stale_s) || stale_s <= 0)
      throw std::invalid_argument("invalid tactile freshness clock");
    std::lock_guard guard(mutex_);
    if (!latest_ || now_s < latest_->stamp_s || now_s - latest_->stamp_s > stale_s)
      return std::nullopt;
    return latest_;
  }
  void reset() { std::lock_guard guard(mutex_); latest_.reset(); }
 private:
  Layout layout_;
  mutable std::mutex mutex_;
  std::optional<Frame> latest_;
};

// Position binning is a simulated sensor model, not privileged actor input.
// Input coordinates are in the tactile frame: u=x, v=y, outward normal=z.
inline bool addNormalForce(Frame& frame, const Layout& layout,
                           double u_m, double v_m, double force_n) {
  if (!std::isfinite(u_m) || !std::isfinite(v_m) ||
      !std::isfinite(force_n) || force_n < 0)
    throw std::invalid_argument("nonfinite or negative simulated contact");
  if (u_m < -layout.size_u_m / 2 || u_m >= layout.size_u_m / 2 ||
      v_m < -layout.size_v_m / 2 || v_m >= layout.size_v_m / 2) return false;
  int col = static_cast<int>(std::floor((u_m / layout.size_u_m + 0.5) * layout.cols));
  int row = static_cast<int>(std::floor((v_m / layout.size_v_m + 0.5) * layout.rows));
  col = std::clamp(col, 0, layout.cols - 1);
  row = std::clamp(row, 0, layout.rows - 1);
  if (layout.flip_cols) col = layout.cols - 1 - col;
  if (layout.flip_rows) row = layout.rows - 1 - row;
  const auto index = static_cast<std::size_t>(row) * layout.cols + col;
  if (index >= frame.normal_force_n.size()) throw std::invalid_argument("unallocated grid");
  const double sum = frame.normal_force_n[index] + force_n;
  if (sum > std::numeric_limits<float>::max()) throw std::invalid_argument("force overflow");
  frame.normal_force_n[index] = static_cast<float>(sum);
  return true;
}

}  // namespace raisin::tactile
