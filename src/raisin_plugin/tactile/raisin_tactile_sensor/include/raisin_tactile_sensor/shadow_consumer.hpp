#pragma once

#include "frame_v2.hpp"

namespace raisin::tactile {

class ShadowPairConsumer {
 public:
  std::optional<BilateralFrameV2> tick(const std::optional<FrameV2>& left,
                                       const std::optional<FrameV2>& right,
                                       std::uint64_t now_ns,
                                       std::uint64_t stale_timeout_ns = 150000000ULL,
                                       std::uint64_t max_skew_ns = 50000000ULL) {
    if (!left || !right) return std::nullopt;
    return pairer_.admit(*left, *right, now_ns, stale_timeout_ns, max_skew_ns);
  }

  void reset() noexcept { pairer_.reset(); }

 private:
  PairAdmitter pairer_;
};

}  // namespace raisin::tactile
