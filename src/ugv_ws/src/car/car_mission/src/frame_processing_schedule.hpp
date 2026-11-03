#pragma once

#include <chrono>
#include <optional>

namespace car_mission {

// A new image may be processed immediately, but inference starts remain rate-limited.
class FrameProcessingSchedule
{
public:
  using Clock = std::chrono::steady_clock;

  explicit FrameProcessingSchedule(Clock::duration period) : period_(period) {}

  void image_received() { pending_ = true; }
  void discard_pending() { pending_ = false; }
  bool has_pending() const { return pending_; }

  bool try_start(Clock::time_point now)
  {
    if (!pending_ || (next_start_ && now < *next_start_)) return false;
    pending_ = false;
    next_start_ = now + period_;
    return true;
  }

private:
  Clock::duration period_;
  std::optional<Clock::time_point> next_start_;
  bool pending_{false};
};

}  // namespace car_mission
