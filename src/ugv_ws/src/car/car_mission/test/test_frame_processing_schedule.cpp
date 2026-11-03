#include <chrono>

#include "gtest/gtest.h"
#include "../src/frame_processing_schedule.hpp"

TEST(VisionScheduling, StartsOnArrivalAndProcessesOnlyTheLatestPendingFrame)
{
  using namespace std::chrono_literals;
  using Clock = car_mission::FrameProcessingSchedule::Clock;
  car_mission::FrameProcessingSchedule schedule(50ms);
  const Clock::time_point t{};

  EXPECT_FALSE(schedule.try_start(t));
  // An eligible image is not delayed until a periodic timer boundary.
  schedule.image_received();
  EXPECT_TRUE(schedule.try_start(t + 35ms));
  EXPECT_FALSE(schedule.try_start(t + 50ms));
  schedule.image_received();
  schedule.image_received();  // Both arrivals refer to the latest cached image.
  EXPECT_FALSE(schedule.try_start(t + 84ms));
  EXPECT_TRUE(schedule.has_pending());
  EXPECT_TRUE(schedule.try_start(t + 85ms));
  EXPECT_FALSE(schedule.has_pending());
  schedule.image_received();
  EXPECT_FALSE(schedule.try_start(t + 100ms));
  EXPECT_TRUE(schedule.try_start(t + 200ms));
  schedule.image_received();
  schedule.discard_pending();
  EXPECT_FALSE(schedule.has_pending());
  EXPECT_FALSE(schedule.try_start(t + 249ms));
  schedule.image_received();
  EXPECT_FALSE(schedule.try_start(t + 249ms));
  EXPECT_TRUE(schedule.try_start(t + 250ms));
}
