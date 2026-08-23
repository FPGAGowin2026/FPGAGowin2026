#include "vision_bridge/detector.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "gtest/gtest.h"

TEST(Detector, DetectsCenteredRedTargetAndRejectsEmptyImage)
{
  constexpr double kPi = 3.14159265358979323846;
  constexpr std::size_t kWidth = 40;
  constexpr std::size_t kHeight = 30;
  const auto image = vision_bridge::render_target(
    kWidth, kHeight, 2.0, 0.0, kPi / 2.0, 10.0);
  const auto observation = vision_bridge::detect_red_target(
    image, kWidth, kHeight, kWidth * 3U, "rgb8", 120, 1.5, 0.002, 1);
  ASSERT_TRUE(observation.has_value());
  EXPECT_LT(std::abs(observation->horizontal_error), 0.05);
  EXPECT_GT(observation->area_fraction, 0.01);

  const std::vector<std::uint8_t> empty(kWidth * kHeight * 3U, 0U);
  EXPECT_FALSE(vision_bridge::detect_red_target(
    empty, kWidth, kHeight, kWidth * 3U, "rgb8").has_value());
}
