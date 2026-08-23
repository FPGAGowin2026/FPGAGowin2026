#include "mission_control/control.hpp"

#include <limits>

#include "gtest/gtest.h"

TEST(Control, AlignsApproachesAndStops)
{
  auto command = mission_control::approach_command(
    0.4, 0.01, 0.06, 0.1, 0.12, 1.2, 0.6);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_NEAR(command.angular, -0.48, 1e-12);
  EXPECT_FALSE(command.complete);

  command = mission_control::approach_command(
    0.05, 0.02, 0.06, 0.1, 0.12, 1.2, 0.6);
  EXPECT_DOUBLE_EQ(command.linear, 0.12);
  EXPECT_NEAR(command.angular, -0.06, 1e-12);
  EXPECT_FALSE(command.complete);

  command = mission_control::approach_command(
    0.0, 0.06, 0.06, 0.1, 0.12, 1.2, 0.6);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
  EXPECT_TRUE(command.complete);

  command = mission_control::approach_command(
    std::numeric_limits<double>::quiet_NaN(), 0.01, 0.06, 0.1, 0.12, 1.2, 0.6);
  EXPECT_DOUBLE_EQ(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(command.angular, 0.0);
  EXPECT_FALSE(command.complete);
}
