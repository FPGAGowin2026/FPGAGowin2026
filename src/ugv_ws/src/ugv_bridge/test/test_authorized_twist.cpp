#include "ugv_bridge/authorized_twist.hpp"

#include "gtest/gtest.h"

TEST(AuthorizedTwist, BindsBridgeEpochAndFreshPostAuthorizationStamp)
{
  ugv_bridge::msg::AuthorizedTwist command;
  command.bridge_id = "bridge-a";
  command.authorization_epoch = 42;
  command.header.frame_id = "base_link";
  command.header.stamp.sec = 10;
  command.header.stamp.nanosec = 100000000;
  constexpr std::int64_t authorized = 10000000000LL;
  constexpr std::int64_t now = 10150000000LL;
  EXPECT_TRUE(bx72::valid_mission_command(command, "bridge-a", 42, authorized, now, 0.2));
  EXPECT_FALSE(bx72::valid_mission_command(command, "bridge-b", 42, authorized, now, 0.2));
  EXPECT_FALSE(bx72::valid_mission_command(command, "bridge-a", 43, authorized, now, 0.2));
  EXPECT_FALSE(bx72::valid_mission_command(command, "bridge-a", 42, 10100000000LL, now, 0.2));
  EXPECT_FALSE(bx72::valid_mission_command(command, "bridge-a", 42, authorized, 10300000001LL, 0.2));
  EXPECT_FALSE(bx72::valid_mission_command(command, "bridge-a", 42, authorized, 10099999999LL, 0.2));
  command.header.frame_id = "camera";
  EXPECT_FALSE(bx72::valid_mission_command(command, "bridge-a", 42, authorized, now, 0.2));
}
