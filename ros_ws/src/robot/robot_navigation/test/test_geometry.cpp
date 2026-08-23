#include "robot_navigation/geometry.hpp"

#include <cmath>

#include "gtest/gtest.h"

TEST(Geometry, RayHitsRoomAndNearerObstacle)
{
  constexpr double kPi = 3.14159265358979323846;
  const robot_navigation::Room room{-3.0, 3.0, -2.0, 2.0};
  EXPECT_DOUBLE_EQ(robot_navigation::ray_distance(0.0, 0.0, 0.0, room), 3.0);
  EXPECT_NEAR(robot_navigation::ray_distance(0.0, 0.0, kPi / 2.0, room), 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(
    robot_navigation::ray_distance(
      0.0, 0.0, 0.0, room,
      robot_navigation::CircularObstacle{1.0, 0.0, 0.25}),
    0.75);
}
