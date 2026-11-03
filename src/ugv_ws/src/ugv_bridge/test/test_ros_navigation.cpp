#include "ugv_bridge/ros_navigation.hpp"

#include <gtest/gtest.h>

TEST(FusionCovariance, GridAndPolarEstimatesUseSquaredSiAndUnknownVelocity) {
  const auto values=ugv_bridge::ros_navigation::estimate_covariance({30.0,40.0,50.0});
  ASSERT_TRUE(values.has_value());
  EXPECT_NEAR((*values)[0],0.0009,1e-12);
  EXPECT_NEAR((*values)[1],0.0016,1e-12);
  EXPECT_NEAR((*values)[2],0.0025,1e-12);
  for (std::size_t i=3;i<values->size();++i) EXPECT_EQ((*values)[i],1e6);
  EXPECT_FALSE(ugv_bridge::ros_navigation::estimate_covariance({NAN,0.0,0.0}).has_value());
}

#include <cmath>
#include <cstdint>
#include <limits>

namespace nav = ugv_bridge::ros_navigation;

TEST(NavigationSync, FailedJetsonCancelOnlyInvalidatesAWaitingJetsonStart) {
  EXPECT_TRUE(nav::failed_jetson_cancel_blocks_start(true, true));
  EXPECT_FALSE(nav::failed_jetson_cancel_blocks_start(false, true));
  EXPECT_FALSE(nav::failed_jetson_cancel_blocks_start(true, false));
  EXPECT_FALSE(nav::failed_jetson_cancel_blocks_start(false, false));
}

TEST(FpgaRosTimeMapper, ExtendsNavigationAndLegacyTimesAcross32BitWrap) {
  nav::FpgaRosTimeMapper mapper;
  constexpr std::uint32_t anchor_us = 0xfffffff0U;
  constexpr std::int64_t receive_ns = 1'700'000'000'000'000'000LL;
  ASSERT_TRUE(mapper.observe_navigation(anchor_us, receive_ns));

  const auto mapped = mapper.map_legacy_us(0xffffffe0U);
  ASSERT_TRUE(mapped);
  EXPECT_EQ(*mapped, receive_ns - 16'000);
  EXPECT_EQ(mapper.anchor_fpga_us(), anchor_us);
  EXPECT_TRUE(mapper.map_legacy_us(0U).has_value());

  ASSERT_TRUE(mapper.observe_navigation(0x10U, receive_ns + 32'000));
  EXPECT_EQ(mapper.anchor_fpga_us(), 0x100000010ULL);
  const auto after_wrap = mapper.map_legacy_us(0x10U);
  ASSERT_TRUE(after_wrap);
  EXPECT_EQ(*after_wrap, receive_ns + 32'000);
  EXPECT_FALSE(mapper.observe_navigation(0xfffffff0U, receive_ns + 16'000));
}

TEST(FpgaRosTimeMapper, RejectsOutOfRangeAndResetInvalidatesOldMapping) {
  nav::FpgaRosTimeMapper mapper;
  EXPECT_FALSE(mapper.observe_navigation(10, -1));
  EXPECT_FALSE(mapper.map_fpga_us(10));
  ASSERT_TRUE(mapper.observe_navigation(100, 1'000'000));
  EXPECT_TRUE(mapper.map_fpga_us(101));
  mapper.reset();
  EXPECT_FALSE(mapper.valid());
  EXPECT_FALSE(mapper.map_legacy_us(101));
}

TEST(FpgaRosTimeMapper, RepeatedNavigationTimestampDoesNotResetClockOffset) {
  nav::FpgaRosTimeMapper mapper;
  ASSERT_TRUE(mapper.observe_navigation(1000, 2'000'000));
  const auto initial_offset = mapper.offset_ns();
  const auto initial_anchor = mapper.anchor_fpga_us();
  ASSERT_TRUE(mapper.observe_navigation(1000, 2'010'000));
  EXPECT_EQ(mapper.offset_ns(), initial_offset);
  EXPECT_EQ(mapper.anchor_fpga_us(), initial_anchor);
  ASSERT_TRUE(mapper.observe_navigation(1001, 2'011'000));
  EXPECT_EQ(mapper.offset_ns(), initial_offset + 625);
}

TEST(N10ScanAssembler, PublishesOnlyAfterACompleteOrderedRevolution) {
  nav::N10ScanAssembler assembler;
  std::optional<sensor_msgs::msg::LaserScan> completed;
  std::uint32_t sequence = 0;
  for (std::uint32_t first = 0; first < 736; first += 16, ++sequence) {
    nav::N10Group group;
    group.sequence = sequence;
    group.timestamp_us = 1'000'000 + sequence * 10'000;
    for (std::uint32_t point = 0; point < group.points.size(); ++point) {
      const auto sample = first + point;
      auto& value = group.points[point];
      value.angle_cdeg = static_cast<std::uint16_t>((sample * 100U) % 36000U);
      value.distance_mm = 1'250;
      if (value.angle_cdeg == 9000) value.distance_mm = 2'500;
      value.strength = 42;
    }
    auto scan = assembler.consume(group,
        1'700'000'000'000'000'000LL + static_cast<std::int64_t>(sequence) * 10'000'000,
        "n10_link", 0.01, 65.534);
    if (scan) completed = std::move(scan);
  }

  ASSERT_TRUE(completed);
  EXPECT_EQ(completed->header.frame_id, "n10_link");
  EXPECT_EQ(completed->ranges.size(), nav::N10ScanAssembler::kAngleBins);
  EXPECT_NEAR(completed->angle_increment, static_cast<float>(std::acos(-1.0) / 18000.0), 1e-8);
  // 扫描保留雷达坐标，车体TF只应用一次实测偏移。
  EXPECT_NEAR(completed->ranges[0], 1.25F, 1e-6F);
  const auto sensor_tf = nav::n10_sensor_transform(
      "base_link", completed->header.frame_id, nav::kDefaultN10OffsetXMetres, 0.0, 0.0);
  EXPECT_EQ(sensor_tf.child_frame_id, completed->header.frame_id);
  EXPECT_NEAR(sensor_tf.transform.translation.x + completed->ranges[0], 1.38, 1e-6);
  // 将包含雷达原点的车体坐标整体旋转到里程计坐标。
  const auto base_yaw = std::acos(-1.0) / 2.0;
  const auto base_x = sensor_tf.transform.translation.x + completed->ranges[0];
  EXPECT_NEAR(1.0 + std::cos(base_yaw) * base_x, 1.0, 1e-6);
  EXPECT_NEAR(2.0 + std::sin(base_yaw) * base_x, 3.38, 1e-6);
  EXPECT_NEAR(completed->ranges[14800], 1.25F, 1e-6F);
  EXPECT_FLOAT_EQ(completed->intensities[14800], 42.0F);
  EXPECT_NEAR(completed->ranges[27000], 2.5F, 1e-6F);
  EXPECT_NEAR(completed->ranges[9000], 1.25F, 1e-6F);
  EXPECT_TRUE(std::isnan(completed->ranges[35999]));
}

TEST(N10ScanAssembler, DropsARevolutionAfterSequenceGapOrCounterChange) {
  // 先越过首个同步回绕，再分别在完整候选扫描内注入序号丢包和错误计数变化。
  // 每包16个角度均有序，避免未填写的15个零角度让夹具自身反复逆序。
  for (bool counter_change : {false, true}) {
    nav::N10ScanAssembler assembler;
    std::uint32_t sequence = 0;
    for (std::uint32_t first = 0; first < 736; first += 16, ++sequence) {
      nav::N10Group group;
      group.sequence = sequence + (!counter_change && sequence >= 30 ? 1 : 0);
      group.crc_errors = counter_change && sequence >= 30 ? 1 : 0;
      for (std::uint32_t point = 0; point < group.points.size(); ++point) {
        group.points[point].angle_cdeg = static_cast<std::uint16_t>(((first + point) * 100U) % 36000U);
        group.points[point].distance_mm = 1000;
      }
      EXPECT_FALSE(assembler.consume(group,
          1'000'000'000LL + static_cast<std::int64_t>(sequence) * 10'000'000,
          "n10_link"));
    }
    EXPECT_EQ(assembler.dropped_scans(), 1U);
  }
}

TEST(WheelOdometryIntegrator, IntegratesHolonomicBodyVelocityAndRejectsOldSamples) {
  nav::WheelOdometryIntegrator integrator;
  const auto first = integrator.update(1.0, 0.5, 0.0, 1'000'000, 2'000'000'000,
      true, "odom", "base_link");
  ASSERT_TRUE(first);
  const auto second = integrator.update(1.0, 0.5, 0.0, 1'100'000, 2'100'000'000,
      true, "odom", "base_link");
  ASSERT_TRUE(second);
  EXPECT_NEAR(second->pose.pose.position.x, 0.1, 1e-9);
  EXPECT_NEAR(second->pose.pose.position.y, 0.05, 1e-9);
  EXPECT_FALSE(integrator.update(1.0, 0.5, 0.0, 1'050'000, 2'050'000'000,
      true, "odom", "base_link"));

  const auto transform = nav::odom_transform(*second);
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, second->pose.pose.position.x);
}

TEST(RosNavigationMessages, ConvertsFusionStateAndCovarianceIntoOdometry) {
  const std::array<double, 7> state{1.0, -2.0, 0.5, 0.2, -0.1, 0.3, 0.01};
  const std::array<double, 7> covariance{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const auto odom = nav::make_fusion_odometry(state, covariance, 3'000'000'000,
      "odom", "base_link");
  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(odom.pose.covariance[0], 0.1);
  EXPECT_DOUBLE_EQ(odom.pose.covariance[7], 0.2);
  EXPECT_DOUBLE_EQ(odom.twist.covariance[35], 0.6);
  EXPECT_NEAR(odom.pose.pose.orientation.z, std::sin(0.25), 1e-12);
}

TEST(RosNavigationMessages, BuildsStaticN10TransformFromBaseFrameParameters) {
  const auto yaw = std::acos(-1.0) / 2.0;
  const auto transform = nav::n10_sensor_transform("base_link", "n10_sensor", 0.25, -0.1, yaw);
  EXPECT_EQ(transform.header.frame_id, "base_link");
  EXPECT_EQ(transform.child_frame_id, "n10_sensor");
  EXPECT_DOUBLE_EQ(transform.transform.translation.x, 0.25);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y, -0.1);
  EXPECT_DOUBLE_EQ(transform.transform.translation.z, 0.0);
  EXPECT_NEAR(transform.transform.rotation.z, std::sin(yaw * 0.5), 1e-12);
  EXPECT_NEAR(transform.transform.rotation.w, std::cos(yaw * 0.5), 1e-12);
}
