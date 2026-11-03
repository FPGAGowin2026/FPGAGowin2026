#include <cmath>
#include <limits>
#include <stdexcept>
#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include "uav_vio_bridge/uav_vio_bridge.hpp"

namespace
{
using uav_vio_bridge::BridgeConfig;
using uav_vio_bridge::OdometryAdapter;
using Odom = nav_msgs::msg::Odometry;
constexpr int64_t kTime = 100000000000LL;

Odom sample(int64_t stamp = kTime)
{
  Odom msg;
  msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
  msg.header.frame_id = "global";
  msg.child_frame_id = "imu";
  msg.pose.pose.orientation.w = 1.0;
  for (size_t i = 0; i < 6; ++i) {
    msg.pose.covariance[i * 7] = 0.01 * (i + 1);
    msg.twist.covariance[i * 7] = 0.1 * (i + 1);
  }
  return msg;
}

diagnostic_msgs::msg::DiagnosticArray health(
  int64_t stamp = kTime, const std::string & session = "test-session", uint64_t generation = 1,
  bool initialized = true)
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header = sample(stamp).header;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "openvins/visual_fusion";
  status.hardware_id = session;
  status.level = status.OK;
  for (const auto & entry : std::vector<std::pair<std::string, std::string>>{
      {"contract_version", "1"}, {"initialized", initialized ? "true" : "false"},
      {"reset_counter", std::to_string(generation)}})
  {
    diagnostic_msgs::msg::KeyValue value;
    value.key = entry.first;
    value.value = entry.second;
    status.values.push_back(value);
  }
  msg.status.push_back(status);
  return msg;
}

void initialize(OdometryAdapter & adapter)
{
  ASSERT_TRUE(adapter.observe_health(health(), kTime, kTime).empty());
}

TEST(VioAdapter, PreservesSampleTimeAndUsesHamiltonChildToWorld)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  auto input = sample();
  input.pose.pose.orientation.z = std::sin(0.3);
  input.pose.pose.orientation.w = std::cos(0.3);
  input.pose.pose.position.x = 2.0;
  Odom output;
  ASSERT_TRUE(adapter.process(input, kTime, kTime, output).empty());
  EXPECT_EQ(output.header.stamp, input.header.stamp);
  EXPECT_EQ(output.header.frame_id, "vio_odom");
  EXPECT_EQ(output.child_frame_id, "vio_body");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 2.0);
  EXPECT_NEAR(output.pose.pose.orientation.z, std::sin(0.3), 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.w, std::cos(0.3), 1e-12);
}

TEST(VioAdapter, WorldYawRotatesPoseButNotBodyTwist)
{
  BridgeConfig config;
  config.world_yaw_rad = std::acos(-1.0) / 2.0;
  OdometryAdapter adapter(config);
  initialize(adapter);
  auto input = sample();
  input.pose.pose.position.x = 1.0;
  input.twist.twist.linear.x = 2.0;
  Odom output;
  ASSERT_TRUE(adapter.process(input, kTime, kTime, output).empty());
  EXPECT_NEAR(output.pose.pose.position.x, 0.0, 1e-12);
  EXPECT_NEAR(output.pose.pose.position.y, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 2.0);
  EXPECT_NEAR(output.pose.covariance[0], input.pose.covariance[7], 1e-12);
}

TEST(VioAdapter, LeverArmAndBodyRotationAffectPoseTwistAndCovariance)
{
  BridgeConfig config;
  config.position_body_in_imu = Eigen::Vector3d(1.0, 0.0, 0.0);
  config.rotation_body_imu = Eigen::Quaterniond(
    Eigen::AngleAxisd(std::acos(-1.0) / 2.0, Eigen::Vector3d::UnitZ()));
  OdometryAdapter adapter(config);
  initialize(adapter);
  auto input = sample();
  input.twist.twist.linear.x = 1.0;
  input.twist.twist.angular.z = 2.0;
  Odom output;
  ASSERT_TRUE(adapter.process(input, kTime, kTime, output).empty());
  EXPECT_NEAR(output.pose.pose.position.x, 1.0, 1e-12);
  EXPECT_NEAR(output.pose.pose.orientation.z, -std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.x, -2.0, 1e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, 1.0, 1e-12);
  EXPECT_NEAR(output.twist.twist.angular.z, 2.0, 1e-12);
  EXPECT_NEAR(output.pose.covariance[7], 0.02 + 0.06 + std::hypot(0.06, 0.05), 1e-12);
  EXPECT_NEAR(output.pose.covariance[14], 0.03 + 0.05 + std::hypot(0.06, 0.05), 1e-12);
  EXPECT_NEAR(output.twist.covariance[0], 0.2 + 0.6, 1e-12);
  EXPECT_NEAR(output.twist.covariance[14], 0.3 + 0.5, 1e-12);
  EXPECT_NEAR(output.twist.covariance[5], -0.6, 1e-12);
}

TEST(VioAdapter, AttitudeUpperBoundIncludesOffDiagonalUncertainty)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  auto input = sample();
  input.pose.covariance[21] = 0.05;
  input.pose.covariance[28] = 0.05;
  input.pose.covariance[22] = input.pose.covariance[27] = 0.02;
  Odom output;
  ASSERT_TRUE(adapter.process(input, kTime, kTime, output).empty());
  for (size_t i = 3; i < 6; ++i) {
    EXPECT_NEAR(output.pose.covariance[i * 7], 0.07, 1e-12);
  }
  EXPECT_DOUBLE_EQ(output.pose.covariance[22], 0.0);
}

TEST(VioAdapter, RejectsInvalidFramesQuaternionAndCovariance)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  Odom output;
  auto input = sample();
  input.child_frame_id = "base_link";
  EXPECT_EQ(adapter.process(input, kTime, kTime, output), "unexpected input frames");
  input = sample();
  input.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(adapter.process(input, kTime, kTime, output).empty());
  input = sample();
  input.pose.covariance[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.process(input, kTime, kTime, output).empty());
  input = sample();
  input.pose.covariance[1] = input.pose.covariance[6] = 1.0;  // symmetric but not PSD
  EXPECT_FALSE(adapter.process(input, kTime, kTime, output).empty());
  EXPECT_FALSE(adapter.faulted());
}

TEST(VioAdapter, RejectsStaleFutureAndMissingSampleTime)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  Odom output;
  EXPECT_EQ(
    adapter.process(sample(kTime - 200000000), kTime, kTime, output),
    "stale sample timestamp");
  EXPECT_EQ(
    adapter.process(sample(kTime + 20000000), kTime, kTime, output),
    "future sample timestamp");
  EXPECT_EQ(adapter.process(sample(0), kTime, kTime, output), "missing sample timestamp");
}

TEST(VioAdapter, RateLimitNeverReplaysOrPublishesDuplicateSamples)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  Odom output;
  ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
  EXPECT_EQ(adapter.process(sample(), kTime, kTime, output), "duplicate odometry timestamp");
  EXPECT_EQ(
    adapter.process(
      sample(kTime + 10000000), kTime + 10000000,
      kTime + 10000000, output), "rate limited");
  auto newest = sample(kTime + 30000000);
  ASSERT_TRUE(adapter.process(newest, kTime + 30000000, kTime + 30000000, output).empty());
  EXPECT_EQ(output.header.stamp, newest.header.stamp);
}

TEST(VioAdapter, VisualTimeoutLatchesEvenWhenImuOdometryContinues)
{
  OdometryAdapter adapter(BridgeConfig{});
  Odom output;
  EXPECT_FALSE(adapter.process(sample(), kTime, kTime, output).empty());
  initialize(adapter);
  ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
  ASSERT_TRUE(
    adapter.process(
      sample(kTime + 150000000), kTime + 150000000,
      kTime + 150000000, output).empty());
  EXPECT_FALSE(
    adapter.process(
      sample(kTime + 310000000), kTime + 310000000,
      kTime + 310000000, output).empty());
  EXPECT_TRUE(adapter.faulted());
  EXPECT_FALSE(
    adapter.observe_health(
      health(kTime + 310000000),
      kTime + 310000000, kTime + 310000000).empty());
}

TEST(VioAdapter, DuplicateVisualUpdatesDoNotRenewWatchdog)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  Odom output;
  ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
  EXPECT_EQ(
    adapter.observe_health(health(), kTime + 100000000, kTime + 100000000),
    "duplicate visual timestamp");
  EXPECT_FALSE(adapter.check_timeout(kTime + 310000000, kTime + 310000000).empty());
  EXPECT_TRUE(adapter.faulted());
}

TEST(VioAdapter, OdometryGapAndClockRollbackLatch)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  Odom output;
  ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
  ASSERT_TRUE(
    adapter.observe_health(
      health(kTime + 100000000),
      kTime + 100000000, kTime + 100000000).empty());
  EXPECT_FALSE(adapter.check_timeout(kTime + 210000000, kTime + 210000000).empty());
  EXPECT_TRUE(adapter.faulted());
  OdometryAdapter clock_adapter(BridgeConfig{});
  initialize(clock_adapter);
  ASSERT_TRUE(clock_adapter.process(sample(), kTime, kTime, output).empty());
  EXPECT_FALSE(clock_adapter.check_timeout(kTime - 1, kTime + 1).empty());
  EXPECT_TRUE(clock_adapter.faulted());
}

TEST(VioAdapter, MonotonicTimestampWithResetPoseStillLatches)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  Odom output;
  ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
  auto reset = sample(kTime + 30000000);
  reset.pose.pose.position.x = 2.0;
  EXPECT_FALSE(adapter.process(reset, kTime + 30000000, kTime + 30000000, output).empty());
  EXPECT_TRUE(adapter.faulted());
}

TEST(VioAdapter, RejectsInvalidConfiguration)
{
  BridgeConfig config;
  config.publish_rate_hz = 0.0;
  EXPECT_THROW(OdometryAdapter adapter(config), std::invalid_argument);
  config = BridgeConfig{};
  config.rotation_body_imu.coeffs().setZero();
  EXPECT_THROW(OdometryAdapter adapter(config), std::invalid_argument);
}

TEST(VioAdapter, MissingFusionCannotEnableOdometry)
{
  OdometryAdapter adapter(BridgeConfig{});
  auto msg = health(0);
  msg.status.front().level = msg.status.front().WARN;
  EXPECT_FALSE(adapter.observe_health(msg, kTime, kTime).empty());
  Odom output;
  EXPECT_FALSE(adapter.process(sample(), kTime, kTime, output).empty());
  EXPECT_FALSE(adapter.faulted());
  initialize(adapter);
  EXPECT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
}

TEST(VioAdapter, SmallResetAndProcessRestartLatchWithoutPoseJump)
{
  for (const bool restart : {false, true}) {
    OdometryAdapter adapter(BridgeConfig{});
    initialize(adapter);
    Odom output;
    ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
    auto changed = health(
      kTime + 30000000, restart ? "new-session" : "test-session",
      restart ? 1 : 2);
    EXPECT_FALSE(adapter.observe_health(changed, kTime + 30000000, kTime + 30000000).empty());
    EXPECT_TRUE(adapter.faulted());
    EXPECT_FALSE(
      adapter.process(
        sample(kTime + 30000000), kTime + 30000000,
        kTime + 30000000, output).empty());
  }
}

TEST(VioAdapter, LostInitializationOrUnhealthyFusionStopsImmediately)
{
  for (const bool lost_initialization : {false, true}) {
    OdometryAdapter adapter(BridgeConfig{});
    initialize(adapter);
    Odom output;
    ASSERT_TRUE(adapter.process(sample(), kTime, kTime, output).empty());
    auto msg = health(kTime + 1000000, "test-session", 1, !lost_initialization);
    if (!lost_initialization) {msg.status.front().level = msg.status.front().ERROR;}
    EXPECT_FALSE(adapter.observe_health(msg, kTime + 1000000, kTime + 1000000).empty());
    EXPECT_TRUE(adapter.faulted());
  }
}

TEST(VioAdapter, RejectsMalformedAndUnsupportedHealth)
{
  OdometryAdapter adapter(BridgeConfig{});
  for (int kind = 0; kind < 7; ++kind) {
    auto msg = health();
    if (kind == 0) {msg.header.frame_id = "map";}
    if (kind == 1) {msg.status.front().values[0].value = "2";}
    if (kind == 2) {msg.status.front().values[2].value = "1junk";}
    if (kind == 3) {msg.status.front().hardware_id.clear();}
    if (kind == 4) {msg.status.front().values.pop_back();}
    if (kind == 5) {msg.status.front().values.push_back(msg.status.front().values.front());}
    if (kind == 6) {msg.status.push_back(msg.status.front());}
    EXPECT_FALSE(adapter.observe_health(msg, kTime, kTime).empty());
  }
  Odom output;
  EXPECT_FALSE(adapter.process(sample(), kTime, kTime, output).empty());
}

TEST(VioAdapter, CorrelationRemovalKeepsFullPoseCovarianceUpperBound)
{
  OdometryAdapter adapter(BridgeConfig{});
  initialize(adapter);
  auto input = sample();
  input.pose.covariance[3] = input.pose.covariance[18] = 0.015;
  Odom output;
  ASSERT_TRUE(adapter.process(input, kTime, kTime, output).empty());
  using Matrix6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  const Matrix6 difference = Eigen::Map<const Matrix6>(output.pose.covariance.data()) -
    Eigen::Map<const Matrix6>(input.pose.covariance.data());
  Eigen::SelfAdjointEigenSolver<Matrix6> solver(difference);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12);
  EXPECT_DOUBLE_EQ(output.pose.covariance[3], 0.0);
}
}  // namespace
