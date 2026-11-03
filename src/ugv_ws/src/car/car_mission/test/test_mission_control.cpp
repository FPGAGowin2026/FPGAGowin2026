#define CAR_MISSION_NO_MAIN
#include "../src/mission_control_node.cpp"

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace car_mission {
namespace {

DetectionArray make_array()
{
  DetectionArray array;
  array.header.frame_id = "camera_optical_frame";
  array.header.stamp.sec = 10;
  array.header.stamp.nanosec = 20;
  return array;
}

Detection make_detection(
  const std::string & id, double score, const std_msgs::msg::Header & header,
  const std::string & class_id = "0")
{
  Detection detection;
  detection.id = id;
  detection.header = header;
  detection.bbox.center.position.x = 100.0;
  detection.bbox.center.position.y = 80.0;
  detection.bbox.center.theta = 0.0;
  detection.bbox.size_x = 30.0;
  detection.bbox.size_y = 24.0;
  detection.results.emplace_back();
  detection.results.back().hypothesis.class_id = class_id;
  detection.results.back().hypothesis.score = score;
  return detection;
}

TargetConfig test_config()
{
  return {"0", "", 0.5, 0.2};
}

}  // namespace

TEST(MissionRange, UsesOnlyFiniteForwardObstacleReturns)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = -1.0f; scan.angle_increment = 0.25f;
  scan.range_min = 0.05f; scan.range_max = 8.0f;
  scan.ranges.assign(9, std::numeric_limits<float>::infinity());
  EXPECT_FALSE(front_obstacle_distance(scan).has_value());
  scan.ranges[0] = 0.2f; scan.ranges[4] = 1.0f; scan.ranges[5] = 0.6f;
  ASSERT_TRUE(front_obstacle_distance(scan).has_value());
  EXPECT_NEAR(*front_obstacle_distance(scan), 0.6, 1e-6);
  scan.ranges[4] = std::numeric_limits<float>::quiet_NaN();
  scan.ranges[5] = 0.01f;
  EXPECT_FALSE(front_obstacle_distance(scan).has_value());
}

TEST(MissionFreshness, ReportsSourceAndReceiptFailuresSeparately)
{
  EXPECT_STREQ(observation_failure_reason(false, INFINITY, false, INFINITY, 0.2), "no_source");
  EXPECT_STREQ(observation_failure_reason(true, -0.01, true, 0.01, 0.2), "future_source");
  EXPECT_STREQ(observation_failure_reason(true, 0.201, true, 0.01, 0.2), "source_timeout");
  EXPECT_STREQ(observation_failure_reason(true, 0.01, false, INFINITY, 0.2), "no_receipt");
  EXPECT_STREQ(observation_failure_reason(true, 0.01, true, 0.201, 0.2), "receipt_timeout");
  EXPECT_EQ(observation_failure_reason(true, 0.2, true, 0.2, 0.2), nullptr);
}

TEST(MissionFreshness, RecordedJitterFitsTheBoundedObservationWindow)
{
  EXPECT_DOUBLE_EQ(kDefaultTimeout, 0.25);
  for (const auto & ages : {std::pair{0.222947, 0.202893},
      std::pair{0.213172, 0.144487}, std::pair{0.208298, 0.188078}})
  {
    EXPECT_STREQ(observation_failure_reason(true, ages.first, true, ages.second, 0.2), "source_timeout");
    EXPECT_EQ(observation_failure_reason(true, ages.first, true, ages.second, kDefaultTimeout), nullptr);
  }
  EXPECT_EQ(observation_failure_reason(true, 0.25, true, 0.25, kDefaultTimeout), nullptr);
  EXPECT_STREQ(observation_failure_reason(true, 0.250001, true, 0.1, kDefaultTimeout), "source_timeout");
  EXPECT_STREQ(observation_failure_reason(true, 0.1, true, 0.250001, kDefaultTimeout), "receipt_timeout");
  EXPECT_STREQ(observation_failure_reason(true, -0.001, true, 0.1, kDefaultTimeout), "future_source");
}

TEST(MissionSelection, UsesConfidenceThenStableIdOrder)
{
  auto array = make_array();
  array.detections.push_back(make_detection("session:z", 0.8, array.header));
  array.detections.push_back(make_detection("session:a", 0.8, array.header));
  array.detections.push_back(make_detection("session:lower", 0.7, array.header));

  const auto selected = select_detection(array, test_config());

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->id, "session:a");
}

TEST(MissionSelection, TrackingCanOnlyRecoverTheLockedFullId)
{
  auto array = make_array();
  array.detections.push_back(make_detection("session:locked", 0.6, array.header));
  array.detections.push_back(make_detection("session:other", 0.99, array.header));

  const auto selected = select_detection(array, test_config(), "session:locked");
  const auto missing = select_detection(array, test_config(), "other-session:locked");

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->id, "session:locked");
  EXPECT_FALSE(missing.has_value());
}

TEST(MissionSelection, ReportsWhyTheLockedTargetDidNotQualify)
{
  auto array = make_array();
  const char * reason = nullptr;
  EXPECT_FALSE(select_detection(array, test_config(), "session:locked", &reason));
  EXPECT_STREQ(reason, "target_id_missing");

  array.detections.push_back(make_detection("session:locked", 0.9, array.header, "1"));
  EXPECT_FALSE(select_detection(array, test_config(), "session:locked", &reason));
  EXPECT_STREQ(reason, "class_mismatch");

  array.detections.front() = make_detection("session:locked", 0.1, array.header);
  EXPECT_FALSE(select_detection(array, test_config(), "session:locked", &reason));
  EXPECT_STREQ(reason, "low_confidence");
}

TEST(MissionSelection, RejectsMalformedBoxes)
{
  auto array = make_array();
  auto detection = make_detection("session:target", 0.9, array.header);
  detection.bbox.size_x = std::numeric_limits<double>::quiet_NaN();
  array.detections.push_back(detection);
  EXPECT_FALSE(select_detection(array, test_config()).has_value());

}

TEST(MissionSelection, AppliesClassAndConfidenceFilters)
{
  auto array = make_array();
  auto wrong_class = make_detection("session:wrong-class", 0.99, array.header);
  wrong_class.results.front().hypothesis.class_id = "1";
  array.detections.push_back(wrong_class);
  array.detections.push_back(make_detection("session:weak", 0.49, array.header));

  EXPECT_FALSE(select_detection(array, test_config()).has_value());
}

TEST(MotionServo, RightTargetTurnsNegativeWithDeadbandAndSpeedLimit)
{
  const auto array = make_array();
  auto detection = make_detection("session:target", 0.9, array.header);
  MotionConfig config;
  EXPECT_DOUBLE_EQ(config.max_alignment_angular_speed, 0.3);
  detection.bbox.center.position.x = config.image_width;
  auto command = compute_servo_command(detection, config);
  EXPECT_FALSE(config.enabled);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);

  config.enabled = true;

  command = compute_servo_command(detection, config);
  EXPECT_NEAR(command.angular.z, -config.max_alignment_angular_speed, 1e-9);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);

  detection.bbox.center.position.x = config.image_width * 0.52;
  command = compute_servo_command(detection, config);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);

  detection.bbox.center.position.x = 0.0;
  command = compute_servo_command(detection, config);
  EXPECT_NEAR(command.angular.z, config.max_alignment_angular_speed, 1e-9);
}

TEST(MotionServo, BoxSizeNeverCreatesUnmeasuredTranslation)
{
  const auto array = make_array();
  auto detection = make_detection("session:target", 0.9, array.header);
  MotionConfig config;
  config.enabled = true;
  detection.bbox.center.position.x = config.image_width / 2.0;
  detection.bbox.size_y = 24.0;

  auto command = compute_servo_command(detection, config);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);

  detection.bbox.size_y = 96.0;
  detection.bbox.center.position.x = 600.0;
  command = compute_servo_command(detection, config);
  EXPECT_LT(command.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);

  detection.bbox.center.position.x = config.image_width / 2.0;
  command = compute_servo_command(detection, config);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);

  detection.bbox.size_y = 288.0;
  command = compute_servo_command(detection, config);
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
}

TEST(MotionServo, RejectsNonFiniteAndOutOfRangeConfiguration)
{
  MotionConfig config;
  EXPECT_TRUE(valid_motion_config(config));

  config.image_width = 0.0;
  EXPECT_FALSE(valid_motion_config(config));

  config = MotionConfig{};
  config.angular_gain = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(valid_motion_config(config));

  config = MotionConfig{};
  config.max_alignment_angular_speed = 0.301;
  EXPECT_FALSE(valid_motion_config(config));
}

TEST(MissionControl, RunsTargetLockAndSafetyFlowThroughRosInterfaces)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions invalid_options;
  invalid_options.parameter_overrides({rclcpp::Parameter("linear_speed_m_s", 0.401)});
  EXPECT_THROW(std::make_shared<MissionControlNode>(invalid_options), std::invalid_argument);
  for (const double speed : {0.0, -0.01, 0.801,
      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
  {
    SCOPED_TRACE(speed);
    invalid_options.parameter_overrides({rclcpp::Parameter("search_angular_speed", speed)});
    EXPECT_THROW(std::make_shared<MissionControlNode>(invalid_options), std::invalid_argument);
  }
  rclcpp::NodeOptions mission_options;
  mission_options.parameter_overrides({
    rclcpp::Parameter("motion_enabled", true)});
  auto mission = std::make_shared<MissionControlNode>(mission_options);
  auto test_node = std::make_shared<rclcpp::Node>("mission_control_test_client");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(mission);
  executor.add_node(test_node);

  struct SpinGuard
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread thread;

    explicit SpinGuard(rclcpp::executors::SingleThreadedExecutor & value)
    : executor(value), thread([this]() {executor.spin();}) {}

    ~SpinGuard()
    {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  };
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<DetectionArray> outputs;
  std::vector<std::string> states;
  std::vector<geometry_msgs::msg::Twist> commands;
  std::vector<ugv_bridge::msg::AuthorizedTwist> authorized_commands;
  auto output_subscription = test_node->create_subscription<DetectionArray>(
    "/mission/selected_target", rclcpp::QoS(10),
    [&](DetectionArray::ConstSharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        outputs.push_back(*message);
      }
      changed.notify_all();
    });
  auto state_subscription = test_node->create_subscription<std_msgs::msg::String>(
    "/mission/state", rclcpp::QoS(1).transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        states.push_back(message->data);
      }
      changed.notify_all();
    });
  auto command_subscription = test_node->create_subscription<ugv_bridge::msg::AuthorizedTwist>(
    "/mission/cmd_vel", rclcpp::QoS(10),
    [&](ugv_bridge::msg::AuthorizedTwist::ConstSharedPtr message) {
      {
        std::lock_guard<std::mutex> lock(mutex);
        commands.push_back(message->twist);
        authorized_commands.push_back(*message);
      }
      changed.notify_all();
    });
  auto detections_publisher = test_node->create_publisher<DetectionArray>(
    "/vision/detections", rclcpp::QoS(10));
  auto emergency_publisher = test_node->create_publisher<std_msgs::msg::Bool>(
    "/emergency_stop", rclcpp::QoS(10));
  auto authorization_publisher = test_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/fpga/bidirectional_status", rclcpp::QoS(10));
  auto start_client = test_node->create_client<std_srvs::srv::Trigger>("/mission/start");
  auto stop_client = test_node->create_client<std_srvs::srv::Trigger>("/mission/stop");

  ASSERT_TRUE(start_client->wait_for_service(std::chrono::seconds(5)));
  ASSERT_TRUE(stop_client->wait_for_service(std::chrono::seconds(5)));
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while ((detections_publisher->get_subscription_count() == 0 ||
    output_subscription->get_publisher_count() == 0 ||
    command_subscription->get_publisher_count() == 0) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(detections_publisher->get_subscription_count(), 0U);
  ASSERT_GT(output_subscription->get_publisher_count(), 0U);
  ASSERT_GT(command_subscription->get_publisher_count(), 0U);

  auto output_count = [&]() {
      std::lock_guard<std::mutex> lock(mutex);
      return outputs.size();
    };
  auto state_count = [&]() {
      std::lock_guard<std::mutex> lock(mutex);
      return states.size();
    };
  auto command_count = [&]() {
      std::lock_guard<std::mutex> lock(mutex);
      return commands.size();
    };
  auto wait_output = [&](std::size_t begin, auto predicate) -> std::optional<DetectionArray> {
      std::unique_lock<std::mutex> lock(mutex);
      const auto matches = [&]() {
          for (std::size_t index = begin; index < outputs.size(); ++index) {
            if (predicate(outputs[index])) {
              return true;
            }
          }
          return false;
        };
      if (!changed.wait_for(lock, std::chrono::seconds(3), matches)) {
        return std::nullopt;
      }
      for (std::size_t index = begin; index < outputs.size(); ++index) {
        if (predicate(outputs[index])) {
          return outputs[index];
        }
      }
      return std::nullopt;
    };
  auto wait_state = [&](std::size_t begin, const std::string & expected) {
      std::unique_lock<std::mutex> lock(mutex);
      const auto matches = [&]() {
          for (std::size_t index = begin; index < states.size(); ++index) {
            if (states[index] == expected) {
              return true;
            }
          }
          return false;
        };
      return changed.wait_for(lock, std::chrono::seconds(3), matches);
    };
  auto wait_command = [&](std::size_t begin, auto predicate)
    -> std::optional<geometry_msgs::msg::Twist> {
      std::unique_lock<std::mutex> lock(mutex);
      const auto matches = [&]() {
          for (std::size_t index = begin; index < commands.size(); ++index) {
            if (predicate(commands[index])) {
              return true;
            }
          }
          return false;
        };
      if (!changed.wait_for(lock, std::chrono::seconds(3), matches)) {
        return std::nullopt;
      }
      for (std::size_t index = begin; index < commands.size(); ++index) {
        if (predicate(commands[index])) {
          return commands[index];
        }
      }
      return std::nullopt;
    };
  auto has_motion = [&](std::size_t begin) {
      std::lock_guard<std::mutex> lock(mutex);
      for (std::size_t index = begin; index < commands.size(); ++index) {
        if (std::abs(commands[index].linear.x) > 1e-9 ||
          std::abs(commands[index].angular.z) > 1e-9)
        {
          return true;
        }
      }
      return false;
    };
  const auto is_zero_command = [](const geometry_msgs::msg::Twist & command) {
      return std::abs(command.linear.x) <= 1e-9 && std::abs(command.angular.z) <= 1e-9;
    };
  const auto is_motion_command = [](const geometry_msgs::msg::Twist & command) {
      return std::abs(command.linear.x) > 1e-9 || std::abs(command.angular.z) > 1e-9;
    };
  auto call_trigger = [](const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client) {
      if (!client->wait_for_service(std::chrono::seconds(3))) {
        return std::shared_ptr<std_srvs::srv::Trigger::Response>{};
      }
      auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        return std::shared_ptr<std_srvs::srv::Trigger::Response>{};
      }
      return future.get();
    };
  auto set_parameters = [&](std::vector<rclcpp::Parameter> parameters) {
      const auto results = mission->set_parameters(parameters);
      return std::all_of(results.begin(), results.end(), [](const auto & result) {
        return result.successful;
      });
    };
  auto make_frame = [&](const std::string & id, double score, const std::string & class_id = "0") {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      auto frame = make_array();
      const auto stamp = test_node->now();
      frame.header.stamp = stamp;
      if (!id.empty()) {
        frame.detections.push_back(make_detection(id, score, frame.header, class_id));
      }
      return frame;
    };
  auto same_stamp = [](const std_msgs::msg::Header & first, const std_msgs::msg::Header & second) {
      return first.stamp.sec == second.stamp.sec &&
             first.stamp.nanosec == second.stamp.nanosec;
    };
  auto wait_for_frame = [&](std::size_t begin, const DetectionArray & source, bool selected) {
      return wait_output(begin, [&](const DetectionArray & output) {
        return same_stamp(output.header, source.header) &&
               output.detections.size() == (selected ? 1U : 0U);
      });
    };

  SpinGuard spin_guard(executor);
  std::mutex authorization_mutex;
  Authorization heartbeat{"test-bridge", 1, true, false, "ros"};
  std::atomic<bool> emit_heartbeat{true};
  struct HeartbeatGuard
  {
    std::atomic<bool> running{true};
    std::thread thread;
    explicit HeartbeatGuard(std::function<void()> publish)
    : thread([this, publish]() {
        while (running.load()) {
          publish();
          std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
      }) {}
    ~HeartbeatGuard() { running = false; thread.join(); }
  } heartbeat_guard([&]() {
      if (!emit_heartbeat.load()) return;
      Authorization current;
      {
        std::lock_guard<std::mutex> lock(authorization_mutex);
        current = heartbeat;
      }
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = test_node->now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "bx72_udp";
      auto value = [&](const std::string & key, const std::string & text) {
          diagnostic_msgs::msg::KeyValue entry;
          entry.key = key; entry.value = text; status.values.push_back(entry);
        };
      value("bridge_id", current.bridge_id);
      value("ros_authorization_epoch", std::to_string(current.epoch));
      value("ros_authorized", current.authorized ? "True" : "False");
      value("emergency_stop", current.emergency_stop ? "True" : "False");
      value("control_source", current.control_source);
      array.status.push_back(status);
      authorization_publisher->publish(array);
    });
  auto set_authorization = [&](const Authorization & value) {
      std::lock_guard<std::mutex> lock(authorization_mutex);
      heartbeat = value;
    };
  std::this_thread::sleep_for(std::chrono::milliseconds(160));

  auto command_begin = command_count();
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());

  ASSERT_TRUE(set_parameters({rclcpp::Parameter("observation_timeout_s", 0.2)}));
  const auto no_observation = call_trigger(start_client);
  ASSERT_NE(no_observation, nullptr);
  EXPECT_FALSE(no_observation->success);
  EXPECT_NE(no_observation->message.find("没有新鲜视觉观测"), std::string::npos);

  const auto first_frame = make_frame("session:locked", 0.9);
  auto output_begin = output_count();
  detections_publisher->publish(first_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, first_frame, false).has_value());

  ASSERT_TRUE(set_parameters({rclcpp::Parameter("min_confidence", 1.01)}));
  const auto invalid_confidence_start = call_trigger(start_client);
  ASSERT_NE(invalid_confidence_start, nullptr);
  EXPECT_FALSE(invalid_confidence_start->success);
  EXPECT_NE(
    invalid_confidence_start->message.find("min_confidence 必须在 [0, 1] 内"),
    std::string::npos);

  ASSERT_TRUE(set_parameters({
    rclcpp::Parameter("min_confidence", 0.5),
    rclcpp::Parameter("observation_timeout_s", 0.0),
  }));
  const auto invalid_timeout_start = call_trigger(start_client);
  ASSERT_NE(invalid_timeout_start, nullptr);
  EXPECT_FALSE(invalid_timeout_start->success);
  EXPECT_NE(
    invalid_timeout_start->message.find("observation_timeout_s 必须在(0, 0.25]秒内"),
    std::string::npos);

  ASSERT_TRUE(set_parameters({rclcpp::Parameter("observation_timeout_s", 0.251)}));
  const auto too_long_timeout_start = call_trigger(start_client);
  ASSERT_NE(too_long_timeout_start, nullptr);
  EXPECT_FALSE(too_long_timeout_start->success);
  EXPECT_NE(too_long_timeout_start->message.find("observation_timeout_s"), std::string::npos);

  // A short detection gap from the field must not alternate LOST and TRACKING.
  ASSERT_TRUE(set_parameters({rclcpp::Parameter("observation_timeout_s", 0.25)}));
  const auto jitter_idle_frame = make_frame("session:jitter", 0.9);
  output_begin = output_count();
  detections_publisher->publish(jitter_idle_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, jitter_idle_frame, false).has_value());
  const auto jitter_start = call_trigger(start_client);
  ASSERT_NE(jitter_start, nullptr);
  ASSERT_TRUE(jitter_start->success);
  const auto jitter_first_frame = make_frame("session:jitter", 0.9);
  output_begin = output_count();
  detections_publisher->publish(jitter_first_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, jitter_first_frame, true).has_value());
  const auto jitter_state_begin = state_count();
  std::this_thread::sleep_for(std::chrono::milliseconds(205));
  auto jitter_next_frame = make_array();
  jitter_next_frame.header.stamp = test_node->now();
  jitter_next_frame.detections.push_back(make_detection("session:jitter", 0.9, jitter_next_frame.header));
  output_begin = output_count();
  detections_publisher->publish(jitter_next_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, jitter_next_frame, true).has_value());
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + jitter_state_begin, states.end(), "LOST"), 0);
  }

  // Repeating the last frame does not extend either freshness clock.
  const auto dropout_state_begin = state_count();
  const auto dropout_command_begin = command_count();
  for (int index = 0; index < 16; ++index) {
    detections_publisher->publish(jitter_next_frame);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(wait_state(dropout_state_begin, "LOST"));
  ASSERT_TRUE(wait_command(dropout_command_begin, is_zero_command).has_value());
  const auto jitter_recovery = make_frame("session:jitter", 0.9);
  const auto recovery_state_begin = state_count();
  output_begin = output_count();
  detections_publisher->publish(jitter_recovery);
  ASSERT_TRUE(wait_for_frame(output_begin, jitter_recovery, true).has_value());
  ASSERT_TRUE(wait_state(recovery_state_begin, "TRACKING"));
  ASSERT_TRUE(call_trigger(stop_client)->success);

  ASSERT_TRUE(set_parameters({rclcpp::Parameter("observation_timeout_s", 0.2)}));
  const auto ready_frame = make_frame("session:locked", 0.9);
  output_begin = output_count();
  detections_publisher->publish(ready_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, ready_frame, false).has_value());
  auto state_begin = state_count();
  command_begin = command_count();
  const auto first_start = call_trigger(start_client);
  ASSERT_NE(first_start, nullptr);
  ASSERT_TRUE(first_start->success);
  ASSERT_TRUE(wait_state(state_begin, "TRACKING"));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_FALSE(has_motion(command_begin));

  // A running task keeps its validated configuration snapshot even if the
  // externally editable parameters are later invalid.
  ASSERT_TRUE(set_parameters({
    rclcpp::Parameter("target_id", "session:other"),
    rclcpp::Parameter("target_class_id", "1"),
    rclcpp::Parameter("min_confidence", 0.99),
    rclcpp::Parameter("observation_timeout_s", 0.0),
  }));
  const auto same_id_frame = make_frame("session:locked", 0.8);
  output_begin = output_count();
  command_begin = command_count();
  detections_publisher->publish(same_id_frame);
  const auto same_id_output = wait_for_frame(output_begin, same_id_frame, true);
  ASSERT_TRUE(same_id_output.has_value());
  ASSERT_EQ(same_id_output->detections.size(), 1U);
  EXPECT_EQ(same_id_output->detections.front().id, "session:locked");
  EXPECT_TRUE(same_stamp(same_id_output->detections.front().header, same_id_frame.header));
  const auto first_motion = wait_command(command_begin, is_motion_command);
  ASSERT_TRUE(first_motion.has_value());
  EXPECT_GT(first_motion->angular.z, 0.0);
  EXPECT_DOUBLE_EQ(first_motion->linear.x, 0.0);

  // Four fresh misses at 60 ms intervals hold the original target even after
  // its own source stamp is over 200 ms old. Each output truthfully reports
  // the empty selection for the new frame instead of restamping that target.
  state_begin = state_count();
  DetectionArray last_good_miss;
  for (int index = 0; index < 4; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    const auto missed = index == 1 ? make_frame("other-session:locked", 0.99) :
      index == 2 ? make_frame("session:locked", 0.9, "1") :
      index == 3 ? make_frame("session:locked", 0.1) : make_frame("", 0.0);
    output_begin = output_count();
    detections_publisher->publish(missed);
    ASSERT_TRUE(wait_for_frame(output_begin, missed, false).has_value());
    last_good_miss = missed;
  }
  command_begin = command_count();
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
  }

  // Duplicate, older, stale, future and malformed-header frames do not
  // consume the fifth miss or move the source watermark. Correcting the last
  // frame with the same outer stamp must still be accepted.
  const auto bad_output_begin = output_count();
  detections_publisher->publish(last_good_miss);
  auto older_frame = last_good_miss;
  if (older_frame.header.stamp.nanosec == 0U) {
    --older_frame.header.stamp.sec;
    older_frame.header.stamp.nanosec = 999999999U;
  } else {
    --older_frame.header.stamp.nanosec;
  }
  older_frame.detections.front().header = older_frame.header;
  detections_publisher->publish(older_frame);
  auto stale_frame = last_good_miss;
  stale_frame.header.stamp.sec -= 10;
  stale_frame.detections.front().header = stale_frame.header;
  detections_publisher->publish(stale_frame);
  auto future_frame = last_good_miss;
  future_frame.header.stamp.sec += 10;
  future_frame.detections.front().header = future_frame.header;
  detections_publisher->publish(future_frame);
  const auto recovery_during_hold = make_frame("session:locked", 0.8);
  auto mismatched_stamp = recovery_during_hold;
  ++mismatched_stamp.detections.front().header.stamp.nanosec;
  detections_publisher->publish(mismatched_stamp);
  detections_publisher->publish(recovery_during_hold);
  ASSERT_TRUE(wait_for_frame(bad_output_begin, recovery_during_hold, true).has_value());
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
    EXPECT_EQ(outputs.size() - bad_output_begin, 1U);
  }

  // Recovery of the same full ID clears the count. Four more misses are
  // tolerated, and the fifth immediately loses the target and sends zero.
  for (int index = 0; index < 4; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    const auto missed = make_frame(index == 1 ? "other-session:locked" : "", 0.9);
    output_begin = output_count();
    detections_publisher->publish(missed);
    ASSERT_TRUE(wait_for_frame(output_begin, missed, false).has_value());
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  const auto fifth_miss = make_frame("other-session:locked", 0.99);
  output_begin = output_count();
  command_begin = command_count();
  detections_publisher->publish(fifth_miss);
  ASSERT_TRUE(wait_for_frame(output_begin, fifth_miss, false).has_value());
  ASSERT_TRUE(wait_state(state_begin, "LOST"));
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());

  const auto other_session = make_frame("other-session:locked", 0.99);
  output_begin = output_count();
  detections_publisher->publish(other_session);
  ASSERT_TRUE(wait_for_frame(output_begin, other_session, false).has_value());

  const auto recovered_frame = make_frame("session:locked", 0.8);
  state_begin = state_count();
  output_begin = output_count();
  detections_publisher->publish(recovered_frame);
  const auto recovered_output = wait_for_frame(output_begin, recovered_frame, true);
  ASSERT_TRUE(recovered_output.has_value());
  EXPECT_EQ(recovered_output->detections.front().id, "session:locked");
  ASSERT_TRUE(wait_state(state_begin, "TRACKING"));

  // Flooding duplicates after a valid miss cannot refresh the stream. Its
  // normal 200 ms timeout and control tick still cause LOST and zero.
  const auto missed_once = make_frame("", 0.0);
  output_begin = output_count();
  const auto last_valid_sent = std::chrono::steady_clock::now();
  detections_publisher->publish(missed_once);
  ASSERT_TRUE(wait_for_frame(output_begin, missed_once, false).has_value());
  state_begin = state_count();
  command_begin = command_count();
  for (int index = 0; index < 15; ++index) {
    detections_publisher->publish(missed_once);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    if (index == 4) {
      std::lock_guard<std::mutex> lock(mutex);
      EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
    }
  }
  ASSERT_TRUE(wait_state(state_begin, "LOST"));
  EXPECT_LT(std::chrono::steady_clock::now() - last_valid_sent, std::chrono::milliseconds(400));
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());

  const auto fresh_recovery = make_frame("session:locked", 0.8);
  state_begin = state_count();
  output_begin = output_count();
  detections_publisher->publish(fresh_recovery);
  ASSERT_TRUE(wait_for_frame(output_begin, fresh_recovery, true).has_value());
  ASSERT_TRUE(wait_state(state_begin, "TRACKING"));

  state_begin = state_count();
  output_begin = output_count();
  command_begin = command_count();
  const auto stop_response = call_trigger(stop_client);
  ASSERT_NE(stop_response, nullptr);
  ASSERT_TRUE(stop_response->success);
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  ASSERT_TRUE(wait_output(output_begin, [](const DetectionArray & output) {
    return output.detections.empty();
  }).has_value());
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  std::this_thread::sleep_for(std::chrono::milliseconds(70));
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_FALSE(authorized_commands.empty());
    EXPECT_EQ(authorized_commands.back().bridge_id, "test-bridge");
    EXPECT_EQ(authorized_commands.back().authorization_epoch, 1U);
    EXPECT_TRUE(is_zero_command(authorized_commands.back().twist));
  }

  ASSERT_TRUE(set_parameters({
    rclcpp::Parameter("target_id", ""),
    rclcpp::Parameter("target_class_id", "0"),
    rclcpp::Parameter("min_confidence", 0.5),
    rclcpp::Parameter("observation_timeout_s", 0.2),
  }));
  detections_publisher->publish(recovered_frame);
  const auto empty_barrier = make_frame("", 0.0);
  output_begin = output_count();
  detections_publisher->publish(empty_barrier);
  ASSERT_TRUE(wait_for_frame(output_begin, empty_barrier, false).has_value());
  const auto duplicate_start = call_trigger(start_client);
  ASSERT_NE(duplicate_start, nullptr);
  EXPECT_FALSE(duplicate_start->success);

  std_msgs::msg::Bool emergency;
  emergency.data = true;
  output_begin = output_count();
  command_begin = command_count();
  emergency_publisher->publish(emergency);
  ASSERT_TRUE(wait_output(output_begin, [](const DetectionArray & output) {
    return output.detections.empty();
  }).has_value());
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  const auto emergency_frame = make_frame("session:locked", 0.9);
  output_begin = output_count();
  detections_publisher->publish(emergency_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, emergency_frame, false).has_value());
  const auto emergency_start = call_trigger(start_client);
  ASSERT_NE(emergency_start, nullptr);
  EXPECT_FALSE(emergency_start->success);
  EXPECT_NE(emergency_start->message.find("急停"), std::string::npos);

  std::string state_before_release;
  {
    std::lock_guard<std::mutex> lock(mutex);
    state_before_release = states.back();
  }
  emergency.data = false;
  emergency_publisher->publish(emergency);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(states.back(), state_before_release);
    EXPECT_EQ(states.back(), "STOPPED");
  }

  ASSERT_TRUE(set_parameters({rclcpp::Parameter("observation_timeout_s", 0.2)}));
  const auto timeout_frame = make_frame("session:timeout", 0.9);
  output_begin = output_count();
  detections_publisher->publish(timeout_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, timeout_frame, false).has_value());
  state_begin = state_count();
  command_begin = command_count();
  const auto timeout_start = call_trigger(start_client);
  ASSERT_NE(timeout_start, nullptr);
  ASSERT_TRUE(timeout_start->success);
  ASSERT_TRUE(wait_state(state_begin, "TRACKING"));
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_FALSE(has_motion(command_begin));

  const auto live_timeout_frame = make_frame("session:timeout", 0.9);
  output_begin = output_count();
  command_begin = command_count();
  detections_publisher->publish(live_timeout_frame);
  ASSERT_TRUE(wait_for_frame(output_begin, live_timeout_frame, true).has_value());
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());

  // The ROS clock is frozen after two distinct source stamps are prepared.
  // A real wall timer must still stop the held target when steady receipt age
  // exceeds 200 ms, even though ROS source age no longer advances.
  {
    const auto first_missing = make_frame("", 0.0);
    const auto last_stream_frame = make_frame("", 0.0);
    struct FrozenRosClock
    {
      rclcpp::Clock::SharedPtr clock;
      bool active{false};
      explicit FrozenRosClock(rclcpp::Clock::SharedPtr value) : clock(std::move(value))
      {
        std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
        rcl_time_point_value_t current{};
        auto * handle = clock->get_clock_handle();
        active = rcl_clock_get_now(handle, &current) == RCL_RET_OK &&
          rcl_set_ros_time_override(handle, current) == RCL_RET_OK &&
          rcl_enable_ros_time_override(handle) == RCL_RET_OK;
      }
      ~FrozenRosClock()
      {
        if (active) {
          std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
          const auto result = rcl_disable_ros_time_override(clock->get_clock_handle());
          (void)result;
        }
      }
    } frozen_clock(mission->get_clock());
    ASSERT_TRUE(frozen_clock.active);

    state_begin = state_count();
    output_begin = output_count();
    detections_publisher->publish(first_missing);
    ASSERT_TRUE(wait_for_frame(output_begin, first_missing, false).has_value());
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    output_begin = output_count();
    const auto last_stream_sent = std::chrono::steady_clock::now();
    detections_publisher->publish(last_stream_frame);
    ASSERT_TRUE(wait_for_frame(output_begin, last_stream_frame, false).has_value());
    {
      std::lock_guard<std::mutex> lock(mutex);
      EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
    }

    output_begin = output_count();
    command_begin = command_count();
    ASSERT_TRUE(wait_state(state_begin, "LOST"));
    EXPECT_LT(std::chrono::steady_clock::now() - last_stream_sent, std::chrono::milliseconds(350));
    ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
    ASSERT_TRUE(wait_output(output_begin, [&](const DetectionArray & output) {
      return output.detections.empty() &&
             output.header.stamp.sec >= last_stream_frame.header.stamp.sec &&
             (output.header.stamp.sec > last_stream_frame.header.stamp.sec ||
             output.header.stamp.nanosec > last_stream_frame.header.stamp.nanosec);
    }).has_value());
  }

  auto scan_publisher = test_node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
  auto configured_client = test_node->create_client<ugv_bridge::srv::MissionStart>("/mission/start_configured");
  ASSERT_TRUE(configured_client->wait_for_service(std::chrono::seconds(3)));
  const auto scan_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (scan_publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < scan_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ASSERT_GT(scan_publisher->get_subscription_count(), 0U);
  ASSERT_TRUE(call_trigger(stop_client)->success);
  auto search_only = std::make_shared<ugv_bridge::srv::MissionStart::Request>();
  search_only->class_id = "0"; search_only->behavior = "search_only";

  // A pre-start frame must not authorize searching after start.
  const auto prestart_empty = make_frame("", 0.9);
  output_begin = output_count();
  detections_publisher->publish(prestart_empty);
  ASSERT_TRUE(wait_for_frame(output_begin, prestart_empty, false).has_value());
  state_begin = state_count(); command_begin = command_count();
  const auto no_stream_start = std::chrono::steady_clock::now();
  auto no_stream_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(no_stream_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(no_stream_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  ASSERT_TRUE(wait_state(state_begin, "LOST"));
  EXPECT_LT(std::chrono::steady_clock::now() - no_stream_start, std::chrono::milliseconds(800));
  EXPECT_FALSE(has_motion(command_begin));
  command_begin = command_count();
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  EXPECT_FALSE(has_motion(command_begin));
  command_begin = command_count();
  detections_publisher->publish(prestart_empty);
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  EXPECT_FALSE(has_motion(command_begin));
  const auto recovered_empty = make_frame("", 0.9);
  state_begin = state_count(); output_begin = output_count(); command_begin = command_count();
  detections_publisher->publish(recovered_empty);
  ASSERT_TRUE(wait_for_frame(output_begin, recovered_empty, false).has_value());
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());

  // Fresh empty frames keep searching alive; a subsequent stream loss stops it.
  auto publish_fresh_empty = [&]() {
      const auto empty = make_frame("", 0.9);
      const auto begin = output_count();
      detections_publisher->publish(empty);
      return wait_for_frame(begin, empty, false).has_value();
    };
  ASSERT_TRUE(call_trigger(stop_client)->success);
  ASSERT_TRUE(publish_fresh_empty());
  search_only->target_id = "session:search-lock";
  state_begin = state_count(); command_begin = command_count();
  auto empty_stream_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(empty_stream_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(empty_stream_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  for (int index = 0; index < 8; ++index) {
    detections_publisher->publish(make_frame("", 0.9));
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  EXPECT_TRUE(has_motion(command_begin));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
  }
  state_begin = state_count();
  ASSERT_TRUE(wait_state(state_begin, "LOST"));
  command_begin = command_count();
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());

  // Recovery to search keeps the requested full target ID.
  const auto other_search_target = make_frame("session:other", 0.99);
  state_begin = state_count(); output_begin = output_count();
  detections_publisher->publish(other_search_target);
  ASSERT_TRUE(wait_for_frame(output_begin, other_search_target, false).has_value());
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  const auto locked_search_target = make_frame("session:search-lock", 0.9);
  state_begin = state_count(); output_begin = output_count();
  detections_publisher->publish(locked_search_target);
  ASSERT_TRUE(wait_state(state_begin, "FOUND"));
  ASSERT_TRUE(wait_for_frame(output_begin, locked_search_target, true).has_value());

  // An invalid source header does not interrupt a healthy search stream.
  ASSERT_TRUE(call_trigger(stop_client)->success);
  ASSERT_TRUE(publish_fresh_empty());
  search_only->target_id.clear();
  state_begin = state_count();
  auto invalid_stream_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(invalid_stream_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(invalid_stream_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  ASSERT_TRUE(publish_fresh_empty());
  auto invalid_search_frame = make_frame("", 0.9);
  invalid_search_frame.header.stamp.sec = 0;
  invalid_search_frame.header.stamp.nanosec = 0;
  state_begin = state_count(); command_begin = command_count();
  detections_publisher->publish(invalid_search_frame);
  ASSERT_TRUE(publish_fresh_empty());
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
  }

  ASSERT_TRUE(call_trigger(stop_client)->success);
  ASSERT_TRUE(publish_fresh_empty());
  search_only->target_id.clear();
  auto search_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(search_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(search_future.get()->success);
  command_begin = command_count();
  detections_publisher->publish(make_frame("", 0.9));
  ASSERT_TRUE(wait_command(command_begin, [](const geometry_msgs::msg::Twist & command) {
    return command.linear.x == 0.0 && command.angular.z == 0.8;
  }).has_value());
  auto found_frame = make_frame("session:found", 0.9);
  found_frame.detections[0].bbox.center.position.x = 0.0;
  state_begin = state_count(); command_begin = command_count(); output_begin = output_count();
  detections_publisher->publish(found_frame);
  ASSERT_TRUE(wait_state(state_begin, "FOUND"));
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  auto found_output = wait_for_frame(output_begin, found_frame, true);
  ASSERT_TRUE(found_output.has_value());
  EXPECT_EQ(found_output->header.frame_id, found_frame.header.frame_id);
  EXPECT_EQ(found_output->detections[0].id, "session:found");
  command_begin = command_count(); output_begin = output_count();
  auto again = make_frame("session:found", 0.9);
  again.detections[0].bbox.center.position.x = 0.0;
  detections_publisher->publish(again);
  ASSERT_TRUE(wait_for_frame(output_begin, again, true).has_value());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_FALSE(has_motion(command_begin));
  state_begin = state_count();
  for (int index = 0; index < 4; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    const auto missing = make_frame("", 0.9);
    output_begin = output_count();
    detections_publisher->publish(missing);
    ASSERT_TRUE(wait_for_frame(output_begin, missing, false).has_value());
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
  }

  // Stopping during a four-frame hold clears the counter for the next task.
  state_begin = state_count(); command_begin = command_count();
  ASSERT_TRUE(call_trigger(stop_client)->success);
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  auto after_hold_restart_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(after_hold_restart_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(after_hold_restart_future.get()->success);
  const auto restart_target = make_frame("session:found", 0.9);
  state_begin = state_count(); output_begin = output_count();
  detections_publisher->publish(restart_target);
  ASSERT_TRUE(wait_for_frame(output_begin, restart_target, true).has_value());
  ASSERT_TRUE(wait_state(state_begin, "FOUND"));
  state_begin = state_count();
  for (int index = 0; index < 5; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    const auto missing = make_frame("", 0.9);
    output_begin = output_count();
    detections_publisher->publish(missing);
    ASSERT_TRUE(wait_for_frame(output_begin, missing, false).has_value());
    if (index < 4) {
      std::lock_guard<std::mutex> lock(mutex);
      EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "LOST"), 0);
    }
  }
  ASSERT_TRUE(wait_state(state_begin, "LOST"));
  for (const auto & behavior : {"search", "approach", "follow"}) {
    ASSERT_TRUE(call_trigger(stop_client)->success);
    ASSERT_TRUE(publish_fresh_empty());
    auto request = std::make_shared<ugv_bridge::srv::MissionStart::Request>();
    request->class_id = "0"; request->behavior = behavior;
    auto future = configured_client->async_send_request(request);
    ASSERT_EQ(future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    ASSERT_TRUE(future.get()->success);
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "laser"; scan.header.stamp = test_node->now(); scan.angle_min = 0.0; scan.angle_increment = 0.1;
    scan.range_min = 0.05; scan.range_max = 8.0; scan.ranges = {0.3f};
    scan_publisher->publish(scan);
    auto frame = make_frame("session:blocked", 0.9);
    state_begin = state_count(); command_begin = command_count(); detections_publisher->publish(frame);
    ASSERT_TRUE(wait_state(state_begin, "BLOCKED"));
    ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
    { std::lock_guard<std::mutex> lock(mutex);
      EXPECT_EQ(std::count(states.begin() + state_begin, states.end(), "ARRIVED"), 0); }
    scan.header.stamp = test_node->now(); scan.ranges = {1.0f}; scan_publisher->publish(scan);
    frame = make_frame("session:blocked", 0.9);
    frame.detections[0].bbox.center.position.x = 320.0;
    state_begin = state_count(); command_begin = command_count(); detections_publisher->publish(frame);
    ASSERT_TRUE(wait_state(state_begin, std::string(behavior) == "follow" ? "FOLLOWING" : "APPROACHING"));
    ASSERT_TRUE(wait_command(command_begin, [](const geometry_msgs::msg::Twist & command) {
      return std::abs(command.linear.x - 0.4) < 1e-9 && command.angular.z == 0.0;
    }).has_value());
  }

  ASSERT_TRUE(call_trigger(stop_client)->success);
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(has_motion(command_begin));
  { std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(states.back(), "STOPPED"); }

  // An unauthorized start has a bounded handshake. Frames captured before
  // the new authorization cannot start searching.
  set_authorization({"test-bridge", 2, false, false, "none"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  search_only->target_id.clear();
  state_begin = state_count(); command_begin = command_count();
  auto pending_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(pending_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(pending_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  const auto before_authorization = make_frame("", 0.9);
  output_begin = output_count();
  detections_publisher->publish(before_authorization);
  ASSERT_TRUE(wait_for_frame(output_begin, before_authorization, false).has_value());
  EXPECT_FALSE(has_motion(command_begin));
  set_authorization({"test-bridge", 3, true, false, "ros"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(has_motion(command_begin));
  ASSERT_TRUE(publish_fresh_empty());
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto last = std::find_if(authorized_commands.rbegin(), authorized_commands.rend(),
      [&](const auto & command) { return is_motion_command(command.twist); });
    ASSERT_NE(last, authorized_commands.rend());
    EXPECT_EQ(last->bridge_id, "test-bridge");
    EXPECT_EQ(last->authorization_epoch, 3U);
    EXPECT_EQ(last->header.frame_id, "base_link");
  }

  // Revocation during a one-frame hold permanently clears the task, including
  // if authorization is immediately issued again and valid frames continue.
  const auto revoke_target = make_frame("session:revoke", 0.9);
  state_begin = state_count(); output_begin = output_count();
  detections_publisher->publish(revoke_target);
  ASSERT_TRUE(wait_for_frame(output_begin, revoke_target, true).has_value());
  ASSERT_TRUE(wait_state(state_begin, "FOUND"));
  const auto revoke_miss = make_frame("", 0.0);
  output_begin = output_count();
  detections_publisher->publish(revoke_miss);
  ASSERT_TRUE(wait_for_frame(output_begin, revoke_miss, false).has_value());
  state_begin = state_count(); command_begin = command_count();
  set_authorization({"test-bridge", 4, false, false, "none"});
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  set_authorization({"test-bridge", 5, true, false, "ros"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(has_motion(command_begin));
  { std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(states.back(), "STOPPED"); }

  // An explicit revoke while waiting invalidates the pending task too.
  set_authorization({"test-bridge", 6, false, false, "none"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  state_begin = state_count(); command_begin = command_count();
  auto revoked_pending = configured_client->async_send_request(search_only);
  ASSERT_EQ(revoked_pending.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(revoked_pending.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  set_authorization({"test-bridge", 7, false, false, "none"});
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  set_authorization({"test-bridge", 8, true, false, "ros"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_TRUE(publish_fresh_empty());
  EXPECT_FALSE(has_motion(command_begin));

  // Waiting for the first authorization is bounded, even with healthy
  // diagnostic heartbeats and fresh video.
  set_authorization({"test-bridge", 9, false, false, "none"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  state_begin = state_count(); command_begin = command_count();
  auto expired_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(expired_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(expired_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  EXPECT_FALSE(has_motion(command_begin));
  set_authorization({"test-bridge", 10, true, false, "ros"});
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_FALSE(has_motion(command_begin));

  // E-stop during a one-frame hold clears the task; clearing the stop and
  // supplying fresh frames cannot resume it.
  state_begin = state_count();
  auto emergency_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(emergency_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(emergency_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());
  const auto emergency_target = make_frame("session:emergency", 0.9);
  state_begin = state_count(); output_begin = output_count();
  detections_publisher->publish(emergency_target);
  ASSERT_TRUE(wait_for_frame(output_begin, emergency_target, true).has_value());
  ASSERT_TRUE(wait_state(state_begin, "FOUND"));
  const auto emergency_miss = make_frame("", 0.0);
  output_begin = output_count();
  detections_publisher->publish(emergency_miss);
  ASSERT_TRUE(wait_for_frame(output_begin, emergency_miss, false).has_value());
  state_begin = state_count(); command_begin = command_count();
  emergency.data = true;
  emergency_publisher->publish(emergency);
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  ASSERT_TRUE(wait_command(command_begin, is_zero_command).has_value());
  emergency.data = false;
  emergency_publisher->publish(emergency);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_FALSE(has_motion(command_begin));

  // A task stops if the bridge identity changes, even when the replacement
  // bridge reports an authorized ROS source.
  state_begin = state_count();
  auto restart_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(restart_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(restart_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());
  state_begin = state_count();
  set_authorization({"new-bridge", 1, true, false, "ros"});
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(has_motion(command_begin));

  // A lost authorization heartbeat also clears the task; receiving the same
  // identity again is not permission to revive it.
  state_begin = state_count();
  auto heartbeat_future = configured_client->async_send_request(search_only);
  ASSERT_EQ(heartbeat_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  ASSERT_TRUE(heartbeat_future.get()->success);
  ASSERT_TRUE(wait_state(state_begin, "SEARCHING"));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  ASSERT_TRUE(wait_command(command_begin, is_motion_command).has_value());
  state_begin = state_count();
  emit_heartbeat = false;
  ASSERT_TRUE(wait_state(state_begin, "STOPPED"));
  emit_heartbeat = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  command_begin = command_count();
  ASSERT_TRUE(publish_fresh_empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(has_motion(command_begin));
}

}  // namespace car_mission
