// Exercise the production node's command handler in-process without DDS discovery.
#define main jetson_local_navigation_program_main
#include "../src/jetson_local_navigation.cpp"
#undef main

#include <iostream>
#include <stdexcept>
#include <thread>

#define REQUIRE(condition) do { if (!(condition)) throw std::runtime_error(#condition); } while (0)

class JetsonNavigationCommandHarness {
  using Command = ugv_bridge::srv::NavigationCommand;

  static std::uint64_t steady_ns() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  static Command::Request request(std::uint8_t action, std::uint32_t task,
                                  const std::string& bridge, std::uint64_t epoch) {
    Command::Request value;
    value.action = action; value.task_id = task; value.bridge_id = bridge;
    value.authorization_epoch = epoch; value.boot_id = 7; value.odom_domain = 1;
    value.mode = 3; value.flags = 2; value.goal_x_m = 2.0; value.goal_y_m = 0.0;
    value.max_speed_mm_s = 200; value.max_accel_mm_s2 = 500;
    value.tolerance_mm = 100; value.yaw_tolerance_mrad = 100; value.duration_s = 20.0;
    value.issued_steady_ns = steady_ns();
    value.start_deadline_steady_ns = value.issued_steady_ns + 2000000000ULL;
    return value;
  }

  static bool call(LocalNavigation& node, const Command::Request& value) {
    Command::Response response;
    node.command(value, response);
    return response.success;
  }

 public:
  static void run() {
    LocalNavigation node;
    std_msgs::msg::String navigation;
    auto state = Json{{"protocol", "velocity_only_v1"}, {"boot_id", 0u}, {"odom_domain", 1u}, {"now_us", 999000u},
                      {"pose_valid", true}, {"guard_stop", false}};
    navigation.data = state.dump();
    node.receive_navigation(navigation);
    node.receive_navigation(navigation);
    REQUIRE(!clock_valid && node.grid_epoch_base_ == 0);
    state["boot_id"] = 7u;
    state["now_us"] = 1000000u;
    navigation.data = state.dump();
    node.receive_navigation(navigation);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const auto initial_epoch = node.grid_epoch_base_;
    clock_anchor -= std::chrono::milliseconds(400);
    state["now_us"] = node.last_nav_source_us_ - 1;
    navigation.data = state.dump();
    for (int i = 0; i < 3; ++i) node.receive_navigation(navigation);
    REQUIRE(node.grid_epoch_base_ == initial_epoch && node.negative_clock_samples_ == 0);
    state["now_us"] = node.last_nav_source_us_ + 1000u;
    state["guard_stop"] = true;
    navigation.data = state.dump();
    for (int i = 0; i < 3; ++i) node.receive_navigation(navigation);
    REQUIRE(node.grid_epoch_base_ == initial_epoch && !node.guard_stop_ && node.negative_clock_samples_ == 1);
    state["now_us"] = device_now();
    navigation.data = state.dump();
    node.receive_navigation(navigation);
    REQUIRE(node.grid_epoch_base_ == initial_epoch && node.guard_stop_ && node.negative_clock_samples_ == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    state["now_us"] = device_now();
    state["guard_stop"] = false;
    navigation.data = state.dump();
    node.receive_navigation(navigation);
    REQUIRE(!node.guard_stop_);

    // A sustained, regularly advancing negative offset can be real clock drift.
    auto lagging_source = node.last_nav_source_us_ + 1000u;
    clock_anchor -= std::chrono::milliseconds(400);
    for (int i = 0; i < 3; ++i) {
      if (i != 0) {
        node.negative_clock_at_ -= std::chrono::milliseconds(100);
        clock_anchor -= std::chrono::milliseconds(100);
        lagging_source += 100000u;
      }
      state["now_us"] = lagging_source;
      navigation.data = state.dump();
      node.receive_navigation(navigation);
      if (i < 2) REQUIRE(node.grid_epoch_base_ == initial_epoch);
    }
    REQUIRE(node.grid_epoch_base_ == initial_epoch + 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const auto old_issue = steady_ns() - 1000000ULL;
    REQUIRE(call(node, request(3, 0, "new-bridge", 1)));
    auto old_start = request(1, 1, "old-bridge", 1);
    old_start.issued_steady_ns = old_issue;
    REQUIRE(!call(node, old_start));
    REQUIRE(!node.output_.autonomous);

    auto expired = request(1, 1, "new-bridge", 2);
    expired.start_deadline_steady_ns = steady_ns() - 1;
    REQUIRE(!call(node, expired));
    auto invalid_time = request(1, 1, "new-bridge", 2);
    invalid_time.issued_steady_ns = steady_ns() + 10000000000ULL;
    REQUIRE(!call(node, invalid_time));
    auto excessive_speed = request(1, 1, "new-bridge", 2);
    excessive_speed.max_speed_mm_s = 401;
    REQUIRE(!call(node, excessive_speed));
    REQUIRE(!node.output_.autonomous && node.output_.last_task_id == 0 &&
            !node.output_.vx_mm_s && !node.output_.vy_mm_s && !node.output_.wz_mrad_s);
    auto invalid_flags = request(1, 1, "new-bridge", 2);
    invalid_flags.flags = 4;
    REQUIRE(!call(node, invalid_flags));
    REQUIRE(!node.output_.autonomous && node.output_.last_task_id == 0 &&
            !node.output_.vx_mm_s && !node.output_.vy_mm_s && !node.output_.wz_mrad_s);
    auto maximum_speed = request(1, 1, "new-bridge", 2);
    maximum_speed.max_speed_mm_s = 400;
    REQUIRE(call(node, maximum_speed));
    REQUIRE(node.output_.autonomous);
    REQUIRE(node.bridge_id_ == "new-bridge" && node.authorization_epoch_ == 2);

    auto late_cancel = request(3, 0, "old-bridge", 1);
    late_cancel.issued_steady_ns = old_issue;
    REQUIRE(call(node, late_cancel));
    REQUIRE(node.output_.autonomous && node.bridge_id_ == "new-bridge");
    auto invalid = request(1, 2, "other-bridge", 3);
    invalid.max_speed_mm_s = 0;
    REQUIRE(!call(node, invalid));
    REQUIRE(node.output_.autonomous && node.bridge_id_ == "new-bridge" && node.authorization_epoch_ == 2);
    REQUIRE(!call(node, request(1, 1, "other-bridge", 3)));
    REQUIRE(node.output_.autonomous && node.bridge_id_ == "new-bridge" && node.authorization_epoch_ == 2);
    REQUIRE(!call(node, request(3, 1, "new-bridge", 2)));
    REQUIRE(node.output_.autonomous && node.output_.last_task_id == 1);

    // A valid newer owner replaces the old task only after the typed entry
    // accepts its transaction; a repeated transaction cannot take it back.
    REQUIRE(call(node, request(1, 2, "other-bridge", 3)));
    REQUIRE(node.output_.autonomous && node.output_.last_task_id == 2 &&
            node.bridge_id_ == "other-bridge" && node.authorization_epoch_ == 3);
    REQUIRE(!call(node, request(1, 2, "new-bridge", 2)));
    REQUIRE(node.output_.autonomous && node.bridge_id_ == "other-bridge");
    REQUIRE(call(node, request(3, 3, "other-bridge", 3)));
    REQUIRE(!node.output_.autonomous && node.output_.last_task_id == 3);

    state["protocol"] = "nav_grid_v1";
    state["now_us"] = node.last_nav_source_us_ + 1000u;
    navigation.data = state.dump();
    node.receive_navigation(navigation);
    REQUIRE(!node.local_execution_enabled_ && !clock_valid && !node.output_.autonomous);
    state["protocol"] = "velocity_only_v1";
    state["now_us"] = state["now_us"].get<std::uint32_t>() + 1000u;
    navigation.data = state.dump();
    node.receive_navigation(navigation);
    REQUIRE(node.local_execution_enabled_ && clock_valid && !node.output_.autonomous);

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    REQUIRE(call(node, request(1, 1, "new-bridge", 4)));
    diagnostic_msgs::msg::DiagnosticArray control;
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = "bx72_udp";
    diagnostic_msgs::msg::KeyValue source;
    source.key = "control_source";
    source.value = "pc";
    diagnostic.values.push_back(source);
    control.status.push_back(diagnostic);
    control.header.stamp = rclcpp::Time(node.latest_start_ns_ - 1);
    node.receive_control_status(control);
    REQUIRE(node.output_.autonomous); // An old external-owner report cannot cancel a new task.
    control.status[0].values[0].value = "none";
    control.header.stamp = rclcpp::Time(node.latest_start_ns_ + 1);
    node.receive_control_status(control);
    REQUIRE(node.output_.autonomous); // Monitor-only zero is not an owner takeover.
    control.status[0].values[0].value = "pc";
    control.header.stamp = rclcpp::Time(node.latest_start_ns_ + 2);
    node.receive_control_status(control);
    REQUIRE(!node.output_.autonomous);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    JetsonNavigationCommandHarness::run();
    rclcpp::shutdown();
    std::cout << "Jetson navigation command ordering passed\n";
    return 0;
  } catch (const std::exception& error) {
    rclcpp::shutdown();
    std::cerr << "Jetson navigation command ordering failed: " << error.what() << '\n';
    return 1;
  }
}
