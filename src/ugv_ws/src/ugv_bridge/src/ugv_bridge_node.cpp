#include "ugv_bridge/bx72_protocol.hpp"
#include "ugv_bridge/ros_navigation.hpp"
#include "ugv_bridge/video.hpp"
#include "ugv_bridge/route_overlay.hpp"
#include "ugv_bridge/authorized_twist.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <ugv_bridge/msg/authorized_twist.hpp>
#include <ugv_bridge/msg/bx72_imu_raw.hpp>
#include <ugv_bridge/msg/encoder_counts.hpp>
#include <ugv_bridge/srv/mission_start.hpp>
#include <ugv_bridge/srv/navigation_command.hpp>
#include <ugv_bridge/srv/global_navigation.hpp>
#include <ugv_bridge/msg/n10_packet.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rmw/rmw.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <exception>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <poll.h>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace bx72 {
namespace ros_navigation = ::ugv_bridge::ros_navigation;

namespace {

constexpr double kVideoTimeoutS = 0.25;

double age(double now, double previous)
{
  return std::round(std::min(999.0, now - previous) * 1000) / 1000;
}

std::string utf8_prefix(const std::string& text, std::size_t limit)
{
  auto end = std::min(limit, text.size());
  while (end < text.size() && end > 0 &&
         (static_cast<unsigned char>(text[end]) & 0xc0U) == 0x80U) --end;
  return text.substr(0, end);
}

bool supported_navigation_protocol(const std::string& protocol)
{
  return protocol == "velocity_only_v1" || protocol == "nav_full_v1" || protocol == "nav_polar_v2" || protocol == "nav_grid_v1";
}

}  // namespace

class Bridge {
  enum class ControlSource { None, Pc, Ros, Navigation };
  enum class NavigationBackend { Unknown, Fpga, Jetson };
  struct Pending
  {
    Bytes packet;
    double sent_at = 0;
    std::uint32_t sequence = 0;
    std::uint8_t type = 0;
    bool auto_enabled = false;
    Velocity velocity{};
    std::uint32_t pc_seq = 0;
    std::string pc_session;
    std::string source;
  };

  Args args_;
  double video_fps_;
  int jpeg_quality_;
  std::shared_ptr<rclcpp::Node> node_;
  Udp pc_;
  Udp fpga_;
  Udp video_;
  Udp telemetry_;
  sockaddr_in fpga_peer_{};
  std::optional<sockaddr_in> pc_filter_;
  PcLease lease_;
  PcObservers observers_;
  TelemetryTracker telemetry_tracker_;
  LocalGridAssembler local_grid_assembler_;
  ControlSource control_source_ = ControlSource::None;
  bool ros_control_enabled_ = false;
  bool ros_authorized_ = false;
  std::uint64_t ros_authorization_epoch_ = 0;
  std::optional<rclcpp::Time> ros_authorized_stamp_;
  bool emergency_stop_ = false;
  bool ros_command_received_ = false;
  Velocity ros_velocity_{};
  double ros_command_at_ = -1e100;
  double ros_authorized_at_ = -1e100;
  double cmd_vel_timeout_s_ = .2;
  std::string video_pixel_format_ = "yuyv";
  std::string web_control_session_, web_control_peer_;
  using TriggerClient = rclcpp::Client<std_srvs::srv::Trigger>;
  using MissionStartClient = rclcpp::Client<ugv_bridge::srv::MissionStart>;
  using NavigationClient = rclcpp::Client<ugv_bridge::srv::NavigationCommand>;
  using GlobalNavigationClient = rclcpp::Client<ugv_bridge::srv::GlobalNavigation>;
  MissionStartClient::SharedPtr mission_start_;
  NavigationClient::SharedPtr navigation_client_;
  GlobalNavigationClient::SharedPtr global_navigation_client_;
  struct GlobalRequest {
    GlobalNavigationClient::FutureAndRequestId future;
    Json request;
    sockaddr_in peer{};
    double sent_at = 0;
    std::uint64_t safety_epoch = 0;
  };
  std::vector<GlobalRequest> global_requests_;
  std::uint32_t global_state_sequence_ = 0;
  bool global_navigation_active_ = false;
  std::optional<GlobalNavigationClient::FutureAndRequestId> global_cancel_future_;
  std::optional<std::uint64_t> global_cancel_accepted_epoch_;
  std::uint64_t global_safety_epoch_ = 0, global_cancel_epoch_ = 0;
  bool global_cancel_needed_ = false;
  double global_cancel_sent_at_ = 0, global_cancel_retry_at_ = 0;
  rclcpp::Service<ugv_bridge::srv::NavigationCommand>::SharedPtr executor_service_;
  std::shared_ptr<rmw_request_id_t> executor_pending_header_;
  TriggerClient::SharedPtr mission_stop_;
  std::optional<MissionStartClient::FutureAndRequestId> start_future_;
  std::optional<TriggerClient::FutureAndRequestId> web_future_;
  std::optional<NavigationClient::FutureAndRequestId> navigation_future_;
  std::optional<NavigationClient::FutureAndRequestId> navigation_cancel_future_;
  Json navigation_deferred_request_;
  sockaddr_in navigation_deferred_peer_{};
  double navigation_deferred_at_ = -1e100;
  double navigation_cancel_retry_at_ = 0;
  std::uint64_t navigation_deferred_epoch_ = 0;
  std::uint64_t navigation_last_issued_ns_ = 0;
  bool start_cancelled_ = false;
  Json pending_web_request_;
  sockaddr_in pending_web_peer_{};
  double pending_web_at_ = 0;
  vision_msgs::msg::Detection2DArray::ConstSharedPtr web_detections_, web_selected_;
  double web_detections_at_ = -1e100, web_selected_at_ = -1e100;
  std::string mission_state_ = "UNKNOWN";
  std::uint32_t vision_sequence_ = 0;
  std::atomic<bool> stop_{false};
  std::exception_ptr video_error_;
  std::atomic<std::uint64_t> video_generation_{0};
  std::mutex frame_mutex_;
  std::shared_ptr<const Bytes> latest_frame_;
  std::vector<sockaddr_in> video_peers_;
  Json motor_ = nullptr;
  Json last_ack_ = nullptr;
  double motor_at_ = -1e100;
  double last_fusion_at_ = -1e100;
  double ack_at_ = -1e100;
  double last_telemetry_at_ = -1e100;
  double last_rtt_ms_ = -1;
  std::atomic<double> frame_at_{-1e100};
  std::optional<bool> actual_auto_;
  std::optional<bool> fpga_auto_;
  std::optional<bool> mode_target_;
  bool ready_ = false;
  bool zero_barrier_ack_ = false;
  bool mode_command_required_ = true;
  bool auto_acknowledged_ = false;
  std::uint64_t token_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  std::string bridge_id_;
  std::uint32_t fpga_seq_ = 0;
  std::uint32_t status_seq_ = 0;
  Velocity sent_velocity_{};
  std::deque<Pending> outstanding_;
  Json navigation_request_;
  sockaddr_in navigation_request_peer_{};
  std::string navigation_action_;
  std::uint32_t queued_navigation_task_id_ = 0;
  std::uint32_t navigation_task_id_ = 0;
  std::uint32_t navigation_start_transaction_id_ = 0;
  std::uint32_t navigation_cancelled_task_id_ = 0;
  std::uint32_t navigation_cancel_transaction_id_ = 0;
  std::uint32_t navigation_boot_id_ = 0;
  std::uint32_t navigation_odom_domain_ = 0;
  std::uint64_t navigation_session_token_ = 0;
  bool navigation_telemetry_rejected_ = false;
  bool navigation_bootstrap_sent_ = false;
  bool navigation_last_task_id_known_ = false;
  std::uint64_t navigation_now_us_ = 0;
  std::uint8_t navigation_mode_ = 0;
  std::uint8_t navigation_flags_ = 0;
  bool navigation_task_flags_known_ = false;
  std::string navigation_owner_session_, navigation_owner_peer_;
  std::uint8_t navigation_status_ = 0;
  double navigation_received_at_ = -1e100;
  bool navigation_state_known_ = false;
  Json navigation_context_ = nullptr;
  Json jetson_navigation_context_ = nullptr;
  double jetson_navigation_received_at_ = -1e100;
  double navigation_request_at_ = -1e100;
  bool navigation_sync_needed_ = true;
  std::uint32_t jetson_navigation_sequence_ = 0;
  std::uint64_t navigation_task_epoch_ = 0;
  std::int32_t jetson_goal_x_q16_ = 0, jetson_goal_y_q16_ = 0;
  std::uint32_t jetson_expiry_us_ = 0;
  bool navigation_transfer_active_ = false;
  NavigationBackend navigation_backend_ = NavigationBackend::Unknown;
  bool navigation_owner_is_ros_ = false;
  std::deque<Bytes> navigation_packet_queue_;
  std::size_t navigation_packets_acked_ = 0;
  double navigation_transfer_started_at_ = -1e100;
  double next_navigation_probe_at_ = 0;
  bool navigation_task_active_ = false;
  bool navigation_stop_pending_ = false;
  double navigation_stop_zero_ack_at_ = -1e100;
  double navigation_stop_ack_at_ = -1e100;
  double next_navigation_stop_at_ = 0;
  std::uint32_t next_navigation_task_id_ = 0;
  bool cpu_health_known_ = false;
  bool cpu_ready_ = false;
  bool cpu_fault_ = false;
  double cpu_health_at_ = -1e100;
  std::uint64_t ack_count_ = 0;
  std::uint64_t timeouts_ = 0;
  std::uint64_t decode_errors_ = 0;
  std::uint64_t source_errors_ = 0;
  std::atomic<std::uint64_t> jpeg_count_{0};
  std::atomic<std::uint64_t> send_errors_{0};
  std::atomic<std::uint64_t> frame_count_{0};
  std::atomic<std::uint64_t> video_dropped_{0};
  std::atomic<std::uint64_t> video_gaps_{0};
  std::atomic<std::uint64_t> ros_image_convert_attempts_{0};
  std::atomic<std::uint64_t> ros_image_publish_attempts_{0};
  std::atomic<std::uint64_t> ros_image_published_{0};
  std::atomic<std::uint64_t> ros_image_skipped_expired_{0};
  std::atomic<std::uint64_t> ros_image_skipped_generation_{0};
  std::atomic<double> ros_image_published_at_{-1e100};
  std::map<std::string, std::uint64_t> received_;
  std::map<std::string, double> last_host_;
  std::optional<std::uint32_t> motor_stamp_;
  std::string imu_frame_id_;
  std::string n10_frame_id_;
  std::string camera_frame_id_;
  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  double lidar_x_m_ = ros_navigation::kDefaultN10OffsetXMetres;
  double lidar_y_m_ = 0.0;
  double lidar_yaw_rad_ = 0.0;
  double imu_x_m_ = 0.0;
  double imu_y_m_ = 0.0;
  double imu_z_m_ = 0.0;
  double imu_roll_rad_ = 3.14159265358979323846;
  double imu_pitch_rad_ = 0.0;
  double imu_yaw_rad_ = 0.0;
  double scan_range_min_m_ = 0.01;
  double scan_range_max_m_ = 65.534;
  int navigation_default_speed_mm_s_ = 400;
  int navigation_default_accel_mm_s2_ = 200;
  int navigation_default_tolerance_mm_ = 100;
  int navigation_default_yaw_tolerance_mrad_ = 100;
  double navigation_default_duration_s_ = 120;
  ros_navigation::FpgaRosTimeMapper time_mapper_;
  ros_navigation::N10ScanAssembler scan_assembler_;
  ros_navigation::WheelOdometryIntegrator wheel_odometry_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> n10_static_tf_broadcaster_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;
  RouteOverlay route_overlay_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> raw_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<ugv_bridge::msg::Bx72ImuRaw>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<ugv_bridge::msg::N10Packet>::SharedPtr n10_pub_;
  rclcpp::Publisher<ugv_bridge::msg::EncoderCounts>::SharedPtr encoder_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_, fusion_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_state_pub_, local_plan_pub_,
      executor_state_pub_, executor_grid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bridge_safety_state_pub_;
  std::uint64_t bridge_safety_sequence_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sensor_status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr bidirectional_status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<ugv_bridge::msg::AuthorizedTwist>::SharedPtr mission_cmd_vel_sub_;
  rclcpp::Subscription<ugv_bridge::msg::AuthorizedTwist>::SharedPtr navigation_cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr jetson_navigation_sub_, jetson_grid_sub_, jetson_plan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr global_navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ros_control_service_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr web_detections_sub_, web_selected_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;

  static const char* source_name(ControlSource source) {
    switch (source) {
      case ControlSource::Pc: return "pc";
      case ControlSource::Ros: return "ros";
      case ControlSource::Navigation: return "navigation";
      default: return "none";
    }
  }

  const char* request_source() const {
    return emergency_stop_ ? "emergency_stop" : source_name(control_source_);
  }

  void clear_ros_command() {
    web_control_session_.clear();
    web_control_peer_.clear();
    ros_command_received_ = false;
    ros_velocity_ = {};
    ros_command_at_ = -1e100;
    ros_authorized_at_ = -1e100;
  }

  void reset_control_path() {
    outstanding_.clear();
    last_ack_ = nullptr;
    ack_at_ = -1e100;
    ready_ = false;
    zero_barrier_ack_ = false;
    mode_command_required_ = true;
    auto_acknowledged_ = false;
    mode_target_.reset();
  }

  void disarm_pc_lease() {
    lease_.disarm();
    lease_.last = -1e100;
  }

  void refresh_video_peers() {
    std::vector<sockaddr_in> peers;
    peers.reserve(observers_.entries().size());
    for (const auto& observer : observers_.entries()) peers.push_back(observer.endpoint);
    std::lock_guard<std::mutex> lock(frame_mutex_);
    video_peers_ = std::move(peers);
  }

  PcInputResult accept_pc_frame(const PcControlFrame& frame, const sockaddr_in& peer,
                                double now, bool explicit_command = false, bool safety_only = false) {
    bool changed = false;
    const auto result = accept_pc_input(lease_, observers_, frame, peer, now,
                                        explicit_command, safety_only, &changed);
    if (changed) refresh_video_peers();
    return result;
  }

  bool accept_pc_command(const Json& monitor, const sockaddr_in& peer, double now,
                         bool safety_only = false) {
    const auto frame = parse_pc_control(monitor);
    // A request that will be rejected because another task is active must not
    // first replace that task's control lease.
    if (frame && !safety_only && !lease_.session.empty() &&
        (frame->session != lease_.session || peer_key(peer) != lease_.peer) &&
        (navigation_task_active_ || navigation_transfer_active_ || navigation_deferred_request_.is_object() ||
         global_navigation_active_ || start_future_ ||
         (ros_authorized_ && !web_control_session_.empty()) || !global_requests_.empty()))
      return false;
    if (!frame) return false;
    const auto previous_session = lease_.session;
    const auto previous_peer = lease_.peer;
    const auto accepted = accept_pc_frame(*frame, peer, now, true, safety_only);
    if (accepted == PcInputResult::Owner && !previous_session.empty() &&
        (previous_session != lease_.session || previous_peer != lease_.peer)) send_pc_status(now);
    return accepted != PcInputResult::Rejected;
  }

  void invalidate_pc_claims(double now) {
    observers_.renew_claim_nonces();
    send_pc_status(now);
  }

  void publish_bridge_safety_state() {
    if (!bridge_safety_state_pub_) return;
    std_msgs::msg::String message;
    message.data = Json{{"bridge_id", bridge_id_}, {"sequence", ++bridge_safety_sequence_},
                        {"emergency_stop", emergency_stop_},
                        {"global_cancel_pending", global_cancel_needed_},
                        {"safety_epoch", global_safety_epoch_},
                        {"control_busy", (control_source_ == ControlSource::Pc && lease_.armed) ||
                                         (control_source_ == ControlSource::Ros && ros_authorized_)}}.dump();
    bridge_safety_state_pub_->publish(message);
  }

  void revoke_ros_authorization(bool claims_already_revoked = false) {
    if (control_source_ == ControlSource::Navigation) {
      begin_navigation_stop(monotonic(), claims_already_revoked);
      return;
    }
    if (start_future_) start_cancelled_ = true;
    ++ros_authorization_epoch_;
    if (!ros_authorized_ && control_source_ != ControlSource::Ros) {
      publish_bridge_safety_state();
      if (!claims_already_revoked) invalidate_pc_claims(monotonic());
      else send_pc_status(monotonic());
      if (bidirectional_status_pub_) publish_bidirectional_status(monotonic());
      return;
    }
    ros_authorized_ = false;
    ros_authorized_stamp_.reset();
    clear_ros_command();
    control_source_ = ControlSource::None;
    reset_control_path();
    publish_bridge_safety_state();
    if (!claims_already_revoked) invalidate_pc_claims(monotonic());
    else send_pc_status(monotonic());
    if (bidirectional_status_pub_) publish_bidirectional_status(monotonic());
  }

  void revoke_all_controls() {
    if (start_future_) start_cancelled_ = true;
    ++ros_authorization_epoch_;
    ros_authorized_ = false;
    ros_authorized_stamp_.reset();
    clear_ros_command();
    disarm_pc_lease();
    control_source_ = ControlSource::None;
    reset_control_path();
    publish_bridge_safety_state();
    if (bidirectional_status_pub_) publish_bidirectional_status(monotonic());
  }

  void accept_ros_velocity(const geometry_msgs::msg::Twist& message) {
    const std::array<double, 6> axes{message.linear.x, message.linear.y, message.linear.z,
                                     message.angular.x, message.angular.y, message.angular.z};
    if (!std::all_of(axes.begin(), axes.end(), [](double value) { return std::isfinite(value); }) ||
        message.linear.z != 0 || message.angular.x != 0 || message.angular.y != 0) {
      if (control_source_ == ControlSource::Navigation) begin_navigation_stop(monotonic());
      else revoke_ros_authorization();
      return;
    }
    ros_velocity_ = {
        static_cast<int>(std::lround(std::clamp(message.linear.x * 1000.0, -400.0, 400.0))),
        static_cast<int>(std::lround(std::clamp(message.linear.y * 1000.0, -400.0, 400.0))),
        static_cast<int>(std::lround(std::clamp(message.angular.z * 1000.0, -800.0, 800.0)))};
    ros_command_at_ = monotonic();
    ros_command_received_ = true;
  }

  void receive_ros_command(const geometry_msgs::msg::Twist& message) {
    if (emergency_stop_ || !ros_control_enabled_ || !ros_authorized_ ||
        control_source_ != ControlSource::Ros) return;
    accept_ros_velocity(message);
  }

  void receive_mission_command(const ugv_bridge::msg::AuthorizedTwist& message) {
    if (!ros_authorized_ || control_source_ != ControlSource::Ros || emergency_stop_ ||
        !ros_authorized_stamp_) return;
    const auto now = node_->now();
    if (!valid_mission_command(message, bridge_id_, ros_authorization_epoch_,
                               ros_authorized_stamp_->nanoseconds(), now.nanoseconds(),
                               cmd_vel_timeout_s_)) return;
    receive_ros_command(message.twist);
  }

  void receive_navigation_command(const ugv_bridge::msg::AuthorizedTwist& message) {
    if (navigation_backend_ != NavigationBackend::Jetson ||
        !ros_authorized_ || control_source_ != ControlSource::Navigation || emergency_stop_ ||
        !navigation_task_active_ || navigation_stop_pending_ || !ros_authorized_stamp_) return;
    if (!valid_mission_command(message, bridge_id_, ros_authorization_epoch_,
                               ros_authorized_stamp_->nanoseconds(), node_->now().nanoseconds(),
                               cmd_vel_timeout_s_)) return;
    accept_ros_velocity(message.twist);
  }

  void set_ros_authorization(const std_srvs::srv::SetBool::Request::SharedPtr request,
                             std_srvs::srv::SetBool::Response::SharedPtr response,
                             bool claims_already_revoked = false) {
    if (!request->data) {
      revoke_ros_authorization(claims_already_revoked);
      response->success = true;
      response->message = "ROS control authorization revoked";
      return;
    }
    const auto now = monotonic();
    if (!ros_control_enabled_) {
      response->success = false;
      response->message = "ROS control is disabled by configuration";
    } else if (emergency_stop_) {
      response->success = false;
      response->message = "emergency stop is active";
    } else if (global_navigation_active_ || global_cancel_needed_ || !global_requests_.empty()) {
      if (!global_cancel_needed_) {
        request_global_cancel(now);
        begin_navigation_stop(now);
      }
      response->success = false;
      response->message = "global navigation is stopping; retry authorization after confirmation";
    } else if (navigation_transfer_active_ || navigation_task_active_) {
      response->success = false;
      response->message = "stop the FPGA local task before authorizing ROS visual control";
    } else if (!ready_ || now - ack_at_ >= .2) {
      response->success = false;
      response->message = "a fresh matching FPGA ACK is required";
    } else {
      disarm_pc_lease();
      clear_ros_command();
      if (!ros_authorized_ || control_source_ != ControlSource::Ros) {
        ++ros_authorization_epoch_;
        ++global_safety_epoch_;
      }
      ros_authorized_ = true;
      ros_authorized_at_ = now;
      ros_authorized_stamp_ = node_->now();
      control_source_ = ControlSource::Ros;
      reset_control_path();
      publish_bridge_safety_state();
      if (bidirectional_status_pub_) publish_bidirectional_status(now);
      response->success = true;
      response->message = "ROS control authorized; publish a new cmd_vel command";
    }
  }

  void set_emergency_stop(bool stop, bool claims_already_revoked = false) {
    if (stop == emergency_stop_) return;
    if (stop) {
      emergency_stop_ = true;
      request_global_cancel(monotonic());
      begin_navigation_stop(monotonic(), claims_already_revoked);
      return;
    }
    emergency_stop_ = false;
    publish_bridge_safety_state();
  }

  void send_web_result(const Json& request, const sockaddr_in& peer, bool success,
                       const std::string& message, bool unknown = false) {
    if (request.value("__executor_ros", false)) {
      reply_executor(success, message);
      return;
    }
    pc_.send(Json{{"v", 1}, {"type", "mission_result"}, {"session", request.at("session")},
        {"request_id", request.at("request_id")}, {"action", request.at("action")},
        {"success", success}, {"unknown", unknown}, {"message", utf8_prefix(message, 240)}}, peer);
  }

  void reply_executor(bool success, const std::string& message) {
    if (!executor_pending_header_) return;
    auto header = std::move(executor_pending_header_);
    ugv_bridge::srv::NavigationCommand::Response response;
    response.success = success;
    response.message = message;
    executor_service_->send_response(*header, response);
  }

  std::uint64_t next_navigation_issue_stamp() {
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    navigation_last_issued_ns_ = std::max(static_cast<std::uint64_t>(now_ns),
                                          navigation_last_issued_ns_ + 1);
    return navigation_last_issued_ns_;
  }

  void discard_deferred_navigation(const std::string& reason) {
    if (!navigation_deferred_request_.is_object()) return;
    send_web_result(navigation_deferred_request_, navigation_deferred_peer_, false, reason);
    navigation_deferred_request_ = nullptr;
  }

  void defer_navigation(const Json& request, const sockaddr_in& peer, double transaction_at) {
    navigation_deferred_request_ = request;
    navigation_deferred_peer_ = peer;
    navigation_deferred_at_ = transaction_at;
    navigation_deferred_epoch_ = ros_authorization_epoch_;
    // Make the existing per-observer sequence watermark visible promptly so
    // the requester can distinguish a received pending request from no reply.
    send_pc_status(monotonic());
  }

  bool manual_navigation_ready(double now) const {
    return !emergency_stop_ && ready_ && now - ack_at_ <= .2 && fpga_auto_ && !*fpga_auto_ &&
           mode_target_ && !*mode_target_ && !mode_command_required_;
  }

  bool navigation_status_fresh(double now) const {
    return navigation_state_known_ && navigation_received_at_ > -1e50 &&
        now >= navigation_received_at_ && now - navigation_received_at_ <= .5;
  }

  bool navigation_clock_fresh(double now) const {
    return navigation_status_fresh(now) && navigation_now_us_ != 0 &&
        navigation_boot_id_ != 0 && navigation_odom_domain_ != 0 &&
        navigation_session_token_ != 0 &&
        navigation_last_task_id_known_ && time_mapper_.valid();
  }

  static std::int32_t q16_from_number(const Json& object, const char* key, double fallback) {
    double value = fallback;
    if (object.contains(key)) {
      const auto& field = object.at(key);
      if (!field.is_number() || field.is_boolean())
        throw std::invalid_argument(std::string(key) + " must be numeric");
      value = field.get<double>();
    }
    if (!std::isfinite(value)) throw std::invalid_argument(std::string(key) + " must be finite");
    if (value < static_cast<double>(std::numeric_limits<std::int32_t>::min()) / 65536.0 ||
        value > static_cast<double>(std::numeric_limits<std::int32_t>::max()) / 65536.0)
      throw std::invalid_argument(std::string(key) + " is outside Q16 range");
    const auto scaled = std::llround(value * 65536.0);
    if (scaled < std::numeric_limits<std::int32_t>::min() ||
        scaled > std::numeric_limits<std::int32_t>::max())
      throw std::invalid_argument(std::string(key) + " is outside Q16 range");
    return static_cast<std::int32_t>(scaled);
  }

  static int bounded_integer(const Json& object, const char* key, int fallback, int maximum) {
    if (!object.contains(key)) return fallback;
    const auto& field = object.at(key);
    if (!field.is_number_integer() || field.is_boolean())
      throw std::invalid_argument(std::string(key) + " must be an integer");
    const auto value = field.get<std::int64_t>();
    if (value < 1 || value > maximum)
      throw std::invalid_argument(std::string(key) + " is outside integer range");
    return static_cast<int>(value);
  }

  static std::uint32_t navigation_task_id_from_request(const Json& request) {
    if (!request.contains("task_id"))
      throw std::invalid_argument("task_id is required from the UDP service watermark");
    const auto& field = request.at("task_id");
    if (!field.is_number_integer() || field.is_boolean())
      throw std::invalid_argument("task_id must be an unsigned 32-bit integer");
    std::uint64_t value = 0;
    if (field.is_number_unsigned()) {
      value = field.get<std::uint64_t>();
    } else {
      const auto signed_value = field.get<std::int64_t>();
      if (signed_value < 0) throw std::invalid_argument("task_id must be positive");
      value = static_cast<std::uint64_t>(signed_value);
    }
    if (value == 0 || value > std::numeric_limits<std::uint32_t>::max())
      throw std::invalid_argument("task_id is outside the positive 32-bit range");
    return static_cast<std::uint32_t>(value);
  }

  void navigation_result(bool success, const std::string& message, bool unknown = false) {
    if (!navigation_request_.is_object()) return;
    if (navigation_request_.value("__executor_ros", false)) {
      navigation_request_ = nullptr;
      reply_executor(success, message);
      return;
    }
    Json result{{"v", 1}, {"type", "mission_result"},
        {"session", navigation_request_.at("session")},
        {"request_id", navigation_request_.at("request_id")},
        {"action", navigation_request_.at("action")},
        {"success", success}, {"unknown", unknown},
        {"message", utf8_prefix(message, 240)},
        {"transaction_task_id", queued_navigation_task_id_}};
    if (navigation_action_ == "local_start")
      result["task_id"] = queued_navigation_task_id_;
    const bool reported_task_active = navigation_task_active_ ||
        (navigation_stop_pending_ && navigation_task_id_ != 0);
    if (reported_task_active)
      result["active_task_id"] = navigation_task_id_;
    if (navigation_action_ != "local_start")
      result["cancelled_task_id"] = navigation_cancelled_task_id_;
    pc_.send(result, navigation_request_peer_);
    navigation_request_ = nullptr;
  }

  bool queue_navigation_task(const Json& request, const sockaddr_in& peer, double now,
                             std::optional<double> first_received_at = std::nullopt) {
    const auto action = request.at("action").get<std::string>();
    const auto transaction_at = first_received_at.value_or(now);
    if (action == "local_start" && global_cancel_needed_) {
      send_web_result(request, peer, false, "全局取消尚未确认，暂不能启动局部段");
      return false;
    }
    if (action == "local_start" && now - transaction_at >= 2.0) {
      send_web_result(request, peer, false, "导航启动请求已过期");
      begin_navigation_stop(now);
      return false;
    }
    if (navigation_transfer_active_ || navigation_deferred_request_.is_object()) {
      send_web_result(request, peer, false, "另一项导航请求仍在处理");
      return false;
    }
    if (navigation_backend_ == NavigationBackend::Unknown) {
      send_web_result(request, peer, false, "FPGA 导航协议尚未确定");
      return false;
    }
    const bool service_ready = navigation_client_ && navigation_client_->service_is_ready();
    if (navigation_backend_ == NavigationBackend::Jetson && action == "local_start" &&
        (navigation_sync_needed_ || !service_ready)) {
      navigation_sync_needed_ = true;
      // Keep the already accepted web request until every prior force-cancel
      // has completed; never rely on DDS service request delivery order.
      defer_navigation(request, peer, transaction_at);
      if (service_ready && !navigation_cancel_future_ && now >= navigation_cancel_retry_at_)
        cancel_jetson_navigation();
      return true;
    }
    if (navigation_backend_ == NavigationBackend::Jetson && !service_ready) {
      send_web_result(request, peer, false, "Jetson 导航服务尚未就绪");
      return false;
    }
    if (!navigation_clock_fresh(now)) {
      send_web_result(request, peer, false, "缺少新鲜的32位导航时钟、boot_id、session_token或odom_domain");
      return false;
    }

    NavigationTask task;
    const auto nav_now32=static_cast<std::uint32_t>(navigation_now_us_);
    task.action = action == "local_start" ? 1 : (action == "local_stop" ? 2 : 3);
    task.odom_domain = navigation_odom_domain_;
    task.boot_id = navigation_boot_id_;
    task.sent_at_us = nav_now32;
    task.expiry_us = static_cast<std::uint32_t>(nav_now32 + 1'000'000U);
    task.mode = navigation_mode_ == 3 ? 3 : 2;
    task.max_speed_mm_s = static_cast<std::uint16_t>(navigation_default_speed_mm_s_);
    task.max_accel_mm_s2 = static_cast<std::uint16_t>(navigation_default_accel_mm_s2_);
    task.tolerance_mm = static_cast<std::uint16_t>(navigation_default_tolerance_mm_);
    task.yaw_tolerance_mrad = static_cast<std::uint16_t>(navigation_default_yaw_tolerance_mrad_);

    if (action == "local_start") {
      if (navigation_stop_pending_) {
        send_web_result(request, peer, false, "FPGA 停车尚未确认，不能开始局部任务");
        return false;
      }
      if (navigation_task_active_ || navigation_status_active(navigation_status_)) {
        send_web_result(request, peer, false, "已有局部导航任务正在运行；请先停止或取消");
        return false;
      }
      if (emergency_stop_) {
        send_web_result(request, peer, false,
            "急停生效，不能开始局部任务");
        return false;
      }
      if (!request.contains("mode") || !request.at("mode").is_string()) {
        send_web_result(request, peer, false, "局部任务必须指定 cooperative 或 independent 模式");
        return false;
      }
      const auto mode = request.at("mode").get<std::string>();
      if (mode == "cooperative") task.mode = 2;
      else if (mode == "independent") task.mode = 3;
      else {
        send_web_result(request, peer, false, "不支持的局部任务模式");
        return false;
      }
      if (request.contains("allow_disconnect") && !request.at("allow_disconnect").is_boolean()) {
        send_web_result(request, peer, false, "allow_disconnect 必须是布尔值");
        return false;
      }
      const bool allow_disconnect = request.value("allow_disconnect", false);
      task.flags = task.mode == 2 && allow_disconnect ? 1 : 0;
      if (request.contains("position_only") && !request.at("position_only").is_boolean()) {
        send_web_result(request, peer, false, "position_only 必须是布尔值");
        return false;
      }
      if (request.value("position_only", false)) task.flags |= 2;
      task.goal_x_q16 = q16_from_number(request, "goal_x_m", 0.0);
      task.goal_y_q16 = q16_from_number(request, "goal_y_m", 0.0);
      task.hold_yaw_q16 = q16_from_number(request, "hold_yaw_rad", 0.0);
      task.final_yaw_q16 = q16_from_number(request, "final_yaw_rad", 0.0);
      if (task.hold_yaw_q16 < -205887 || task.hold_yaw_q16 > 205887 ||
          task.final_yaw_q16 < -205887 || task.final_yaw_q16 > 205887)
        throw std::invalid_argument("yaw is outside navigation bounds");
      task.max_speed_mm_s = static_cast<std::uint16_t>(bounded_integer(
          request, "max_speed_mm_s", navigation_default_speed_mm_s_, 400));
      task.max_accel_mm_s2 = static_cast<std::uint16_t>(bounded_integer(
          request, "max_accel_mm_s2", navigation_default_accel_mm_s2_, 1000));
      task.tolerance_mm = static_cast<std::uint16_t>(bounded_integer(
          request, "tolerance_mm", navigation_default_tolerance_mm_, 500));
      task.yaw_tolerance_mrad = static_cast<std::uint16_t>(bounded_integer(
          request, "yaw_tolerance_mrad", navigation_default_yaw_tolerance_mrad_, 1000));
      double duration_s = navigation_default_duration_s_;
      if (request.contains("duration_s")) {
        const auto& field = request.at("duration_s");
        if (!field.is_number() || field.is_boolean())
          throw std::invalid_argument("duration_s must be numeric");
        duration_s = field.get<double>();
      }
      if (!std::isfinite(duration_s) || duration_s <= 0 || duration_s > 120)
        throw std::invalid_argument("duration_s must be within (0, 120]");
      const auto duration_us = static_cast<std::uint64_t>(std::llround(duration_s * 1'000'000.0));
      if (duration_us == 0) throw std::invalid_argument("duration_s rounds to zero microseconds");
      task.expiry_us = static_cast<std::uint32_t>(
          nav_now32 + static_cast<std::uint32_t>(duration_us));
      task.horizon_ms = 0;
    }

    {
      // 协调器基于执行器最近水位分配事务号，网桥统一校验。
      std::optional<std::uint32_t> previous_task_id;
      if (next_navigation_task_id_ != 0) previous_task_id = next_navigation_task_id_;
      else if (navigation_task_id_ != 0) previous_task_id = navigation_task_id_;
      task.task_id = navigation_task_id_from_request(request);
      const auto task_error = navigation_task_error(task, navigation_now_us_,
          navigation_odom_domain_, navigation_boot_id_, previous_task_id);
      if (task_error != 0) {
        send_web_result(request, peer, false,
            "局部任务被源时标/里程域/递增编号校验拒绝，原因码 " + std::to_string(task_error));
        return false;
      }
      if (task.action == 1 && !manual_navigation_ready(now)) {
        // A freshly accepted zero-speed ACK precedes the manual-mode ACK.
        // Preserve the request and its original deadline while that fence completes.
        defer_navigation(request, peer, transaction_at);
        return true;
      }
      if (task.action == 1) {
        revoke_all_controls();
        if (mission_stop_->service_is_ready())
          mission_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        navigation_owner_is_ros_ = request.value("__executor_ros", false);
        if (!navigation_owner_is_ros_) {
          lease_.last = now;
          navigation_owner_session_ = lease_.session;
          navigation_owner_peer_ = lease_.peer;
        }
        if (navigation_backend_ == NavigationBackend::Jetson) {
          ros_authorized_ = true;
          ros_authorized_at_ = now;
          ros_authorized_stamp_ = node_->now();
          navigation_task_epoch_ = ros_authorization_epoch_;
          control_source_ = ControlSource::Navigation;
          navigation_task_active_ = true;
          navigation_task_id_ = task.task_id;
          navigation_status_ = 1;
        }
        jetson_goal_x_q16_ = task.goal_x_q16;
        jetson_goal_y_q16_ = task.goal_y_q16;
        jetson_expiry_us_ = static_cast<std::uint32_t>(task.expiry_us);
        jetson_navigation_context_ = nullptr;
      }
      next_navigation_task_id_ = task.task_id;
      navigation_flags_ = task.flags;
      navigation_task_flags_known_ = true;
    }
    navigation_request_ = request;
    navigation_request_peer_ = peer;
    navigation_action_ = action;
    queued_navigation_task_id_ = task.task_id;
    if (task.action != 1) {
      navigation_cancel_transaction_id_ = task.task_id;
    }
    navigation_mode_ = task.mode;
    navigation_flags_ = task.flags;
    navigation_request_at_ = transaction_at;
    navigation_transfer_active_ = true;
    if (navigation_backend_ == NavigationBackend::Fpga) {
      const auto packets = fpga_task_packets(fpga_seq_ + 1U, navigation_session_token_, task);
      navigation_packet_queue_.assign(packets.begin(), packets.end());
      navigation_packets_acked_ = 0;
      navigation_transfer_started_at_ = -1e100;
      return true;
    }
    auto command = std::make_shared<ugv_bridge::srv::NavigationCommand::Request>();
    command->action = task.action;
    command->mode = task.mode;
    command->flags = task.flags;
    command->task_id = task.task_id;
    command->boot_id = task.boot_id;
    command->odom_domain = task.odom_domain;
    command->goal_x_m = static_cast<double>(task.goal_x_q16) / 65536.0;
    command->goal_y_m = static_cast<double>(task.goal_y_q16) / 65536.0;
    command->hold_yaw_rad = static_cast<double>(task.hold_yaw_q16) / 65536.0;
    command->final_yaw_rad = static_cast<double>(task.final_yaw_q16) / 65536.0;
    command->max_speed_mm_s = task.max_speed_mm_s;
    command->max_accel_mm_s2 = task.max_accel_mm_s2;
    command->tolerance_mm = task.tolerance_mm;
    command->yaw_tolerance_mrad = task.yaw_tolerance_mrad;
    command->duration_s = static_cast<std::uint32_t>(task.expiry_us - task.sent_at_us) / 1000000.0;
    command->bridge_id = bridge_id_;
    command->authorization_epoch = ros_authorization_epoch_;
    command->issued_steady_ns = next_navigation_issue_stamp();
    if (task.action == 1)
      command->start_deadline_steady_ns = static_cast<std::uint64_t>((transaction_at + 2.0) * 1e9);
    navigation_future_.emplace(navigation_client_->async_send_request(command));
    return true;
  }

  void abort_navigation_transfer(bool send_neutral) {
    const bool had_transaction = navigation_transfer_active_;
    if (navigation_future_) {
      navigation_client_->remove_pending_request(navigation_future_->request_id);
      navigation_future_.reset();
    }
    outstanding_.clear();
    navigation_packet_queue_.clear();
    navigation_start_transaction_id_ = 0;
    navigation_transfer_active_ = false;
    navigation_task_active_ = false;
    navigation_task_flags_known_ = false;
    navigation_owner_session_.clear();
    navigation_owner_peer_.clear();
    if (had_transaction)
      navigation_result(false, "导航请求被停车或人工接管中断；任务结果未知", true);
    if (send_neutral) send_request(3, {}, false, monotonic(), request_source());
  }

  void cancel_jetson_navigation() {
    navigation_sync_needed_ = true;
    // Coalesce force-cancels while the old one is outstanding. No new start
    // may pass it, so a single successful cancel covers all intervening stops.
    if (navigation_cancel_future_ || !navigation_client_ ||
        !navigation_client_->service_is_ready()) return;
    auto command = std::make_shared<ugv_bridge::srv::NavigationCommand::Request>();
    command->action = 3;
    command->task_id = 0;
    command->boot_id = navigation_boot_id_;
    command->odom_domain = navigation_odom_domain_;
    command->bridge_id = bridge_id_;
    command->authorization_epoch = ros_authorization_epoch_;
    command->issued_steady_ns = next_navigation_issue_stamp();
    navigation_cancel_future_.emplace(navigation_client_->async_send_request(command));
  }

  void poll_navigation_sync(double now) {
    if (navigation_deferred_request_.is_object() && now - navigation_deferred_at_ >= 2.0) {
      discard_deferred_navigation("等待导航服务、手动模式确认或旧任务取消超时，启动未发送");
      begin_navigation_stop(now);
    }
    if (navigation_cancel_future_ &&
        navigation_cancel_future_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      bool success = false;
      try { success = navigation_cancel_future_->future.get()->success; }
      catch (const std::exception&) {}
      navigation_cancel_future_.reset();
      if (success) {
        navigation_sync_needed_ = false;
      } else {
        navigation_sync_needed_ = true;
        navigation_cancel_retry_at_ = now + .25;
        if (ros_navigation::failed_jetson_cancel_blocks_start(
                navigation_backend_ == NavigationBackend::Jetson, navigation_deferred_request_.is_object())) {
          discard_deferred_navigation("Jetson 旧任务取消失败，启动未发送");
          begin_navigation_stop(now);
        }
      }
    }
    if (navigation_deferred_request_.is_object()) {
      const auto& request = navigation_deferred_request_;
      const bool cooperative = request.contains("mode") && request["mode"].is_string() &&
          request["mode"].get<std::string>() == "cooperative";
      const bool allow_disconnect = request.contains("allow_disconnect") &&
          request["allow_disconnect"].is_boolean() && request["allow_disconnect"].get<bool>();
      const bool ros_owner = request.value("__executor_ros", false);
      if (navigation_deferred_epoch_ != ros_authorization_epoch_ || (!ros_owner && (
          request.at("session").get<std::string>() != lease_.session ||
          peer_key(navigation_deferred_peer_) != lease_.peer ||
          (cooperative && !allow_disconnect && now - lease_.last >= .2)))) {
        discard_deferred_navigation("等待导航同步或模式确认期间控制权已失效，启动未发送");
      } else if (emergency_stop_ || global_cancel_needed_ || navigation_stop_pending_) {
        discard_deferred_navigation("等待期间收到停车或急停，启动未发送");
      } else {
        const bool waiting_for_sync = navigation_backend_ == NavigationBackend::Jetson &&
            (navigation_sync_needed_ || navigation_cancel_future_ || !navigation_client_->service_is_ready());
        if (!waiting_for_sync && manual_navigation_ready(now)) {
          const auto saved_request = navigation_deferred_request_;
          const auto peer = navigation_deferred_peer_;
          const auto first_received_at = navigation_deferred_at_;
          navigation_deferred_request_ = nullptr;
          try { queue_navigation_task(saved_request, peer, now, first_received_at); }
          catch (const std::exception& error) {
            send_web_result(saved_request, peer, false, std::string("局部任务字段无效：") + error.what());
          }
        }
      }
    }
    // Do not abandon a timed-out future: an old cancel could still arrive.
    // Once it resolves, waiting for a new start no longer adds a delay.
    if (navigation_sync_needed_ && !navigation_cancel_future_ &&
        (navigation_backend_ == NavigationBackend::Fpga ||
         (navigation_backend_ == NavigationBackend::Jetson &&
          !navigation_task_active_ && !navigation_transfer_active_)) &&
        now >= navigation_cancel_retry_at_ && navigation_client_->service_is_ready())
      cancel_jetson_navigation();
  }

  void begin_navigation_stop(double now, bool claims_already_revoked = false) {
    discard_deferred_navigation("导航启动已被停车或人工接管取消");
    if (navigation_task_active_) navigation_cancelled_task_id_ = navigation_task_id_;
    if (navigation_transfer_active_) abort_navigation_transfer(false);
    cancel_jetson_navigation();
    revoke_all_controls();
    navigation_task_active_ = false;
    if (navigation_task_id_ != 0) navigation_status_ = 7;
    navigation_task_flags_known_ = false;
    navigation_owner_session_.clear();
    navigation_owner_peer_.clear();
    navigation_stop_pending_ = true;
    navigation_stop_zero_ack_at_ = -1e100;
    navigation_stop_ack_at_ = -1e100;
    next_navigation_stop_at_ = now + .1;
    // 撤销 Jetson 授权后立即发送零速和关闭模式屏障。
    send_request(3, {}, false, now, "navigation-stop-fence");
    send_request(5, {}, false, now, "navigation-stop-fence");
    if (!claims_already_revoked) invalidate_pc_claims(now);
    else send_pc_status(now);
  }

  void receive_executor_request(std::shared_ptr<rmw_request_id_t> header,
      const std::shared_ptr<ugv_bridge::srv::NavigationCommand::Request>& command) {
    const auto reject = [&](const std::string& message) {
      ugv_bridge::srv::NavigationCommand::Response response;
      response.success = false;
      response.message = message;
      executor_service_->send_response(*header, response);
    };
    if (command->action < 1 || command->action > 3 ||
        (command->action == 1 && (command->mode < 2 || command->mode > 3)) ||
        (command->flags & ~3U)) {
      reject("invalid executor action, mode or flags");
      return;
    }
    if (command->boot_id != navigation_boot_id_ || command->odom_domain != navigation_odom_domain_ ||
        !command->boot_id || !command->odom_domain || !command->task_id) {
      reject("executor request requires the current boot/domain and a nonzero task_id");
      return;
    }
    if (command->action == 1 && executor_pending_header_) {
      reject("another executor request is still pending");
      return;
    }
    if (command->action == 1 &&
        ((control_source_ == ControlSource::Pc && lease_.armed) ||
         (control_source_ == ControlSource::Ros && ros_authorized_))) {
      reject("manual or visual control owns the robot; stop it before starting a global segment");
      return;
    }
    const auto now = monotonic();
    if (command->action != 1) begin_navigation_stop(now);
    executor_pending_header_ = std::move(header);
    Json request{{"__executor_ros", true}, {"session", "ros-executor"}, {"request_id", "ros-executor"},
        {"action", command->action == 1 ? "local_start" : (command->action == 2 ? "local_stop" : "local_cancel")},
        {"task_id", command->task_id}, {"mode", command->mode == 3 ? "independent" : "cooperative"},
        {"allow_disconnect", (command->flags & 1U) != 0}, {"position_only", (command->flags & 2U) != 0},
        {"goal_x_m", command->goal_x_m}, {"goal_y_m", command->goal_y_m},
        {"hold_yaw_rad", command->hold_yaw_rad}, {"final_yaw_rad", command->final_yaw_rad},
        {"max_speed_mm_s", command->max_speed_mm_s}, {"max_accel_mm_s2", command->max_accel_mm_s2},
        {"tolerance_mm", command->tolerance_mm}, {"yaw_tolerance_mrad", command->yaw_tolerance_mrad},
        {"duration_s", command->duration_s}};
    try {
      queue_navigation_task(request, {}, now);
    } catch (const std::exception& error) {
      reply_executor(false, std::string("invalid executor request: ") + error.what());
    }
  }

  void send_global_result(const Json& request, const sockaddr_in& peer, bool success,
                          bool unknown, const std::string& message) {
    pc_.send(Json{{"v", 1}, {"type", "navigation_task_result"},
        {"session", request.at("session")}, {"bridge_id", bridge_id_},
        {"request_id", request.at("request_id")}, {"action", request.at("action")},
        {"success", success}, {"unknown", unknown}, {"message", utf8_prefix(message, 240)}}, peer);
  }

  void request_global_cancel(double now) {
    ++global_safety_epoch_;
    global_cancel_needed_ = true;
    global_cancel_accepted_epoch_.reset();
    global_cancel_retry_at_ = now;
    publish_bridge_safety_state();
  }

  void poll_global_cancel(double now) {
    if (global_cancel_future_) {
      if (global_cancel_future_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        bool inactive = false;
        try {
          const auto result = global_cancel_future_->future.get();
          const auto state = Json::parse(result->state_json, nullptr, false);
          inactive = result->success && !result->unknown && state.is_object() &&
                     state.contains("active") && state["active"].is_boolean() && !state["active"].get<bool>();
        } catch (const std::exception&) {}
        global_cancel_future_.reset();
        if (inactive && global_cancel_epoch_ == global_safety_epoch_) {
          global_cancel_accepted_epoch_ = global_cancel_epoch_;
        } else global_cancel_retry_at_ = now + .25;
      } else if (now - global_cancel_sent_at_ > 3.0) {
        global_navigation_client_->remove_pending_request(global_cancel_future_->request_id);
        global_cancel_future_.reset();
        global_cancel_retry_at_ = now + .25;
      }
    }
    if (global_cancel_needed_ && global_cancel_accepted_epoch_ &&
        *global_cancel_accepted_epoch_ == global_safety_epoch_ &&
        !navigation_stop_pending_ && !navigation_task_active_ && !navigation_transfer_active_) {
      global_cancel_needed_ = false;
      global_cancel_accepted_epoch_.reset();
      // A later start uses a newer epoch than any retried cancel.
      ++global_safety_epoch_;
      publish_bridge_safety_state();
    }
    if (!global_cancel_needed_ || global_cancel_accepted_epoch_ || global_cancel_future_ || now < global_cancel_retry_at_ ||
        !global_navigation_client_->service_is_ready()) return;
    auto command = std::make_shared<ugv_bridge::srv::GlobalNavigation::Request>();
    command->request_json = Json{{"action", "cancel"}, {"_bridge_id", bridge_id_},
                                 {"_bridge_safety_epoch", global_safety_epoch_}}.dump();
    global_cancel_epoch_ = global_safety_epoch_;
    global_cancel_sent_at_ = now;
    global_cancel_future_.emplace(global_navigation_client_->async_send_request(command));
  }

  void receive_global_request(const Json& request, const sockaddr_in& peer, double now) {
    if (!request.contains("session") || !request["session"].is_string() ||
        !request.contains("request_id") || !request["request_id"].is_string() ||
        !request.contains("action") || !request["action"].is_string()) return;
    const auto id = request["request_id"].get<std::string>();
    const auto action = request["action"].get<std::string>();
    if (id.empty() || id.size() > 64 || (action != "start" && action != "stop" && action != "cancel")) return;
    if (action == "start" && (emergency_stop_ || global_cancel_needed_)) {
      send_global_result(request, peer, false, false, "急停或全局取消尚未确认");
      return;
    }
    if (action == "start" && control_source_ == ControlSource::Pc && lease_.armed &&
        (request["session"].get<std::string>() != lease_.session || peer_key(peer) != lease_.peer)) {
      send_global_result(request, peer, false, false, "其他页面正在进行人工控制");
      return;
    }
    if (action == "start" && control_source_ == ControlSource::Ros && ros_authorized_ &&
        (web_control_session_.empty() || request["session"].get<std::string>() != web_control_session_ ||
         peer_key(peer) != web_control_peer_)) {
      send_global_result(request, peer, false, false, "其他来源正在进行 ROS 控制");
      return;
    }
    auto monitor = request;
    monitor["type"] = "control";
    monitor["take_control"] = false;
    monitor["vx"] = monitor["vy"] = monitor["wz"] = 0;
    monitor["auto"] = false;
    if (!accept_pc_command(monitor, peer, now, action != "start")) {
      send_global_result(request, peer, false, false, "会话或序号已更新，请等待状态后重试");
      return;
    }
    if (action != "start") {
      request_global_cancel(now);
      begin_navigation_stop(now, true);
      send_global_result(request, peer, true, true, "已请求本地停车，全局取消等待确认");
      return;
    }
    if (!global_navigation_client_->service_is_ready()) {
      send_global_result(request, peer, false, false, "Jetson 全局导航服务尚未就绪");
      return;
    }
    if (global_requests_.size() >= 16) {
      send_global_result(request, peer, false, false, "过多全局导航请求正在处理");
      return;
    }
    if (control_source_ == ControlSource::Ros && ros_authorized_) revoke_ros_authorization();
    publish_bridge_safety_state();
    auto command = std::make_shared<ugv_bridge::srv::GlobalNavigation::Request>();
    auto coordinator_request = request;
    coordinator_request["_bridge_id"] = bridge_id_;
    coordinator_request["_bridge_safety_epoch"] = global_safety_epoch_;
    coordinator_request["_bridge_control_seq"] = bridge_safety_sequence_;
    command->request_json = coordinator_request.dump();
    global_requests_.push_back({global_navigation_client_->async_send_request(command), request, peer, now,
                                global_safety_epoch_});
  }

  void poll_global_results(double now) {
    for (auto it = global_requests_.begin(); it != global_requests_.end();) {
      const bool ready = it->future.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      if (!ready && now - it->sent_at <= 3.0) { ++it; continue; }
      if (ready) {
        try {
          const auto response = it->future.future.get();
          if (it->safety_epoch != global_safety_epoch_)
            send_global_result(it->request, it->peer, false, true, "启动期间已请求停车，先前结果未知");
          else
            send_global_result(it->request, it->peer, response->success, response->unknown, response->message);
        } catch (const std::exception&) {
          send_global_result(it->request, it->peer, false, true, "全局导航服务结果未知");
        }
      } else {
        global_navigation_client_->remove_pending_request(it->future.request_id);
        send_global_result(it->request, it->peer, false, true, "全局导航服务超时，结果未知");
      }
      it = global_requests_.erase(it);
    }
  }

  void receive_global_state(const std_msgs::msg::String& message) {
    const auto state = Json::parse(message.data, nullptr, false);
    if (!state.is_object() && !state.is_null()) return;
    if (state.is_object() && state.contains("active") && state["active"].is_boolean()) {
      global_navigation_active_ = state["active"].get<bool>();
    }
    if (observers_.entries().empty()) return;
    const auto now = monotonic();
    const auto sequence = ++global_state_sequence_;
    Json data{{"type", "navigation_task"}, {"sequence", sequence},
        {"time_us", static_cast<std::uint32_t>(static_cast<std::uint64_t>(now * 1000000.0))},
        {"session", std::string(64, '\x01')}, {"state", state}};
    try {
      // Reserve the largest allowed session before splitting; all clients share one fragment layout.
      const auto packets = telemetry_envelopes(bridge_id_, telemetry_tracker_.epoch, data, now, now);
      for (const auto& observer : observers_.entries()) for (const auto& original : packets) {
        auto packet = original;
        if (packet["data"].contains("session")) packet["data"]["session"] = observer.session;
        if (!pc_.send(packet, observer.endpoint)) ++send_errors_;
      }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(node_->get_logger(), "Global navigation state cannot be transmitted: %s", error.what());
      for (const auto& observer : observers_.entries())
        pc_.send(Json{{"v", 1}, {"type", "navigation_task_state_error"},
            {"session", observer.session}, {"bridge_id", bridge_id_},
            {"message", "全局导航状态超出传输容量，完整路线不可用"}}, observer.endpoint);
    }
  }

  void handle_web_command(const Json& request, const sockaddr_in& peer, double now) {
    static const std::array<std::string, 9> actions{
        "enable_ros", "disable_ros", "start", "stop", "emergency_stop", "release_emergency_stop",
        "local_start", "local_stop", "local_cancel"};
    if (request.contains("__executor_ros") || !request.contains("session") || !request["session"].is_string() ||
        !request.contains("action") || !request["action"].is_string() ||
        !request.contains("request_id") || !request["request_id"].is_string()) return;
    const auto action = request["action"].get<std::string>();
    const auto id = request["request_id"].get<std::string>();
    if (id.empty() || id.size() > 64 ||
        std::find(actions.begin(), actions.end(), action) == actions.end()) return;
    // Commands and monitoring share the control session's sequence watermark.
    auto monitor = request;
    monitor["type"] = "control";
    monitor["take_control"] = false;
    monitor["vx"] = monitor["vy"] = monitor["wz"] = 0;
    monitor["auto"] = false;
    const bool safety_only = action == "stop" || action == "local_stop" ||
        action == "local_cancel" || action == "emergency_stop" || action == "disable_ros";
    if (!accept_pc_command(monitor, peer, now, safety_only)) {
      send_web_result(request, peer, false, "会话或序号已更新，请等待状态后重试");
      return;
    }
    if (action == "local_start" || action == "local_stop" || action == "local_cancel") {
      if (action != "local_start") {
        request_global_cancel(now);
        begin_navigation_stop(now, true);
      }
      try {
        queue_navigation_task(request, peer, now);
      } catch (const std::exception& error) {
        send_web_result(request, peer, false, std::string("局部任务字段无效：") + error.what());
      }
      return;
    }
    if (action == "enable_ros" || action == "disable_ros") {
      auto command = std::make_shared<std_srvs::srv::SetBool::Request>();
      auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
      command->data = action == "enable_ros";
      set_ros_authorization(command, response, safety_only);
      if (command->data && response->success) {
        web_control_session_ = lease_.session;
        web_control_peer_ = lease_.peer;
        lease_.last = now;
      }
      send_web_result(request, peer, response->success, response->message);
      return;
    }
    if (action == "emergency_stop" || action == "release_emergency_stop") {
      const bool stop = action == "emergency_stop";
      const bool already_stopped = emergency_stop_;
      set_emergency_stop(stop, safety_only);
      if (stop && already_stopped) send_pc_status(now);
      std_msgs::msg::Bool message;
      message.data = stop;
      emergency_stop_pub_->publish(message);
      send_web_result(request, peer, true, stop ? "软件急停已请求，机械停车待确认" :
          "软件急停已解除，未恢复控制授权");
      return;
    }
    if (action == "stop") {
      request_global_cancel(now);
      begin_navigation_stop(now, true);
    }
    if (emergency_stop_ && action == "start") {
      send_web_result(request, peer, false, "急停激活，不能开始任务");
      return;
    }
    if (action == "stop" && start_future_) {
      start_cancelled_ = true;
      if (mission_stop_->service_is_ready())
        mission_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      send_web_result(request, peer, true, "已撤销控制；正在等待启动请求完成", true);
      return;
    }
    if (web_future_ || start_future_) {
      send_web_result(request, peer, false, "任务服务请求仍在处理，请等待结果");
      return;
    }
    pending_web_request_ = request;
    pending_web_peer_ = peer;
    pending_web_at_ = now;
    if (action == "start") {
      if (!mission_start_->service_is_ready()) {
        send_web_result(request, peer, false, "任务服务尚未就绪"); return;
      }
      auto command = std::make_shared<ugv_bridge::srv::MissionStart::Request>();
      try {
        command->target_id = request.value("target_id", std::string{});
        command->class_id = request.value("class_id", std::string("0"));
        command->behavior = request.value("behavior", std::string("follow"));
        command->color = request.value("color", std::string{});
      } catch (const Json::exception&) {
        send_web_result(request, peer, false, "目标参数类型无效"); return;
      }
      const bool numeric = !command->class_id.empty() && command->class_id.size() <= 2 &&
          std::all_of(command->class_id.begin(), command->class_id.end(),
                      [](unsigned char ch) { return ch >= '0' && ch <= '9'; });
      if (!numeric || std::stoi(command->class_id) > 79 || command->target_id.size() > 128 ||
          (command->behavior != "follow" && command->behavior != "approach" && command->behavior != "search" &&
          command->behavior != "search_only") ||
          (command->color != "" && command->color != "red")) {
        send_web_result(request, peer, false, "目标或任务参数无效"); return;
      }
      if (!ros_control_enabled_ || !ready_ || now - ack_at_ >= .2 ||
          navigation_transfer_active_ || navigation_task_active_) {
        send_web_result(request, peer, false, "控制不可用或局部任务正在运行"); return;
      }
      start_cancelled_ = false;
      start_future_.emplace(mission_start_->async_send_request(command));
    } else {
      if (!mission_stop_->service_is_ready()) {
        send_web_result(request, peer, true, "已撤销控制，任务服务不可用", true); return;
      }
      web_future_.emplace(mission_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()));
    }
  }

  void poll_web_result(double now) {
    if (start_future_) {
      if (start_future_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          const auto response = start_future_->future.get();
          if (response->success && !start_cancelled_ && !emergency_stop_ &&
              lease_.session == pending_web_request_.at("session").get<std::string>() &&
              lease_.peer == peer_key(pending_web_peer_) &&
              now - lease_.last <= .5) {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            auto result = std::make_shared<std_srvs::srv::SetBool::Response>();
            req->data = true;
            set_ros_authorization(req, result);
            if (result->success) {
              web_control_session_ = lease_.session;
              web_control_peer_ = lease_.peer;
              lease_.last = now;
              send_web_result(pending_web_request_, pending_web_peer_, true, response->message);
            } else {
              if (mission_stop_->service_is_ready()) mission_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
              send_web_result(pending_web_request_, pending_web_peer_, false, result->message);
            }
          } else {
            if (response->success && mission_stop_->service_is_ready()) mission_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            send_web_result(pending_web_request_, pending_web_peer_, false,
                response->success ? "启动期间控制已撤销" : response->message);
          }
        } catch (const std::exception&) {
          send_web_result(pending_web_request_, pending_web_peer_, false, "服务结果未知", true);
        }
        start_future_.reset();
      } else if (now - pending_web_at_ > 2.0) {
        mission_start_->remove_pending_request(start_future_->request_id);
        if (mission_stop_->service_is_ready()) mission_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        send_web_result(pending_web_request_, pending_web_peer_, false, "服务超时，结果未知", true);
        start_future_.reset();
      }
      return;
    }
    if (!web_future_) return;
    if (web_future_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = web_future_->future.get();
        send_web_result(pending_web_request_, pending_web_peer_, response->success, response->message);
      } catch (const std::exception&) {
        send_web_result(pending_web_request_, pending_web_peer_, false, "服务结果未知", true);
      }
    } else if (now - pending_web_at_ > 2.0) {
      mission_stop_->remove_pending_request(web_future_->request_id);
      send_web_result(pending_web_request_, pending_web_peer_, false, "服务超时，结果未知", true);
    } else return;
    web_future_.reset();
  }

  double observation_age(const vision_msgs::msg::Detection2DArray::ConstSharedPtr& data,
                         double received, double now) const {
    if (!data || data->header.frame_id.empty() || data->header.stamp.sec < 0 ||
        data->header.stamp.nanosec >= 1000000000U) return 999;
    const double stamp = data->header.stamp.sec + data->header.stamp.nanosec / 1e9;
    const double source_age = node_->now().seconds() - stamp;
    if (stamp <= 0 || source_age < 0 || now < received) return 999;
    return std::min(999.0, std::max(source_age, now - received));
  }

  static Json web_box(const vision_msgs::msg::Detection2D& detection) {
    const auto& box = detection.bbox;
    if (detection.id.empty() || detection.id.size() > 128 || detection.results.empty() ||
        !std::isfinite(box.center.position.x) || !std::isfinite(box.center.position.y) ||
        !std::isfinite(box.size_x) || !std::isfinite(box.size_y) ||
        box.center.position.x < 0 || box.center.position.x > video_width ||
        box.center.position.y < 0 || box.center.position.y > video_height ||
        box.size_x <= 0 || box.size_x > video_width || box.size_y <= 0 || box.size_y > video_height) return nullptr;
    const auto& result = detection.results.front().hypothesis;
    if (!std::isfinite(result.score) || result.score < 0 || result.score > 1 ||
        result.class_id.size() > 32) return nullptr;
    return Json{{"id", detection.id}, {"class_id", result.class_id}, {"score", result.score},
        {"cx", box.center.position.x}, {"cy", box.center.position.y},
        {"width", box.size_x}, {"height", box.size_y}};
  }

  void send_vision(double now) {
    if (observers_.entries().empty()) return;
    const double age_s = observation_age(web_detections_, web_detections_at_, now);
    const bool stale = age_s > .5;
    Json boxes = Json::array();
    bool truncated = false;
    if (!stale) for (const auto& detection : web_detections_->detections) {
      if (boxes.size() == 8) { truncated = true; break; }
      auto box = web_box(detection);
      if (!box.is_null()) boxes.push_back(std::move(box));
    }
    Json selected = nullptr;
    if (!stale && observation_age(web_selected_, web_selected_at_, now) <= .5 &&
        web_selected_->detections.size() == 1) selected = web_box(web_selected_->detections.front());
    auto header = web_detections_ ? web_detections_->header : std_msgs::msg::Header{};
    if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) header = std_msgs::msg::Header{};
    if (header.frame_id.size() > 128) header.frame_id.clear();
    Json message{{"v", 1}, {"type", "vision"}, {"bridge_id", bridge_id_},
        {"session", ""}, {"seq", vision_sequence_++}, {"mission_state", mission_state_},
        {"control_source", request_source()}, {"ros_authorized", ros_authorized_},
        {"emergency_stop", emergency_stop_}, {"frame_id", header.frame_id},
        {"stamp", {{"sec", header.stamp.sec}, {"nanosec", header.stamp.nanosec}}},
        {"width", video_width}, {"height", video_height}, {"age_ms", std::round(age_s * 1000)}, {"stale", stale},
        {"detections", boxes}, {"selected_target", selected}, {"truncated", truncated}};
    for (const auto& observer : observers_.entries()) {
      if (now - observer.last > .5) continue;
      auto per_observer = message;
      per_observer["session"] = observer.session;
      while (per_observer.dump().size() > 1400 && !per_observer["detections"].empty()) {
        per_observer["detections"].erase(per_observer["detections"].end() - 1);
        per_observer["truncated"] = true;
      }
      if (per_observer.dump().size() > 1400) {
        per_observer["selected_target"] = nullptr; per_observer["truncated"] = true;
      }
      if (!pc_.send(per_observer, observer.endpoint)) ++send_errors_;
    }
  }

  Bytes make_request(std::uint8_t type, Velocity velocity, bool auto_enabled)
  {
    const auto sequence = ++fpga_seq_;
    if (type == 5) return fpga_auto_packet(sequence, token_, auto_enabled);
    return fpga_packet(sequence, token_, velocity);
  }

  bool send_navigation_probe(double now) {
    ProbePacket probe;
    probe.kind = 1;
    probe.sequence = fpga_seq_ + 1U;
    probe.token = navigation_session_token_;
    return send_packet(encode_probe(probe), 1, probe.sequence, {}, false, now, "navigation-heartbeat");
  }

  void poll_navigation_transfer(double now) {
    if (navigation_backend_ == NavigationBackend::Fpga) {
      if (!navigation_transfer_active_) return;
      if (navigation_transfer_started_at_ > -1e50 && now - navigation_transfer_started_at_ >= .18) {
        navigation_packet_queue_.clear();
        outstanding_.clear();
        navigation_transfer_active_ = false;
        navigation_result(false, "FPGA 任务分片 ACK 超时，执行结果未知", true);
        begin_navigation_stop(now);
        return;
      }
      if (navigation_packets_acked_ == 8 && navigation_packet_queue_.empty()) {
        navigation_transfer_active_ = false;
        if (navigation_action_ == "local_start") {
          navigation_task_active_ = true;
          navigation_start_transaction_id_ = queued_navigation_task_id_;
        }
        navigation_result(true, "8 个 FPGA 任务分片均收到匹配 ACK；执行结果以遥测为准");
        return;
      }
      if (!outstanding_.empty()) {
        auto& pending = outstanding_.front();
        if (pending.source == "navigation-task" && now - pending.sent_at >= .02) {
          if (fpga_.send(pending.packet, fpga_peer_)) pending.sent_at = now;
          else { ++send_errors_; fail_safe(); }
        }
        return;
      }
      if (navigation_packet_queue_.empty()) return;
      if (navigation_transfer_started_at_ < -1e50) navigation_transfer_started_at_ = now;
      auto packet = std::move(navigation_packet_queue_.front());
      navigation_packet_queue_.pop_front();
      const auto sequence = fpga_seq_ + 1U;
      send_packet(std::move(packet), 7, sequence, {}, false, now, "navigation-task");
      return;
    }
    if (!navigation_future_) return;
    const bool starting = navigation_action_ == "local_start";
    // A ready future can itself be late. Enforce the original web deadline
    // before accepting its response, then revoke any possibly issued motion.
    if (starting && now - navigation_request_at_ >= 2.0) {
      navigation_client_->remove_pending_request(navigation_future_->request_id);
      navigation_future_.reset();
      navigation_transfer_active_ = false;
      navigation_result(false, "Jetson 导航服务超时，任务结果未知", true);
      begin_navigation_stop(now);
      return;
    }
    if (navigation_future_->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      bool success = false;
      std::string message = "Jetson 导航服务结果未知";
      try {
        const auto result = navigation_future_->future.get();
        success = result->success;
        message = result->message;
      } catch (const std::exception&) {}
      navigation_future_.reset();
      navigation_transfer_active_ = false;
      navigation_result(success, message, !success && message == "Jetson 导航服务结果未知");
      if (!success && starting) begin_navigation_stop(now);
    } else if (now - navigation_request_at_ > 2.0) {
      navigation_client_->remove_pending_request(navigation_future_->request_id);
      navigation_future_.reset();
      navigation_transfer_active_ = false;
      navigation_result(false, "Jetson 导航服务超时，任务结果未知", true);
      if (starting) begin_navigation_stop(now);
    }
  }

  void fail_safe() {
    ++timeouts_;
    discard_deferred_navigation("FPGA 连接失效，导航启动未发送");
    if (navigation_transfer_active_ || navigation_task_active_) {
      cancel_jetson_navigation();
      if (navigation_future_) {
        navigation_client_->remove_pending_request(navigation_future_->request_id);
        navigation_future_.reset();
      }
      navigation_transfer_active_ = false;
      navigation_task_active_ = false;
      if (navigation_task_id_ != 0) navigation_status_ = 8;
      navigation_task_flags_known_ = false;
      navigation_owner_session_.clear();
      navigation_owner_peer_.clear();
      navigation_result(false, "FPGA ACK超时；局部任务状态未知并已停止网桥保活", true);
    }
    revoke_all_controls();
  }

  bool send_packet(Bytes packet, std::uint8_t type, std::uint32_t sequence,
                   Velocity velocity, bool auto_enabled, double now,
                   const std::string& source) {
    if (!fpga_.send(packet, fpga_peer_)) {
      ++send_errors_;
      fail_safe();
      return false;
    }
    fpga_seq_ = sequence;
    if (type == 3) sent_velocity_ = velocity;
    const bool pc_source = source == "pc" || (source == "none" && type == 3 &&
        velocity == Velocity{} && !lease_.peer.empty() && now - lease_.last <= .25);
    outstanding_.push_back({std::move(packet), now, sequence, type, auto_enabled,
                            velocity, pc_source ? lease_.seq : 0,
                            pc_source ? lease_.session : std::string{}, source});
    return true;
  }

  bool send_request(std::uint8_t type, Velocity velocity, bool auto_enabled, double now,
                    const std::string& source) {
    auto packet = make_request(type, velocity, auto_enabled);
    return send_packet(std::move(packet), type, fpga_seq_, velocity, auto_enabled, now, source);
  }

  void receive_ack(const Bytes& data, const sockaddr_in& peer, double now) {
    if (!same_peer(peer, fpga_peer_)) return;
    auto found = std::find_if(outstanding_.begin(), outstanding_.end(), [&](const Pending& item) {
      return valid_ack(data, item.packet);
    });
    if (found == outstanding_.end()) return;
    Pending accepted = *found;
    if (accepted.type == 7 && accepted.source == "navigation-task" && navigation_transfer_active_)
      ++navigation_packets_acked_;
    ack_at_ = now;
    if (accepted.source == "navigation-stop-fence") {
      if (accepted.type == 3) navigation_stop_zero_ack_at_ = now;
      else if (accepted.type == 5) navigation_stop_ack_at_ = now;
    }
    last_rtt_ms_ = (now - accepted.sent_at) * 1000.0;
    ++ack_count_;
    last_ack_ = {{"type", accepted.type}, {"fpga_seq", accepted.sequence},
                 {"pc_seq", accepted.pc_seq}, {"pc_session", accepted.pc_session},
                 {"source", accepted.source},
                 {"jetson_rx_mono_s", now}};
    if (accepted.type == 3)
      last_ack_["velocity"] = Json::array({accepted.velocity[0], accepted.velocity[1], accepted.velocity[2]});
    else
      last_ack_["auto_enabled"] = accepted.auto_enabled;

    // A newer ACK supersedes older commands, but the other stop-fence ACK
    // may already be in flight even though its command was applied first.
    for (auto it = outstanding_.begin(); it != outstanding_.end();) {
      const bool waiting_other_fence = navigation_stop_pending_ &&
          it->source == "navigation-stop-fence" &&
          ((it->type == 3 && navigation_stop_zero_ack_at_ < -1e50) ||
           (it->type == 5 && navigation_stop_ack_at_ < -1e50));
      if (!newer(it->sequence, accepted.sequence) && !waiting_other_fence)
        it = outstanding_.erase(it);
      else ++it;
    }
    if (accepted.type == 3) {
      if (accepted.velocity == Velocity{}) {
        zero_barrier_ack_ = true;
        ready_ = true;
      }
    } else if (accepted.type == 5) {
      fpga_auto_ = accepted.auto_enabled;
      if (mode_target_ && accepted.auto_enabled == *mode_target_) {
        mode_command_required_ = false;
        auto_acknowledged_ = true;
      }
    }
    // 手动来源的零速屏障一旦获得真实ACK就立即通知PC。
    // 等待100ms周期状态可能错过网页200ms握手期限，尤其在type5切换ACK覆盖后。
    if (accepted.type == 3 && accepted.velocity == Velocity{} && accepted.source == "pc")
      send_pc_status(now);
  }

  void clear_sensor_cache() {
    discard_deferred_navigation("FPGA 会话变化，导航启动未发送");
    cancel_jetson_navigation();
    if (navigation_future_) {
      navigation_client_->remove_pending_request(navigation_future_->request_id);
      navigation_future_.reset();
    }
    route_overlay_.clear();
    last_host_.clear();
    motor_stamp_.reset();
    motor_ = nullptr;
    motor_at_ = -1e100;
    actual_auto_.reset();
    last_fusion_at_ = -1e100;
    last_telemetry_at_ = -1e100;
    sent_velocity_ = {};
    time_mapper_.reset();
    scan_assembler_.reset();
    wheel_odometry_.reset();
    navigation_task_id_ = 0;
    next_navigation_task_id_ = 0;
    navigation_cancelled_task_id_ = 0;
    navigation_cancel_transaction_id_ = 0;
    navigation_boot_id_ = 0;
    navigation_odom_domain_ = 0;
    navigation_session_token_ = 0;
    navigation_telemetry_rejected_ = false;
    navigation_bootstrap_sent_ = false;
    navigation_last_task_id_known_ = false;
    navigation_now_us_ = 0;
    navigation_mode_ = 0;
    navigation_flags_ = 0;
    navigation_task_flags_known_ = false;
    navigation_owner_session_.clear();
    navigation_owner_peer_.clear();
    navigation_status_ = 0;
    navigation_received_at_ = -1e100;
    navigation_state_known_ = false;
    navigation_context_ = nullptr;
    jetson_navigation_context_ = nullptr;
    jetson_navigation_received_at_ = -1e100;
    navigation_task_epoch_ = 0;
    navigation_transfer_active_ = false;
    navigation_task_active_ = false;
    navigation_backend_ = NavigationBackend::Unknown;
    navigation_packet_queue_.clear();
    navigation_start_transaction_id_ = 0;
    cpu_health_known_ = false;
    cpu_ready_ = false;
    cpu_fault_ = false;
    cpu_health_at_ = -1e100;
    navigation_request_ = nullptr;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      ++video_generation_;
      latest_frame_.reset();
      frame_at_ = -1e100;
    }
  }

  void invalidate_navigation_domain(bool boot_changed) {
    discard_deferred_navigation("FPGA 里程域变化，导航启动未发送");
    cancel_jetson_navigation();
    if (navigation_future_) {
      navigation_client_->remove_pending_request(navigation_future_->request_id);
      navigation_future_.reset();
    }
    local_grid_assembler_.reset();
    route_overlay_.clear();
    if (navigation_transfer_active_)
      navigation_result(false, "FPGA boot或odom_domain发生变化；导航请求结果未知", true);
    navigation_transfer_active_ = false;
    navigation_task_active_ = false;
    navigation_packet_queue_.clear();
    navigation_start_transaction_id_ = 0;
    if (boot_changed) navigation_backend_ = NavigationBackend::Unknown;
    navigation_state_known_ = false;
    navigation_context_ = nullptr;
    jetson_navigation_context_ = nullptr;
    jetson_navigation_received_at_ = -1e100;
    navigation_task_epoch_ = 0;
    navigation_task_id_ = 0;
    navigation_last_task_id_known_ = false;
    navigation_status_ = 0;
    navigation_mode_ = 0;
    navigation_flags_ = 0;
    navigation_task_flags_known_ = false;
    navigation_owner_session_.clear();
    navigation_owner_peer_.clear();
    navigation_now_us_ = 0;
    navigation_session_token_ = 0;
    navigation_telemetry_rejected_ = false;
    navigation_bootstrap_sent_ = false;
    navigation_received_at_ = -1e100;
    navigation_cancelled_task_id_ = 0;
    navigation_cancel_transaction_id_ = 0;
    scan_assembler_.reset();
    wheel_odometry_.reset();
    if (boot_changed) {
      time_mapper_.reset();
      next_navigation_task_id_ = 0;
      navigation_boot_id_ = 0;
      navigation_odom_domain_ = 0;
    }
    revoke_all_controls();
  }

  void forward_telemetry(const Json& decoded, double received_at, const char* description) {
    if (observers_.entries().empty()) return;
    try {
      const auto tx = monotonic();
      for (const auto& envelope : telemetry_envelopes(
               bridge_id_, telemetry_tracker_.epoch, decoded, received_at, tx)) {
        const auto text = envelope.dump();
        const Bytes packet(text.begin(), text.end());
        for (const auto& observer : observers_.entries())
          if (!pc_.send(packet, observer.endpoint)) ++send_errors_;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(node_->get_logger(), "%s: %s", description, e.what());
    }
  }

  void observe_telemetry(const Bytes& bytes, double received_at) {
    Json decoded;
    try {
      decoded = decode_telemetry(bytes);
      if (decoded.at("type")=="local_grid_chunk") {
        auto snapshot=local_grid_assembler_.feed(decoded,received_at);
        if (!snapshot) return;
        decoded=std::move(*snapshot);
      }
    } catch (const std::invalid_argument&) {
      ++decode_errors_;
      return;
    }
    // Map chunks share telemetry type 8 with navigation, arrive as a frozen
    // snapshot, and may be reordered. Forward them without the per-stream
    // monotonic sequence filter used by ordinary sensor telemetry.
    if (decoded.at("type") == "map") {
      forward_telemetry(decoded, received_at, "map telemetry envelope");
      return;
    }
    bool restarted = false;
    if (!telemetry_tracker_.observe(decoded, received_at, &restarted)) return;
    if (restarted) {
      // A confirmed FPGA reboot invalidates old sensor samples and interval state.
      if (navigation_transfer_active_)
        navigation_result(false, "检测到FPGA重启；导航请求结果未知", true);
      revoke_all_controls();
      fpga_auto_.reset();
      clear_sensor_cache();
      local_grid_assembler_.reset();
    }

    if (decoded.at("type") == "navigation") {
      const bool rx_rejected = decoded.value("rejected", false);
      const auto incoming_boot = decoded.at("boot_id").get<std::uint32_t>();
      const auto incoming_domain = decoded.at("odom_domain").get<std::uint32_t>();
      const bool boot_changed = !rx_rejected && navigation_state_known_ &&
          incoming_boot != navigation_boot_id_;
      const bool domain_changed = !rx_rejected && navigation_state_known_ &&
          incoming_domain != navigation_odom_domain_;
      const bool bootstrap_activated = navigation_bootstrap_sent_ && zero_barrier_ack_ &&
          navigation_boot_id_ == 0 && incoming_boot != 0;
      if ((boot_changed || domain_changed) && !bootstrap_activated)
        invalidate_navigation_domain(boot_changed);
      if (!rx_rejected) {
        const auto protocol = decoded.value("protocol", std::string{});
        const auto backend = protocol == "velocity_only_v1" ? NavigationBackend::Jetson :
            (supported_navigation_protocol(protocol) ? NavigationBackend::Fpga : NavigationBackend::Unknown);
        if (backend == NavigationBackend::Unknown) return;
        if (navigation_backend_ != NavigationBackend::Unknown && navigation_backend_ != backend) {
          RCLCPP_WARN(node_->get_logger(), "Ignoring conflicting navigation protocol in the same FPGA boot");
          return;
        }
        if (navigation_backend_ == NavigationBackend::Unknown && backend == NavigationBackend::Fpga)
          navigation_sync_needed_ = true;  // Quiesce any leftover Jetson local task.
        navigation_backend_ = backend;
      }
      time_mapper_.observe_navigation(decoded.at("now_us").get<std::uint32_t>(),
                                     node_->now().nanoseconds());
      normalize_navigation_packet(decoded);
      if (!rx_rejected) local_grid_assembler_.set_domain(incoming_boot,incoming_domain);
      if (navigation_backend_ == NavigationBackend::Fpga && decoded.value("rx_rejection", false) &&
          decoded.value("rejected_task_id", std::uint32_t{0}) == queued_navigation_task_id_) {
        if (navigation_transfer_active_) {
          navigation_packet_queue_.clear();
          for (auto it = outstanding_.begin(); it != outstanding_.end();) {
            if (it->source == "navigation-task") it = outstanding_.erase(it);
            else ++it;
          }
          navigation_transfer_active_ = false;
          navigation_result(false, "FPGA 拒绝本次任务；具体原因以导航遥测为准");
        }
        navigation_start_transaction_id_ = 0;
      }
    }

    publish_sensor_packet(decoded, received_at);
    if (decoded["type"] == "motor") {
      motor_ = decoded;
      motor_at_ = received_at;
    } else if (decoded["type"] == "fusion") {
      const auto flags = decoded.value("flags", std::uint32_t{0});
      actual_auto_ = (flags & (std::uint32_t{1} << 11)) != 0;
      last_fusion_at_ = received_at;
    } else if (decoded["type"] == "health") {
      cpu_health_known_ = decoded.value("cpu_health_valid", false);
      cpu_ready_ = cpu_health_known_ && decoded.value("cpu_ready", false);
      cpu_fault_ = !cpu_health_known_ || decoded.value("cpu_fault", true);
      cpu_health_at_ = received_at;
    }

    const auto forwarded = decoded["type"] == "navigation" ? navigation_for_web(decoded) : decoded;
    if (decoded["type"] == "navigation") route_overlay_.observe(forwarded, received_at);
    if (navigation_backend_ != NavigationBackend::Jetson ||
        (decoded["type"] != "local_grid" && decoded["type"] != "local_plan"))
      forward_telemetry(forwarded, received_at, "telemetry envelope");
    last_telemetry_at_ = received_at;
  }

  template <class Message>
  void common(Message& msg, const Json& packet) {
    msg.header.stamp = node_->now();
    msg.sequence = packet.at("sequence").get<std::uint32_t>();
    msg.timestamp_us = packet.at("time_us").get<std::uint64_t>();
    msg.dropped = packet.at("queue_dropped").get<std::uint32_t>();
  }

  void normalize_navigation_packet(Json& packet) {
    const auto error = packet.at("error").get<std::uint8_t>();
    const bool rejected = (error & 0x80U) != 0;
    const auto status = packet.at("status").get<std::uint8_t>();
    const auto mode = packet.at("mode").get<std::uint8_t>();
    const auto guard_error = packet.value("guard_error", std::uint8_t{0});
    packet["planner_error"] = (!rejected && guard_error == 0 && (error & 0x40U) == 0)
        ? static_cast<std::uint8_t>(error & 0x3fU) : 0;
    packet["guard_error"] = rejected ? 0 : guard_error;
    packet["rx_rejection"] = rejected;
    packet["active_task"] = rejected
        ? (navigation_context_.is_object() && navigation_context_.value("active_task", false))
        : (navigation_status_active(status) && (mode == 2 || mode == 3));
    if (!rejected) {
      const auto protocol=packet.value("protocol", std::string{});
      const bool modern_cancel_transaction =
          (protocol == "nav_polar_v2" || protocol == "nav_grid_v1") &&
          packet.value("last_task_id", std::uint32_t{0}) == navigation_cancel_transaction_id_;
      if (status == 7 && navigation_cancelled_task_id_ != 0 &&
          (packet.at("task_id").get<std::uint32_t>() == navigation_cancelled_task_id_ ||
           modern_cancel_transaction)) {
        packet["cancelled_task_id"] = navigation_cancelled_task_id_;
        packet["cancel_transaction_id"] = navigation_cancel_transaction_id_;
      }
      navigation_context_ = packet;
      return;
    }

    packet["rejected_task_id"] = packet.at("task_id");
    packet["rx_rejection_error"] = error;
    packet["rejection_reason"] = static_cast<std::uint8_t>(error & 0x7fU);
    if (navigation_context_.is_object()) {
      for (const auto* key : {"protocol", "deadline_known", "boot_id", "odom_domain", "task_id",
                              "last_task_id", "expiry_us", "now_us", "session_token", "progress",
                              "status", "planner_error", "guard_error", "mode", "flags",
                              "cancelled_task_id", "cancel_transaction_id", "vx_mm_s", "vy_mm_s",
                              "wz_mrad_s", "goal_x_q16", "goal_y_q16", "pose_age_us"}) {
        if (navigation_context_.contains(key)) packet[key] = navigation_context_.at(key);
      }
      const bool polar_context = navigation_context_.value("protocol", std::string{}) == "nav_polar_v2";
      const bool grid_context = navigation_context_.value("protocol", std::string{}) == "nav_grid_v1";
      const auto copy_if_present = [&](const char* key) {
        if (navigation_context_.contains(key)) packet[key] = navigation_context_.at(key);
      };
      // 拒绝事件继承之前的活动任务，清除当前报文另一架构的字段后再拷贝。
      for (const auto* key : {"candidates_evaluated","map_age_us","map_version",
                              "planner_expansions","polar_candidates_evaluated",
                              "polar_age_us","polar_generation"}) packet.erase(key);
      if (grid_context) {
        copy_if_present("candidates_evaluated");
        copy_if_present("map_age_us");
        copy_if_present("map_version");
      } else if (polar_context) {
        copy_if_present("polar_candidates_evaluated");
        copy_if_present("polar_age_us");
        copy_if_present("polar_generation");
      } else {
        copy_if_present("planner_expansions");
        copy_if_present("map_age_us");
        copy_if_present("map_version");
      }
      packet["rejected"] = true;
      packet["rx_rejection"] = true;
      packet["active_task"] = navigation_context_.value("active_task", false);
    }
  }

  void update_navigation_state(const Json& packet, double received_at) {
    navigation_state_known_ = true;
    navigation_now_us_ = packet.at("now_us").get<std::uint64_t>();
    navigation_session_token_ = packet.at("session_token").get<std::uint64_t>();
    navigation_telemetry_rejected_ = packet.value("rx_rejection", false);
    navigation_received_at_ = received_at;

    if (!packet.value("rx_rejection", false)) {
      navigation_boot_id_ = packet.at("boot_id").get<std::uint32_t>();
      navigation_odom_domain_ = packet.at("odom_domain").get<std::uint32_t>();
      if (navigation_boot_id_ != 0) navigation_bootstrap_sent_ = false;
      navigation_last_task_id_known_ = packet.contains("last_task_id") &&
          packet.at("last_task_id").is_number_unsigned();
      if (navigation_last_task_id_known_) {
        const auto last = packet.at("last_task_id").get<std::uint32_t>();
        if (last != 0 && (next_navigation_task_id_ == 0 || newer(last, next_navigation_task_id_)))
          next_navigation_task_id_ = last;
      }
      if (navigation_backend_ == NavigationBackend::Fpga) {
        const auto task = packet.at("task_id").get<std::uint32_t>();
        const auto status = packet.at("status").get<std::uint8_t>();
        const auto mode = packet.at("mode").get<std::uint8_t>();
        navigation_task_id_ = task;
        navigation_status_ = status;
        navigation_mode_ = mode;
        if (navigation_status_active(status) && (mode == 2 || mode == 3)) {
          navigation_task_active_ = true;
          if (task == navigation_start_transaction_id_) navigation_start_transaction_id_ = 0;
        } else if (navigation_start_transaction_id_ != 0) {
          if (task == navigation_start_transaction_id_ && navigation_status_terminal(status)) {
            navigation_task_active_ = false;
            navigation_start_transaction_id_ = 0;
            navigation_task_flags_known_ = false;
          }
        } else if (!navigation_transfer_active_) {
          navigation_task_active_ = false;
          navigation_task_flags_known_ = false;
        }
      }
    }

    std_msgs::msg::String state;
    state.data = packet.dump();
    navigation_state_pub_->publish(state);
    state.data = navigation_for_web(packet).dump();
    executor_state_pub_->publish(state);
  }

  Json navigation_for_web(const Json& fpga) {
    if (navigation_backend_ != NavigationBackend::Jetson) return fpga;
    Json result = fpga;
    result["type"] = "navigation";
    result["protocol"] = "jetson_nav_v1";
    result["source"] = "jetson";
    result["sequence"] = ++jetson_navigation_sequence_;
    result["task_id"] = navigation_task_id_;
    result["last_task_id"] = next_navigation_task_id_;
    result["task_counter_valid"] = true;
    const bool stopping = navigation_stop_pending_ && navigation_task_id_ != 0;
    result["status"] = stopping ? 2 : navigation_status_;
    result["mode"] = navigation_task_active_ || stopping ? navigation_mode_ : 1;
    result["active_task"] = navigation_task_active_ || stopping;
    result["goal_x_q16"] = jetson_goal_x_q16_;
    result["goal_y_q16"] = jetson_goal_y_q16_;
    result["expiry_us"] = navigation_task_active_ ? jetson_expiry_us_ : 0;
    result["deadline_known"] = navigation_task_active_;
    result["progress"] = jetson_navigation_context_.is_object()
        ? jetson_navigation_context_.value("progress", 0u) : 0u;
    result["planner_error"] = 0;
    result["cancelled_task_id"] = navigation_cancelled_task_id_;
    result["cancel_transaction_id"] = navigation_cancel_transaction_id_;
    return result;
  }

  void receive_jetson_navigation(const std_msgs::msg::String& message) {
    if (navigation_backend_ != NavigationBackend::Jetson) return;
    const auto state = Json::parse(message.data, nullptr, false);
    if (!state.is_object()) return;
    try {
      const auto boot = state.at("boot_id").get<std::uint32_t>();
      const auto domain = state.at("odom_domain").get<std::uint32_t>();
      const auto task = state.at("task_id").get<std::uint32_t>();
      const auto last = state.at("last_task_id").get<std::uint32_t>();
      const auto status = state.at("status").get<unsigned>();
      const auto mode = state.at("mode").get<unsigned>();
      const auto progress = state.at("progress").get<unsigned>();
      if (!boot || !domain || boot != navigation_boot_id_ || domain != navigation_odom_domain_ ||
          status > 9 || mode > 3 || progress > 1000) return;
      // 网桥重启后仍须延续节点的事务水位，旧任务状态不能因此重新获得运动权。
      if (last != 0 && (next_navigation_task_id_ == 0 || newer(last, next_navigation_task_id_)))
        next_navigation_task_id_ = last;
      if (task != 0) {
        if (task != navigation_task_id_ || state.at("bridge_id").get<std::string>() != bridge_id_ ||
            state.at("authorization_epoch").get<std::uint64_t>() != navigation_task_epoch_) return;
        if (navigation_status_active(static_cast<std::uint8_t>(status)) &&
            (!navigation_task_active_ || control_source_ != ControlSource::Navigation ||
             navigation_task_epoch_ != ros_authorization_epoch_)) return;
      } else if (navigation_task_active_ || navigation_task_id_ != 0) return;
      jetson_navigation_context_ = state;
      jetson_navigation_received_at_ = monotonic();
      if (task != 0) {
        navigation_status_ = static_cast<std::uint8_t>(status);
        if (navigation_status_terminal(navigation_status_)) {
          navigation_task_active_ = false;
          navigation_task_flags_known_ = false;
          navigation_owner_session_.clear();
          navigation_owner_peer_.clear();
          if (control_source_ == ControlSource::Navigation) revoke_all_controls();
        }
      }
      if (navigation_context_.is_object()) {
        std_msgs::msg::String current;
        current.data = navigation_for_web(navigation_context_).dump();
        executor_state_pub_->publish(current);
      }
    } catch (const Json::exception&) {
      ++decode_errors_;
    }
  }

  void receive_jetson_grid(const std_msgs::msg::String& message) {
    if (navigation_backend_ != NavigationBackend::Jetson) return;
    const auto grid = Json::parse(message.data, nullptr, false);
    if (!grid.is_object()) return;
    try {
      if (grid.value("type", std::string{}) != "local_grid" ||
          grid.value("protocol", std::string{}) != "jetson_grid_snapshot_v1" ||
          grid.value("source", std::string{}) != "jetson" ||
          grid.at("boot_id").get<std::uint32_t>() != navigation_boot_id_ ||
          grid.at("odom_domain").get<std::uint32_t>() != navigation_odom_domain_ ||
          !grid.contains("packed_cells") || !grid.at("packed_cells").is_array() ||
          grid.at("packed_cells").size() != 1024) return;
      forward_telemetry(grid, monotonic(), "Jetson local grid envelope");
      executor_grid_pub_->publish(message);
    } catch (const Json::exception&) {
      ++decode_errors_;
    } catch (const std::invalid_argument&) {
      ++decode_errors_;
    }
  }

  void receive_jetson_plan(const std_msgs::msg::String& message) {
    if (navigation_backend_ != NavigationBackend::Jetson) return;
    const auto plan = Json::parse(message.data, nullptr, false);
    if (!plan.is_object()) return;
    try {
      if (plan.value("type", std::string{}) != "local_plan" ||
          plan.value("protocol", std::string{}) != "jetson_local_segment_v1" ||
          plan.value("source", std::string{}) != "jetson" ||
          plan.at("boot_id").get<std::uint32_t>() != navigation_boot_id_ ||
          plan.at("odom_domain").get<std::uint32_t>() != navigation_odom_domain_ ||
          plan.at("task_id").get<std::uint32_t>() != navigation_task_id_ ||
          !navigation_task_active_) return;
      const auto received = monotonic();
      route_overlay_.observe(plan, received);
      forward_telemetry(plan, received, "Jetson local plan envelope");
    } catch (const Json::exception&) {
      ++decode_errors_;
    } catch (const std::invalid_argument&) {
      ++decode_errors_;
    }
  }

  void publish_laser_scan(const Json& packet) {
    ros_navigation::N10Group group;
    group.sequence = packet.at("sequence").get<std::uint32_t>();
    group.timestamp_us = packet.at("time_us").get<std::uint32_t>();
    group.dropped = packet.at("queue_dropped").get<std::uint32_t>();
    group.crc_errors = packet.at("crc_errors").get<std::uint32_t>();
    group.length_errors = packet.at("length_errors").get<std::uint32_t>();
    const auto& points = packet.at("points");
    for (std::size_t index = 0; index < group.points.size(); ++index) {
      const auto& source = points.at(index);
      group.points[index].angle_cdeg = source.at("angle_cdeg").get<std::uint16_t>();
      group.points[index].distance_mm = source.at("distance_mm").get<std::uint16_t>();
      group.points[index].strength = source.at("strength").get<std::uint8_t>();
    }
    const auto stamp = time_mapper_.map_legacy_us(group.timestamp_us);
    if (!stamp) {
      scan_assembler_.reset();
      return;
    }
    auto scan = scan_assembler_.consume(group, *stamp, n10_frame_id_,
                                         scan_range_min_m_, scan_range_max_m_);
    if (scan) scan_pub_->publish(*scan);
  }

  void publish_wheel_odometry(const Json& packet) {
    const auto source_low = packet.at("time_us").get<std::uint32_t>();
    const auto source_us = time_mapper_.unwrap_legacy_us(source_low);
    const auto stamp_ns = source_us ? time_mapper_.map_fpga_us(*source_us) : std::nullopt;
    if (!source_us || !stamp_ns) {
      wheel_odometry_.reset();
      return;
    }
    const auto& velocity = packet.at("odometry");
    const bool valid = packet.at("encoder_invalid_mask").get<std::uint8_t>() == 0;
    const auto odom = wheel_odometry_.update(
        velocity.at("vx_mm_s").get<std::int32_t>() * 0.001,
        velocity.at("vy_mm_s").get<std::int32_t>() * 0.001,
        velocity.at("wz_mrad_s").get<std::int32_t>() * 0.001,
        *source_us, *stamp_ns, valid, odom_frame_id_, base_frame_id_);
    if (odom) wheel_odom_pub_->publish(*odom);
  }

  void publish_fusion_odometry(const Json& packet) {
    if ((packet.at("flags").get<std::uint32_t>() & 1U) == 0) return;
    // 用扩展后的32位导航时标映射融合包源时间；不把处理时刻当作报文采样时刻。
    const auto stamp = time_mapper_.map_legacy_us(packet.at("time_us").get<std::uint32_t>());
    if (!stamp) return;
    const auto& source_state = packet.at("state");
    const std::array<const char*, 7> keys{
        "x_m", "y_m", "yaw_rad", "vx_m_s", "vy_m_s", "wz_rad_s", "gyro_bias_rad_s"};
    std::array<double, 7> state{};
    std::array<double, 7> covariance;
    const bool estimate_layout=(packet.at("flags").get<std::uint32_t>()&(1U<<14))!=0;
    covariance.fill(-1.0);
    for (std::size_t index = 0; index < keys.size(); ++index) {
      state[index] = source_state.at(keys[index]).get<double>();
      if (!std::isfinite(state[index])) return;
    }
    if (estimate_layout) {
      // 极坐标和局部栅格复用相同的误差估计格式；不得把计数字当作协方差。
      const auto& estimate=packet.at("error_estimate");
      if (!estimate.is_array() || estimate.size()!=3) return;
      const std::array<double,3> values{estimate.at(0).get<double>(),estimate.at(1).get<double>(),
                                       estimate.at(2).get<double>()};
      const auto result=ros_navigation::estimate_covariance(values);
      if (!result) return;
      covariance=*result;
    } else if (packet.contains("covariance_diag") && packet.at("covariance_diag").is_array()) {
      for (std::size_t index=0;index<7 && index<packet.at("covariance_diag").size();++index)
        covariance[index]=packet.at("covariance_diag").at(index).get<double>();
    }
    auto odom = ros_navigation::make_fusion_odometry(
        state, covariance, *stamp, odom_frame_id_, base_frame_id_);
    fusion_odom_pub_->publish(odom);
    // FPGA融合已经消费N10；该单一变换直接来自融合输出，不再把同一雷达送入ROS EKF。
    tf_broadcaster_->sendTransform(ros_navigation::odom_transform(odom));
  }

  void publish_sensor_packet(const Json& packet, double received_at) {
    const auto name = packet.at("type").get<std::string>();
    if (navigation_backend_ != NavigationBackend::Jetson || (name != "local_grid" && name != "local_plan"))
      route_overlay_.observe(packet, received_at);
    ++received_[name];
    last_host_[name] = received_at;
    std_msgs::msg::String raw;
    raw.data = packet.dump();
    raw_pubs_.at(name)->publish(raw);
    if (name == "local_grid" && navigation_backend_ == NavigationBackend::Fpga)
      executor_grid_pub_->publish(raw);
    if (name == "local_plan") local_plan_pub_->publish(raw);

    if (name == "imu")
     {
      ugv_bridge::msg::Bx72ImuRaw raw_imu;
      common(raw_imu, packet);
      raw_imu.header.frame_id = imu_frame_id_;
      raw_imu.reader_sequence = packet.at("sample_sequence").get<std::uint16_t>();
      raw_imu.flags = packet.at("flags").get<std::uint8_t>();
      raw_imu.who_am_i = packet.at("who_am_i").get<std::uint8_t>();
      const std::array<const char*, 3> axes{"x", "y", "z"};
      for (int i = 0; i < 3; ++i) {
        raw_imu.accel_raw[i] = packet.at("accel_raw").at(axes[i]).get<std::int16_t>();
        raw_imu.gyro_raw[i] = packet.at("gyro_raw").at(axes[i]).get<std::int16_t>();
      }
      imu_raw_pub_->publish(raw_imu);

      if (raw_imu.flags == 15 && raw_imu.who_am_i == 0x70)
       {
        sensor_msgs::msg::Imu imu;
        imu.header = raw_imu.header;
        imu.orientation_covariance[0] = -1.0;
        imu.linear_acceleration.x = raw_imu.accel_raw[0] * .000122 * 9.80665;
        imu.linear_acceleration.y = raw_imu.accel_raw[1] * .000122 * 9.80665;
        imu.linear_acceleration.z = raw_imu.accel_raw[2] * .000122 * 9.80665;
        const double scale = .0175 * std::acos(-1) / 180;
        imu.angular_velocity.x = raw_imu.gyro_raw[0] * scale;
        imu.angular_velocity.y = raw_imu.gyro_raw[1] * scale;
        imu.angular_velocity.z = raw_imu.gyro_raw[2] * scale;
        imu_pub_->publish(imu);
      }
    }
    else if (name == "lidar")
    {
      ugv_bridge::msg::N10Packet lidar;
      common(lidar, packet);
      lidar.header.frame_id = n10_frame_id_;
      lidar.crc_errors = packet.at("crc_errors").get<std::uint32_t>();
      lidar.length_errors = packet.at("length_errors").get<std::uint32_t>();
      lidar.framing_errors = 0;
      lidar.framing_errors_available = false;
      for (int i = 0; i < 16; ++i) {
        const auto& point = packet.at("points").at(i);
        lidar.angle_cdeg[i] = point.at("angle_cdeg").get<std::uint16_t>();
        lidar.distance_mm[i] = point.at("distance_mm").get<std::uint16_t>();
        lidar.strength[i] = point.at("strength").get<std::uint8_t>();
      }
      n10_pub_->publish(lidar);
      publish_laser_scan(packet);
    }
     else if (name == "motor")
      {
      ugv_bridge::msg::EncoderCounts encoder;
      common(encoder, packet);
      const auto stamp = packet.at("time_us").get<std::uint32_t>();
      encoder.interval_us = motor_stamp_ ? std::uint32_t(stamp - *motor_stamp_) : 0;
      motor_stamp_ = stamp;
      const auto invalid = packet.at("encoder_invalid_mask").get<std::uint8_t>();
      const auto map = packet.at("encoder_channel_map").get<std::uint8_t>();
      for (int i = 0; i < 4; ++i) {
        encoder.ticks[i] = packet.at("physical_count_ticks")
                               .at("MOTOR" + std::to_string(i + 1)).get<std::int32_t>();
        encoder.invalid_mask |= ((invalid >> i) & 1) << ((map >> (2 * i)) & 3);
      }
      encoder.ready = true;
      encoder_pub_->publish(encoder);
      publish_wheel_odometry(packet);
    }
    else if (name == "fusion") publish_fusion_odometry(packet);
    else if (name == "navigation") update_navigation_state(packet, received_at);
  }

  void poll_sockets() {
    pollfd descriptors[] = {{pc_.fd, POLLIN, 0}, {fpga_.fd, POLLIN, 0},
                            {telemetry_.fd, POLLIN, 0}};
    if (poll(descriptors, 3, 10) < 0 && errno != EINTR)
      throw std::runtime_error(std::strerror(errno));
    Udp* sockets[] = {&pc_, &fpga_, &telemetry_};
    for (int index = 0; index < 3; ++index) {
      if (!(descriptors[index].revents & POLLIN)) continue;
      const auto batch_started = monotonic();
      for (int count = 0; count < 64; ++count) {
        // Telemetry may be continuously readable. Return to ACKs, task
        // fragments and lease timers within 4 ms instead of doing 64 costly
        // decode/ROS/JSON operations before servicing control again.
        if (index == 2 && monotonic() - batch_started >= .004) break;
        auto datagram = sockets[index]->receive();
        if (!datagram) break;
        const auto now = monotonic();
        if (index == 0) {
          if (observers_.expire(now)) refresh_video_peers();
          if (datagram->data.size() > 16384 ||
              (pc_filter_ && datagram->peer.sin_addr.s_addr != pc_filter_->sin_addr.s_addr)) continue;
          auto msg = Json::parse(datagram->data.begin(), datagram->data.end(), nullptr, false);
          if (msg.is_object() && msg.value("type", Json{}) == "navigation_task") {
            receive_global_request(msg, datagram->peer, now);
            continue;
          }
          if (msg.is_object() && msg.value("type", Json{}) == "mission_command") {
            handle_web_command(msg, datagram->peer, now);
            continue;
          }
          const bool requested_auto = msg.is_object() && msg.contains("auto") &&
              msg["auto"].is_boolean() && msg["auto"].get<bool>();
          const bool monitor = msg.is_object() && msg.contains("take_control") &&
              msg["take_control"].is_boolean() && !msg["take_control"].get<bool>();
          if (requested_auto) {
            ++source_errors_;
            if (msg.contains("session") && msg["session"].is_string() &&
                msg["session"].get<std::string>() == lease_.session &&
                peer_key(datagram->peer) == lease_.peer)
              pc_.send(Json{{"v", 1}, {"type", "event"}, {"event", "legacy_auto_rejected"},
                  {"reason", "无目标type5自动模式已停用；请提交odom局部任务"}}, datagram->peer);
            continue;
          }
          if ((emergency_stop_ || navigation_stop_pending_) && !monitor) continue;
          if (msg.is_object() && msg.contains("wz") && msg["wz"].is_number_integer() &&
              !msg["wz"].is_boolean()) {
            const auto angular = msg["wz"].get<std::int64_t>();
            if (angular < -800 || angular > 800)
              msg["wz"] = std::clamp<std::int64_t>(angular, -800, 800);
          }
          const auto frame = parse_pc_control(msg);
          if (!frame) continue;
          const bool was_manual = control_source_ == ControlSource::Pc && lease_.armed && now - lease_.last < .2;
          const auto old_session = lease_.session;
          const auto old_peer = lease_.peer;
          const auto accepted = accept_pc_frame(*frame, datagram->peer, now);
          if (accepted == PcInputResult::Owner) {
            if (frame->claim && (global_navigation_active_ || global_cancel_needed_ || !global_requests_.empty())) {
              if (!global_cancel_needed_) request_global_cancel(now);
              begin_navigation_stop(now);
              pc_.send(Json{{"v", 1}, {"type", "event"}, {"event", "global_cancel_pending"},
                            {"reason", "全局导航正在停止，请确认后重新申请人工控制"}}, datagram->peer);
              continue;
            }
            if (lease_.armed && (navigation_transfer_active_ || navigation_task_active_ ||
                                  navigation_deferred_request_.is_object()))
              begin_navigation_stop(now);
            const bool session_changed = old_session != lease_.session || old_peer != lease_.peer;
            if (lease_.armed) {
              if (frame->claim && (!was_manual || session_changed)) ++global_safety_epoch_;
              if (start_future_) start_cancelled_ = true;
              const bool source_changed = control_source_ != ControlSource::Pc;
              if (ros_authorized_) {
                ros_authorized_ = false;
                ros_authorized_stamp_.reset();
                clear_ros_command();
              }
              if (source_changed || session_changed) ++ros_authorization_epoch_;
              control_source_ = ControlSource::Pc;
              if (source_changed || session_changed) reset_control_path();
              publish_bridge_safety_state();
              if ((source_changed || session_changed) && bidirectional_status_pub_)
                publish_bidirectional_status(now);
            }
            if (!lease_.armed) publish_bridge_safety_state();
            // Broadcast only after the control source and lease agree. A
            // false owner status here would reject a valid pending claim.
            if (frame->claim || session_changed) send_pc_status(now);
          }
        } else if (!same_peer(datagram->peer, fpga_peer_)) {
          ++source_errors_;
        } else if (index == 1) {
          receive_ack(datagram->data, datagram->peer, now);
        } else {
          observe_telemetry(datagram->data, now);
        }
      }
    }
  }

  static void add_values(diagnostic_msgs::msg::DiagnosticStatus& status, const Json& data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = it.key();
      if (it.value().is_boolean())
        value.value = it.value().get<bool>() ? "True" : "False";
      else if (it.value().is_null())
        value.value = "None";
      else if (it.value().is_string())
        value.value = it.value().get<std::string>();
      else
        value.value = it.value().dump();
      status.values.push_back(std::move(value));
    }
  }

  void publish_sensor_status() {
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node_->now();
    const auto host = monotonic();
    for (const auto& entry : raw_pubs_) {
      const auto& name = entry.first;
      const bool seen = last_host_.count(name) != 0;
      const double age_s = seen ? host - last_host_.at(name) : 0;
      const bool fresh = seen && age_s < (name == "health" ? 2.5 : .5);
      Status status;
      status.name = "bx72/" + name;
      status.hardware_id = args_.get("--fpga-ip");
      status.level = fresh ? Status::OK : Status::ERROR;
      status.message = fresh ? "fresh telemetry; calibration pending" : "missing/stale telemetry";
      add_values(status, {{"received", received_.at(name)},
                          {"age_s", seen ? Json(age_s) : Json(nullptr)}});
      message.status.push_back(std::move(status));
    }
    Status protocol;
    protocol.name = "bx72/protocol";
    protocol.hardware_id = args_.get("--fpga-ip");
    protocol.level = (decode_errors_ || source_errors_) ? Status::ERROR : Status::OK;
    protocol.message = "BX72-v1; 32-bit FPGA timestamp; ROS header is receive time";
    add_values(protocol, {{"decode_errors", decode_errors_}, {"source_errors", source_errors_},
                          {"epoch", telemetry_tracker_.epoch}});
    message.status.push_back(std::move(protocol));
    sensor_status_pub_->publish(message);
  }

  bool telemetry_fresh(double now) const {
    for (const auto& entry : last_host_) {
      const double maximum_age = entry.first == "health" ? 2.5 : .5;
      if (now - entry.second < maximum_age) return true;
    }
    return false;
  }

  void publish_bidirectional_status(double now) {
    using Status = diagnostic_msgs::msg::DiagnosticStatus;
    const bool ack_fresh = now - ack_at_ < .2;
    const bool fresh_telemetry = telemetry_fresh(now);
    const bool bidirectional = ack_fresh && fresh_telemetry;
    const bool desired_auto = false;
    const bool mode_acknowledged = mode_target_ && !mode_command_required_ && auto_acknowledged_;
    const bool actual_mode_known = actual_auto_ && now - last_fusion_at_ < 1.0;
    const bool actual_matches = !actual_mode_known || *actual_auto_ == desired_auto;
    const bool mode_ready = ready_ && zero_barrier_ack_ && mode_acknowledged && actual_matches;

    Status status;
    status.name = "bx72_udp";
    status.hardware_id = args_.get("--fpga-ip");
    status.level = !bidirectional ? Status::ERROR : (mode_ready ? Status::OK : Status::WARN);
    if (!ack_fresh) status.message = "BX72 control ACK timeout";
    else if (!fresh_telemetry) status.message = "BX72 telemetry missing/stale";
    else if (!mode_ready) status.message = "bidirectional UDP OK; control mode not ready";
    else status.message = "bidirectional UDP OK; control mode ready";

    Json actual_mode = nullptr;
    if (actual_mode_known) actual_mode = *actual_auto_ ? "auto" : "manual";
    add_values(status, {{"bidirectional", bidirectional}, {"control_ack_fresh", ack_fresh},
                        {"telemetry_fresh", fresh_telemetry}, {"control_ready", ready_},
                        {"mode_ready", mode_ready}, {"mode_acknowledged", mode_acknowledged},
                        {"actual_mode_matches", actual_matches}, {"actual_mode", actual_mode},
                        {"control_source", request_source()},
                        {"pc_peer", lease_.peer}, {"pc_session", lease_.session},
                        {"pc_seq", lease_.seq}, {"pc_lease_age_s", age(now, lease_.last)},
                        {"bridge_id", bridge_id_},
                        {"ros_authorization_epoch", ros_authorization_epoch_},
                        {"ros_control_enabled", ros_control_enabled_},
                        {"ros_authorized", ros_authorized_},
                        {"ros_command_age_s", age(now, ros_command_received_ ?
                            ros_command_at_ : ros_authorized_at_)},
                        {"emergency_stop", emergency_stop_},
                        {"video_complete_frames", frame_count_.load()},
                        {"ros_image_convert_attempts", ros_image_convert_attempts_.load()},
                        {"ros_image_publish_attempts", ros_image_publish_attempts_.load()},
                        {"ros_image_published", ros_image_published_.load()},
                        {"ros_image_skipped_expired", ros_image_skipped_expired_.load()},
                        {"ros_image_skipped_generation", ros_image_skipped_generation_.load()},
                        {"ros_image_publish_age_s", age(now, ros_image_published_at_.load())},
                        {"ack_age_s", age(now, ack_at_)},
                        {"telemetry_age_s", age(now, last_telemetry_at_)},
                        {"acknowledged", ack_count_}, {"timeouts", timeouts_},
                        {"send_errors", send_errors_.load()},
                        {"rtt_ms", last_rtt_ms_ >= 0 ? Json(last_rtt_ms_) : Json(nullptr)}});
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = node_->now();
    message.status.push_back(std::move(status));
    bidirectional_status_pub_->publish(message);
  }

  void send_pc_status(double now) {
    if (observers_.entries().empty()) return;
    Json actual_mode = nullptr;
    if (actual_auto_ && now - last_fusion_at_ < 1.0)
      actual_mode = *actual_auto_ ? "auto" : "manual";
    const auto tx = monotonic();
    Json status = {{"v", 1}, {"type", "status"}, {"session", ""},
                   {"pc_seq", 0}, {"ready", ready_},
                   {"pc_timeout", true},
                   {"fpga_ack_age", age(now, ack_at_)}, {"ack_count", ack_count_},
                   {"timeouts", timeouts_}, {"velocity", sent_velocity_},
                   {"frames", frame_count_.load()}, {"video_dropped", video_dropped_.load()},
                   {"video_gaps", video_gaps_.load()}, {"video_age", age(now, frame_at_.load())},
                   {"jpeg_frames", jpeg_count_.load()},
                   {"motor", now - motor_at_ < 1 ? motor_ : Json(nullptr)},
                   {"bridge_id", bridge_id_}, {"seq", status_seq_++}, {"jetson_tx_mono_s", tx},
                   {"control_source", source_name(control_source_)},
                   {"ros_authorized", ros_authorized_},
                   {"ros_command_age_s", age(now, ros_command_received_ ?
                       ros_command_at_ : ros_authorized_at_)},
                   {"emergency_stop", emergency_stop_}, {"global_cancel_pending", global_cancel_needed_},
                   {"auto_requested", false}, {"auto_acknowledged", auto_acknowledged_},
                   {"legacy_auto_supported", false}, {"actual_mode", actual_mode},
                   {"last_ack", last_ack_}};
    for (const auto& observer : observers_.entries()) {
      status["session"] = observer.session;
      status["pc_seq"] = observer.seq;
      status["pc_timeout"] = now - observer.last > .25;
      status["claim_nonce"] = observer.claim_nonce;
      status["pc_control_owner"] = control_source_ == ControlSource::Pc && lease_.armed &&
          lease_.session == observer.session && lease_.peer == peer_key(observer.endpoint);
      try {
        if (!pc_.send(bounded_status(status), observer.endpoint)) ++send_errors_;
      } catch (const std::length_error& e) {
        RCLCPP_WARN(node_->get_logger(), "PC status: %s", e.what());
      }
    }
  }

  void video_worker() {
    try {
      RtpFrames rtp;
      std::mt19937 random(std::random_device{}());
      std::uint32_t frame_id = random();
      std::uint32_t raw_frame_id = 0;
      bool raw_frame_active = false;
      double raw_frame_started = -1e100;
      std::shared_ptr<const Bytes> image_source;
      std::shared_ptr<const Bytes> jpeg_source;
      std::weak_ptr<const Bytes> last_expired_source;
      cv::Mat image;
      double next_encode = 0;
      std::uint64_t worker_generation = video_generation_.load();
      auto reset_video_state = [&] {
        rtp.expire_incomplete();
        raw_frame_id = 0;
        raw_frame_active = false;
        raw_frame_started = -1e100;
        image_source.reset();
        jpeg_source.reset();
        last_expired_source.reset();
        image.release();
        next_encode = 0;
        video_dropped_ = rtp.dropped;
        video_gaps_ = rtp.gaps;
      };
      auto flush_video_queue = [&] {
        for (int count = 0; count < 8192 && video_.receive(); ++count) {}
      };
      while (!stop_.load()) {
        pollfd descriptor{video_.fd, POLLIN, 0};
        const auto polled = poll(&descriptor, 1, 10);
        if (polled < 0 && errno != EINTR) throw std::runtime_error(std::strerror(errno));
        auto current_generation = video_generation_.load();
        if (current_generation != worker_generation) {
          worker_generation = current_generation;
          reset_video_state();
          flush_video_queue();
          continue;
        }
        if (descriptor.revents & POLLIN) {
          bool generation_changed = false;
          for (int count = 0; count < 128; ++count) {
            if (video_generation_.load() != worker_generation) {
              generation_changed = true;
              break;
            }
            auto datagram = video_.receive();
            if (!datagram) break;
            if (!same_peer(datagram->peer, fpga_peer_)) continue;
            const auto received = monotonic();
            const auto& raw = datagram->data;
            if (rtp.stamp && received - raw_frame_started > kVideoTimeoutS) {
              rtp.expire_incomplete();
              raw_frame_active = false;
            }
            const bool had_stamp = rtp.stamp.has_value();
            const auto previous_packets = rtp.packets;
            auto frame = rtp.feed(raw);
            if (rtp.packets != previous_packets) {
              const std::uint32_t incoming = (std::uint32_t(raw[4]) << 24) |
                  (std::uint32_t(raw[5]) << 16) | (std::uint32_t(raw[6]) << 8) | raw[7];
              if (!had_stamp || !raw_frame_active || incoming != raw_frame_id) {
                raw_frame_id = incoming;
                raw_frame_active = true;
                raw_frame_started = received;
              }
            }
            frame_count_ = rtp.complete;
            video_dropped_ = rtp.dropped;
            video_gaps_ = rtp.gaps;
            if (frame) {
              raw_frame_active = false;
              {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                if (video_generation_.load() == worker_generation) {
                  latest_frame_ = std::make_shared<Bytes>(std::move(*frame));
                  frame_at_ = received;
                } else {
                  ++ros_image_skipped_generation_;
                  generation_changed = true;
                }
              }
              if (generation_changed) break;
            }
          }
          if (generation_changed) {
            worker_generation = video_generation_.load();
            reset_video_state();
            flush_video_queue();
            continue;
          }
        }
        if (video_generation_.load() != worker_generation) {
          worker_generation = video_generation_.load();
          reset_video_state();
          flush_video_queue();
          continue;
        }
        const auto now = monotonic();
        if (rtp.stamp && now - raw_frame_started > kVideoTimeoutS) {
          rtp.expire_incomplete();
          raw_frame_active = false;
          video_dropped_ = rtp.dropped;
          video_gaps_ = rtp.gaps;
        }

        std::shared_ptr<const Bytes> raw;
        std::vector<sockaddr_in> peers;
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          raw = latest_frame_;
          peers = video_peers_;
          // A PC joining after a video outage must not receive the cached old frame.
          if (now - frame_at_.load() > kVideoTimeoutS) {
            if (raw && raw != image_source && raw != last_expired_source.lock()) {
              ++ros_image_skipped_expired_;
              last_expired_source = raw;
            }
            raw.reset();
          }
        }
        if (raw && raw != image_source) {
          image_source = raw;
          ++ros_image_convert_attempts_;
          image = yuv422_to_bgr(*raw, video_pixel_format_);
          if (video_generation_.load() != worker_generation) {
            ++ros_image_skipped_generation_;
            continue;
          }
          sensor_msgs::msg::Image msg;
          msg.header.stamp = node_->now();
          msg.header.frame_id = camera_frame_id_;
          msg.height = video_height;
          msg.width = video_width;
          msg.encoding = sensor_msgs::image_encodings::BGR8;
          msg.is_bigendian = 0;
          msg.step = static_cast<std::uint32_t>(image.step);
          msg.data.assign(image.data, image.data + image.total() * image.elemSize());
          if (video_generation_.load() != worker_generation) {
            ++ros_image_skipped_generation_;
            continue;
          }
          ++ros_image_publish_attempts_;
          image_pub_->publish(msg);
          // A successful return means ROS accepted the publish call, not that DDS delivered it.
          ros_image_published_at_ = monotonic();
          ++ros_image_published_;
          if (camera_info_ && video_generation_.load() == worker_generation) {
            auto info = *camera_info_;
            info.header = msg.header;
            camera_info_pub_->publish(info);
          }
        }
        if (raw && raw != jpeg_source && !peers.empty() && now >= next_encode) {
          if (image.empty() || raw != image_source) {
            image = yuv422_to_bgr(*raw, video_pixel_format_);
            image_source = raw;
          }
          // Keep ROS images pristine for vision; annotate only the browser JPEG.
          auto display_image = route_overlay_.enabled() ? image.clone() : image;
          route_overlay_.draw(display_image, now, frame_at_.load());
          Bytes jpeg;
          if (cv::imencode(".jpg", display_image, jpeg,
                           {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
            if (video_generation_.load() != worker_generation) continue;
            auto packets = jpeg_packets(++frame_id, jpeg);
            if (!packets.empty()) {
              for (const auto& packet : packets) {
                if (video_generation_.load() != worker_generation) break;
                for (const auto& peer : peers)
                  if (!pc_.send(packet, peer)) ++send_errors_;
              }
              ++jpeg_count_;
            }
          }
          jpeg_source = raw;
          next_encode = now + 1.0 / video_fps_;
        }
      }
    } catch (const std::exception& e) {
      video_error_ = std::current_exception();
      RCLCPP_ERROR(node_->get_logger(), "video worker: %s", e.what());
      stop_ = true;
    }
  }

  bool send_and_wait_stop(std::uint8_t type, bool enabled) {
    for (int attempt = 0; attempt < 3; ++attempt) {
      auto packet = make_request(type, {}, enabled);
      if (!fpga_.send(packet, fpga_peer_)) { ++send_errors_; continue; }
      const auto deadline = monotonic() + .1;
      while (monotonic() < deadline) {
        pollfd descriptor{fpga_.fd, POLLIN, 0};
        const auto remain = static_cast<int>(std::max(1.0, (deadline - monotonic()) * 1000));
        if (poll(&descriptor, 1, remain) <= 0) break;
        auto datagram = fpga_.receive();
        if (datagram && same_peer(datagram->peer, fpga_peer_) && valid_ack(datagram->data, packet)) return true;
      }
    }
    return false;
  }

  void stop_fpga()
  {
    if (send_and_wait_stop(3, false))
    {
      if (send_and_wait_stop(5, false)) send_and_wait_stop(3, false);
    }
  }

 public:
  Bridge(Args args, std::shared_ptr<rclcpp::Node> node, double video_fps, int jpeg_quality)
      : args_(std::move(args)), video_fps_(video_fps),
        jpeg_quality_(jpeg_quality), node_(std::move(node)),
        pc_(args_.get("--bind-pc"), args_.integer("--pc-port")),
        fpga_(args_.get("--bind-fpga"), args_.integer("--ack-port")),
        video_(args_.get("--bind-fpga"), args_.integer("--video-port"), 8 * 1024 * 1024),
        telemetry_(args_.get("--bind-fpga"), args_.integer("--telemetry-port"), 1024 * 1024),
        fpga_peer_(endpoint(args_.get("--fpga-ip"), args_.integer("--fpga-port"))),
        imu_frame_id_(node_->get_parameter("imu_frame_id").as_string()),
        n10_frame_id_(node_->get_parameter("n10_frame_id").as_string()),
        camera_frame_id_(node_->get_parameter("camera_frame_id").as_string()) {
    ros_control_enabled_ = node_->get_parameter("ros_control_enabled").as_bool();
    cmd_vel_timeout_s_ = node_->get_parameter("cmd_vel_timeout_s").as_double();
    video_pixel_format_ = node_->get_parameter("video_pixel_format").as_string();
    odom_frame_id_ = node_->get_parameter("odom_frame_id").as_string();
    base_frame_id_ = node_->get_parameter("base_frame_id").as_string();
    lidar_x_m_ = node_->get_parameter("lidar_x_m").as_double();
    lidar_y_m_ = node_->get_parameter("lidar_y_m").as_double();
    lidar_yaw_rad_ = node_->get_parameter("lidar_yaw_rad").as_double();
    imu_x_m_ = node_->get_parameter("imu_x_m").as_double();
    imu_y_m_ = node_->get_parameter("imu_y_m").as_double();
    imu_z_m_ = node_->get_parameter("imu_z_m").as_double();
    imu_roll_rad_ = node_->get_parameter("imu_roll_rad").as_double();
    imu_pitch_rad_ = node_->get_parameter("imu_pitch_rad").as_double();
    imu_yaw_rad_ = node_->get_parameter("imu_yaw_rad").as_double();
    scan_range_min_m_ = node_->get_parameter("scan_range_min_m").as_double();
    scan_range_max_m_ = node_->get_parameter("scan_range_max_m").as_double();
    navigation_default_speed_mm_s_ = node_->get_parameter("navigation_default_speed_mm_s").as_int();
    navigation_default_accel_mm_s2_ = node_->get_parameter("navigation_default_accel_mm_s2").as_int();
    navigation_default_tolerance_mm_ = node_->get_parameter("navigation_default_tolerance_mm").as_int();
    navigation_default_yaw_tolerance_mrad_ = node_->get_parameter("navigation_default_yaw_tolerance_mrad").as_int();
    navigation_default_duration_s_ = node_->get_parameter("navigation_default_duration_s").as_double();
    if (!std::isfinite(cmd_vel_timeout_s_) || cmd_vel_timeout_s_ <= 0 || cmd_vel_timeout_s_ > .2)
      throw std::invalid_argument("cmd_vel_timeout_s must be within (0, 0.2]");
    if (video_pixel_format_ != "yuyv" && video_pixel_format_ != "uyvy")
      throw std::invalid_argument("video_pixel_format must be yuyv or uyvy");
    if (odom_frame_id_.empty() || base_frame_id_.empty() || n10_frame_id_.empty() ||
        imu_frame_id_.empty() || odom_frame_id_ == base_frame_id_ ||
        odom_frame_id_ == imu_frame_id_ ||
        base_frame_id_ == n10_frame_id_ || base_frame_id_ == imu_frame_id_ ||
        n10_frame_id_ == imu_frame_id_ ||
        !std::isfinite(lidar_x_m_) || !std::isfinite(lidar_y_m_) ||
        !std::isfinite(lidar_yaw_rad_) ||
        !std::isfinite(imu_x_m_) || !std::isfinite(imu_y_m_) || !std::isfinite(imu_z_m_) ||
        !std::isfinite(imu_roll_rad_) || !std::isfinite(imu_pitch_rad_) ||
        !std::isfinite(imu_yaw_rad_) ||
        !std::isfinite(scan_range_min_m_) || !std::isfinite(scan_range_max_m_) ||
        scan_range_min_m_ < 0 || scan_range_max_m_ <= scan_range_min_m_ ||
        scan_range_max_m_ > 65.534 || navigation_default_speed_mm_s_ < 1 ||
        navigation_default_speed_mm_s_ > 400 || navigation_default_accel_mm_s2_ < 1 ||
        navigation_default_accel_mm_s2_ > 1000 || navigation_default_tolerance_mm_ < 1 ||
        navigation_default_tolerance_mm_ > 500 || navigation_default_yaw_tolerance_mrad_ < 1 ||
        navigation_default_yaw_tolerance_mrad_ > 1000 ||
        !std::isfinite(navigation_default_duration_s_) || navigation_default_duration_s_ <= 0 ||
        navigation_default_duration_s_ > 120)
      throw std::invalid_argument("invalid odom, scan or local navigation parameters");

    std::random_device source;
    std::mt19937 generator(source());
    std::ostringstream bridge_id;
    bridge_id << std::hex << std::setfill('0');
    for (int i = 0; i < 4; ++i) bridge_id << std::setw(8) << generator();
    bridge_id_ = bridge_id.str();

    if (!args_.get("--pc-ip").empty()) pc_filter_ = endpoint(args_.get("--pc-ip"), 1);

    const auto qos = rclcpp::SensorDataQoS();
    auto image_qos = rclcpp::SensorDataQoS();
    image_qos.keep_last(1);
    for (const auto* name : {"imu", "motor", "lidar", "fusion", "health", "navigation", "local_plan", "local_grid"}) {
      raw_pubs_[name] = node_->create_publisher<std_msgs::msg::String>(
          std::string("/fpga/round5/") + name, qos);
      received_[name] = 0;
    }
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", qos);
    imu_raw_pub_ = node_->create_publisher<ugv_bridge::msg::Bx72ImuRaw>("/fpga/imu_raw", qos);
    n10_pub_ = node_->create_publisher<ugv_bridge::msg::N10Packet>("/fpga/n10_packets", qos);
    encoder_pub_ = node_->create_publisher<ugv_bridge::msg::EncoderCounts>("/fpga/encoder_counts", qos);
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", image_qos);
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("/scan", qos);
    wheel_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom/wheel", qos);
    fusion_odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom/filtered", qos);
    navigation_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/fpga/navigation_state", rclcpp::QoS(10).transient_local());
    executor_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/navigation/executor/state", qos);
    executor_grid_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/navigation/executor/local_grid", qos);
    local_plan_pub_ = node_->create_publisher<std_msgs::msg::String>("/fpga/local_plan", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    n10_static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);
    auto lidar_transform = ros_navigation::n10_sensor_transform(
        base_frame_id_, n10_frame_id_, lidar_x_m_, lidar_y_m_, lidar_yaw_rad_);
    lidar_transform.header.stamp = node_->get_clock()->now();
    n10_static_tf_broadcaster_->sendTransform(lidar_transform);
    geometry_msgs::msg::TransformStamped imu_transform;
    imu_transform.header.stamp = node_->get_clock()->now();
    imu_transform.header.frame_id = base_frame_id_;
    imu_transform.child_frame_id = imu_frame_id_;
    imu_transform.transform.translation.x = imu_x_m_;
    imu_transform.transform.translation.y = imu_y_m_;
    imu_transform.transform.translation.z = imu_z_m_;
    const double cr = std::cos(imu_roll_rad_ * .5);
    const double sr = std::sin(imu_roll_rad_ * .5);
    const double cp = std::cos(imu_pitch_rad_ * .5);
    const double sp = std::sin(imu_pitch_rad_ * .5);
    const double cy = std::cos(imu_yaw_rad_ * .5);
    const double sy = std::sin(imu_yaw_rad_ * .5);
    imu_transform.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    imu_transform.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    imu_transform.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    imu_transform.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    n10_static_tf_broadcaster_->sendTransform(imu_transform);
    sensor_status_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/fpga/sensor_status", 10);
    bidirectional_status_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/fpga/bidirectional_status", 10);
    bridge_safety_state_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/ugv_bridge/safety_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publish_bridge_safety_state();
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
          receive_ros_command(*message);
        });
    mission_cmd_vel_sub_ = node_->create_subscription<ugv_bridge::msg::AuthorizedTwist>(
        "/mission/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
        [this](ugv_bridge::msg::AuthorizedTwist::ConstSharedPtr message) {
          receive_mission_command(*message);
        });
    navigation_cmd_vel_sub_ = node_->create_subscription<ugv_bridge::msg::AuthorizedTwist>(
        "/jetson/navigation/cmd_vel", rclcpp::QoS(10),
        [this](ugv_bridge::msg::AuthorizedTwist::ConstSharedPtr message) {
          receive_navigation_command(*message);
        });
    jetson_navigation_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/jetson/navigation/state", rclcpp::QoS(10),
        [this](std_msgs::msg::String::ConstSharedPtr message) { receive_jetson_navigation(*message); });
    jetson_grid_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/jetson/navigation/local_grid", rclcpp::QoS(10),
        [this](std_msgs::msg::String::ConstSharedPtr message) { receive_jetson_grid(*message); });
    jetson_plan_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/jetson/navigation/local_plan", rclcpp::QoS(10),
        [this](std_msgs::msg::String::ConstSharedPtr message) { receive_jetson_plan(*message); });
    ros_control_service_ = node_->create_service<std_srvs::srv::SetBool>(
        "/control/enable_ros",
        [this](std_srvs::srv::SetBool::Request::SharedPtr request,
               std_srvs::srv::SetBool::Response::SharedPtr response) {
          set_ros_authorization(std::move(request), std::move(response));
        });
    emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", 10,
        [this](std_msgs::msg::Bool::ConstSharedPtr message, const rclcpp::MessageInfo& info) {
          bool own = false;
          if (emergency_stop_pub_ &&
              rmw_compare_gids_equal(&info.get_rmw_message_info().publisher_gid,
                                     &emergency_stop_pub_->get_gid(), &own) == RMW_RET_OK && own) return;
          set_emergency_stop(message->data);
        });
    emergency_stop_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/emergency_stop", 10);
    mission_start_ = node_->create_client<ugv_bridge::srv::MissionStart>("/mission/start_configured");
    mission_stop_ = node_->create_client<std_srvs::srv::Trigger>("/mission/stop");
    navigation_client_ = node_->create_client<ugv_bridge::srv::NavigationCommand>("/jetson/navigation/command");
    global_navigation_client_ = node_->create_client<ugv_bridge::srv::GlobalNavigation>(
        "/jetson/global_navigation/command");
    executor_service_ = node_->create_service<ugv_bridge::srv::NavigationCommand>(
        "/navigation/executor/command",
        [this](std::shared_ptr<rmw_request_id_t> header,
               std::shared_ptr<ugv_bridge::srv::NavigationCommand::Request> request) {
          receive_executor_request(std::move(header), request);
        });
    global_navigation_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/jetson/global_navigation/state", rclcpp::QoS(10),
        [this](std_msgs::msg::String::ConstSharedPtr message) { receive_global_state(*message); });
    web_detections_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/vision/detections", rclcpp::SensorDataQoS().keep_last(1),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr message) {
          web_detections_ = message; web_detections_at_ = monotonic();
        });
    web_selected_sub_ = node_->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/mission/selected_target", rclcpp::QoS(1),
        [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr message) {
          web_selected_ = message; web_selected_at_ = monotonic();
        });
    mission_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/mission/state", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr message) {
          const auto& state = message->data;
          mission_state_ = state == "IDLE" || state == "TRACKING" || state == "LOST" ||
              state == "STOPPED" || state == "SEARCHING" || state == "FOLLOWING" ||
              state == "APPROACHING" || state == "FOUND" || state == "ARRIVED" || state == "BLOCKED" ? state : "UNKNOWN";
        });
    try {
      const auto width = node_->get_parameter("camera_width").as_int();
      const auto height = node_->get_parameter("camera_height").as_int();
      const auto k = node_->get_parameter("camera_matrix").as_double_array();
      const auto d = node_->get_parameter("distortion_coefficients").as_double_array();
      const auto r = node_->get_parameter("rectification_matrix").as_double_array();
      const auto p = node_->get_parameter("projection_matrix").as_double_array();
      const auto model = node_->get_parameter("distortion_model").as_string();
      const bool calibration_provided = width != 0 || height != 0 || !k.empty() || !d.empty() ||
          !r.empty() || !p.empty() || !model.empty();
      if (calibration_provided) {
        if (width != video_width || height != video_height || k.size() != 9 || d.empty() || r.size() != 9 ||
            p.size() != 12 || model.empty() || camera_frame_id_.empty())
          throw std::invalid_argument(
              "camera calibration needs the BX72 640x360 mode, K[9], D, R[9], P[12], model and frame_id");
        bool finite = true;
        for (const auto value : k) finite = finite && std::isfinite(value);
        for (const auto value : d) finite = finite && std::isfinite(value);
        for (const auto value : r) finite = finite && std::isfinite(value);
        for (const auto value : p) finite = finite && std::isfinite(value);
        if (!finite || k[0] <= 0 || k[4] <= 0)
          throw std::invalid_argument(
              "camera calibration matrices must be finite with positive focal lengths");

        sensor_msgs::msg::CameraInfo info;
        info.header.frame_id = camera_frame_id_;
        info.width = static_cast<std::uint32_t>(width);
        info.height = static_cast<std::uint32_t>(height);
        info.distortion_model = model;
        info.d = d;
        std::copy(k.begin(), k.end(), info.k.begin());
        std::copy(r.begin(), r.end(), info.r.begin());
        std::copy(p.begin(), p.end(), info.p.begin());
        camera_info_ = std::move(info);
      }
      if (camera_info_)
        camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", image_qos);
    } catch (const std::invalid_argument& e) {
      RCLCPP_WARN(node_->get_logger(), "CameraInfo disabled: %s", e.what());
    }
    if (node_->get_parameter("route_overlay_enabled").as_bool()) {
      if (!camera_info_ || camera_info_->distortion_model != "plumb_bob")
        throw std::invalid_argument("route overlay requires calibrated CameraInfo with plumb_bob D[5]");
      route_overlay_.configure(camera_info_->k, camera_info_->d,
          node_->get_parameter("route_overlay_camera_from_base").as_double_array());
    }
  }

  void run() {
    std::thread worker(&Bridge::video_worker, this);
    std::exception_ptr error;
    double next_send = 0;
    double next_sensor_status = 0;
    double next_bidirectional_status = 0;
    double next_pc_status = 0;
    RCLCPP_INFO(node_->get_logger(), "PC UDP :%d; FPGA %s; bridge_id=%s",
                args_.integer("--pc-port"), peer_key(fpga_peer_).c_str(), bridge_id_.c_str());
    try {
      while (rclcpp::ok() && !stop_.load()) {
        poll_sockets();
        const auto now = monotonic();
        lease_.expire(now);
        if (control_source_ == ControlSource::Pc && !lease_.armed) {
          control_source_ = ControlSource::None;
          reset_control_path();
          publish_bridge_safety_state();
        }
        if (control_source_ == ControlSource::Ros && ros_authorized_) {
          if (!web_control_session_.empty() && (web_control_session_ != lease_.session ||
              web_control_peer_ != lease_.peer || now - lease_.last > .5)) revoke_ros_authorization();
          const auto command_at = ros_command_received_ ? ros_command_at_ : ros_authorized_at_;
          if (control_source_ == ControlSource::Ros && now - command_at >= cmd_vel_timeout_s_) {
            revoke_ros_authorization();
            send_request(3, {}, false, now, "ros-command-expired");
            next_send = now + .05;
          }
        }
        if (navigation_backend_ == NavigationBackend::Jetson &&
            control_source_ == ControlSource::Navigation && ros_authorized_ && navigation_task_active_) {
          const bool cooperative_lost = cooperative_navigation_owner_lost(lease_, navigation_owner_is_ros_,
              navigation_mode_, navigation_task_flags_known_, navigation_flags_,
              navigation_owner_session_, navigation_owner_peer_, now);
          const auto command_at = ros_command_received_ ? ros_command_at_ : ros_authorized_at_;
          const bool awaiting_start = navigation_transfer_active_ && navigation_action_ == "local_start";
          const bool command_lost = (!awaiting_start || ros_command_received_) &&
              now - command_at >= cmd_vel_timeout_s_;
          const bool state_lost = !awaiting_start &&
              now - std::max(jetson_navigation_received_at_, ros_authorized_at_) > .75;
          if (cooperative_lost || command_lost || state_lost) begin_navigation_stop(now);
        }
        // A lost probe ACK must not switch an FPGA-owned task to the manual
        // velocity stream. Retry liveness while telemetry remains authoritative.
        for (auto it = outstanding_.begin(); it != outstanding_.end();) {
          if (it->source == "navigation-heartbeat" && now - it->sent_at > .1) {
            ++timeouts_;
            it = outstanding_.erase(it);
          } else ++it;
        }
        const auto expired = std::find_if(outstanding_.begin(), outstanding_.end(),
            [&](const Pending& pending) {
              return pending.source != "navigation-task" && now - pending.sent_at > .1;
            });
        if (expired != outstanding_.end()) {
          fail_safe();
        }
        if (now - ack_at_ > .2) ready_ = false;

        if (navigation_stop_pending_ && navigation_stop_zero_ack_at_ > -1e50 && navigation_stop_ack_at_ > -1e50 &&
            navigation_status_fresh(now) &&
            navigation_received_at_ > std::max(navigation_stop_zero_ack_at_, navigation_stop_ack_at_) &&
            navigation_context_.is_object() && !navigation_context_.value("active_task", true) &&
            navigation_context_.value("vx_mm_s", 1) == 0 && navigation_context_.value("vy_mm_s", 1) == 0 &&
            navigation_context_.value("wz_mrad_s", 1) == 0)
          navigation_stop_pending_ = false;

        if (navigation_stop_pending_ && !(navigation_backend_ == NavigationBackend::Fpga &&
                                          navigation_transfer_active_)) {
          if (outstanding_.empty() && now >= next_navigation_stop_at_) {
            // Retry only unconfirmed fences. If the newer request reached
            // the FPGA first, retrying the older kind with a new sequence
            // permits recovery even when the original was discarded.
            if (navigation_stop_zero_ack_at_ < -1e50)
              send_request(3, {}, false, now, "navigation-stop-fence");
            if (navigation_stop_ack_at_ < -1e50)
              send_request(5, {}, false, now, "navigation-stop-fence");
            next_navigation_stop_at_ = now + .1;
          }
        } else if (navigation_backend_ == NavigationBackend::Fpga && navigation_transfer_active_) {
          // The fragment pump runs after ROS callbacks and serializes against all other traffic.
        } else if (navigation_backend_ == NavigationBackend::Fpga && navigation_task_active_) {
          const bool owner_lost = cooperative_navigation_owner_lost(lease_, navigation_owner_is_ros_,
              navigation_mode_, navigation_task_flags_known_, navigation_flags_,
              navigation_owner_session_, navigation_owner_peer_, now);
          if (owner_lost) begin_navigation_stop(now);
          else if (navigation_session_token_ && outstanding_.empty() && now >= next_navigation_probe_at_) {
            send_navigation_probe(now);
            next_navigation_probe_at_ = now + .075;
          }
        } else if (!navigation_state_known_) {
          // 首次导航状态到达前只被动监听，不自动发type3/type5改写硬件会话。
        } else if (navigation_boot_id_ == 0) {
            // 相位扫描期间初始请求可能丢失；只在新鲜、无任务的初始域重试零速。
            if (navigation_status_fresh(now) && now >= next_send &&
              !navigation_telemetry_rejected_ && navigation_context_.is_object() &&
              supported_navigation_protocol(navigation_context_.value("protocol", std::string{})) &&
              !navigation_context_.value("deadline_known", true) &&
              !navigation_context_.value("active_task", true) &&
              navigation_status_ == 0 &&
              !navigation_task_active_ &&
              outstanding_.empty()) {
            navigation_bootstrap_sent_ = true;
            send_request(3, {}, false, now, "navigation-session-bootstrap");
            next_send = now + .05;
          }
        } else {
          const bool desired_auto = false;
          if (!mode_target_ || *mode_target_ != desired_auto) {
            outstanding_.clear();
            last_ack_ = nullptr;
            mode_target_ = desired_auto;
            zero_barrier_ack_ = false;
            mode_command_required_ = true;
            auto_acknowledged_ = false;
          }
          if (now >= next_send) {
            if (!zero_barrier_ack_)
              send_request(3, {}, false, now, request_source());
            else if (mode_command_required_ || !fpga_auto_ || *fpga_auto_ != desired_auto)
              send_request(5, {}, desired_auto, now, request_source());
            else {
              Velocity velocity{};
              if (ready_ && !emergency_stop_ &&
                  (control_source_ == ControlSource::Ros || control_source_ == ControlSource::Navigation) &&
                  ros_authorized_ && ros_command_received_)
                velocity = ros_velocity_;
              else if (ready_ && !emergency_stop_ && control_source_ == ControlSource::Pc &&
                       lease_.armed)
                velocity = lease_.velocity;
              send_request(3, velocity, false, now, request_source());
            }
            next_send = now + .05;
          }
        }
        if (observers_.expire(now)) refresh_video_peers();
        if (now >= next_sensor_status) {
          publish_sensor_status();
          next_sensor_status = now + 1.0;
        }
        if (now >= next_bidirectional_status) {
          publish_bidirectional_status(now);
          next_bidirectional_status = now + .1;
        }
        if (now >= next_pc_status) {
          send_pc_status(now);
          send_vision(now);
          next_pc_status = now + .1;
        }
        // SIGINT可在循环条件之后关闭context；只将这类预期退出作为正常清理。
        if (!rclcpp::ok()) break;
        try {
          rclcpp::spin_some(node_);
        } catch (const rclcpp::exceptions::RCLError&) {
          if (rclcpp::ok()) throw;
          break;
        }
        poll_navigation_sync(monotonic());
        poll_navigation_transfer(monotonic());
        poll_web_result(monotonic());
        poll_global_results(monotonic());
        poll_global_cancel(monotonic());
      }
    } catch (...) {
      error = std::current_exception();
    }
    stop_ = true;
    worker.join();
    discard_deferred_navigation("网桥已停止，导航启动未发送");
    cancel_jetson_navigation();
    stop_fpga();
    if (error) std::rethrow_exception(error);
    // join() makes the worker's exception visible to the calling thread.
    if (video_error_) std::rethrow_exception(video_error_);
  }
};

}  // namespace bx72

int main(int argc, char** argv) {
  int result = 0;
  try {
    rclcpp::init(argc, argv);
    auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char*> bridge_argv;
    bridge_argv.reserve(arguments.size());
    for (auto& argument : arguments) bridge_argv.push_back(argument.data());

    bx72::Args args(static_cast<int>(bridge_argv.size()), bridge_argv.data(),
                    {{"--bind-pc", "0.0.0.0"}, {"--pc-ip", ""}, {"--pc-port", "6200"},
                     {"--bind-fpga", "192.168.0.4"}, {"--fpga-ip", "192.168.0.2"},
                     {"--fpga-port", "5000"}, {"--ack-port", "6103"},
                     {"--video-port", "6102"}, {"--telemetry-port", "6104"},
                     {"--video-fps", "10"}, {"--jpeg-quality", "60"}},
                    "BX72 ROS/PC/FPGA UDP gateway");

    auto node = std::make_shared<rclcpp::Node>("ugv_bridge_node");
    node->declare_parameter<std::string>("bind_pc", args.get("--bind-pc"));
    node->declare_parameter<std::string>("pc_ip", args.get("--pc-ip"));
    node->declare_parameter<int>("pc_port", args.integer("--pc-port"));
    node->declare_parameter<std::string>("bind_fpga", args.get("--bind-fpga"));
    node->declare_parameter<std::string>("fpga_ip", args.get("--fpga-ip"));
    node->declare_parameter<int>("fpga_port", args.integer("--fpga-port"));
    node->declare_parameter<int>("ack_port", args.integer("--ack-port"));
    node->declare_parameter<int>("video_port", args.integer("--video-port"));
    node->declare_parameter<int>("telemetry_port", args.integer("--telemetry-port"));
    node->declare_parameter<double>("video_fps", args.number("--video-fps"));
    node->declare_parameter<int>("jpeg_quality", args.integer("--jpeg-quality"));
    node->declare_parameter<bool>("ros_control_enabled", false);
    node->declare_parameter<double>("cmd_vel_timeout_s", .2);
    node->declare_parameter<std::string>("video_pixel_format", "yuyv");
    node->declare_parameter<int>("camera_width", 0);
    node->declare_parameter<int>("camera_height", 0);
    node->declare_parameter<std::vector<double>>("camera_matrix", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("distortion_coefficients", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("rectification_matrix", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("projection_matrix", std::vector<double>{});
    node->declare_parameter<std::string>("distortion_model", "");
    node->declare_parameter<bool>("route_overlay_enabled", false);
    node->declare_parameter<std::vector<double>>("route_overlay_camera_from_base", std::vector<double>{});
    node->declare_parameter<std::string>("imu_frame_id", "imu660rc_sensor");
    node->declare_parameter<std::string>("n10_frame_id", "n10_sensor");
    node->declare_parameter<std::string>("camera_frame_id", "camera");
    node->declare_parameter<std::string>("odom_frame_id", "odom");
    node->declare_parameter<std::string>("base_frame_id", "base_link");
    node->declare_parameter<double>("lidar_x_m", ugv_bridge::ros_navigation::kDefaultN10OffsetXMetres);
    node->declare_parameter<double>("lidar_y_m", 0.0);
    node->declare_parameter<double>("lidar_yaw_rad", 0.0);
    node->declare_parameter<double>("imu_x_m", 0.0);
    node->declare_parameter<double>("imu_y_m", 0.0);
    node->declare_parameter<double>("imu_z_m", 0.0);
    node->declare_parameter<double>("imu_roll_rad", 3.14159265358979323846);
    node->declare_parameter<double>("imu_pitch_rad", 0.0);
    node->declare_parameter<double>("imu_yaw_rad", 0.0);
    node->declare_parameter<double>("scan_range_min_m", 0.01);
    node->declare_parameter<double>("scan_range_max_m", 65.534);
    node->declare_parameter<int>("navigation_default_speed_mm_s", 400);
    node->declare_parameter<int>("navigation_default_accel_mm_s2", 200);
    node->declare_parameter<int>("navigation_default_tolerance_mm", 100);
    node->declare_parameter<int>("navigation_default_yaw_tolerance_mrad", 100);
    node->declare_parameter<double>("navigation_default_duration_s", 120.0);

    const std::array<std::pair<const char*, const char*>, 11> options{{
        {"--bind-pc", "bind_pc"}, {"--pc-ip", "pc_ip"}, {"--pc-port", "pc_port"},
        {"--bind-fpga", "bind_fpga"}, {"--fpga-ip", "fpga_ip"}, {"--fpga-port", "fpga_port"},
        {"--ack-port", "ack_port"}, {"--video-port", "video_port"},
        {"--telemetry-port", "telemetry_port"}, {"--video-fps", "video_fps"},
        {"--jpeg-quality", "jpeg_quality"}}};
    for (const auto& option : options) {
      bool cli_specified = false;
      for (int i = 1; i < static_cast<int>(bridge_argv.size()); ++i) {
        const std::string arg(bridge_argv[i]);
        if (arg == option.first || arg.rfind(std::string(option.first) + "=", 0) == 0) {
          cli_specified = true;
          break;
        }
      }
      if (cli_specified) continue;
      const auto parameter = node->get_parameter(option.second);
      const std::string key(option.first);
      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
        args.values[key] = parameter.as_string();
      else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        args.values[key] = std::to_string(parameter.as_double());
      else
        args.values[key] = std::to_string(parameter.as_int());
    }
    const double video_fps = args.number("--video-fps");
    const int jpeg_quality = args.integer("--jpeg-quality");
    if (video_fps < 1 || video_fps > 30 || jpeg_quality < 20 || jpeg_quality > 90)
      throw std::invalid_argument("video-fps: 1..30; jpeg-quality: 20..90");

    bx72::Bridge(std::move(args), std::move(node), video_fps, jpeg_quality).run();
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    result = 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
