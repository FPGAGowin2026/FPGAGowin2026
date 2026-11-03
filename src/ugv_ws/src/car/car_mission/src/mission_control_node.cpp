#include <algorithm>
#include <chrono>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <functional>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "ugv_bridge/srv/mission_start.hpp"
#include "ugv_bridge/msg/authorized_twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace car_mission {
namespace {

constexpr char kIdle[] = "IDLE";
constexpr char kTracking[] = "TRACKING";
constexpr char kLost[] = "LOST";
constexpr char kStopped[] = "STOPPED";
constexpr double kDefaultTimeout = 0.25;
constexpr double kMaxSearchAngularSpeed = 0.8;
constexpr double kAuthorizationTimeout = 0.35;
constexpr double kAuthorizationWait = 0.75;
constexpr unsigned int kConsecutiveMissLimit = 5;
using SteadyClock = std::chrono::steady_clock;
using Detection = vision_msgs::msg::Detection2D;
using DetectionArray = vision_msgs::msg::Detection2DArray;

struct Authorization
{
  std::string bridge_id;
  std::uint64_t epoch{};
  bool authorized{};
  bool emergency_stop{};
  std::string control_source;
};

std::optional<Authorization> parse_authorization(const diagnostic_msgs::msg::DiagnosticArray & array)
{
  std::optional<Authorization> result;
  for (const auto & status : array.status) {
    if (status.name != "bx72_udp") continue;
    if (result) return std::nullopt;
    std::optional<std::string> bridge, epoch, authorized, emergency, source;
    for (const auto & value : status.values) {
      std::optional<std::string> * field = nullptr;
      if (value.key == "bridge_id") field = &bridge;
      else if (value.key == "ros_authorization_epoch") field = &epoch;
      else if (value.key == "ros_authorized") field = &authorized;
      else if (value.key == "emergency_stop") field = &emergency;
      else if (value.key == "control_source") field = &source;
      if (field) {
        if (*field) return std::nullopt;
        *field = value.value;
      }
    }
    if (!bridge || bridge->empty() || bridge->size() > 128 || !epoch || epoch->empty() ||
      !authorized || (*authorized != "True" && *authorized != "False") ||
      !emergency || (*emergency != "True" && *emergency != "False") || !source ||
      (*source != "ros" && *source != "none" && *source != "pc" && *source != "emergency_stop"))
      return std::nullopt;
    std::uint64_t parsed_epoch = 0;
    const auto parsed = std::from_chars(epoch->data(), epoch->data() + epoch->size(), parsed_epoch);
    if (parsed.ec != std::errc{} || parsed.ptr != epoch->data() + epoch->size()) return std::nullopt;
    result = Authorization{*bridge, parsed_epoch, *authorized == "True", *emergency == "True", *source};
    if (result->authorized && (result->epoch == 0 || result->control_source != "ros" || result->emergency_stop))
      return std::nullopt;
  }
  return result;
}

struct TargetConfig
{
  std::string class_id;
  std::string target_id;
  double min_confidence{};
  double timeout_s{};
};

struct MotionConfig
{
  bool enabled{false};
  double image_width{640.0};
  double image_height{360.0};
  double angular_gain{1.2};
  double angular_deadband{0.05};
  double max_alignment_angular_speed{0.3};
};

bool newer_stamp(
  const std_msgs::msg::Header & current, const std_msgs::msg::Header & previous)
{
  return current.stamp.sec > previous.stamp.sec ||
         (current.stamp.sec == previous.stamp.sec &&
         current.stamp.nanosec > previous.stamp.nanosec);
}

const char * header_failure_reason(
  const std_msgs::msg::Header & header, const rclcpp::Time & now, double timeout_s)
{
  const auto & stamp = header.stamp;
  if (header.frame_id.empty()) return "empty_frame_id";
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) return "invalid_stamp";
  if (stamp.sec == 0 && stamp.nanosec == 0U) return "zero_stamp";
  const rclcpp::Time source_time(header.stamp, now.get_clock_type());
  const double age_s = (now - source_time).seconds();
  if (!std::isfinite(age_s)) return "invalid_source_age";
  if (age_s < 0.0) return "future_stamp";
  if (age_s > timeout_s) return "source_timeout";
  return nullptr;
}

bool fresh_header(
  const std_msgs::msg::Header & header, const rclcpp::Time & now, double timeout_s)
{
  return header_failure_reason(header, now, timeout_s) == nullptr;
}

const char * observation_failure_reason(
  bool has_source, double source_age_s, bool has_receipt, double receipt_age_s,
  double timeout_s)
{
  if (!has_source) return "no_source";
  if (!std::isfinite(source_age_s)) return "invalid_source_age";
  if (source_age_s < 0.0) return "future_source";
  if (source_age_s > timeout_s) return "source_timeout";
  if (!has_receipt) return "no_receipt";
  if (!std::isfinite(receipt_age_s) || receipt_age_s < 0.0) return "invalid_receipt_age";
  if (receipt_age_s > timeout_s) return "receipt_timeout";
  return nullptr;
}

bool valid_motion_config(const MotionConfig & config)
{
  return std::isfinite(config.image_width) && config.image_width >= 1.0 &&
         config.image_width <= 16384.0 && std::isfinite(config.image_height) &&
         config.image_height >= 1.0 && config.image_height <= 16384.0 &&
         std::isfinite(config.angular_gain) && config.angular_gain >= 0.0 &&
         config.angular_gain <= 10.0 && std::isfinite(config.angular_deadband) &&
         config.angular_deadband >= 0.0 && config.angular_deadband <= 1.0 &&
         std::isfinite(config.max_alignment_angular_speed) && config.max_alignment_angular_speed >= 0.0 &&
         config.max_alignment_angular_speed <= 0.3;
}

geometry_msgs::msg::Twist compute_servo_command(
  const Detection & detection, const MotionConfig & config)
{
  geometry_msgs::msg::Twist command;
  if (!config.enabled) {
    return command;
  }

  const double horizontal_error = std::clamp(
    (detection.bbox.center.position.x - config.image_width / 2.0) /
    (config.image_width / 2.0), -1.0, 1.0);
  const bool aligned = std::abs(horizontal_error) <= config.angular_deadband;
  if (!aligned) {
    command.angular.z = std::clamp(
      -config.angular_gain * horizontal_error,
      -config.max_alignment_angular_speed, config.max_alignment_angular_speed);
    return command;
  }

  // 单目检测框没有可靠距离：只允许视觉转向，不由框大小生成平移请求。
  return command;
}

std::optional<Detection> select_detection(
  const DetectionArray & array, const TargetConfig & config,
  const std::string & locked_id = "", const char ** miss_reason = nullptr)
{
  std::optional<Detection> best;
  double best_score = -1.0;
  bool saw_id = false;
  bool saw_valid_box = false;
  bool saw_class = false;
  bool saw_low_confidence = false;
  for (const auto & detection : array.detections) {
    if ((!config.target_id.empty() && detection.id != config.target_id) ||
      (!locked_id.empty() && detection.id != locked_id) || detection.id.empty())
    {
      continue;
    }
    saw_id = true;

    const auto & center = detection.bbox.center;
    if (!std::isfinite(center.position.x) || !std::isfinite(center.position.y) ||
      !std::isfinite(center.theta) || center.position.x < 0.0 ||
      center.position.y < 0.0 || !std::isfinite(detection.bbox.size_x) ||
      !std::isfinite(detection.bbox.size_y) || detection.bbox.size_x <= 0.0 ||
      detection.bbox.size_y <= 0.0)
    {
      continue;
    }
    saw_valid_box = true;

    double score = -1.0;
    for (const auto & result : detection.results) {
      const double candidate_score = result.hypothesis.score;
      if (result.hypothesis.class_id == config.class_id) {
        saw_class = true;
        if (std::isfinite(candidate_score) && candidate_score >= 0.0 &&
          candidate_score < config.min_confidence)
        {
          saw_low_confidence = true;
        }
      }
      if (result.hypothesis.class_id == config.class_id &&
        std::isfinite(candidate_score) && candidate_score >= config.min_confidence &&
        candidate_score <= 1.0)
      {
        score = std::max(score, candidate_score);
      }
    }
    if (score < 0.0) {
      continue;
    }

    if (!best || score > best_score || (score == best_score && detection.id < best->id))
    {
      best = detection;
      best_score = score;
    }
  }
  if (miss_reason && !best) {
    *miss_reason = !saw_id ? "target_id_missing" :
      !saw_valid_box ? "invalid_box" :
      !saw_class ? "class_mismatch" :
      saw_low_confidence ? "low_confidence" : "invalid_confidence";
  }
  return best;
}

std::optional<double> front_obstacle_distance(const sensor_msgs::msg::LaserScan & scan)
{
    double nearest = INFINITY;
    for (std::size_t i=0; i<scan.ranges.size(); ++i) {
      const double a = scan.angle_min + i * scan.angle_increment;
      const double angle = std::atan2(std::sin(a), std::cos(a));
      const double r = scan.ranges[i];
      if (std::abs(angle) <= 0.35 && std::isfinite(r) && r >= scan.range_min && r <= scan.range_max)
        nearest = std::min(nearest, r);
    }
    if (!std::isfinite(nearest)) return std::nullopt;
    return nearest;
}

}  // namespace

class MissionControlNode : public rclcpp::Node
{
 public:
  explicit MissionControlNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_control_node", options)
  {
    const bool autostart = declare_parameter<bool>("autostart", false);
    const auto detections_topic = declare_parameter<std::string>(
      "detections_topic", "/vision/detections");
    const auto selected_target_topic = declare_parameter<std::string>(
      "selected_target_topic", "/mission/selected_target");
    declare_parameter<std::string>("target_class_id", "0");
    declare_parameter<std::string>("target_id", "");
    declare_parameter<double>("min_confidence", 0.5);
    declare_parameter<double>("observation_timeout_s", kDefaultTimeout);
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    standoff_m_ = declare_parameter<double>("standoff_m", 0.5);
    linear_speed_m_s_ = declare_parameter<double>("linear_speed_m_s", 0.4);
    search_angular_speed_ = declare_parameter<double>("search_angular_speed", kMaxSearchAngularSpeed);
    scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.5);
    if (image_topic_.empty() || scan_topic_.empty() || !std::isfinite(standoff_m_) || standoff_m_ < 0.15 || standoff_m_ > 5.0 ||
        !std::isfinite(linear_speed_m_s_) || linear_speed_m_s_ <= 0 || linear_speed_m_s_ > 0.4 ||
        !std::isfinite(search_angular_speed_) || search_angular_speed_ <= 0 || search_angular_speed_ > kMaxSearchAngularSpeed ||
        !std::isfinite(scan_timeout_s_) || scan_timeout_s_ <= 0 || scan_timeout_s_ > 1.0)
      throw std::invalid_argument("任务测距或速度参数无效");
    motion_config_.enabled = declare_parameter<bool>("motion_enabled", false);
    motion_config_.image_width = static_cast<double>(
      declare_parameter<std::int64_t>("image_width", 640));
    motion_config_.image_height = static_cast<double>(
      declare_parameter<std::int64_t>("image_height", 360));
    motion_config_.angular_gain = declare_parameter<double>("angular_gain", 1.2);
    motion_config_.angular_deadband = declare_parameter<double>("angular_deadband", 0.05);
    // Existing parameter name is retained; this limit applies only to target alignment.
    motion_config_.max_alignment_angular_speed = declare_parameter<double>("max_angular_speed", 0.3);

    if (detections_topic.empty() || selected_target_topic.empty()) {
      throw std::invalid_argument("mission topic 参数不能为空");
    }
    if (!valid_motion_config(motion_config_)) {
      throw std::invalid_argument("motion 参数超出有效范围");
    }

    cmd_vel_publisher_ = create_publisher<ugv_bridge::msg::AuthorizedTwist>(
      "/mission/cmd_vel", rclcpp::QoS(1));
    selected_target_publisher_ = create_publisher<DetectionArray>(
      selected_target_topic, rclcpp::QoS(10));
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/mission/state", rclcpp::QoS(1).transient_local());
    detections_subscription_ = create_subscription<DetectionArray>(
      detections_topic, rclcpp::SensorDataQoS(),
      std::bind(&MissionControlNode::on_detections, this, std::placeholders::_1));
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        images_.push_back(msg); if (images_.size() > 16) images_.pop_front();
      });
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        scan_ = msg; scan_receipt_ = SteadyClock::now();
      });
    configured_start_service_ = create_service<ugv_bridge::srv::MissionStart>(
      "/mission/start_configured", std::bind(&MissionControlNode::on_start_configured, this,
        std::placeholders::_1, std::placeholders::_2));
    emergency_stop_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(10),
      std::bind(&MissionControlNode::on_emergency_stop, this, std::placeholders::_1));
    authorization_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/fpga/bidirectional_status", rclcpp::QoS(10),
      std::bind(&MissionControlNode::on_authorization_status, this, std::placeholders::_1));
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/mission/start",
      std::bind(
        &MissionControlNode::on_start, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/mission/stop",
      std::bind(
        &MissionControlNode::on_stop, this,
        std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&MissionControlNode::on_timer, this));

    last_ros_time_ = now();
    publish_state(kIdle);
    publish_empty_at_now(*last_ros_time_);
    publish_zero_velocity();
    if (autostart) {
      RCLCPP_WARN(get_logger(), "autostart=true 已停用；请等待新鲜目标后调用 /mission/start");
    }
  }

  void stop()
  {
    stop_mission();
  }

  void publish_zero_velocity()
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    publish_command_locked(geometry_msgs::msg::Twist{});
  }

  void prepare_shutdown()
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    shutting_down_ = true;
    publish_command_locked(geometry_msgs::msg::Twist{});
  }

 private:
  void publish_command_locked(const geometry_msgs::msg::Twist & command)
  {
    ugv_bridge::msg::AuthorizedTwist message;
    message.header.stamp = now();
    message.header.frame_id = "base_link";
    if (command_authorization_) {
      message.bridge_id = command_authorization_->bridge_id;
      message.authorization_epoch = command_authorization_->epoch;
    }
    message.twist = command;
    cmd_vel_publisher_->publish(message);
  }

  void publish_velocity(const geometry_msgs::msg::Twist & command)
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    publish_command_locked(shutting_down_ ? geometry_msgs::msg::Twist{} : command);
  }

  std::optional<TargetConfig> read_config(std::string & reason) const
  {
    TargetConfig config;
    config.class_id = get_parameter("target_class_id").as_string();
    config.target_id = get_parameter("target_id").as_string();
    config.min_confidence = get_parameter("min_confidence").as_double();
    config.timeout_s = get_parameter("observation_timeout_s").as_double();

    const bool numeric_class_id = !config.class_id.empty() && std::all_of(
      config.class_id.begin(), config.class_id.end(), [](char ch) {
        return ch >= '0' && ch <= '9';
      });
    if (!numeric_class_id) {
      reason = "target_class_id 必须是非空数字字符串";
    } else if (!std::isfinite(config.min_confidence) ||
      config.min_confidence < 0.0 || config.min_confidence > 1.0)
    {
      reason = "min_confidence 必须在 [0, 1] 内";
    } else if (!std::isfinite(config.timeout_s) || config.timeout_s <= 0.0 || config.timeout_s > kDefaultTimeout) {
      reason = "observation_timeout_s 必须在(0, 0.25]秒内";
    } else {
      return config;
    }
    return std::nullopt;
  }

  void publish_state(const char * state)
  {
    if (state_ == state) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "任务状态: %s -> %s",
      state_.empty() ? "START" : state_.c_str(), state);
    state_ = state;
    std_msgs::msg::String message;
    message.data = state_;
    state_publisher_->publish(message);
  }

  void publish_empty(const std_msgs::msg::Header & source_header)
  {
    DetectionArray output;
    output.header = source_header;
    if (output.header.frame_id.empty()) {
      output.header.frame_id = last_source_frame_id_.empty() ? "unknown" : last_source_frame_id_;
    }
    selected_target_publisher_->publish(output);
  }

  void publish_empty_at_now(const rclcpp::Time & stamp)
  {
    DetectionArray output;
    output.header.stamp = stamp;
    output.header.frame_id = last_source_frame_id_.empty() ? "unknown" : last_source_frame_id_;
    selected_target_publisher_->publish(output);
  }

  void publish_detection(
    const std_msgs::msg::Header & source_header, const Detection & detection)
  {
    DetectionArray output;
    output.header = source_header;
    output.detections.push_back(detection);
    selected_target_publisher_->publish(output);
  }

  void clear_latest_observation()
  {
    latest_observation_.reset();
    latest_receipt_.reset();
  }

  void clear_current_target()
  {
    target_receipt_.reset();
    target_source_stamp_.reset();
    control_target_.reset();
    consecutive_misses_ = 0;
  }

  void lose_target(const char * reason, const std_msgs::msg::Header * source_header = nullptr)
  {
    const bool entering_lost = active_config_.has_value() && state_ != kLost;
    clear_current_target();
    if (active_config_) {
      publish_state(kLost);
    }
    publish_zero_velocity();
    if (source_header) {
      publish_empty(*source_header);
    } else {
      publish_empty_at_now(now());
    }
    if (entering_lost) {
      RCLCPP_WARN(get_logger(), "目标失效，进入 LOST: %s", reason);
    }
  }

  bool update_ros_clock(const rclcpp::Time & current)
  {
    const bool moved_back = last_ros_time_ && current < *last_ros_time_;
    last_ros_time_ = current;
    if (!moved_back) {
      return true;
    }

    RCLCPP_WARN(get_logger(), "ROS 时间回退，清除观测缓存");
    last_source_header_.reset();
    authorization_stamp_.reset();
    authorization_receipt_.reset();
    last_authorization_.reset();
    clear_latest_observation();
    if (active_config_) {
      stop_mission();
    } else {
      clear_current_target();
      publish_empty_at_now(current);
    }
    return false;
  }

  bool fresh_steady_receipt(
    const std::optional<SteadyClock::time_point> & receipt, double timeout_s) const
  {
    if (!receipt) {
      return false;
    }
    const double age_s = std::chrono::duration<double>(SteadyClock::now() - *receipt).count();
    return std::isfinite(age_s) && age_s >= 0.0 && age_s <= timeout_s;
  }

  bool authorization_is_fresh(const rclcpp::Time & current) const
  {
    if (!last_authorization_ || !authorization_stamp_ ||
      !fresh_steady_receipt(authorization_receipt_, kAuthorizationTimeout)) return false;
    const double source_age = (current - *authorization_stamp_).seconds();
    return std::isfinite(source_age) && source_age >= 0.0 && source_age <= kAuthorizationTimeout;
  }

  void bind_authorization(const Authorization & authorization)
  {
    task_authorization_ = authorization;
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    command_authorization_ = authorization;
  }

  void begin_authorization()
  {
    if (last_authorization_->authorized) {
      bind_authorization(*last_authorization_);
    } else {
      {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        command_authorization_.reset();
      }
      pending_authorization_ = *last_authorization_;
      authorization_deadline_ = SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(kAuthorizationWait));
      RCLCPP_INFO(get_logger(), "等待本次任务的 ROS 授权，最长 %.2f 秒", kAuthorizationWait);
    }
  }

  void on_authorization_status(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
  {
    const auto current = now();
    if (!update_ros_clock(current)) return;
    const auto & stamp = message->header.stamp;
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
      (stamp.sec == 0 && stamp.nanosec == 0U)) return;
    const rclcpp::Time source_stamp(stamp, current.get_clock_type());
    const double age_s = (current - source_stamp).seconds();
    const auto authorization = parse_authorization(*message);
    if (!authorization || !std::isfinite(age_s) || age_s < 0.0 || age_s > kAuthorizationTimeout ||
      (authorization_stamp_ && source_stamp <= *authorization_stamp_)) return;

    last_authorization_ = *authorization;
    authorization_stamp_ = source_stamp;
    authorization_receipt_ = SteadyClock::now();
    if (!active_config_) return;
    if (authorization->emergency_stop || authorization->control_source == "emergency_stop") {
      RCLCPP_WARN(get_logger(), "授权状态报告急停，停止视觉任务");
      stop_mission();
      return;
    }
    if (pending_authorization_) {
      const auto & pending = *pending_authorization_;
      if (authorization->bridge_id == pending.bridge_id && authorization->epoch == pending.epoch &&
        !authorization->authorized) return;
      const bool next_epoch = pending.epoch != std::numeric_limits<std::uint64_t>::max() &&
        authorization->epoch == pending.epoch + 1;
      if (authorization->bridge_id == pending.bridge_id && next_epoch && authorization->authorized &&
        authorization_deadline_ && SteadyClock::now() <= *authorization_deadline_ &&
        motion_start_stamp_ && source_stamp > *motion_start_stamp_)
      {
        bind_authorization(*authorization);
        pending_authorization_.reset();
        authorization_deadline_.reset();
        // A frame captured before authorization cannot initiate motion.
        motion_start_stamp_ = source_stamp;
        clear_latest_observation();
        RCLCPP_INFO(get_logger(), "视觉任务已绑定授权代次 %llu，等待授权后的新帧",
          static_cast<unsigned long long>(authorization->epoch));
        publish_zero_velocity();
        return;
      }
      RCLCPP_WARN(get_logger(), "等待授权期间授权实例或代次已变化，停止视觉任务");
      stop_mission();
      return;
    }
    if (!task_authorization_ || !authorization->authorized ||
      authorization->bridge_id != task_authorization_->bridge_id ||
      authorization->epoch != task_authorization_->epoch)
    {
      RCLCPP_WARN(get_logger(), "ROS 授权已撤销或授权实例/代次改变，停止视觉任务");
      stop_mission();
    }
  }

  void on_detections(DetectionArray::ConstSharedPtr message)
  {
    const auto receipt = SteadyClock::now();
    const auto ros_now = now();
    if (!update_ros_clock(ros_now)) {
      return;
    }
    if (active_config_ && !authorization_is_fresh(ros_now)) {
      RCLCPP_WARN(get_logger(), "ROS 授权状态已过期，停止视觉任务");
      stop_mission();
      return;
    }

    const double configured_timeout = active_config_ ? active_config_->timeout_s :
      get_parameter("observation_timeout_s").as_double();
    const bool valid_timeout = active_config_ ||
      (std::isfinite(configured_timeout) && configured_timeout > 0.0 && configured_timeout <= kDefaultTimeout);
    const double timeout_s = valid_timeout ? configured_timeout : 0.0;
    const auto header_failure = valid_timeout ?
      header_failure_reason(message->header, ros_now, timeout_s) : "invalid_timeout";
    if (header_failure) {
      const auto & stamp = message->header.stamp;
      const double source_age_ms = stamp.sec >= 0 && stamp.nanosec < 1000000000U ?
        (ros_now.seconds() - (static_cast<double>(stamp.sec) + stamp.nanosec / 1.0e9)) * 1000.0 :
        -1.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "拒绝视觉观测 reason=%s state=%s source_stamp=%d.%09u source_age_ms=%.3f timeout_ms=%.3f frame_id=%s",
        header_failure, state_.c_str(), stamp.sec, stamp.nanosec, source_age_ms,
        timeout_s * 1000.0, message->header.frame_id.c_str());
      return;
    }
    if (last_source_header_ && !newer_stamp(message->header, *last_source_header_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "拒绝视觉观测 reason=non_increasing_stamp state=%s source_stamp=%d.%09u previous_stamp=%d.%09u",
        state_.c_str(), message->header.stamp.sec, message->header.stamp.nanosec,
        last_source_header_->stamp.sec, last_source_header_->stamp.nanosec);
      return;
    }

    const bool bad_detection_header = std::any_of(
      message->detections.begin(), message->detections.end(), [&message](const Detection & detection) {
        return detection.header.frame_id != message->header.frame_id ||
          detection.header.stamp.sec != message->header.stamp.sec ||
          detection.header.stamp.nanosec != message->header.stamp.nanosec;
      });
    if (bad_detection_header) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "丢弃视觉观测 reason=detection_header_mismatch state=%s", state_.c_str());
      return;
    }

    // A frame arriving after a detection-stream timeout cannot conceal the gap.
    if (active_config_ && state_ != "SEARCHING" && state_ != kLost && target_acquired_ &&
      latest_observation_ && latest_receipt_)
    {
      const auto prior_failure = header_failure_reason(
        latest_observation_->header, ros_now, active_config_->timeout_s);
      if (prior_failure || !fresh_steady_receipt(latest_receipt_, active_config_->timeout_s)) {
        clear_latest_observation();
        lose_target(prior_failure ? prior_failure : "receipt_timeout");
      }
    }

    last_source_header_ = message->header;
    last_source_frame_id_ = message->header.frame_id;
    const rclcpp::Time source_stamp(message->header.stamp, ros_now.get_clock_type());
    if (active_config_ && (pending_authorization_ ||
      (motion_start_stamp_ && source_stamp <= *motion_start_stamp_))) {
      publish_empty(message->header);
      return;
    }
    latest_observation_ = *message;
    latest_receipt_ = receipt;

    if (!active_config_) {
      publish_empty(message->header);
      return;
    }

    DetectionArray filtered = *message;
    if (color_ == "red") {
      filtered.detections.erase(std::remove_if(filtered.detections.begin(), filtered.detections.end(),
        [this, &message](const Detection & d) { return !red_detection(*message, d); }), filtered.detections.end());
    }
    const char * miss_reason = nullptr;
    const auto selected = select_detection(filtered, *active_config_, locked_target_id_, &miss_reason);
    if (!selected) {
      if (color_ == "red" && !locked_target_id_.empty() &&
        std::any_of(message->detections.begin(), message->detections.end(),
          [this](const Detection & detection) {return detection.id == locked_target_id_;}) &&
        std::none_of(filtered.detections.begin(), filtered.detections.end(),
          [this](const Detection & detection) {return detection.id == locked_target_id_;}))
      {
        miss_reason = "color_unavailable_or_rejected";
      }
      if (state_ == kLost && !target_acquired_ && behavior_ != "tracking") {
        publish_state("SEARCHING");
        RCLCPP_INFO(get_logger(), "授权仍连续，新鲜观测恢复搜索");
        publish_empty(message->header);
      } else if (state_ != "SEARCHING" && state_ != kLost && target_acquired_) {
        ++consecutive_misses_;
        if (consecutive_misses_ >= kConsecutiveMissLimit) {
          lose_target("连续5帧没有同一 ID 的合格目标", &message->header);
        } else {
          if (consecutive_misses_ == 1) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "目标暂时漏检 reason=%s target_id=%s，连续5帧才进入 LOST",
              miss_reason, locked_target_id_.c_str());
          }
          publish_empty(message->header);
        }
      } else if (state_ != "SEARCHING") {
        lose_target("新帧中没有同一 ID 的合格目标", &message->header);
      } else {
        publish_empty(message->header);
      }
      return;
    }

    if (locked_target_id_.empty()) locked_target_id_ = selected->id;
    target_acquired_ = true;
    consecutive_misses_ = 0;
    target_receipt_ = receipt;
    target_source_stamp_ = source_stamp;
    control_target_ = *selected;
    if (behavior_ == "search_only") {
      publish_zero_velocity();
      publish_state("FOUND");
    } else if (state_ != "BLOCKED") {
      if (behavior_ == "follow") publish_state("FOLLOWING");
      else if (behavior_ == "approach" || behavior_ == "search") publish_state("APPROACHING");
      else publish_state(kTracking);
    }
    publish_detection(message->header, *selected);
  }

  void on_start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (active_config_) {
      response->success = false;
      response->message = "任务正在运行或等待原目标恢复；请先调用 /mission/stop";
      return;
    }
    if (emergency_stop_) {
      response->success = false;
      response->message = "急停仍激活，不能启动任务";
      return;
    }

    std::string reason;
    const auto config = read_config(reason);
    if (!config) {
      response->success = false;
      response->message = reason;
      return;
    }

    const auto ros_now = now();
    if (!update_ros_clock(ros_now)) {
      response->success = false;
      response->message = "ROS 时间刚发生回退，请等待新的视觉观测后重试";
      return;
    }
    if (!authorization_is_fresh(ros_now) || last_authorization_->emergency_stop ||
      last_authorization_->control_source == "emergency_stop") {
      response->success = false;
      response->message = "缺少新鲜的网桥授权状态或急停激活";
      return;
    }
    if (!latest_observation_ || !latest_receipt_ ||
      !fresh_header(latest_observation_->header, ros_now, config->timeout_s) ||
      !fresh_steady_receipt(latest_receipt_, config->timeout_s))
    {
      response->success = false;
      response->message = "没有新鲜视觉观测；请等待新帧后重试";
      return;
    }

    const auto selected = select_detection(*latest_observation_, *config);
    if (!selected) {
      response->success = false;
      response->message = config->target_id.empty() ?
        "新鲜观测中没有满足类别和置信度的合法目标" :
        "新鲜观测中没有指定 target_id 的合法目标";
      return;
    }

    behavior_ = "tracking"; color_.clear();
    active_config_ = *config;
    begin_authorization();
    locked_target_id_ = selected->id;
    target_acquired_ = true;
    control_target_.reset();
    target_receipt_ = latest_receipt_;
    target_source_stamp_ = rclcpp::Time(
      latest_observation_->header.stamp, ros_now.get_clock_type());
    motion_start_stamp_ = ros_now;
    last_source_frame_id_ = latest_observation_->header.frame_id;
    publish_state(kTracking);
    publish_empty(latest_observation_->header);
    publish_zero_velocity();
    response->success = true;
    response->message = "已锁定目标 " + locked_target_id_ + "，等待下一帧观测";
  }

  bool red_detection(const DetectionArray & detections, const Detection & d) const
  {
    if (!std::isfinite(d.bbox.center.position.x) || !std::isfinite(d.bbox.center.position.y) ||
        !std::isfinite(d.bbox.size_x) || !std::isfinite(d.bbox.size_y) ||
        d.bbox.size_x <= 0 || d.bbox.size_y <= 0 || d.bbox.size_x > 16384 || d.bbox.size_y > 16384 ||
        std::abs(d.bbox.center.position.x) > 16384 || std::abs(d.bbox.center.position.y) > 16384) return false;
    for (auto it = images_.rbegin(); it != images_.rend(); ++it) {
      const auto & h = (*it)->header;
      if (h.frame_id != detections.header.frame_id || h.stamp.sec != detections.header.stamp.sec ||
          h.stamp.nanosec != detections.header.stamp.nanosec) continue;
      try {
        auto img = cv_bridge::toCvShare(*it, sensor_msgs::image_encodings::BGR8)->image;
        int x0 = std::max(0, static_cast<int>(std::floor(d.bbox.center.position.x - d.bbox.size_x / 2)));
        int y0 = std::max(0, static_cast<int>(std::floor(d.bbox.center.position.y - d.bbox.size_y / 2)));
        int x1 = std::min(img.cols, static_cast<int>(std::ceil(d.bbox.center.position.x + d.bbox.size_x / 2)));
        int y1 = std::min(img.rows, static_cast<int>(std::ceil(d.bbox.center.position.y + d.bbox.size_y / 2)));
        if (x1 <= x0 || y1 <= y0) return false;
        cv::Mat hsv, low, high;
        cv::cvtColor(img(cv::Rect(x0,y0,x1-x0,y1-y0)), hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0,100,70), cv::Scalar(10,255,255), low);
        cv::inRange(hsv, cv::Scalar(170,100,70), cv::Scalar(179,255,255), high);
        return cv::countNonZero(low | high) >= (x1-x0)*(y1-y0)/10.0;
      } catch (const cv_bridge::Exception &) { return false; }
    }
    return false;
  }

  void on_start_configured(const std::shared_ptr<ugv_bridge::srv::MissionStart::Request> & request,
                           std::shared_ptr<ugv_bridge::srv::MissionStart::Response> response)
  {
    if (active_config_ || emergency_stop_) { response->message = "任务运行中或急停激活"; return; }
    if (request->class_id.empty() || !std::all_of(request->class_id.begin(), request->class_id.end(),
        [](char c) { return c >= '0' && c <= '9'; }) ||
        (request->behavior != "follow" && request->behavior != "approach" && request->behavior != "search" &&
        request->behavior != "search_only") ||
        (request->color != "" && request->color != "red")) {
      response->message = "类别、行为或颜色无效"; return;
    }
    std::string reason;
    auto config = read_config(reason);
    if (!config) { response->message = reason; return; }
    config->class_id = request->class_id; config->target_id = request->target_id;
    const auto t = now();
    if (!update_ros_clock(t)) { response->message = "ROS 时间回退"; return; }
    if (!authorization_is_fresh(t) || last_authorization_->emergency_stop ||
      last_authorization_->control_source == "emergency_stop") {
      response->message = "缺少新鲜的网桥授权状态或急停激活"; return;
    }
    active_config_ = *config; behavior_ = request->behavior; color_ = request->color;
    begin_authorization();
    locked_target_id_ = request->target_id;
    target_acquired_ = false;
    clear_current_target(); clear_latest_observation(); motion_start_stamp_ = t;
    publish_state("SEARCHING"); publish_empty_at_now(t); publish_zero_velocity();
    response->success = true; response->message = "任务已启动，等待新观测";
  }

  std::optional<double> front_distance(const rclcpp::Time & t) const
  {
    if (!scan_ || !fresh_steady_receipt(scan_receipt_, scan_timeout_s_) ||
        !fresh_header(scan_->header, t, scan_timeout_s_) || !std::isfinite(scan_->angle_increment) || scan_->angle_increment <= 0) return std::nullopt;
    return front_obstacle_distance(*scan_);
  }

  void on_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stop_mission();
    response->success = true;
    response->message = "任务已停止，选中目标已清空";
  }

  void stop_mission()
  {
    // Clear the bridge's last command while the old authorization is still attached.
    publish_zero_velocity();
    active_config_.reset();
    behavior_.clear(); color_.clear();
    locked_target_id_.clear();
    target_acquired_ = false;
    task_authorization_.reset();
    pending_authorization_.reset();
    authorization_deadline_.reset();
    // Keep the old identity only for repeated zero commands. The bridge will
    // reject it after revocation; retaining it avoids losing the first zero
    // to the depth-one command queue when /mission/stop is called directly.
    motion_start_stamp_.reset();
    clear_current_target();
    clear_latest_observation();
    publish_state(kStopped);
    publish_empty_at_now(now());
    publish_zero_velocity();
  }

  void on_emergency_stop(std_msgs::msg::Bool::ConstSharedPtr message)
  {
    emergency_stop_ = message->data;
    if (emergency_stop_) {
      RCLCPP_WARN(get_logger(), "收到急停，停止任务并清空选中目标");
      stop_mission();
    }
  }

  void on_timer()
  {
    const auto ros_now = now();
    if (!update_ros_clock(ros_now)) {
      return;
    }
    if (!active_config_) { publish_zero_velocity(); return; }
    if (!authorization_is_fresh(ros_now)) {
      RCLCPP_WARN(get_logger(), "ROS 授权状态超时，停止视觉任务");
      stop_mission();
      return;
    }
    if (pending_authorization_) {
      if (!authorization_deadline_ || SteadyClock::now() > *authorization_deadline_) {
        RCLCPP_WARN(get_logger(), "等待 ROS 授权超时，停止视觉任务");
        stop_mission();
      } else {
        publish_zero_velocity();
      }
      return;
    }
    if (!task_authorization_) { stop_mission(); return; }
    if (state_ == kLost) { publish_zero_velocity(); return; }

    const bool searching = state_ == "SEARCHING";
    std::optional<rclcpp::Time> source_stamp;
    const bool use_stream_freshness = searching || consecutive_misses_ > 0;
    if (use_stream_freshness && latest_observation_) {
      source_stamp.emplace(latest_observation_->header.stamp, ros_now.get_clock_type());
    } else if (!use_stream_freshness) {
      source_stamp = target_source_stamp_;
    }
    const auto & receipt = use_stream_freshness ? latest_receipt_ : target_receipt_;
    const double ros_age_s = source_stamp ?
      (ros_now - *source_stamp).seconds() : INFINITY;
    const double steady_age_s = receipt ?
      std::chrono::duration<double>(SteadyClock::now() - *receipt).count() : INFINITY;
    const auto failure = observation_failure_reason(
      source_stamp.has_value(), ros_age_s, receipt.has_value(), steady_age_s,
      active_config_->timeout_s);
    if (failure) {
      RCLCPP_WARN(
        get_logger(),
        "视觉观测失效 reason=%s state=%s kind=%s source_age_ms=%.3f receive_age_ms=%.3f timeout_ms=%.3f source_stamp_ns=%lld",
        failure, state_.c_str(), searching ? "search" :
        (use_stream_freshness ? "miss_stream" : "target"),
        source_stamp ? ros_age_s * 1000.0 : -1.0,
        receipt ? steady_age_s * 1000.0 : -1.0, active_config_->timeout_s * 1000.0,
        static_cast<long long>(source_stamp ? source_stamp->nanoseconds() : 0));
      clear_latest_observation();
      lose_target(failure);
      return;
    }

    if (searching) {
      geometry_msgs::msg::Twist command;
      if ((behavior_ == "search" || behavior_ == "search_only") && motion_config_.enabled) command.angular.z = search_angular_speed_;
      publish_velocity(command); return;
    }
    if (behavior_ == "search_only" || !motion_config_.enabled || !control_target_) {
      publish_zero_velocity();
      return;
    }
    auto command = compute_servo_command(*control_target_, motion_config_);
    if (motion_config_.enabled && (behavior_ == "follow" || behavior_ == "approach" || behavior_ == "search")) {
      const auto distance = front_distance(ros_now);
      if (distance && *distance <= standoff_m_) {
        publish_state("BLOCKED");
        command = geometry_msgs::msg::Twist{};
      } else {
        if (state_ == "BLOCKED") publish_state(behavior_ == "follow" ? "FOLLOWING" : "APPROACHING");
        if (distance && command.angular.z == 0.0) command.linear.x = linear_speed_m_s_;
      }
    }
    publish_velocity(command);
  }

  std::string image_topic_, scan_topic_, behavior_, color_;
  double standoff_m_, linear_speed_m_s_, search_angular_speed_, scan_timeout_s_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  std::optional<SteadyClock::time_point> scan_receipt_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Service<ugv_bridge::srv::MissionStart>::SharedPtr configured_start_service_;
  std::string state_;
  bool emergency_stop_{false};
  bool shutting_down_{false};
  bool target_acquired_{false};
  unsigned int consecutive_misses_{0};
  MotionConfig motion_config_;
  std::optional<Authorization> last_authorization_;
  std::optional<Authorization> pending_authorization_;
  std::optional<Authorization> task_authorization_;
  std::optional<Authorization> command_authorization_;  // Protected by velocity_mutex_.
  std::optional<SteadyClock::time_point> authorization_receipt_;
  std::optional<SteadyClock::time_point> authorization_deadline_;
  std::optional<rclcpp::Time> authorization_stamp_;
  std::optional<TargetConfig> active_config_;
  std::string locked_target_id_;
  std::optional<Detection> control_target_;
  std::optional<SteadyClock::time_point> target_receipt_;
  std::optional<rclcpp::Time> target_source_stamp_;
  std::optional<rclcpp::Time> motion_start_stamp_;
  std::optional<DetectionArray> latest_observation_;
  std::optional<SteadyClock::time_point> latest_receipt_;
  std::optional<std_msgs::msg::Header> last_source_header_;
  std::optional<rclcpp::Time> last_ros_time_;
  std::string last_source_frame_id_;

  std::mutex velocity_mutex_;
  rclcpp::Publisher<ugv_bridge::msg::AuthorizedTwist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<DetectionArray>::SharedPtr selected_target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<DetectionArray>::SharedPtr detections_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr authorization_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace car_mission

#ifndef CAR_MISSION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<car_mission::MissionControlNode>();
    const std::weak_ptr<car_mission::MissionControlNode> weak_node = node;
    node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak_node]() {
      try {
        if (const auto stopping_node = weak_node.lock()) {
          stopping_node->prepare_shutdown();
        }
      } catch (...) {
        // Shutdown callbacks must not propagate exceptions.
      }
    });
    rclcpp::spin(node);
    if (rclcpp::ok()) {
      node->stop();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_control_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
