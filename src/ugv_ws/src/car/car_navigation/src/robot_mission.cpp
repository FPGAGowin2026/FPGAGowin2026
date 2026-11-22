#include "car_navigation/fpga_reference.hpp"
#include "car_navigation/grid_route.hpp"
#include "car_navigation/navigation_world.hpp"
#include "car_navigation/waypoint_follower.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <optional>
#include <utility>

namespace car_navigation {
namespace {
using Json = nlohmann::json;
using Empty = std_msgs::msg::Empty;
using String = std_msgs::msg::String;
using Pose = geometry_msgs::msg::PoseStamped;
using Path = nav_msgs::msg::Path;
using Steady = std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr double blocked_retry_delay = 4.;
constexpr double mppi_terminal_retry_radius = 0.30;
constexpr double mppi_terminal_progress_distance = 0.05;

double seconds() { return std::chrono::duration<double>(Steady::now().time_since_epoch()).count(); }
double yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}
Point transform_point(Point p, Point from, Point to, double angle) {
  const double x = p.x - from.x, y = p.y - from.y, c = std::cos(angle), s = std::sin(angle);
  return {to.x + c * x - s * y, to.y + s * x + c * y};
}
std::optional<Waypoint> waypoint(const Pose &p) {
  const auto &q = p.pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if ((!p.header.frame_id.empty() && p.header.frame_id != "map") || !(norm >= .25 && norm <= 2.25) ||
      !(std::abs(p.pose.position.x) <= 10000 && std::abs(p.pose.position.y) <= 10000))
    return {};
  return Waypoint{{p.pose.position.x, p.pose.position.y}, yaw(q)};
}

// Consume the first contact with this waypoint on the measured travel segment.
// The suffix can then be used to check the next waypoint in chronological order.
bool pass_waypoint(Point &from, Point to, Point goal) {
  constexpr double radius = intermediate_waypoint_radius;
  const double x = goal.x - from.x, y = goal.y - from.y;
  if (std::hypot(x, y) <= radius) return true;
  const double dx = to.x - from.x, dy = to.y - from.y, length = std::hypot(dx, dy);
  if (length == 0) return false;
  const double along = (x * dx + y * dy) / length, lateral = (x * dy - y * dx) / length;
  if (along < 0 || std::abs(lateral) > radius) return false;
  const double first = along - std::sqrt(std::max(0., radius * radius - lateral * lateral));
  if (first > length) return false;
  const double u = std::max(0., first) / length;
  from = {from.x + u * dx, from.y + u * dy};
  return true;
}

struct PlanWork {
  MissionRoute result;
  std::uint64_t task, generation, map_version;
  std::size_t index;
  Point start;
  bool detour;
};

struct MppiBlockage {
  Point odom, target;
  double yaw, since, hard_stop_at;
  std::size_t index, next;
};

struct MppiTerminalAbort {
  Point odom, goal;
  std::size_t index;
};

struct FpgaBlockage {
  Point start, target;
  std::size_t index;
  std::uint64_t token;
  double retry_at;
  std::int64_t observe_after_ns{};
  bool open{};
};

struct ObservationTurn {
  Point map_center, odom_center;
  double yaw, remaining, progress_until;
  std::optional<double> retry_from_yaw;
};

std::string plan_failure(const std::string &reason) {
  if (reason == "goal_occupied") return "目标中心格已有障碍占据";
  if (reason == "goal_footprint_blocked") return "指定终点朝向的车身轮廓与障碍相交";
  if (reason == "no_route") return "未找到可到达目标的通路";
  if (reason == "no_reachable_frontier") return "未找到可到达的观测前沿";
  if (reason == "start_unknown_or_occupied") return "起点格尚未观测或已被障碍占据";
  if (reason == "start_outside_map") return "起点位于规划地图范围外";
  if (reason == "no_known_goal") return "目标格位于已知区域之外";
  if (reason == "blocked_leg") return "当前受阻方向尚无可验证绕行";
  return reason;
}
}  // namespace

class Mission : public rclcpp::Node {
 public:
  Mission() : Node("robot_mission"), world_(*this), follower_(*this), reference_(*this) {
    enable_mppi_ = declare_parameter<bool>("enable_waypoint_mppi", true);
    stop_pub_ = create_publisher<Empty>("/fpga/stop", 10);
    status_pub_ = create_publisher<String>("/robot/mission/status", 10);
    path_pub_ = create_publisher<Path>("/robot/global_path", 1);
    vision_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/fpga/vision_velocity", 1);
    subscribe<Pose>("/robot/mission/goal", [this](const Pose &p) { single_goal(p, false); });
    subscribe<Pose>("/swarm/car/goal", [this](const Pose &p) { single_goal(p, true); });
    subscribe<Path>("/robot/mission/waypoints", [this](const Path &path) {
      if (path.header.frame_id != "map" || path.poses.empty() || path.poses.size() > 32) {
        reason_ = "多航点格式无效";
        return;
      }
      std::vector<Waypoint> points;
      for (const auto &p : path.poses) {
        auto w = waypoint(p);
        if (!w) {
          reason_ = "多航点格式无效";
          return;
        }
        points.push_back(*w);
      }
      begin("goal", std::move(points), false);
    });
    subscribe<Empty>("/robot/mission/explore", [this](const Empty &) { begin("explore", {}, false); });
    subscribe<Empty>("/robot/mission/stop", [this](const Empty &) { finish("idle", "已请求停车", true); });
    listen_json("/fpga/navigation_status", [this](const Json &j) { on_device(j); });
    listen_json("/fpga/command_status", [this](const Json &j) { on_command(j); });
    listen_json("/robot/vision/command", [this](const Json &j) { on_vision(j); });
    timer_ = create_wall_timer(50ms, [this] { tick(); });
  }
  ~Mission() override {
    if (cancel_) cancel_->store(true);
    if (job_.valid()) job_.wait();
  }

 private:
  template <class Message, class Callback>
  void subscribe(const char *topic, Callback cb) {
    subs_.push_back(create_subscription<Message>(topic, 10, std::move(cb)));
  }
  template <class Callback>
  void listen_json(const char *topic, Callback cb) {
    subscribe<String>(topic, [this, cb, topic](const String &s) {
      try {
        const auto j = Json::parse(s.data);
        if (j.is_object()) cb(j);
      } catch (const Json::exception &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s: %s", topic, e.what());
      }
    });
  }
  bool navigating() const { return mode_ == "goal" || mode_ == "explore"; }
  bool device_fresh() const { return seconds() - device_at_ < 0.5 && device_.contains("session_token"); }
  bool ready_session() const {
    return device_fresh() && device_.value("session_token", std::uint64_t{}) && boot_ && domain_;
  }
  bool stopped() const {
    return device_.value("mode", 0) == 1 && !device_.value("vx_mm_s", 1) && !device_.value("vy_mm_s", 1) &&
           !device_.value("wz_mrad_s", 1);
  }
  bool front_stop() const {
    return device_fresh() && stopped() && device_.value("guard_obstacle", 0) != 0 &&
           !device_.value("guard_missing", 0) && device_.value("error", 0) == 0x4e;
  }
  void single_goal(const Pose &p, bool coop) {
    if (coop && (navigating() || mode_ == "vision")) {
      reason_ = "已有任务，协同目标等待";
      return;
    }
    auto w = waypoint(p);
    if (p.header.frame_id != "map" || !w) {
      reason_ = "地图航点格式无效";
      return;
    }
    begin("goal", {*w}, coop);
  }
  void cancel_plan() {
    ++task_revision_;
    if (cancel_) cancel_->store(true);
  }
  void clear_route() {
    cancel_plan();
    follower_.cancel();
    reference_.reset();
    route_ = {};
    next_ = 0;
    previous_.reset();
    using_mppi_ = false;
    replacement_needed_ = false;
    mppi_blockage_.reset();
    pending_detour_.reset();
    observation_turn_.reset();
    checked_from_.reset();
    projected_from_.reset();
    frontier_revision_.reset();
    map_checked_ = 0;
    next_plan_at_ = 0;
    publish_path();
  }
  void request_stop() {
    previous_.reset();
    waypoint_sample_.reset();
    stop_stamp_ = now().nanoseconds();
    last_stop_send_at_ = seconds();
    stop_sent_ = stop_pub_->get_subscription_count() != 0;
    stop_acked_ = false;
    ack_sample_us_.reset();
    waiting_stop_ = true;
    if (stop_sent_) stop_pub_->publish(Empty{});
  }
  void begin(const std::string &mode, std::vector<Waypoint> points, bool coop) {
    const bool wants_mppi = mode == "goal" && !coop && enable_mppi_;
    const bool stack_ready = wants_mppi && follower_.server_ready();
    // A confirmed streamed session can accept another path directly.
    const bool continuous = stack_ready && mode_ != "vision" && !reference_.active() &&
                            !waiting_stop_ && stream_authorized_ && ready_session() &&
                            owned_token_ == device_.value("session_token", std::uint64_t{}) &&
                            device_.value("mode", 0) == 1;
    const bool new_session = !stream_authorized_ && !reference_.active();
    clear_route();
    fpga_blockage_.reset();
    last_look_from_.reset();
    next_look_at_ = 0.;
    mppi_terminal_abort_.reset();
    waypoint_sample_.reset();
    points_ = std::move(points);
    index_ = 0;
    completed_ = 0;
    task_state_ = points_.empty() ? "idle" : "running";
    failure_.clear();
    excluded_.clear();
    cooperative_ = coop;
    world_.set_cooperative(coop);
    epoch_.reset();
    map_session_ = mode == "vision" ? "" : world_.snapshot().map_session;
    if (continuous) epoch_ = std::make_pair(boot_, domain_);
    generation_.reset();
    force_fpga_ = wants_mppi && !stack_ready;
    suspended_ = false;
    start_pose_.reset();
    start_point_.reset();
    may_set_start_ = new_session && mode != "vision";
    command_after_ns_ = now().nanoseconds();
    waypoint_task_id_ = points_.empty() ? "" : std::to_string(command_after_ns_);
    if (!points_.empty()) report_at_ = 0;
    vision_forward_at_ = 0;
    mode_ = mode;
    reason_ = "等待导航输入";
    if (!continuous) {
      stream_authorized_ = false;
      request_stop();
    }
  }
  void finish(const std::string &mode, const std::string &reason, bool send_stop) {
    clear_route();
    force_fpga_ = false;
    fpga_blockage_.reset();
    last_look_from_.reset();
    mppi_terminal_abort_.reset();
    if (task_state_ == "running") {
      report_at_ = 0;
      task_state_ = mode == "arrived" ? "completed" : mode == "blocked" ? "failed" : "cancelled";
      if (mode == "arrived") completed_ = points_.size();
      if (mode == "blocked") failure_ = reason;
    }
    mode_ = mode;
    reason_ = reason;
    suspended_ = false;
    cooperative_ = false;
    world_.set_cooperative(false);
    start_pose_.reset();
    start_point_.reset();
    may_set_start_ = false;
    epoch_.reset();
    generation_.reset();
    map_session_.clear();
    command_after_ns_ = now().nanoseconds();
    vision_forward_at_ = 0;
    waiting_stop_ = false;
    stream_authorized_ = false;
    if (send_stop) request_stop();
  }
  void suspend(const std::string &reason) {
    if (!suspended_) {
      if (observation_turn_) {
        retry_look_yaw_ = observation_turn_->retry_from_yaw;
        if (fpga_blockage_) fpga_blockage_->observe_after_ns = now().nanoseconds();
      }
      clear_route();
      mppi_terminal_abort_.reset();
      stream_authorized_ = false;
      if (!waiting_stop_) request_stop();
      start_pose_.reset();
      start_point_.reset();
      may_set_start_ = false;
      suspended_ = true;
    }
    reason_ = reason;
  }
  void on_device(const Json &j) {
    const auto boot = j.at("boot_id").get<std::uint32_t>(), domain = j.at("odom_domain").get<std::uint32_t>();
    j.at("session_token").get<std::uint64_t>();
    for (auto key : {"status", "mode", "error", "guard_obstacle", "guard_missing", "vx_mm_s", "vy_mm_s",
                     "wz_mrad_s", "task_id", "now_us", "expiry_us"})
      if (!j.contains(key) || !j.at(key).is_number_integer()) return;
    boot_ = boot;
    domain_ = domain;
    device_ = j;
    device_at_ = seconds();
    if (!ready_session()) return;
    reference_.temporary_start_active(device_);
    if (navigating() && epoch_ && *epoch_ != std::make_pair(boot_, domain_))
      finish("blocked", "FPGA 控制会话或里程计坐标系已改变，请重新提交航点", true);
  }
  void on_command(const Json &j) {
    const auto state = j.at("state").get<std::string>();
    reference_.on_command(j);
    if (state == "manual_takeover") {
      const auto stamp = j.at("stamp_ns").get<std::int64_t>(), current = now().nanoseconds();
      if (stamp > 0 && stamp <= current) {
        manual_at_ = std::max(manual_at_, seconds() - (current - stamp) * 1e-9);
        if (stamp > command_after_ns_ && (navigating() || mode_ == "vision"))
          finish("idle", "手动接管", false);
      }
      return;
    }
    if (state == "submitted" && j.contains("speed_mm_s"))
      world_.set_fpga_goal_speed_mm_s(j.at("speed_mm_s").get<int>());
    if (state == "stop_requested" && waiting_stop_ && stop_sent_ &&
        j.value("stop_stamp_ns", std::int64_t{}) >= stop_stamp_) {
      stop_acked_ = true;
      ack_at_ = seconds();
      ack_token_ = j.value("session_token", std::uint64_t{});
      ack_sample_us_.reset();
      if (j.contains("stop_sample_us")) {
        ack_sample_us_ = j.at("stop_sample_us").get<std::uint32_t>();
        ack_boot_ = j.at("stop_boot_id").get<std::uint32_t>();
        ack_domain_ = j.at("stop_odom_domain").get<std::uint32_t>();
      }
    } else if (waiting_stop_ && (state == "stop_unconfirmed" || state == "link_recovering")) {
      last_stop_send_at_ = seconds();
    }
    if (state == "link_recovering") {
      stream_authorized_ = false;
      if (mode_ == "vision")
        finish("blocked", "视觉任务控制链路中断", true);
      else if (navigating()) {
        suspend("控制链路中断，等待同一底盘恢复");
        if (!waiting_stop_) request_stop();
      }
    }
  }
  void on_vision(const Json &j) {
    const auto stamp = j.at("stamp_ns").get<std::int64_t>();
    const auto current = now().nanoseconds();
    if (stamp <= command_after_ns_ || stamp > current || current - stamp > 1000000000) return;
    const auto action = j.at("action").get<std::string>();
    if (action == "start" && !navigating() && mode_ != "vision" && seconds() - manual_at_ >= 1.) {
      begin("vision", {}, false);
      vision_at_ = seconds();
      reason_ = "视觉检测框控制";
      return;
    }
    if (mode_ != "vision") return;
    if (action == "stop") {
      finish("idle", "视觉任务已停止", true);
      return;
    }
    if (action != "servo" || current - stamp >= 150000000) return;
    const double vx = j.at("vx").get<double>(), wz = j.at("wz").get<double>();
    if (!(vx >= 0 && vx <= .4 && std::abs(wz) <= 1.0)) return;
    vision_at_ = seconds();
    if (!ready_session() || waiting_stop_ || !stream_authorized_ ||
        owned_token_ != device_.value("session_token", std::uint64_t{})) {
      vision_forward_at_ = 0;
      return;
    }
    if (!vx)
      vision_forward_at_ = 0;
    else if (!vision_forward_at_)
      vision_forward_at_ = vision_at_;
    geometry_msgs::msg::TwistStamped v;
    v.header.stamp = rclcpp::Time(stamp);
    v.twist.linear.x = vx;
    v.twist.angular.z = wz;
    vision_pub_->publish(v);
  }
  std::optional<Point> confirmed_start() const {
    return start_point_ && seconds() - start_at_ < 10. ? start_point_ : std::nullopt;
  }
  bool detour_inputs(const WorldSnapshot &w) const {
    const auto error = device_.value("error", 0);
    // 0x4c also reports an expired velocity lease after an acknowledged stop.
    const bool idle_lease = error == 0x4c && !waiting_stop_ && stream_authorized_ && stopped();
    return w.scan_fresh && w.wheel_trusted && w.local_map && ready_session() &&
           owned_token_ == device_.value("session_token", std::uint64_t{}) &&
           !device_.value("guard_missing", 0) && (error == 0 || error == 0x4e || idle_lease);
  }
  std::optional<Point> blocked_target(const WorldSnapshot &w) const {
    if (mppi_blockage_) return mppi_blockage_->target;
    if (!fpga_blockage_ || fpga_blockage_->open) return {};
    return transform_point(fpga_blockage_->target, {w.odom_x, w.odom_y}, {w.x, w.y}, w.yaw - w.odom_yaw);
  }
  bool blockage_observation_ready(const WorldSnapshot &w) const {
    return w.front_scan && detour_inputs(w) &&
           rclcpp::Time(w.local_map->header.stamp).nanoseconds() > fpga_blockage_->observe_after_ns;
  }
  bool waiting_for_blockage_observation(const WorldSnapshot &w) {
    if (!fpga_blockage_ || !fpga_blockage_->observe_after_ns || blockage_observation_ready(w)) return false;
    reason_ = "等待安全状态及更新后的局部观测，保留航点并自动重试";
    return true;
  }
  bool stopped_for_recovery(const WorldSnapshot &w) const {
    return !waiting_stop_ && stream_authorized_ && stopped() && std::hypot(w.vx, w.vy) <= 0.05 &&
           std::abs(w.wz) <= 0.05;
  }
  void remember_blocked_leg(const WorldSnapshot &w, Point odom_target, bool require_sweep_evidence = false) {
    const Point odom{w.odom_x, w.odom_y};
    const double span = distance(odom, odom_target);
    if (span <= 1e-8) return;
    if (span > obstacle_check_distance) {
      const double f = obstacle_check_distance / span;
      odom_target = {odom.x + f * (odom_target.x - odom.x), odom.y + f * (odom_target.y - odom.y)};
    }
    if (fpga_blockage_ && !fpga_blockage_->open && fpga_blockage_->index == index_ &&
        fpga_blockage_->token == owned_token_ && distance(odom, fpga_blockage_->start) < 0.10)
      return;
    if (require_sweep_evidence) {
      const Point robot{w.x, w.y};
      const auto target = transform_point(odom_target, odom, robot, w.yaw - w.odom_yaw);
      // A missed ACK or lack of a candidate alone says nothing about an
      // obstacle. Only retain this direction if a measured hit blocks it.
      if (!w.front_scan || !w.local_map ||
          (!w.front_scan->blocks_translation(odom, odom_target, w.odom_yaw) &&
           clear_body_segment(*w.local_map, odom, odom_target, w.odom_yaw, true, {}, nullptr) &&
           clear_body_segment(*w.map, robot, target, w.yaw, true, {}, w.peer_map.get())))
        return;
    }
    fpga_blockage_ = FpgaBlockage{odom, odom_target, index_, owned_token_, seconds() + blocked_retry_delay,
                                   now().nanoseconds()};
  }
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr detour_local_map(const WorldSnapshot &w) const {
    if (!mppi_blockage_ && !fpga_blockage_) return w.local_map;
    if (!(fpga_blockage_ ? blockage_observation_ready(w) : detour_inputs(w))) return {};
    const Point robot{w.x, w.y};
    if (!stopped_for_recovery(w) ||
        !clear_body_segment(*w.map, robot, robot, w.yaw, false, {}, w.peer_map.get()))
      return w.local_map;
    return recovery_local_map(w.local_map, *w.map, {w.odom_x, w.odom_y}, robot,
                              w.odom_yaw, w.yaw - w.odom_yaw);
  }
  void update_fpga_blockage(const WorldSnapshot &w) {
    if (!fpga_blockage_) return;
    const Point robot{w.x, w.y}, odom{w.odom_x, w.odom_y};
    auto &block = *fpga_blockage_;
    if (index_ != block.index || block.token != owned_token_ ||
        (w.wheel_trusted && distance(odom, block.start) >= 0.10)) {
      fpga_blockage_.reset();
    } else {
      const auto target = transform_point(block.target, odom, robot, w.yaw - w.odom_yaw);
      // The guard bit can clear as soon as the command goes to zero. Recheck
      // the observed opening until a new path has been accepted.
      bool open = seconds() >= block.retry_at && stopped_for_recovery(w) && blockage_observation_ready(w) &&
                  w.front_range > front_stop_range && !w.front_stop_held && !device_.value("guard_obstacle", 0) &&
                  w.front_scan && !w.front_scan->blocks_translation(odom, block.target, w.odom_yaw) &&
                  clear_body_segment(*w.map, robot, target, w.yaw, false, {}, w.peer_map.get());
      if (open) {
        const double forward = (block.target.x - odom.x) * std::cos(w.odom_yaw) +
                               (block.target.y - odom.y) * std::sin(w.odom_yaw);
        // The latest board map vetoes local hits. Only a front/side departure may
        // use globally confirmed free space to bridge unknown board cells.
        open = clear_observed_departure(*w.local_map, odom, block.target, w.odom_yaw);
        if (!open && forward >= 0.) {
          const auto recovered = detour_local_map(w);
          open = recovered && recovered != w.local_map &&
                 clear_observed_departure(*recovered, odom, block.target, w.odom_yaw);
        }
      }
      if (open == block.open) return;
      block.open = open;
    }
    cancel_plan();
    next_plan_at_ = 0;
  }
  bool clear_observation_turn(const WorldSnapshot &w, double odom_yaw) const {
    const Point robot{w.x, w.y}, odom{w.odom_x, w.odom_y};
    return w.front_scan && !w.front_scan->blocks_turn(odom, w.odom_yaw, odom_yaw) &&
           w.wheel_trusted && w.local_map && !device_.value("guard_missing", 0) &&
           !device_.value("guard_obstacle", 0) &&
           clear_body_segment(*w.map, robot, robot, w.yaw, false, {}, w.peer_map.get(), {},
                              odom_yaw + w.yaw - w.odom_yaw) &&
           clear_body_segment(*w.local_map, odom, odom, w.odom_yaw, true, {}, nullptr, {}, odom_yaw);
  }
  bool start_observation_turn(const WorldSnapshot &w) {
    if (!fpga_blockage_ || fpga_blockage_->open || w.front_stop_held || !(w.front_range > front_stop_range) ||
        !route_.route.points.empty() || reference_.active() || !stopped_for_recovery(w) ||
        !blockage_observation_ready(w) || seconds() < std::max(next_look_at_, fpga_blockage_->retry_at))
      return false;
    const Point robot{w.x, w.y}, odom{w.odom_x, w.odom_y};
    auto &block = *fpga_blockage_;
    const double pi = std::acos(-1.);
    // An interruption before any measured turn gets one more attempt after recovery.
    if ((last_look_from_ && (!retry_look_yaw_ ||
         std::abs(std::remainder(w.odom_yaw - *retry_look_yaw_, 2 * pi)) >= 0.05)) ||
        distance(odom, block.target) <= 1e-8)
      return false;
    next_look_at_ = seconds() + blocked_retry_delay;
    const double bearing = std::atan2(block.target.y - odom.y, block.target.x - odom.x);
    const double target_turn = std::remainder(bearing - w.odom_yaw, 2 * pi);
    // Prefer the side with unobserved space. A rear target can still use its
    // original direct observation turn when the full rotational sweep is clear.
    const double frontier = frontier_heading(*w.local_map, odom, bearing);
    const double side = std::copysign(pi / 4, std::remainder(frontier - w.odom_yaw, 2 * pi));
    std::optional<double> heading;
    if (std::abs(target_turn) > pi / 2 && clear_observation_turn(w, bearing)) heading = w.odom_yaw + target_turn;
    for (double turn : {side, -side, side / 2, -side / 2}) {
      if (!heading && clear_observation_turn(w, w.odom_yaw + turn)) {
        heading = w.odom_yaw + turn;
        break;
      }
    }
    if (!heading) return false;
    reference_.update(w, {robot}, "goal", true, *heading + w.yaw - w.odom_yaw, device_, {});
    if (!reference_.active()) return false;
    block.start = odom;
    observation_turn_ = ObservationTurn{robot, odom, *heading, std::abs(*heading - w.odom_yaw), seconds() + 2.,
                                        last_look_from_ ? std::nullopt : std::optional{w.odom_yaw}};
    last_look_from_ = odom;
    retry_look_yaw_.reset();
    stream_authorized_ = false;
    cancel_plan();
    reason_ = "原地转向观察受阻航段";
    return true;
  }
  bool update_observation_turn(const WorldSnapshot &w) {
    if (!observation_turn_) return false;
    auto &turn = *observation_turn_;
    const Point odom{w.odom_x, w.odom_y};
    const double time = seconds();
    const double remaining = std::abs(std::remainder(turn.yaw - w.odom_yaw, 2 * std::acos(-1.)));
    if (remaining + 0.05 <= turn.remaining) {
      turn.remaining = remaining;
      turn.progress_until = time + 2.;
      turn.retry_from_yaw.reset();
    }
    const bool safe = w.front_range > front_stop_range && !w.front_stop_held &&
                      distance(odom, turn.odom_center) < 0.05 && clear_observation_turn(w, turn.yaw);
    const bool arrived = safe && reference_.arrived_at(device_, "goal", turn.map_center);
    if (safe && !arrived && time < turn.progress_until) return true;
    if (fpga_blockage_) fpga_blockage_->observe_after_ns = now().nanoseconds();
    clear_route();
    request_stop();
    reason_ = arrived ? "转向完成，等待停车及更新后的局部观测" : "观测转向暂停，等待可通行路线";
    return true;
  }
  void launch_plan(const WorldSnapshot &w) {
    const Point robot{w.x, w.y};
    // Nav2 checks only the path endpoint, so each required waypoint ends its own path.
    const auto goal = index_ < points_.size() ? std::optional{points_[index_].point} : std::nullopt;
    const auto final_yaw = index_ + 1 == points_.size() ? std::optional{points_.back().yaw} : std::nullopt;
    const auto blocked = blocked_target(w);
    const auto local = detour_local_map(w);
    auto cancel = std::make_shared<std::atomic_bool>(false);
    cancel_ = cancel;
    job_ =
        std::async(std::launch::async,
                   [map = w.map, peer = w.peer_map, local, odom = Point{w.odom_x, w.odom_y},
                    map_to_odom_yaw = w.odom_yaw - w.yaw, allow_detour = !using_mppi_, blocked, robot,
                    heading = w.yaw, goal, final_yaw, i = index_,
                    use_mppi = enable_mppi_ && !cooperative_ && !force_fpga_,
                    start = confirmed_start(), excluded = excluded_, cancel, task = task_revision_,
                    generation = w.generation, version = w.map_version] {
                     const auto cancelled = [cancel] { return cancel->load(); };
                     auto route = build_route(*map, robot, heading, goal, final_yaw, use_mppi, start, excluded,
                                              cancelled, peer.get());
                     const bool repeats = blocked &&
                         (!route.known || repeats_blocked_leg(*map, route.route.points, 0, robot, *blocked,
                                                              heading, false, start, peer.get(), final_yaw, !use_mppi));
                     const bool terminal_blocked =
                         route.route.reason == "goal_occupied" || route.route.reason == "goal_footprint_blocked";
                     bool used_detour = false;
                     if (local && goal && !terminal_blocked && !cancelled() &&
                         (repeats || (allow_detour && route.route.points.empty()))) {
                       auto detour = local_detour(
                           *map, *local, robot, odom, map_to_odom_yaw, *goal, excluded, start,
                           cancelled, peer.get(), heading, final_yaw, blocked);
                       if (!detour.points.empty()) {
                         route = {std::move(detour), false};
                         used_detour = true;
                       }
                     }
                     if (repeats && !terminal_blocked && !used_detour)
                       route = {Route{{}, "blocked_leg"}, false};
                     return PlanWork{std::move(route), task, generation, version, i, robot, used_detour};
                   });
    next_plan_at_ = seconds() + 0.4;
  }
  void publish_path() {
    Path path;
    path.header.frame_id = "map";
    path.header.stamp = now();
    for (auto p : route_.route.points) {
      Pose pose;
      pose.header = path.header;
      pose.pose.position.x = p.x;
      pose.pose.position.y = p.y;
      pose.pose.orientation.w = 1.;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }
  std::optional<double> final_heading(const MissionRoute &r) const {
    if (index_ + 1 == points_.size() && (r.route.reason == "goal" || r.route.reason == "unknown_goal"))
      return points_.back().yaw;
    return {};
  }
  bool repeats_blockage(const WorldSnapshot &w, const MissionRoute &r, Point blocked, bool mppi) const {
    return repeats_blocked_leg(*w.map, r.route.points, 1, {w.x, w.y}, blocked, w.yaw, !r.known,
                               confirmed_start(), w.peer_map.get(), final_heading(r), !mppi);
  }
  bool detour_usable(const WorldSnapshot &w, const MissionRoute &r) const {
    const auto observed = detour_local_map(w);
    if (!observed) return false;
    const Point robot{w.x, w.y}, odom{w.odom_x, w.odom_y};
    const double theta = w.odom_yaw - w.yaw;
    std::vector<Point> local;
    local.reserve(r.route.points.size());
    for (auto p : r.route.points) local.push_back(transform_point(p, robot, odom, theta));
    const auto final = final_heading(r);
    const auto odom_final = final ? std::optional{*final + theta} : std::nullopt;
    const auto clear = [&](std::optional<double> map_hold, std::optional<double> odom_hold) {
      return clear_ahead(*w.map, r.route.points, 1, robot, true, confirmed_start(), w.peer_map.get(), w.yaw,
                         final, map_hold) &&
             clear_ahead(*observed, local, 1, odom, false, {}, nullptr, w.odom_yaw, odom_final, odom_hold);
    };
    return clear(w.yaw, w.odom_yaw) || clear({}, {});
  }
  void submit_controller_path(const WorldSnapshot &w) {
    const double theta = w.odom_yaw - w.yaw, c = std::cos(theta), s = std::sin(theta);
    const Point translation{w.odom_x - c * w.x + s * w.y, w.odom_y - s * w.x - c * w.y};
    const auto &pts = route_.route.points;
    std::vector<Point> local{{w.odom_x, w.odom_y}};
    for (std::size_t i = next_; i < pts.size(); ++i)
      local.push_back(
          {translation.x + c * pts[i].x - s * pts[i].y, translation.y + s * pts[i].x + c * pts[i].y});
    std_msgs::msg::Header header;
    header.frame_id = "odom";
    header.stamp = now();
    double end_yaw = w.odom_yaw;
    if (auto required = final_heading(route_)) {
      end_yaw = *required + theta;
    } else {
      for (auto it = local.rbegin() + 1; it != local.rend(); ++it) {
        if (distance(*it, local.back()) <= 1e-8) continue;
        end_yaw = std::atan2(local.back().y - it->y, local.back().x - it->x);
        break;
      }
    }
    if (mppi_terminal_abort_ &&
        distance(local.back(), mppi_terminal_abort_->goal) >= mppi_terminal_progress_distance)
      mppi_terminal_abort_.reset();
    follower_.start_or_update(controller_path(local, end_yaw, header, w.map->info.resolution));
    mppi_path_at_ = seconds();
    projected_from_ = translation;
    projected_yaw_ = theta;
  }
  void take_plan(const WorldSnapshot &w) {
    if (!job_.valid() || job_.wait_for(0s) != std::future_status::ready) return;
    auto work = job_.get();
    cancel_.reset();
    if (work.task != task_revision_ || work.generation != w.generation || index_ != work.index ||
        !navigating() || !w.localized || !w.map || waiting_stop_)
      return;
    if (waiting_for_blockage_observation(w)) return;
    const bool changed_map = w.map_version != work.map_version;
    const bool moved = distance({w.x, w.y}, work.start) > 0.10;
    if (work.result.route.points.empty()) {
      const auto &failure_kind = work.result.route.reason;
      const bool environment_blocked = failure_kind == "blocked_leg" || failure_kind == "no_route" ||
                                       failure_kind == "no_reachable_frontier" || failure_kind == "goal_occupied" ||
                                       failure_kind == "goal_footprint_blocked";
      if (environment_blocked && !using_mppi_ && route_.route.points.empty() &&
          index_ < points_.size() && detour_inputs(w) && stopped_for_recovery(w)) {
        if (!fpga_blockage_) {
          const Point robot{w.x, w.y}, odom{w.odom_x, w.odom_y};
          const auto goal = points_[index_].point;
          const double span = distance(robot, goal);
          if (span > 0.10) {
            const double f = std::min(1., obstacle_check_distance / span);
            const Point ahead{robot.x + f * (goal.x - robot.x), robot.y + f * (goal.y - robot.y)};
            const auto local = transform_point(ahead, robot, odom, w.odom_yaw - w.yaw);
            if ((w.front_scan && w.front_scan->blocks_translation(odom, local, w.odom_yaw)) ||
                !clear_body_segment(*w.map, robot, ahead, w.yaw, false, {}, w.peer_map.get()) ||
                !clear_observed_departure(*w.local_map, odom, local, w.odom_yaw))
              remember_blocked_leg(w, local);
          }
        }
        if (start_observation_turn(w)) return;
        if (fpga_blockage_ && last_look_from_) {
          reason_ = "已尝试原地观测，仍无可通行路线，保留航点并自动重试";
          return;
        }
      }
      if (work.result.route.reason == "blocked_leg" && fpga_blockage_ && fpga_blockage_->observe_after_ns) {
        reason_ = "航段仍受阻，保留航点，等待新观测自动重试";
        return;
      }
      const auto failure = plan_failure(work.result.route.reason);
      if (changed_map || moved) {
        // Publish the completed result even during continuous map updates. The next
        // solve starts on the latest snapshot at the normal planning cadence.
        reason_ = (changed_map ? "上一版地图规划结果：" : "上一起点规划结果：") + failure + "，正在重新规划";
      } else if (using_mppi_) {
        reason_ = "MPPI 局部绕障，等待可通行路线：" + failure;
      } else {
        reason_ = "等待可通行路线：" + failure;
        if (!reference_.active() && device_fresh() && device_.value("mode", 0) == 2) request_stop();
      }
      return;
    }
    auto &points = work.result.route.points;
    if (points.size() == 1 || distance(points.front(), work.start) > 1e-8)
      points.insert(points.begin(), work.start);
    const bool mppi = enable_mppi_ && !force_fpga_ && !cooperative_ && work.result.known && !points_.empty();
    if (mppi && !follower_.server_ready()) {
      if (reference_.active()) {
        // The board already owns this leg. Replan with its motion model.
        force_fpga_ = true;
        next_plan_at_ = 0;
        reason_ = "Nav2 控制未就绪，继续使用 FPGA";
      } else {
        handoff_to_fpga("Nav2 控制未就绪，确认停车后交给 FPGA");
      }
      return;
    }
    const auto blocked = blocked_target(w);
    if ((blocked && repeats_blockage(w, work.result, *blocked, mppi)) ||
        (work.detour && (mppi_blockage_ || fpga_blockage_) && (moved || !detour_usable(w, work.result)))) {
      if (!using_mppi_) start_observation_turn(w);
      return;
    }
    if (using_mppi_ && !work.result.known) {
      if (mppi_blockage_ && work.detour) pending_detour_ = std::move(work);
      return;
    }
    if ((changed_map || moved) && (mppi || fpga_blockage_)) {
      const auto ahead = [&](std::optional<double> held) {
        return clear_ahead(*w.map, points, 1, {w.x, w.y}, !work.result.known, confirmed_start(),
                           w.peer_map.get(), w.yaw, final_heading(work.result), held);
      };
      if (!ahead({}) && (mppi || !ahead(w.yaw))) return;
    }
    if (!route_.route.points.empty() && route_.route.reason == work.result.route.reason &&
        points.size() - 1 == route_.route.points.size() - next_ &&
        std::equal(points.begin() + 1, points.end(), route_.route.points.begin() + next_,
                   [](Point a, Point b) { return distance(a, b) < 1e-6; }))
      return;
    if (route_.route.points.empty() || route_.route.reason != work.result.route.reason ||
        distance(route_.route.points.back(), points.back()) > 1e-6)
      frontier_revision_.reset();
    if (mppi_terminal_abort_ &&
        (!mppi || work.index != mppi_terminal_abort_->index ||
         std::any_of(points.begin() + 1, points.end() - 1, [&](Point p) {
           return segment_distance(p, points.front(), points.back()) >= mppi_terminal_progress_distance;
         })))
      mppi_terminal_abort_.reset();
    route_ = std::move(work.result);
    if (fpga_blockage_ && fpga_blockage_->open) fpga_blockage_.reset();
    next_ = 1;
    replacement_needed_ = false;
    mppi_blockage_.reset();
    pending_detour_.reset();
    checked_from_.reset();
    map_checked_ = w.map_version;
    using_mppi_ = mppi;
    reason_ = using_mppi_ ? "MPPI 跟踪航点" : "FPGA 执行局部参考";
    publish_path();
    if (using_mppi_) {
      if (!stream_authorized_) {
        // A previous autonomous segment owns the board until stop is confirmed.
        clear_route();
        request_stop();
        return;
      }
      submit_controller_path(w);
      follower_.tick(w.front_range);
    }
  }
  bool update_mppi_blockage(const WorldSnapshot &w, bool aborted);
  void handoff_to_fpga(const std::string &reason);
  bool handle_mppi_terminal_abort(const WorldSnapshot &w);
  void advance_waypoints(const WorldSnapshot &w);
  void track(const WorldSnapshot &w);
  void tick();
  void report(const WorldSnapshot &w);

  NavigationWorld world_;
  WaypointFollower follower_;
  FpgaReference reference_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<Empty>::SharedPtr stop_pub_;
  rclcpp::Publisher<String>::SharedPtr status_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vision_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool enable_mppi_{}, force_fpga_{}, using_mppi_{}, cooperative_{}, suspended_{}, replacement_needed_{};
  bool waiting_stop_{}, stop_sent_{}, stop_acked_{}, stream_authorized_{}, may_set_start_{};
  std::int64_t stop_stamp_{}, command_after_ns_{};
  std::string mode_{"idle"}, reason_, task_state_{"idle"}, failure_, map_session_;
  std::string waypoint_task_id_;
  std::vector<Waypoint> points_;
  std::vector<Point> excluded_;
  std::size_t index_{}, completed_{}, next_{}, checked_next_{};
  std::optional<Point> previous_, start_point_, checked_from_, projected_from_;
  std::optional<Point> waypoint_sample_;
  bool waypoint_sample_in_odom_{};
  double projected_yaw_{}, checked_yaw_{}, mppi_path_at_{};
  std::optional<Pose> start_pose_;
  std::optional<std::pair<std::uint32_t, std::uint32_t>> epoch_;
  std::optional<std::uint64_t> generation_, frontier_revision_;
  std::uint32_t boot_{}, domain_{};
  std::uint64_t task_revision_{}, map_checked_{};
  std::uint64_t ack_token_{}, owned_token_{};
  std::uint32_t ack_boot_{}, ack_domain_{};
  std::optional<std::uint32_t> ack_sample_us_;
  double ack_at_{}, last_stop_send_at_{};
  std::uint64_t last_travel_revision_{};
  double manual_at_{-10}, vision_at_{}, vision_forward_at_{}, device_at_{-10}, start_at_{};
  double next_plan_at_{}, report_at_{};
  Json device_ = Json::object();
  MissionRoute route_;
  std::optional<MppiBlockage> mppi_blockage_;
  std::optional<MppiTerminalAbort> mppi_terminal_abort_;
  std::optional<FpgaBlockage> fpga_blockage_;
  std::optional<ObservationTurn> observation_turn_;
  std::optional<Point> last_look_from_;
  std::optional<double> retry_look_yaw_;
  double next_look_at_{};
  std::optional<PlanWork> pending_detour_;
  std::shared_ptr<std::atomic_bool> cancel_;
  std::future<PlanWork> job_;
};

bool Mission::update_mppi_blockage(const WorldSnapshot &w, bool aborted) {
  const Point robot{w.x, w.y}, odom{w.odom_x, w.odom_y};
  const bool same_leg = mppi_blockage_ && mppi_blockage_->index == index_ && mppi_blockage_->next == next_;
  const bool moved =
      same_leg && (distance(odom, mppi_blockage_->odom) >= 0.05 ||
                   std::abs(std::remainder(w.odom_yaw - mppi_blockage_->yaw, 2 * std::acos(-1.))) >= 0.05);
  const auto t = seconds();
  double hard_stop_at = same_leg && !moved ? mppi_blockage_->hard_stop_at : -10.;
  std::optional<Point> blocked;
  if (index_ < points_.size() && distance(robot, points_[index_].point) > 0.10 &&
      stream_authorized_ && owned_token_ == device_.value("session_token", std::uint64_t{}) &&
      device_.value("mode", 0) == 1 && detour_inputs(w) && follower_.recently_following(same_leg) &&
      !reference_.temporary_start_active(device_)) {
    const Point target = route_.route.points[next_];
    const double span = distance(robot, target);
    if (span > 1e-8) {
      // The preceding streamed path may still have a 150 ms command in flight.
      if (device_at_ - mppi_path_at_ >= 0.15 && front_stop()) hard_stop_at = device_at_;
      const double f = std::min(1., obstacle_check_distance / span);
      const double heading = std::atan2(target.y - robot.y, target.x - robot.x);
      const Point ahead{robot.x + f * (target.x - robot.x), robot.y + f * (target.y - robot.y)};
      const double theta = w.odom_yaw - w.yaw;
      // Unknown cells alone do not confirm a physical obstacle in the current leg.
      if (t - hard_stop_at < 1. ||
          !clear_body_segment(*w.map, robot, ahead, heading, true, confirmed_start(), w.peer_map.get(), w.yaw) ||
          !clear_body_segment(*w.local_map, odom, transform_point(ahead, robot, odom, theta), heading + theta,
                              true, {}, nullptr, w.odom_yaw))
        blocked = ahead;
    }
  }
  if (!same_leg || !blocked) {
    if (mppi_blockage_ || blocked) {
      mppi_blockage_.reset();
      pending_detour_.reset();
      cancel_plan();
      next_plan_at_ = 0;
    }
    if (!blocked) return false;
  }
  if (!mppi_blockage_ || moved || std::hypot(w.vx, w.vy) > 0.05 || std::abs(w.wz) > 0.05)
    mppi_blockage_ = MppiBlockage{odom, *blocked, w.odom_yaw, t, hard_stop_at, index_, next_};
  mppi_blockage_->target = *blocked;
  mppi_blockage_->hard_stop_at = hard_stop_at;
  reason_ = "当前航段受阻，正在验证绕行";
  if (t - mppi_blockage_->since < 0.50) return false;
  if (pending_detour_) {
    const auto &detour = pending_detour_->result;
    if (distance(robot, pending_detour_->start) > 0.10 ||
        repeats_blockage(w, detour, *blocked, false) || !detour_usable(w, detour))
      pending_detour_.reset();
  }
  // After a followed action aborts at a confirmed blockage, retain that
  // direction across the stop ACK even if the detour search is still running.
  if ((!pending_detour_ && !(aborted && stopped_for_recovery(w))) || !follower_.recently_following(true))
    return false;
  const bool found_detour = pending_detour_.has_value();
  remember_blocked_leg(w, transform_point(*blocked, robot, odom, w.odom_yaw - w.yaw));
  handoff_to_fpga(found_detour ? "航段持续受阻且有可行绕行，确认停车后交给 FPGA"
                               : "受阻航段已中止，确认停车后重新选择可行方向");
  return true;
}

void Mission::handoff_to_fpga(const std::string &reason) {
  force_fpga_ = true;
  may_set_start_ = false;
  start_point_.reset();
  start_pose_.reset();
  clear_route();
  stream_authorized_ = false;
  request_stop();
  reason_ = reason;
}

bool Mission::handle_mppi_terminal_abort(const WorldSnapshot &w) {
  const auto &points = route_.route.points;
  // A failed action retains recent feedback only after Nav2 aborts the followed goal.
  if (index_ + 1 != points_.size() || next_ + 1 != points.size() ||
      !w.wheel_trusted || !projected_from_ || !follower_.recently_following(true))
    return false;

  const Point odom{w.odom_x, w.odom_y};
  const auto goal = transform_point(points.back(), {}, *projected_from_, projected_yaw_);
  const double remaining = distance(odom, goal);
  if (remaining <= 0.10 || remaining > mppi_terminal_retry_radius) return false;

  if (mppi_terminal_abort_ && mppi_terminal_abort_->index == index_) {
    // CPU/self-test, out-of-bounds output, or firmware-forced stop. Missing
    // measurements and command/lease transitions wait behind the recovery gate.
    const auto error = device_.value("error", 0);
    if (device_fresh() && (error == 0x41 || error == 0x47 || error == 0x4d)) {
      finish("blocked", "MPPI 末段连续中止，底盘报告控制故障", true);
    } else {
      remember_blocked_leg(w, goal);
      handoff_to_fpga("MPPI 末段持续无平移进展，保留航点，确认停车及观测就绪后重新规划");
    }
  } else {
    mppi_terminal_abort_ = MppiTerminalAbort{odom, goal, index_};
    clear_route();
    stream_authorized_ = false;
    request_stop();
    reason_ = "MPPI 末段中止，确认停车后重新规划";
  }
  return true;
}

void Mission::advance_waypoints(const WorldSnapshot &w) {
  const Point robot{w.x, w.y};
  if (w.correction_revision != last_travel_revision_) {
    previous_.reset();
    waypoint_sample_.reset();
    last_travel_revision_ = w.correction_revision;
  }
  if (index_ + 1 >= points_.size()) return;
  const Point odom{w.odom_x, w.odom_y};
  Point travel = robot;
  if (waypoint_sample_ && waypoint_sample_in_odom_ == w.wheel_trusted) {
    // Use a single map projection for two wheel samples. Without trusted wheels,
    // compare successive accepted laser positions within the same map revision.
    travel = w.wheel_trusted ? transform_point(*waypoint_sample_, odom, robot, w.yaw - w.odom_yaw)
                             : *waypoint_sample_;
  }
  waypoint_sample_ = w.wheel_trusted ? odom : robot;
  waypoint_sample_in_odom_ = w.wheel_trusted;
  const auto old_index = index_;
  while (index_ + 1 < points_.size() && pass_waypoint(travel, robot, points_[index_].point))
    completed_ = ++index_;
  if (index_ == old_index) return;
  report_at_ = 0;
  last_look_from_.reset();
  next_look_at_ = 0.;
  clear_route();
}

void Mission::track(const WorldSnapshot &w) {
  const Point robot{w.x, w.y};
  const auto &points = route_.route.points;
  if (points.empty()) return;
  const bool board_arrived = reference_.arrived_at(device_, route_.route.reason, points.back());
  const auto action_state = using_mppi_ ? follower_.state() : std::string{};
  if (action_state == "unavailable") {
    handoff_to_fpga("FollowPath 不可用，确认停车后交给 FPGA");
    return;
  }
  const bool goal_leg = route_.route.reason == "goal" || route_.route.reason == "unknown_goal";
  if (index_ + 1 == points_.size() && distance(robot, points_.back().point) <= 0.10 &&
      std::abs(std::remainder(w.yaw - points_.back().yaw, 2 * std::acos(-1.))) <= 0.10 &&
      (action_state == "succeeded" || (board_arrived && goal_leg))) {
    finish("arrived", "已到达最终航点和指定朝向", true);
    return;
  }
  const auto cursor = tracking_vertex(*w.map, points, next_, robot, w.yaw, !route_.known, confirmed_start(),
                                      w.peer_map.get(), final_heading(route_), !using_mppi_, previous_);
  next_ = cursor.next;
  previous_ = robot;
  if (using_mppi_) {
    if (update_mppi_blockage(w, action_state == "failed")) return;
    if ((action_state == "failed" && !mppi_blockage_) || action_state == "succeeded") {
      if (action_state == "failed" && handle_mppi_terminal_abort(w)) return;
      if (action_state == "succeeded") mppi_terminal_abort_.reset();
      clear_route();
      reason_ = "根据当前位置重新规划剩余航点";
      return;
    }
    if (action_state != "failed" && projected_from_) {
      const double c = std::cos(projected_yaw_), s = std::sin(projected_yaw_);
      Point predicted{projected_from_->x + c * w.x - s * w.y, projected_from_->y + s * w.x + c * w.y};
      if (distance(predicted, {w.odom_x, w.odom_y}) >= 0.05 ||
          std::abs(std::remainder(projected_yaw_ - (w.odom_yaw - w.yaw), 2 * std::acos(-1.))) >= 0.10)
        submit_controller_path(w);
    }
    follower_.tick(w.front_range);
  }
  // Check the actual polyline whenever either the map or the lookahead changes.
  if (map_checked_ != w.map_version || !checked_from_ || checked_next_ != next_ ||
      distance(robot, *checked_from_) >= w.map->info.resolution ||
      std::abs(std::remainder(w.yaw - checked_yaw_, 2 * std::acos(-1.))) >= 0.10) {
    map_checked_ = w.map_version;
    checked_from_ = robot;
    checked_next_ = next_;
    checked_yaw_ = w.yaw;
    const auto ahead = [&](std::optional<double> held) {
      return clear_ahead(*w.map, points, next_, robot, !route_.known, confirmed_start(), w.peer_map.get(),
                         w.yaw, final_heading(route_), held);
    };
    const bool clear = (!using_mppi_ && ahead(w.yaw)) || ahead(std::nullopt);
    replacement_needed_ = !clear;
    if (!clear && !mppi_blockage_)
      reason_ = using_mppi_ ? "MPPI 局部绕障，全局路径正在重算" : "前方路线出现障碍，重新规划";
  }
  if (using_mppi_) return;
  if (board_arrived && route_.route.reason == "frontier" && distance(robot, points.back()) <= 0.18) {
    if (!frontier_revision_) frontier_revision_ = w.base_sequence;
    if (*frontier_revision_ == w.base_sequence) {
      reason_ = "前沿观察已到位，等待下一张 SLAM 地图";
      return;
    }
    if (excluded_.size() == 32) excluded_.erase(excluded_.begin());
    excluded_.push_back(points.back());
    clear_route();
    reason_ = "已观察前沿，选择下一个观测点";
    return;
  }
  if (!route_.known && (route_.route.reason == "unknown" || route_.route.reason == "local_fallback") &&
      distance(robot, points.back()) <= 0.10) {
    clear_route();
    return;
  }
  std::vector<Point> remaining{robot};
  double preview = 0;
  auto i = next_;
  for (; i < points.size() && preview < 1.2; ++i) {
    preview += distance(remaining.back(), points[i]);
    remaining.push_back(points[i]);
  }
  if (i < points.size()) remaining.push_back(points.back());
  const bool final = !points_.empty() && index_ + 1 == points_.size() && goal_leg;
  const double desired = points_.empty() ? w.yaw : points_[index_].yaw;
  reference_.update(w, remaining, route_.route.reason, final, desired, device_,
                    confirmed_start() ? start_pose_ : std::nullopt, cursor.retained_corner,
                    index_ + 1 < points_.size() ? std::optional{points_[index_].point} : std::nullopt);
  if (reference_.active()) stream_authorized_ = false;
  if (reference_.needs_replan()) {
    const auto why = reference_.block_reason();
    if (auto blocked = reference_.blocked_target(); blocked && (route_.known || reference_.scan_blocked()) &&
                                                    index_ < points_.size() && w.wheel_trusted)
      remember_blocked_leg(w, *blocked);
    if (auto failed = reference_.excluded_frontier()) {
      if (excluded_.size() == 32) excluded_.erase(excluded_.begin());
      excluded_.push_back(*failed);
    }
    clear_route();
    stream_authorized_ = false;
    request_stop();
    reason_ = "FPGA 局部段持续受阻，重新规划：" + why;
  } else if (reference_.blocked()) {
    const bool moving = device_.value("mode", 0) == 2 || reference_.active();
    if (index_ < points_.size() && w.wheel_trusted && w.front_scan && w.local_map &&
        !device_.value("guard_missing", 0) && distance(robot, points[next_]) > 0.02) {
      const Point odom{w.odom_x, w.odom_y};
      remember_blocked_leg(w, transform_point(points[next_], robot, odom, w.odom_yaw - w.yaw), true);
    }
    const auto next_retry = next_plan_at_;
    clear_route();
    next_plan_at_ = next_retry;
    if (moving) {
      stream_authorized_ = false;
      request_stop();
    }
    reason_ = "局部参考受阻，重新规划";
  }
}

void Mission::tick() {
  auto w = world_.snapshot();
  if (last_look_from_ && w.wheel_trusted && distance(*last_look_from_, {w.odom_x, w.odom_y}) >= 0.10)
    last_look_from_.reset();
  if (mppi_terminal_abort_ &&
      (mppi_terminal_abort_->index != index_ ||
       (w.wheel_trusted && distance({w.odom_x, w.odom_y}, mppi_terminal_abort_->odom) >=
                               mppi_terminal_progress_distance)))
    mppi_terminal_abort_.reset();
  if (job_.valid() && cancel_ && cancel_->load() && job_.wait_for(0s) == std::future_status::ready) {
    job_.get();
    cancel_.reset();
  }
  if (waiting_stop_) {
    // A bridge restart can discard an outstanding volatile Empty and its ACK.
    // Keep the original lower bound so an ACK already in flight still counts.
    if (!stop_acked_ && stop_pub_->get_subscription_count() &&
        (!stop_sent_ || seconds() - last_stop_send_at_ >= 1.)) {
      stop_pub_->publish(Empty{});
      stop_sent_ = true;
      last_stop_send_at_ = seconds();
    }
    if (stop_acked_ && ready_session() && device_at_ > ack_at_) {
      const auto advance = ack_sample_us_ ? std::uint32_t(device_.value("now_us", 0u) - *ack_sample_us_) : 1u;
      if (device_.value("session_token", std::uint64_t{}) != ack_token_ ||
          (ack_sample_us_ && (ack_boot_ != boot_ || ack_domain_ != domain_)))
        request_stop();
      else if (advance && advance < 0x80000000u && stopped()) {
        waiting_stop_ = false;
        stream_authorized_ = true;
        if (owned_token_ && owned_token_ != ack_token_) {
          force_fpga_ = false;
          mppi_terminal_abort_.reset();
        }
        owned_token_ = ack_token_;
        if (fpga_blockage_ && fpga_blockage_->token == owned_token_ && !fpga_blockage_->observe_after_ns)
          fpga_blockage_->retry_at = seconds() + blocked_retry_delay;
      }
    }
  }
  if (mode_ == "vision" && !waiting_stop_ && device_fresh() &&
      device_.value("session_token", std::uint64_t{}) != owned_token_)
    finish("blocked", "视觉任务的底盘会话已更换", true);
  if (mode_ == "vision" && seconds() - vision_at_ > 1.5) finish("idle", "视觉控制心跳超时", true);
  if (navigating()) {
    if (map_session_.empty()) map_session_ = w.map_session;
    if (!map_session_.empty() && !w.map_session.empty() && map_session_ != w.map_session) {
      finish("blocked", "建图会话已更换，请在新地图上重新提交航点", true);
    } else if (!ready_session() || !w.localized || !w.map) {
      const std::string reason = !ready_session() ? "等待 FPGA 控制会话"
                                 : !w.map         ? "等待规划地图"
                                                  : "等待雷达定位：" + w.localization_reason;
      if (generation_ || !route_.route.points.empty())
        suspend(reason);
      else
        reason_ = reason;
    } else if (!waiting_stop_ && owned_token_ &&
               owned_token_ != device_.value("session_token", std::uint64_t{})) {
      suspend("控制会话已更换，等待底盘重新确认停车");
      if (!waiting_stop_) request_stop();
    } else if (generation_ && w.generation != *generation_) {
      generation_ = w.generation;
      suspend("地图位姿已修正，重新定位剩余航点");
    } else if (w.front_stop_held) {
      suspend("雷达前方达到 0.15 米停车线，保留航点，连续确认净空后自动继续");
    } else if (!waiting_stop_) {
      epoch_ = std::make_pair(boot_, domain_);
      generation_ = w.generation;
      map_session_ = w.map_session;
      suspended_ = false;
      if (may_set_start_) {
        may_set_start_ = false;
        start_at_ = seconds();
        start_point_ = Point{w.x, w.y};
        start_pose_.emplace();
        start_pose_->header.frame_id = "odom";
        start_pose_->header.stamp = now();
        start_pose_->pose.position.x = w.odom_x;
        start_pose_->pose.position.y = w.odom_y;
        start_pose_->pose.orientation.w = 1.;
      }
      if (!update_observation_turn(w)) {
        advance_waypoints(w);
        update_fpga_blockage(w);
        const bool tracking_mppi = using_mppi_;
        if (!tracking_mppi) take_plan(w);
        if (!waiting_stop_ && !route_.route.points.empty()) track(w);
        if (tracking_mppi) take_plan(w);
        const bool needs_plan =
            mppi_blockage_ ? !pending_detour_ : route_.route.points.empty() || replacement_needed_;
        if (navigating() && !waiting_stop_ && !observation_turn_ && needs_plan && !job_.valid() &&
            seconds() >= next_plan_at_ && !waiting_for_blockage_observation(w))
          launch_plan(w);
      }
    } else
      reason_ = "等待底盘确认停车";
  }
  if (seconds() >= report_at_) {
    report_at_ = seconds() + 0.2;
    report(w);
  }
}

void Mission::report(const WorldSnapshot &w) {
  Json path = Json::array(), waypoints = Json::array();
  for (const auto &p : points_) waypoints.push_back({{"x", p.point.x}, {"y", p.point.y}, {"yaw", p.yaw}});
  const auto &pts = route_.route.points;
  const auto step = std::max<std::size_t>(1, (pts.size() + 199) / 200);
  for (std::size_t i = 0; i < pts.size(); i += step) path.push_back({pts[i].x, pts[i].y});
  if (!pts.empty() && (pts.size() - 1) % step) path.push_back({pts.back().x, pts.back().y});
  Json grid = nullptr, robot = nullptr, goal = nullptr;
  if (w.map) {
    const auto &m = w.map->info;
    grid = {{"width", m.width},
            {"height", m.height},
            {"resolution", m.resolution},
            {"origin_x", m.origin.position.x},
            {"origin_y", m.origin.position.y},
            {"origin_yaw", yaw(m.origin.orientation)}};
  }
  if (w.localized) robot = {{"x", w.x}, {"y", w.y}, {"yaw", w.yaw}};
  if (!points_.empty() && (navigating() || mode_ == "blocked"))
    goal = {points_[index_].point.x, points_[index_].point.y};
  std::string guard;
  if (mode_ == "vision") {
    if (!ready_session() || waiting_stop_)
      guard = "unavailable";
    else if (device_.value("guard_missing", 0) || device_.value("error", 0) == 0x45)
      guard = "sensor";
    else if (front_stop())
      guard = vision_forward_at_ && device_at_ > vision_forward_at_ ? "front_stop" : "obstacle";
  }
  Json task = {{"state", task_state_},
               {"task_id", waypoint_task_id_},
               {"total", points_.size()},
               {"current_index", points_.empty() ? 0 : index_ + 1},
               {"completed", completed_},
               {"failure", failure_},
               {"points", std::move(waypoints)},
               {"failed_index", task_state_ == "failed" ? Json(index_ + 1) : Json(nullptr)},
               {"controller", using_mppi_           ? "mppi"
                              : reference_.active() ? "fpga"
                                                    : ""},
               {"handoff", force_fpga_ ? (waiting_stop_ ? "stopping" : "fpga") : ""}};
  Json status = {{"mode", mode_},
                 {"reason", reason_},
                 {"owner", mode_ == "vision" ? "vision" : ""},
                 {"vision_guard", guard},
                 {"cooperative", cooperative_},
                 {"peer_map_ready", w.peer_map_ready},
                 {"robot", robot},
                 {"map", grid},
                 {"map_session", w.map_session},
                 {"goal", goal},
                 {"route", std::move(path)},
                 {"route_kind", route_.route.reason},
                 {"waypoint_task", std::move(task)},
                 {"planning_local_map", w.planning_local_map},
                 {"planning_laser_clear", w.planning_laser_clear},
                 {"localization_generation", w.generation},
                 {"localization",
                  {{"state", w.localization_state},
                   {"reason", w.localization_reason},
                   {"reset_counter", w.generation},
                   {"confirmations", w.confirmations},
                   {"required_confirmations", 3},
                   {"laser_age_s", w.laser_age_s},
                   {"wheel_trusted", w.wheel_trusted}}},
                 {"navigation_map", {{"version", w.map_version}, {"localization_generation", w.generation}}},
                 {"fpga_error", device_.contains("error") ? device_["error"] : Json(nullptr)}};
  String msg;
  msg.data = status.dump();
  status_pub_->publish(msg);
}
}  // namespace car_navigation

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_navigation::Mission>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 4);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
