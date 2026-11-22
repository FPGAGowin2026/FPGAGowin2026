#include "car_navigation/fpga_reference.hpp"

#include <algorithm>
#include <cmath>

namespace car_navigation {
namespace {
using namespace std::chrono_literals;
constexpr double departure_lookahead = 0.45;
constexpr double corner_radius = 0.020;
// Reserve the board's position tolerance and another 2 mm for coordinate rounding.
constexpr double pass_target_radius = intermediate_waypoint_radius - corner_radius - 0.002;

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
Point interpolate(Point a, Point b, double u) { return {a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)}; }

struct Preview {
  Point point;
  std::size_t edge;
};

Preview lookahead(const std::vector<Point>& route, double budget, double deviation) {
  Preview best{route.front(), 0};
  for (std::size_t i = 1; i < route.size(); ++i) {
    const auto a = route[i - 1], b = route[i];
    const double length = distance(a, b);
    if (length == 0) continue;
    const auto candidate = length > budget ? interpolate(a, b, budget / length) : b;
    for (std::size_t j = 1; j < i; ++j) {
      if (segment_distance(route[j], route.front(), candidate) > deviation + 1e-9) return best;
    }
    best = {candidate, i};
    if (length >= budget) break;
    budget -= length;
  }
  return best;
}

Point in_odom(const WorldSnapshot& w, Point p) {
  const double angle = w.odom_yaw - w.yaw;
  const double c = std::cos(angle), s = std::sin(angle);
  return {w.odom_x + c * (p.x - w.x) - s * (p.y - w.y), w.odom_y + s * (p.x - w.x) + c * (p.y - w.y)};
}
Point in_map(const WorldSnapshot& w, Point p) {
  const double angle = w.yaw - w.odom_yaw;
  const double c = std::cos(angle), s = std::sin(angle);
  return {w.x + c * (p.x - w.odom_x) - s * (p.y - w.odom_y),
          w.y + s * (p.x - w.odom_x) + c * (p.y - w.odom_y)};
}
}  // namespace

FpgaReference::FpgaReference(rclcpp::Node& node) : node_(node) {
  goal_ = node.create_publisher<Pose>("/fpga/local_goal", 10);
  reference_ = node.create_publisher<Pose>("/fpga/local_reference", 10);
  confirmed_goal_ = node.create_publisher<Path>("/fpga/confirmed_goal", 10);
  confirmed_reference_ = node.create_publisher<Path>("/fpga/confirmed_reference", 10);
  explore_reference_ = node.create_publisher<Path>("/fpga/explore_reference", 10);
  corner_reference_ = node.create_publisher<Path>("/fpga/corner_reference", 10);
}

void FpgaReference::reset() {
  sent_.reset();
  blocked_ = false;
  clear_blockage();
}

void FpgaReference::on_command(const nlohmann::json& status) {
  if (status.contains("confirmed_until_us")) {
    board_start_until_ = status.at("confirmed_until_us").get<std::uint32_t>();
    reported_boot_ = status.at("confirmation_boot_id").get<std::uint32_t>();
    reported_domain_ = status.at("confirmation_odom_domain").get<std::uint32_t>();
  }
  if (!sent_ || !status.is_object() || status.value("goal_stamp_ns", std::int64_t{}) != sent_->stamp) return;
  const auto state = status.value("state", std::string{});
  if (state == "submitted")
    sent_->task = status.value("task_id", 0u);
  else if (state == "reference_unconfirmed")
    sent_->task = 0;
}

bool FpgaReference::temporary_start_active(const nlohmann::json& t) {
  const auto boot = t.value("boot_id", 0u), domain = t.value("odom_domain", 0u);
  bool reported = false;
  if (board_start_until_) {
    const auto remaining = std::uint32_t(*board_start_until_ - t.value("now_us", 0u));
    reported = boot == reported_boot_ && domain == reported_domain_ && remaining && remaining < 0x80000000u;
    if (!reported) board_start_until_.reset();
  }
  return reported || (boot == start_boot_ && domain == start_domain_ && Clock::now() < possible_start_until_);
}

bool FpgaReference::matches(const nlohmann::json& t) const {
  return sent_ && sent_->task && t.is_object() && t.value("task_id", 0u) == sent_->task &&
         t.value("boot_id", 0u) == sent_->boot && t.value("odom_domain", 0u) == sent_->domain &&
         t.value("session_token", std::uint64_t{}) == sent_->token;
}

bool FpgaReference::arrived_at(const nlohmann::json& telemetry, const std::string& kind,
                               Point endpoint) const {
  return matches(telemetry) && telemetry.value("status", 0) == 4 && sent_->route_kind == kind &&
         distance(sent_->route_endpoint, endpoint) <= 1e-6 &&
         sent_->target.role == (kind == "frontier" ? Role::frontier : Role::goal);
}

void FpgaReference::clear_blockage() {
  blockage_.reset();
  scan_block_.reset();
  needs_replan_ = false;
  block_reason_.clear();
  excluded_frontier_.reset();
}

std::optional<Point> FpgaReference::blocked_target() const {
  if (scan_block_) return scan_block_;
  if (needs_replan_ && blockage_ && blockage_->obstacle && sent_ &&
      (sent_->target.role == Role::segment || sent_->target.role == Role::corner ||
       sent_->target.role == Role::approach))
    return sent_->odom;
  return {};
}

bool FpgaReference::observe_blockage(const WorldSnapshot& w, const nlohmann::json& t) {
  const auto word = w.fusion_nav_status;
  const int guard = word && (*word & 0xffu) == 5 ? static_cast<int>((*word >> 8) & 15u) : -1;
  const int error = t.value("error", 0);
  if (t.value("guard_missing", 0) || guard == 3 || guard == 2 || (error && error != 0x4e)) {
    clear_blockage();
    block_reason_ = (t.value("guard_missing", 0) || guard == 3) ? "FPGA 等待有效雷达观测"
                    : guard == 2                                ? "FPGA 局部保护参数或位姿无效"
                                 : "FPGA 执行器闭锁，错误码 " + std::to_string(error);
    return false;
  }
  const bool front_obstacle = t.value("guard_obstacle", 0) || guard == 4;
  const bool obstacle = front_obstacle || guard == 8;
  if (front_obstacle)
    block_reason_ = "FPGA 前方近障";
  else if (guard == 8)
    block_reason_ = "FPGA 制动范围内有障碍";
  else if (guard == 6)
    block_reason_ = "FPGA 制动范围含未观测区域";
  else if (guard == 7)
    block_reason_ = "FPGA 制动范围的观测已过期";
  else if (guard == 5)
    block_reason_ = "FPGA 横移指令低于起动速度";
  else
    block_reason_ = "FPGA 局部规划未产生可执行速度";

  const auto now_us = t.value("now_us", 0u);
  const Point position = w.wheel_trusted ? Point{w.odom_x, w.odom_y} : Point{w.x, w.y};
  const double yaw = w.wheel_trusted ? w.odom_yaw : w.yaw;
  if (!blockage_ || blockage_->wheel_trusted != w.wheel_trusted || blockage_->obstacle != obstacle ||
      distance(position, blockage_->position) >= 0.05 || std::abs(wrap(yaw - blockage_->yaw)) >= 0.05) {
    blockage_ = Blockage{now_us, position, yaw, w.wheel_trusted, obstacle};
  }
  const auto elapsed_us = std::uint32_t(now_us - blockage_->since_us);
  needs_replan_ = elapsed_us >= 2000000u && elapsed_us < 0x80000000u;
  excluded_frontier_.reset();
  const auto& kind = sent_->route_kind;
  if (needs_replan_ && (kind == "frontier" || kind == "unknown" || kind == "local_fallback"))
    excluded_frontier_ = sent_->route_endpoint;
  return true;
}

void FpgaReference::update(const WorldSnapshot& w, const std::vector<Point>& route, const std::string& kind,
                           bool final_waypoint, double goal_yaw, const nlohmann::json& t,
                           std::optional<Pose> start_odom, bool retained_corner,
                           std::optional<Point> intermediate_waypoint) {
  blocked_ = false;
  if (!w.localized || !w.map || route.empty() || !t.is_object()) {
    clear_blockage();
    return;
  }
  if (!w.front_scan) {
    clear_blockage();
    blocked_ = active();
    return;
  }
  const Point robot{w.x, w.y};
  const bool known = kind == "goal";
  std::optional<Point> start_area;
  if (start_odom && start_odom->header.frame_id == "odom") {
    const auto stamp = node_.now();
    const auto age = stamp - rclcpp::Time(start_odom->header.stamp, stamp.get_clock_type());
    if (age.nanoseconds() >= 0 && age.seconds() < 10.0) {
      const auto& p = start_odom->pose.position;
      start_area = in_map(w, {p.x, p.y});
    }
  }
  const Point body{w.odom_x, w.odom_y};
  std::optional<Point> rejected_scan_leg;
  const auto clear = [&](const Target& target, double hold_yaw) {
    const auto end_yaw = target.role == Role::segment || target.role == Role::corner
                             ? std::nullopt : std::optional{target.yaw};
    const auto end = in_odom(w, target.map);
    const double hold = hold_yaw + w.odom_yaw - w.yaw;
    if (w.front_scan->blocks_turn(body, w.odom_yaw, hold)) return false;
    if (w.front_scan->blocks_translation(body, end, hold)) {
      if (!rejected_scan_leg && distance(body, end) > 1e-8) rejected_scan_leg = end;
      return false;
    }
    if (end_yaw) {
      const double final = *end_yaw + w.odom_yaw - w.yaw;
      if (w.front_scan->blocks_turn(end, hold, final) ||
          (distance(body, end) <= 0.10 && w.front_scan->blocks_turn(body, hold, final))) return false;
    }
    if (target.role == Role::align && w.local_map) {
      if (!clear_body_segment(*w.local_map, body, body, w.odom_yaw, true, {}, nullptr, {},
                              target.yaw + w.odom_yaw - w.yaw))
        return false;
    }
    if (target.pass_point) {
      // The fused sweep below requires known free cells. Keep local hits as a
      // veto and require local observation before moving into the rear blind spot.
      if (!w.local_map || !clear_body_segment(*w.local_map, body, end, hold, true, {}, nullptr)) return false;
      const double forward = (end.x - body.x) * std::cos(hold) + (end.y - body.y) * std::sin(hold);
      if (forward < 0. && !clear_observed_departure(*w.local_map, body, end, hold)) return false;
    }
    // The board may start the final turn anywhere within its position tolerance.
    if (end_yaw && distance(robot, target.map) <= 0.10 &&
        !clear_body_segment(*w.map, robot, robot, hold_yaw, target.explore, start_area, w.peer_map.get(),
                            w.yaw, end_yaw))
      return false;
    return clear_body_segment(*w.map, robot, target.map, hold_yaw, target.explore,
                              target.pass_point ? std::nullopt : start_area,
                              w.peer_map.get(), w.yaw, end_yaw);
  };
  const auto reject_scan_leg = [&]() {
    clear_blockage();
    scan_block_ = rejected_scan_leg;
    needs_replan_ = true;
    block_reason_ = "当前参考将进入雷达前方停车扇区";
  };
  const bool confirmed = matches(t);
  const int status = confirmed ? t.value("status", 0) : 0;
  const auto expiry = t.value("expiry_us", 0u), device_now = t.value("now_us", 0u);
  const std::uint32_t time_left = expiry - device_now;
  const bool renew = confirmed && expiry && device_now && (time_left <= 2000000u || time_left >= 0x80000000u);
  bool old_clear = false;
  if (sent_) {
    // A light SLAM correction moves the board's fixed odom reference in the map.
    // The bridge captures the hold yaw separately from the final target yaw.
    auto physical = sent_->target;
    physical.map = in_map(w, sent_->odom);
    physical.yaw = wrap(sent_->odom_yaw + w.yaw - w.odom_yaw);
    const double hold =
        (status == 3 || status == 4 || status == 5) ? w.yaw : wrap(sent_->hold_odom_yaw + w.yaw - w.odom_yaw);
    old_clear = clear(physical, hold);
    // A new planner endpoint need not redirect an already usable measured pass.
    if (physical.pass_point && intermediate_waypoint && old_clear && confirmed && status != 4 &&
        status != 5 && status != 6 && status != 7 && !renew &&
        distance(physical.map, *intermediate_waypoint) <= pass_target_radius + 1e-8)
      return;
  }
  blocked_ = sent_ && !old_clear;
  if (blocked_ && status != 4 && status != 6 && status != 7) {
    // A newly published replacement cannot revoke an unsafe odom reference
    // until the bridge and board actually accept it.
    if (rejected_scan_leg) reject_scan_leg();
    else clear_blockage();
    return;
  }
  if (status == 5 && old_clear) {
    if (!observe_blockage(w, t) || needs_replan_) return;
  } else {
    clear_blockage();
  }
  const auto target_for = [&](Point point, bool corner) -> std::optional<Target> {
    const bool endpoint = distance(point, route.back()) < 1e-6;
    const double range = distance(point, robot);
    Target target{point, w.yaw, corner ? Role::corner : Role::segment, !known};
    if (endpoint && final_waypoint && (known || (kind == "unknown_goal" && range <= 0.10))) {
      const bool turning = range <= 0.10 || (sent_ && sent_->target.role == Role::goal &&
                                             distance(point, sent_->target.map) < 0.05);
      target.role = turning ? Role::goal : Role::approach;
      if (turning) target.yaw = goal_yaw;
    } else if (endpoint && kind == "frontier") {
      target.role = Role::frontier;
      target.yaw = frontier_heading(*w.map, point, w.yaw);
      if (range < 0.18) target.map = robot;
    }
    const bool translation_clear = clear(target, w.yaw);
    const double heading = std::atan2(point.y - robot.y, point.x - robot.x);
    const double turn = std::abs(wrap(heading - w.yaw));
    if ((target.role == Role::goal && range <= 0.10) || distance(robot, target.map) <= 1e-8)
      return translation_clear ? std::optional{target} : std::nullopt;

    const bool rear = turn > std::acos(0.);
    const bool turned_to_rear = status == 4 && sent_->target.role == Role::align &&
                                std::abs(wrap(sent_->odom_yaw - sent_->hold_odom_yaw)) > std::acos(0.);
    const auto nearby = interpolate(body, in_odom(w, point), std::min(1.0, departure_lookahead / range));
    if ((rear || turned_to_rear) && (!sent_ || status == 4) &&
        (!w.local_map || !clear_observed_departure(*w.local_map, body, nearby, w.odom_yaw))) {
      if (w.local_map && !clear_body_segment(*w.local_map, body, nearby, w.odom_yaw, true, {}, nullptr))
        return {};
      // A rear blind spot needs a turn at the current position before checking the departure again.
      Target observation{robot, heading, Role::align, !known};
      if (rear && w.local_map && clear(observation, w.yaw) &&
          clear_body_segment(*w.local_map, body, body, w.odom_yaw, true, {}, nullptr, {},
                             w.odom_yaw + wrap(heading - w.yaw)))
        return observation;
      return {};
    }
    // In a tight passage the board may need to align even for a short translation.
    bool align = !translation_clear && turn > 0.10;
    if (!align && !sent_ && !rear && range > 0.25 && turn > 0.50 && (known || w.local_map)) {
      align = !w.local_map ||
              !clear_body_segment(*w.local_map, body, nearby, w.odom_yaw, !known, std::nullopt, nullptr);
    }
    if (align) {
      auto aligned = target;
      if (aligned.role == Role::approach) aligned.yaw = heading;
      Target rotation{robot, heading, Role::align, !known};
      if (clear(aligned, heading) && clear(rotation, w.yaw)) return rotation;
    }
    if (translation_clear) return target;
    return {};
  };
  const double budget = known ? 1.2 : 0.7;
  const auto preview = retained_corner ? Preview{route[1], 1} : lookahead(route, budget, known ? 0.04 : 0.075);
  const auto i = preview.edge;
  const bool corner = i > 0 && i + 1 < route.size() && distance(preview.point, route[i]) < 1e-8 &&
                      segment_distance(route[i], route[i - 1], route[i + 1]) > 1e-8;
  std::optional<Target> candidate;
  if (intermediate_waypoint && w.local_map && distance(robot, *intermediate_waypoint) <= departure_lookahead) {
    const auto goal = *intermediate_waypoint;
    const double incoming = std::atan2(robot.y - goal.y, robot.x - goal.x);
    const auto previous_rejection = rejected_scan_leg;
    // Start on the approaching side of the acceptance disk. Every candidate
    // must pass the same held-heading/full-footprint checks as a real corner.
    for (double radius : {pass_target_radius, pass_target_radius / 2., 0.}) {
      const int samples = radius > 0. ? 16 : 1;
      for (int i = 0; i < samples; ++i) {
        const int offset = (i % 2 ? 1 : -1) * ((i + 1) / 2);
        const double a = incoming + offset * (2. * std::acos(-1.) / samples);
        const Target pass{{goal.x + radius * std::cos(a), goal.y + radius * std::sin(a)},
                          w.yaw, Role::corner, false, true};
        if (clear(pass, w.yaw)) {
          candidate = pass;
          break;
        }
      }
      if (candidate) break;
    }
    // A rejected alternative is not the route to blacklist.
    rejected_scan_leg = previous_rejection;
  }
  if (!candidate) candidate = target_for(preview.point, retained_corner || corner);
  // Keep the farthest usable vertex if the smoothed reference clips an obstacle.
  for (auto edge = i; !candidate && edge > 1;) {
    const Point vertex = route[--edge];
    if (distance(robot, vertex) > 1e-8) candidate = target_for(vertex, true);
  }
  if (!candidate) {
    const bool can_progress = confirmed ? status != 4 && status != 6 && status != 7 && !renew
                                        : sent_ && Clock::now() - sent_->sent_at < 750ms;
    blocked_ = !old_clear || !can_progress;
    if (blocked_ && rejected_scan_leg) reject_scan_leg();
    return;
  }
  Target next = *candidate;
  // A retained corner inside the board's arrival radius cannot advance this
  // path. Replan from the measured pose instead of resending the same corner.
  if (next.role == Role::corner &&
      (distance(robot, next.map) <= corner_radius ||
       (status == 4 && sent_->target.role == Role::corner && distance(sent_->target.map, next.map) < 1e-8 &&
        distance(sent_->odom, in_odom(w, next.map)) <= corner_radius))) {
    blocked_ = true;
    return;
  }
  const auto now = Clock::now();

  if (sent_ && old_clear && !renew && status != 6 && status != 7) {
    if (!confirmed && now - sent_->sent_at < 750ms) return;
    if (confirmed) {
      const auto& old = sent_->target;
      const double remaining = distance(robot, old.map);
      const bool route_changed =
          sent_->route_kind != kind || distance(sent_->route_endpoint, route.back()) > 1e-6;
      const bool precise = old.role == Role::corner || next.role == Role::corner;
      const bool role_changed = route_changed || old.explore != next.explore ||
                                (old.role != next.role && (precise || distance(old.map, next.map) < 0.05));
      const bool waiting_turn = old.role == Role::align && status != 4;
      const auto corrected = in_odom(w, old.map);
      const bool correction = distance(corrected, sent_->odom) >= 0.05 ||
                              std::abs(wrap(old.yaw + w.odom_yaw - w.yaw - sent_->odom_yaw)) >= 0.10;
      const bool short_of_goal = status == 4 && (next.role == Role::approach || next.role == Role::segment) &&
                                 distance(next.map, robot) > 0.10 && now - sent_->sent_at >= 750ms;
      if (correction && !role_changed && status != 4 && status != 5) {
        if (clear(old, w.yaw)) next = old;
      } else if (waiting_turn) {
        return;
      } else if (!role_changed && old.role == next.role && distance(old.map, next.map) < (precise ? 1e-8 : 0.05) &&
                 !short_of_goal && !(correction && status == 4) &&
                 !((next.role == Role::goal || next.role == Role::align || next.role == Role::frontier) &&
                   std::abs(wrap(old.yaw - next.yaw)) >= 0.10)) {
        return;
      } else if (!role_changed && status != 4 && status != 5 && remaining > 0.25) {
        // Replace a following leg before its endpoint only when the new
        // reference makes real progress along the old direction.
        const double advance = ((next.map.x - robot.x) * (old.map.x - robot.x) +
                                (next.map.y - robot.y) * (old.map.y - robot.y)) /
                               remaining;
        if (!known || old.role != Role::segment || remaining > 0.45 || advance < remaining + 0.10) return;
      }
    }
  }

  const auto stamp = node_.now();
  if (start_odom) {
    const auto age = stamp - rclcpp::Time(start_odom->header.stamp, stamp.get_clock_type());
    const auto& p = start_odom->pose.position;
    const auto original_stamp = rclcpp::Time(start_odom->header.stamp, stamp.get_clock_type()).nanoseconds();
    if (start_odom->header.frame_id != "odom" || age.nanoseconds() < 0 || age.seconds() >= 10.0 ||
        original_stamp == last_start_stamp_ || std::hypot(w.odom_x - p.x, w.odom_y - p.y) > .05 ||
        std::abs(w.vx) > .02 || std::abs(w.vy) > .02 || std::abs(w.wz) > .02) {
      start_odom.reset();
    }
  }
  const Point odom = in_odom(w, next.map);
  const double yaw = wrap(next.yaw + w.odom_yaw - w.yaw);
  Pose local;
  local.header.frame_id = "odom";
  local.header.stamp = stamp;
  local.pose.position.x = odom.x;
  local.pose.position.y = odom.y;
  local.pose.orientation.z = std::sin(yaw / 2);
  local.pose.orientation.w = std::cos(yaw / 2);
  const bool final = next.role != Role::segment;
  if (next.role == Role::corner || start_odom || (next.explore && !final)) {
    const auto publisher =
        next.role == Role::corner ? corner_reference_ :
        next.explore && !final ? explore_reference_ : (final ? confirmed_goal_ : confirmed_reference_);
    if (publisher->get_subscription_count() == 0) return;
    Path path;
    path.header = local.header;
    if (start_odom) path.poses.push_back(*start_odom);
    path.poses.push_back(local);
    publisher->publish(path);
    if (start_odom) {
      const rclcpp::Time original(start_odom->header.stamp, stamp.get_clock_type());
      last_start_stamp_ = original.nanoseconds();
      possible_start_until_ = now + 10s - std::chrono::nanoseconds((stamp - original).nanoseconds());
      start_boot_ = t.value("boot_id", 0u);
      start_domain_ = t.value("odom_domain", 0u);
    }
  } else {
    const auto publisher = final ? goal_ : reference_;
    if (publisher->get_subscription_count() == 0) return;
    publisher->publish(local);
  }
  sent_ = Submission{next,
                     odom,
                     yaw,
                     stamp.nanoseconds(),
                     now,
                     0u,
                     t.value("boot_id", 0u),
                     t.value("odom_domain", 0u),
                     t.value("session_token", std::uint64_t{}),
                     w.odom_yaw,
                     kind,
                     route.back()};
  blocked_ = false;
  clear_blockage();
}
}  // namespace car_navigation
