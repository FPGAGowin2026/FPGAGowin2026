#include "car_navigation/navigation_world.hpp"
#include "car_navigation/grid_route.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iterator>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nlohmann/json.hpp>

namespace car_navigation {
namespace {
using Grid = nav_msgs::msg::OccupancyGrid;
using Odom = nav_msgs::msg::Odometry;
using Scan = sensor_msgs::msg::LaserScan;
using LaserPose = geometry_msgs::msg::PoseWithCovarianceStamped;
using String = std_msgs::msg::String;
using namespace std::chrono_literals;
constexpr double scan_fresh_s = .35;
constexpr double front_clear_s = .75;

double steady_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double nearest_front_hit(const Scan &scan) {
  // The BX72 tests +/-45 degrees in the lidar frame. Include the edge bins
  // since the bridge rounds each return to the nearest scan angle.
  const double sector = std::acos(-1.) / 4 + std::abs(scan.angle_increment) / 2;
  double nearest = 0.;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (std::abs(scan.angle_min + i * double(scan.angle_increment)) <= sector &&
        std::isfinite(range) && range > 0. && range >= scan.range_min && range <= scan.range_max &&
        (nearest == 0. || range < nearest))
      nearest = range;
  }
  return nearest;
}

struct Pose {
  double x{}, y{}, a{};
};

Pose compose(Pose p, Pose q) {
  const double c = std::cos(p.a), s = std::sin(p.a);
  return {p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y, angle(p.a + q.a)};
}

Pose inverse(Pose p) {
  const double c = std::cos(p.a), s = std::sin(p.a);
  return {-c * p.x - s * p.y, s * p.x - c * p.y, -p.a};
}

double yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

Pose planar(const geometry_msgs::msg::Pose &p) { return {p.position.x, p.position.y, yaw(p.orientation)}; }
bool valid_pose(const geometry_msgs::msg::Pose &p) {
  const auto &q = p.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) && std::isfinite(p.position.z) &&
         norm >= 0.25 && norm <= 2.25;
}

bool valid_grid(const Grid &g, const char *frame) {
  const auto &i = g.info;
  return g.header.frame_id == frame && i.width && i.height && i.width <= 2000 && i.height <= 2000 &&
         i.width * i.height <= 500000 && g.data.size() == i.width * i.height && std::isfinite(i.resolution) &&
         i.resolution > 0 && valid_pose(i.origin);
}

bool same_cells(const Grid::ConstSharedPtr &a, const Grid::ConstSharedPtr &b) {
  return a && b && a->info.width == b->info.width && a->info.height == b->info.height &&
         a->info.resolution == b->info.resolution && a->info.origin == b->info.origin && a->data == b->data;
}

std::int64_t ns(const builtin_interfaces::msg::Time &t) {
  return t.sec >= 0 && t.nanosec < 1000000000u ? static_cast<std::int64_t>(t.sec) * 1000000000 + t.nanosec
                                               : 0;
}
double elapsed(std::int64_t now, std::int64_t then) { return static_cast<double>(now - then) * 1e-9; }

bool recent(std::int64_t now, std::int64_t stamp, double limit) {
  return stamp > 0 && elapsed(now, stamp) >= -0.2 && elapsed(now, stamp) < limit;
}

std::optional<std::array<double, 3>> covariance(const LaserPose &p) {
  const auto &v = p.pose.covariance;
  const std::array<int, 3> axes{0, 1, 5};
  for (int i : axes)
    for (int j : axes) {
      if (!std::isfinite(v[i * 6 + j]) || std::abs(v[i * 6 + j] - v[j * 6 + i]) > 1e-6 ||
          v[i * 6 + j] * v[i * 6 + j] > v[i * 6 + i] * v[j * 6 + j] + 1e-9)
        return std::nullopt;
    }
  const double a = v[0], b = v[1], c = v[5], d = v[7], e = v[11], f = v[35];
  if (a < 0 || d < 0 || f < 0 || a > 1 || d > 1 || f > 0.5 ||
      a * d * f + 2 * b * c * e - a * e * e - d * c * c - f * b * b < -1e-9)
    return std::nullopt;
  return std::array<double, 3>{std::max(a, .0004), std::max(d, .0004), std::max(f, .0025)};
}

struct Sample {
  Pose map, odom;
  std::int64_t stamp{};
  std::array<double, 3> variance{};
};

bool consistent(const Sample &a, const Sample &b) {
  const double dt = elapsed(b.stamp, a.stamp);
  return dt > 0 && dt < 1 && std::hypot(a.map.x - b.map.x, a.map.y - b.map.y) <= .12 + .8 * dt &&
         std::abs(angle(a.map.a - b.map.a)) <= .12 + 1.5 * dt;
}

bool wheel_disagrees(const Sample &a, const Sample &b) {
  const Pose predicted = compose(compose(a.map, inverse(a.odom)), b.odom);
  const double heading = a.map.a + .5 * angle(b.odom.a - a.odom.a);
  const double c = std::cos(heading), s = std::sin(heading);
  const double dt = std::min(2., elapsed(b.stamp, a.stamp));
  const std::array<double, 3> noise{.0004 * c * c + .0016 * s * s, .0004 * s * s + .0016 * c * c, .0004};
  const std::array<double, 3> error{b.map.x - predicted.x, b.map.y - predicted.y,
                                    angle(b.map.a - predicted.a)};
  for (int i = 0; i < 3; ++i)
    if (error[i] * error[i] > 9 * (a.variance[i] + b.variance[i] + noise[i] * dt)) return true;
  return false;
}

bool changed_at_robot(Pose a, Pose b, Pose robot, double resolution) {
  const auto p = compose(a, robot), q = compose(b, robot);
  return std::hypot(p.x - q.x, p.y - q.y) > resolution || std::abs(angle(p.a - q.a)) > .03;
}

// Project occupied cell squares, accounting for the rotation between the grids.
// Merely touching a cell edge is not an overlap.
void overlay_hits(Grid &target, const Grid &source, Pose target_from_source,
                  std::vector<std::uint8_t> *live = nullptr) {
  const Pose t =
      compose(inverse(planar(target.info.origin)), compose(target_from_source, planar(source.info.origin)));
  const double r = target.info.resolution, sr = source.info.resolution;
  const double c = std::cos(t.a), s = std::sin(t.a);
  const int w = target.info.width, ht = target.info.height;
  for (std::size_t n = 0; n < source.data.size(); ++n) {
    if (source.data[n] <= 20) continue;
    const double sx = (n % source.info.width + .5) * sr, sy = (n / source.info.width + .5) * sr;
    const double x = t.x + c * sx - s * sy, y = t.y + s * sx + c * sy;
    visit_overlapping_cells({x / r, y / r}, sr / (2 * r), c, s, w, ht, [&](int xx, int yy) {
      const int index = yy * w + xx;
      target.data[index] = 100;
      if (live) (*live)[index] = 1;
      return false;
    });
  }
}

struct ScanEvidence {
  std::vector<std::uint8_t> free;
  std::vector<int> hits;
};

// Project the whole uncertainty square into the board grid. A partial overlap
// at the edge of the rolling window cannot certify the square as free.
bool locally_observed_free(const Grid &local, Pose local_from_grid, double resolution, int x, int y, int pad) {
  const auto p = compose(local_from_grid, {(x + .5) * resolution, (y + .5) * resolution, 0});
  const Point center{p.x / local.info.resolution, p.y / local.info.resolution};
  const double half = (pad + .5) * resolution / local.info.resolution;
  const double c = std::cos(local_from_grid.a), s = std::sin(local_from_grid.a);
  return known_free_square(local, center, half, c, s);
}

void overlay_local_free(Grid &target, const Grid &local, Pose map_from_odom,
                        const std::vector<std::uint8_t> &live) {
  const Pose t = compose(inverse(planar(target.info.origin)), compose(map_from_odom, planar(local.info.origin)));
  const Pose back = inverse(t);
  const double r = target.info.resolution, sr = local.info.resolution;
  const auto center = compose(t, {.5 * local.info.width * sr, .5 * local.info.height * sr, 0});
  const double half = .5 * std::max(local.info.width, local.info.height) * sr / r;
  const int w = target.info.width;
  visit_overlapping_cells({center.x / r, center.y / r}, half, std::cos(t.a), std::sin(t.a),
                          w, target.info.height, [&](int x, int y) {
    const int i = y * w + x;
    if (target.data[i] != 0 && !live[i] && locally_observed_free(local, back, r, x, y, 0))
      target.data[i] = 0;
    return false;
  });
}

ScanEvidence scan_cells(const Grid &grid, const Scan &scan, Pose laser, double speed, double turn,
                        const std::array<double, 3> *variance, const Grid *local = nullptr,
                        Pose map_from_odom = {}) {
  ScanEvidence out;
  if (scan.ranges.size() > 4096 || !(scan.scan_time >= 0 && scan.scan_time <= .5) ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
      !(scan.range_max > scan.range_min && scan.range_min >= 0))
    return out;
  const auto origin = compose(inverse(planar(grid.info.origin)), laser);
  const int w = grid.info.width, h = grid.info.height;
  const double r = grid.info.resolution, ox = origin.x / r, oy = origin.y / r;
  const double half = scan.scan_time > 0 ? std::max(.05, .5 * scan.scan_time) : .10;
  if (variance) out.free.resize(grid.data.size());
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double d = scan.ranges[i];
    if (!std::isfinite(d) || d <= 0 || d < scan.range_min || d > scan.range_max) continue;
    const double a = origin.a + scan.angle_min + i * scan.angle_increment, ux = std::cos(a), uy = std::sin(a);
    const double error = half * (speed + turn * d), hx = ox + ux * d / r, hy = oy + uy * d / r;
    if (error <= 2 * r && hx >= 0 && hx < w && hy >= 0 && hy < h)
      out.hits.push_back(static_cast<int>(hy) * w + static_cast<int>(hx));
    if (!variance || error > .4 * r) continue;
    const double length = (d - error - 2 * r) / r;
    if (!(length > 0)) continue;
    // Clip the ray before sampling so work is bounded by visible grid extent.
    double low = 0, high = length;
    auto clip = [&](double x, double u, int limit) {
      if (std::abs(u) < 1e-12) return x >= 0 && x < limit;
      double t0 = -x / u, t1 = (limit - 1e-7 - x) / u;
      if (t0 > t1) std::swap(t0, t1);
      low = std::max(low, t0);
      high = std::min(high, t1);
      return low <= high;
    };
    if (!clip(ox, ux, w) || !clip(oy, uy, h)) continue;
    const int steps =
        std::max(1, static_cast<int>(std::ceil(2 * (high - low) * std::max(std::abs(ux), std::abs(uy)))));
    for (int k = 0; k <= steps; ++k) {
      const double t = low + (high - low) * k / steps, x = ox + ux * t, y = oy + uy * t;
      if (x >= 0 && x < w && y >= 0 && y < h) out.free[static_cast<int>(y) * w + static_cast<int>(x)] = 1;
    }
  }
  if (!variance) return out;
  const Pose local_from_grid =
      local ? compose(inverse(compose(map_from_odom, planar(local->info.origin))), planar(grid.info.origin))
            : Pose{};
  // Use summed areas to require a fully observed 2-sigma neighbourhood.
  struct Area {
    int free{}, hits{};
  };
  std::vector<Area> sums((w + 1) * (h + 1));
  for (int n : out.hits) {
    out.free[n] = 0;
    sums[(n / w + 1) * (w + 1) + n % w + 1].hits = 1;
  }
  for (int y = 0; y < h; ++y) {
    Area row;
    for (int x = 0; x < w; ++x) {
      auto &sum = sums[(y + 1) * (w + 1) + x + 1];
      const auto &above = sums[y * (w + 1) + x + 1];
      row.free += out.free[y * w + x];
      row.hits += sum.hits;
      sum = {above.free + row.free, above.hits + row.hits};
    }
  }
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      if (!out.free[y * w + x]) continue;
      const double margin = 2 * (std::sqrt(std::max((*variance)[0], (*variance)[1])) / r +
                                 std::hypot(x + .5 - ox, y + .5 - oy) * std::sqrt((*variance)[2]));
      const int pad = static_cast<int>(std::ceil(std::min(margin, static_cast<double>(std::max(w, h)))));
      const int x0 = x - pad, x1 = x + pad + 1, y0 = y - pad, y1 = y + pad + 1;
      auto &free = out.free[y * w + x];
      if (x0 < 0 || x1 > w || y0 < 0 || y1 > h) {
        free = 0;
        continue;
      }
      const auto &low_left = sums[y0 * (w + 1) + x0], &low_right = sums[y0 * (w + 1) + x1];
      const auto &high_left = sums[y1 * (w + 1) + x0], &high_right = sums[y1 * (w + 1) + x1];
      const int observed = high_right.free - high_left.free - low_right.free + low_left.free;
      const int hits = high_right.hits - high_left.hits - low_right.hits + low_left.hits;
      free = observed == (x1 - x0) * (y1 - y0) ||
             (local && !hits && locally_observed_free(*local, local_from_grid, r, x, y, pad));
    }
  return out;
}
}  // namespace

bool FrontScan::blocks_translation(Point from, Point to, double yaw) const {
  const double c = std::cos(yaw + mount_yaw), s = std::sin(yaw + mount_yaw);
  const double dx = (to.x - from.x) * c + (to.y - from.y) * s;
  const double dy = -(to.x - from.x) * s + (to.y - from.y) * c;
  const double len2 = dx * dx + dy * dy, slope = std::tan(half_angle);
  const Point laser{from.x + std::cos(yaw) * mount.x - std::sin(yaw) * mount.y,
                    from.y + std::sin(yaw) * mount.x + std::cos(yaw) * mount.y};
  for (const auto hit : hits) {
    const double x = (hit.x - laser.x) * c + (hit.y - laser.y) * s;
    const double y = -(hit.x - laser.x) * s + (hit.y - laser.y) * c;
    double lo = 0., hi = 1.;
    // Clip the relative hit trajectory to the two sides of the laser cone.
    for (double sign : {-1., 1.}) {
      const double value = x * slope + sign * y, rate = dx * slope + sign * dy;
      if (rate > 0.) hi = std::min(hi, value / rate);
      else if (rate < 0.) lo = std::max(lo, value / rate);
      else if (value < 0.) {
        hi = -1.;
        break;
      }
    }
    if (lo > hi) continue;
    const double t = len2 > 0. ? std::clamp((x * dx + y * dy) / len2, lo, hi) : lo;
    if (std::hypot(x - t * dx, y - t * dy) <= front_stop_range) return true;
  }
  return false;
}

bool FrontScan::blocks_turn(Point center, double from_yaw, double to_yaw) const {
  const double turn = angle(to_yaw - from_yaw);
  if (std::abs(turn) < 1e-8) return false;
  const double reach = std::hypot(mount.x, mount.y) + front_stop_range;
  const double slope = std::tan(half_angle);
  // The hit rotates in laser-aligned body axes about the robot centre. Within
  // the cone its minimum range is at an endpoint, a side crossing or the
  // bearing of the laser mount. Test those angles on the commanded arc.
  const double c = std::cos(mount_yaw), s = std::sin(mount_yaw);
  const Point origin{mount.x * c + mount.y * s, -mount.x * s + mount.y * c};
  for (const auto hit : hits) {
    const double dx = hit.x - center.x, dy = hit.y - center.y, r = std::hypot(dx, dy);
    if (r > reach) continue;
    const double a = std::atan2(dy, dx) - from_yaw - mount_yaw;
    const auto on_arc = [&](double b) {
      const double step = angle(a - b);
      return turn > 0. ? step >= 0. && step <= turn : step <= 0. && step >= turn;
    };
    const auto inside = [&](double b) {
      const double x = r * std::cos(b) - origin.x, y = r * std::sin(b) - origin.y;
      return x * slope >= std::abs(y) && std::hypot(x, y) <= front_stop_range;
    };
    const double closest = std::atan2(origin.y, origin.x);
    if (inside(a) || inside(a - turn) || (on_arc(closest) && inside(closest))) return true;
    for (double side : {-half_angle, half_angle}) {
      const double u = std::cos(side), v = std::sin(side), dot = origin.x * u + origin.y * v;
      const double cross = origin.x * v - origin.y * u;
      const double disc = r * r - cross * cross;
      if (disc < 0.) continue;
      const double root = std::sqrt(disc);
      for (const double t : {-dot - root, -dot + root})
        if (t >= 0. && t <= front_stop_range && on_arc(std::atan2(origin.y + t * v, origin.x + t * u)))
          return true;
    }
  }
  return false;
}

struct NavigationWorld::Impl {
  rclcpp::Node &node;
  tf2_ros::Buffer tf;
  tf2_ros::TransformListener listener;
  rclcpp::CallbackGroup::SharedPtr input_group, fusion_group;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<Grid>::SharedPtr observation_pub, map_pub;
  mutable std::mutex mutex;
  std::mutex publish_mutex;
  Grid::ConstSharedPtr base, local, peer, observation, navigation;
  Odom::ConstSharedPtr odom;
  std::deque<Odom::ConstSharedPtr> wheel_history;
  LaserPose::ConstSharedPtr pending_pose;
  std::deque<Scan::ConstSharedPtr> scans;
  std::shared_ptr<const FrontScan> projected_scan;
  std::int64_t projected_stamp{};
  std::deque<Sample> history;
  std::optional<Sample> accepted, candidate;
  std::optional<Pose> retained_transform, committed_from_odom;
  std::string session, state{"waiting"}, reason{"waiting_for_laser"};
  std::uint64_t generation{}, map_version{}, epoch{}, base_revision{}, view_revision{}, committed_epoch{};
  std::uint32_t boot{}, domain{};
  unsigned confirmations{}, agreements{};
  std::optional<std::uint32_t> fusion_nav_status;
  bool wheel_ok{true}, plane_ok{true}, cooperative{}, local_used{}, laser_clear{};
  bool front_stop_held{};
  std::int64_t clear_scan_start{};
  double clear_received_start{};
  double local_invalidated_at{}, fpga_ttl_s{4.}, odom_at{}, base_at{}, local_at{}, scan_at{}, health_at{},
      laser_speed{}, laser_turn{}, front_range{};
  double committed_until{};
  std::uint64_t base_sequence{}, committed_sequence{};
  std::int64_t last_laser{}, health_stamp{}, input_after{};
  // Clear evidence belongs to the fusion callback group. World coordinates survive map resizing.
  struct Evidence {
    std::int64_t stamp{};
    unsigned count{};
  };
  std::map<std::pair<std::int64_t, std::int64_t>, Evidence> cleared;
  std::uint64_t evidence_epoch{~std::uint64_t{0}};
  double evidence_resolution{};
  std::int64_t previous_observation{}, previous_fusion_time{};
  bool local_has_free{};

  explicit Impl(rclcpp::Node &n) : node(n), tf(node.get_clock()), listener(tf) {
    input_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    fusion_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = input_group;
    auto retained = rclcpp::QoS(1).reliable().transient_local();
    observation_pub = node.create_publisher<Grid>("/robot/observation_map", retained);
    map_pub = node.create_publisher<Grid>("/robot/navigation_map", 1);
    subscriptions.push_back(node.create_subscription<Grid>(
        "/map", retained,
        [this](Grid::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
          if (!valid_grid(*msg, "map")) return;
          std::ostringstream id;
          for (auto b : info.get_rmw_message_info().publisher_gid.data)
            id << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(b);
          const std::string next = id.str();
          Grid::ConstSharedPtr previous;
          {
            std::lock_guard<std::mutex> lock(mutex);
            previous = base;
          }
          const bool changed_grid = !same_cells(previous, msg);
          std::unique_lock<std::mutex> lock(mutex);
          const bool changed_session = !session.empty() && session != next;
          if (changed_session) {
            reset("map_session_changed");
            peer.reset();
          }
          session = next;
          base = std::move(msg);
          base_at = steady_now();
          ++base_sequence;
          if (changed_grid || changed_session) ++base_revision;
          lock.unlock();
          if (changed_session) publish_current();
        },
        opt));
    subscriptions.push_back(node.create_subscription<Odom>(
        "/odom", rclcpp::SensorDataQoS(),
        [this](Odom::ConstSharedPtr msg) {
          if (msg->header.frame_id != "odom" || msg->child_frame_id != "base_link" ||
              !valid_pose(msg->pose.pose))
            return;
          const auto &v = msg->twist.twist;
          if (!std::isfinite(v.linear.x) || !std::isfinite(v.linear.y) || !std::isfinite(v.angular.z)) return;
          {
            std::lock_guard<std::mutex> lock(mutex);
            const auto stamp = ns(msg->header.stamp);
            if (stamp <= input_after || (odom && stamp <= ns(odom->header.stamp))) return;
            odom = std::move(msg);
            odom_at = steady_now();
            wheel_history.push_back(odom);
            while (wheel_history.size() > 256 || elapsed(stamp, ns(wheel_history.front()->header.stamp)) > 2.)
              wheel_history.pop_front();
          }
          try_pose();
        },
        opt));
    subscriptions.push_back(node.create_subscription<LaserPose>(
        "/pose", rclcpp::QoS(1),
        [this](LaserPose::ConstSharedPtr msg) {
          if (msg->header.frame_id != "map" || !valid_pose(msg->pose.pose)) return;
          {
            std::lock_guard<std::mutex> lock(mutex);
            const auto stamp = ns(msg->header.stamp);
            if (stamp <= std::max(last_laser, input_after) || !recent(node.now().nanoseconds(), stamp, 1.) ||
                (pending_pose && ns(pending_pose->header.stamp) >= stamp))
              return;
            pending_pose = std::move(msg);
          }
          try_pose();
        },
        opt));
    subscriptions.push_back(node.create_subscription<Scan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](Scan::ConstSharedPtr msg) {
          const auto stamp = ns(msg->header.stamp);
          const auto projected = project_scan(*msg, stamp);
          std::lock_guard<std::mutex> lock(mutex);
          const auto clock = node.now().nanoseconds();
          const auto previous = scans.empty() ? 0 : ns(scans.back()->header.stamp);
          if (!plane_ok || stamp <= input_after || msg->header.frame_id.empty() ||
              stamp <= previous || elapsed(clock, stamp) < -.2) {
            clear_scan_start = 0;
            return;
          }
          const auto received = steady_now();
          if (projected) {
            projected_scan = projected;
            projected_stamp = stamp;
          } else if (previous > projected_stamp && scans.back()->header.frame_id == msg->header.frame_id) {
            // A scan can finish before its last odometry TF reaches this process.
            if (auto bracketed = project_scan(*scans.back(), previous)) {
              projected_scan = std::move(bracketed);
              projected_stamp = previous;
            }
          }
          front_range = nearest_front_hit(*msg);
          if (!recent(clock, stamp, scan_fresh_s) || front_range == 0.) {
            clear_scan_start = 0;
          } else if (front_range <= front_stop_range) {
            front_stop_held = true;
            clear_scan_start = 0;
          } else if (front_stop_held) {
            // A single longer return can be a missed near point. Require continuous
            // clear scans in both sensor time and receipt time before resuming.
            if (!clear_scan_start || !previous || elapsed(stamp, previous) >= scan_fresh_s ||
                received - scan_at >= scan_fresh_s || msg->header.frame_id != scans.back()->header.frame_id) {
              clear_scan_start = stamp;
              clear_received_start = received;
            } else if (elapsed(stamp, clear_scan_start) >= front_clear_s &&
                       received - clear_received_start >= front_clear_s) {
              front_stop_held = false;
              clear_scan_start = 0;
            }
          }
          scans.push_back(std::move(msg));
          scan_at = received;
          if (scans.size() > 20) scans.pop_front();
        },
        opt));
    subscriptions.push_back(node.create_subscription<Grid>(
        "/fpga/local_map", 1,
        [this](Grid::ConstSharedPtr msg) {
          if (!valid_grid(*msg, "odom")) return;
          const bool has_free =
              std::any_of(msg->data.begin(), msg->data.end(), [](int v) { return v >= 0 && v <= 20; });
          std::lock_guard<std::mutex> lock(mutex);
          const auto stamp = ns(msg->header.stamp);
          if (!plane_ok || stamp <= input_after || (local && stamp <= ns(local->header.stamp)) ||
              !recent(node.now().nanoseconds(), stamp, 1.5))
            return;
          local = std::move(msg);
          local_at = steady_now();
          local_has_free = has_free;
        },
        opt));
    subscriptions.push_back(node.create_subscription<Grid>(
        "/swarm/car/peer_map", retained,
        [this](Grid::ConstSharedPtr msg) {
          const auto next = valid_grid(*msg, "map") ? msg : nullptr;
          bool changed_view = false;
          Grid::ConstSharedPtr previous;
          {
            std::lock_guard<std::mutex> lock(mutex);
            previous = peer;
          }
          if ((!previous && !next) || same_cells(previous, next)) return;
          {
            std::lock_guard<std::mutex> lock(mutex);
            peer = next;
            if (cooperative) {
              ++view_revision;
              ++map_version;
              navigation = observation;
              changed_view = true;
            }
          }
          if (changed_view) publish_current(false);
        },
        opt));
    subscriptions.push_back(node.create_subscription<String>(
        "/fpga/navigation_status", 10,
        [this](String::ConstSharedPtr msg) {
          auto j = nlohmann::json::parse(msg->data, nullptr, false);
          if (!j.is_object() || !j.contains("boot_id") || !j["boot_id"].is_number_unsigned() ||
              !j.contains("odom_domain") || !j["odom_domain"].is_number_unsigned())
            return;
          const auto b = j["boot_id"].get<std::uint32_t>(), d = j["odom_domain"].get<std::uint32_t>();
          if (!b || !d) return;
          std::unique_lock<std::mutex> lock(mutex);
          const bool changed_domain = boot && (b != boot || d != domain);
          if (changed_domain) {
            reset("odom_domain_changed");
            fpga_ttl_s = 4.;
          }
          boot = b;
          domain = d;
          lock.unlock();
          if (changed_domain) publish_current();
        },
        opt));
    subscriptions.push_back(node.create_subscription<String>(
        "/fpga/fusion_health", 10,
        [this](String::ConstSharedPtr msg) {
          auto j = nlohmann::json::parse(msg->data, nullptr, false);
          if (!j.is_object() || !j.contains("stamp_ns") || !j["stamp_ns"].is_number_integer() ||
              !j.contains("scan_plane_valid") || !j["scan_plane_valid"].is_boolean())
            return;
          const auto stamp = j["stamp_ns"].get<std::int64_t>();
          std::unique_lock<std::mutex> lock(mutex);
          if (stamp <= health_stamp || stamp > node.now().nanoseconds() + 200000000) return;
          health_stamp = stamp;
          health_at = steady_now();
          fusion_nav_status.reset();
          if (auto it = j.find("nav_status");
              it != j.end() && it->is_number_unsigned() && it->get<std::uint64_t>() <= UINT32_MAX)
            fusion_nav_status = it->get<std::uint32_t>();
          const bool was_valid = plane_ok;
          plane_ok = j["scan_plane_valid"].get<bool>();
          if (!plane_ok) {
            if (was_valid) retire_layers();
            lose("invalid_scan_plane");
            input_after = std::max(input_after, stamp);
            local.reset();
            scans.clear();
            projected_scan.reset();
            pending_pose.reset();
            local_invalidated_at = steady_now();
            lock.unlock();
            if (was_valid) publish_current();
          }
        },
        opt));
    timer = node.create_wall_timer(200ms, [this]() { fuse(); }, fusion_group);
  }

  void lose(const char *why) {
    state = "lost";
    reason = why;
    wheel_ok = false;
    candidate.reset();
    confirmations = agreements = 0;
  }

  void retire_layers() {
    ++epoch;
    ++map_version;
    navigation.reset();
    observation.reset();
    local_used = laser_clear = false;
  }

  void reset(const char *why) {
    ++generation;
    retire_layers();
    lose(why);
    input_after = node.now().nanoseconds();
    health_stamp = input_after;
    fusion_nav_status.reset();
    last_laser = 0;
    accepted.reset();
    pending_pose.reset();
    history.clear();
    retained_transform.reset();
    odom.reset();
    wheel_history.clear();
    scans.clear();
    projected_scan.reset();
    local.reset();
    local_invalidated_at = steady_now();
    laser_speed = laser_turn = 0;
  }

  std::optional<Pose> transform(const char *target, const std::string &source, std::int64_t stamp) {
    try {
      const auto t =
          tf.lookupTransform(target, source, rclcpp::Time(stamp, node.get_clock()->get_clock_type()),
                             rclcpp::Duration::from_seconds(0))
              .transform;
      const Pose p{t.translation.x, t.translation.y, yaw(t.rotation)};
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.a)) return p;
      return std::nullopt;
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
  }

  std::shared_ptr<const FrontScan> project_scan(const Scan &scan, std::int64_t stamp) {
    if (scan.header.frame_id.empty() || !recent(node.now().nanoseconds(), stamp, scan_fresh_s)) return {};
    const auto mount = transform("base_link", scan.header.frame_id, stamp);
    const auto lidar = transform("odom", scan.header.frame_id, stamp);
    if (!mount || !lidar) return {};
    auto out = std::make_shared<FrontScan>();
    out->mount = {mount->x, mount->y};
    out->mount_yaw = mount->a;
    out->half_angle = std::acos(-1.) / 4 + std::abs(scan.angle_increment) / 2;
    out->hits.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range <= 0. || range < scan.range_min || range > scan.range_max) continue;
      const double a = lidar->a + scan.angle_min + i * double(scan.angle_increment);
      out->hits.push_back({lidar->x + range * std::cos(a), lidar->y + range * std::sin(a)});
    }
    return out;
  }

  void try_pose() {
    LaserPose::ConstSharedPtr msg;
    {
      std::lock_guard<std::mutex> lock(mutex);
      msg = pending_pose;
    }
    if (!msg) return;
    const auto stamp = ns(msg->header.stamp);
    const auto aligned = transform("odom", "base_link", stamp);
    if (!aligned) return;  // Odometry and its TF may arrive after the matching scan.
    const auto variance = covariance(*msg);
    std::unique_lock<std::mutex> lock(mutex);
    if (pending_pose != msg) return;
    pending_pose.reset();
    if (!plane_ok || stamp <= std::max(last_laser, input_after) ||
        !recent(node.now().nanoseconds(), stamp, 1.))
      return;
    const auto old_epoch = epoch;
    last_laser = stamp;
    if (!variance) {
      const bool was_tracking = state == "tracking";
      if (was_tracking) retire_layers();
      lose("invalid_covariance");
      lock.unlock();
      if (was_tracking) publish_current();
      return;
    }
    Sample next{planar(msg->pose.pose), *aligned, stamp, *variance};
    auto motion_anchor = accepted;
    bool conflict = accepted && wheel_disagrees(*accepted, next);
    for (const auto &anchor : history) {
      const double dt = elapsed(stamp, anchor.stamp);
      if (dt >= .8 && dt <= 2.) {
        conflict = conflict || wheel_disagrees(anchor, next);
        break;
      }
    }
    if ((!accepted && state != "waiting") ||
        (accepted && (state != "tracking" || !consistent(*accepted, next) || (conflict && wheel_ok)))) {
      const bool was_tracking = state == "tracking";
      if (was_tracking) retire_layers();
      confirmations = (candidate && consistent(*candidate, next)) ? confirmations + 1 : 1;
      motion_anchor = candidate;
      candidate = next;
      state = "recovering";
      reason = "confirming_laser_pose";
      wheel_ok = false;
      agreements = 0;
      if (confirmations < 3) {
        lock.unlock();
        if (was_tracking) publish_current();
        return;
      }
      ++generation;
      retire_layers();
      history.clear();
      candidate.reset();
      retained_transform.reset();
      local_invalidated_at = steady_now();
      reason = "laser_recovered";
    } else if (conflict) {
      wheel_ok = false;
      agreements = 0;
      reason = "wheel_laser_disagreement";
    } else {
      if (++agreements >= 3) wheel_ok = true;
      reason = wheel_ok ? "laser_tracking" : "confirming_wheel_motion";
    }
    if (motion_anchor) {
      // After relocalization, the last two confirming poses share a frame.
      const double dt = elapsed(stamp, motion_anchor->stamp);
      laser_speed = std::hypot(next.map.x - motion_anchor->map.x, next.map.y - motion_anchor->map.y) / dt;
      laser_turn = std::abs(angle(next.map.a - motion_anchor->map.a)) / dt;
    }
    state = "tracking";
    history.push_back(next);
    accepted = next;
    if (history.size() > 16) history.pop_front();
    const Pose tf_now = compose(next.map, inverse(next.odom));
    if (!retained_transform)
      retained_transform = tf_now;
    else if (changed_at_robot(*retained_transform, tf_now, next.odom, base ? base->info.resolution : .05)) {
      ++epoch;
      retained_transform = tf_now;
      // Keep the last valid map paired with its projection until fuse() commits
      // the corrected pair. Publishing the bare SLAM map here revives old walls.
    }
    if (!wheel_ok) local_invalidated_at = steady_now();
    const bool retired = old_epoch != epoch && !navigation;
    lock.unlock();
    if (retired) publish_current();
  }

  WorldSnapshot read_locked(double now, std::int64_t clock) const {
    WorldSnapshot w;
    w.map_session = session;
    w.generation = generation;
    w.correction_revision = epoch;
    w.map_version = map_version;
    w.base_sequence = committed_sequence;
    w.localization_state = state;
    w.localization_reason = reason;
    w.confirmations = confirmations;
    // The coordinator sends a retained map only when it changes, and an empty map to revoke it.
    w.peer_map_ready = static_cast<bool>(peer);
    if (cooperative) w.peer_map = peer;
    if (now - health_at < .5 && recent(clock, health_stamp, .5)) w.fusion_nav_status = fusion_nav_status;
    if (base && now < committed_until) {
      w.map = navigation;
      w.observation_map = observation;
      if (w.map) w.correction_revision = committed_epoch;
      w.planning_local_map = navigation && local_used;
      w.planning_laser_clear = navigation && laser_clear;
    }
    w.front_stop_held = front_stop_held;
    w.scan_fresh = plane_ok && !scans.empty() && now - scan_at < scan_fresh_s &&
                   recent(clock, ns(scans.back()->header.stamp), scan_fresh_s);
    if (w.scan_fresh) w.front_range = front_range;
    if (w.scan_fresh && recent(clock, projected_stamp, scan_fresh_s)) w.front_scan = projected_scan;
    if (accepted) w.laser_age_s = std::max(0., elapsed(clock, accepted->stamp));
    const bool laser_fresh = accepted && recent(clock, accepted->stamp, 1.);
    if (state == "tracking" && !laser_fresh) {
      w.localization_state = "lost";
      w.localization_reason = "laser_timeout";
      w.confirmations = 0;
    }
    if (!odom) return w;
    const Pose o = planar(odom->pose.pose);
    w.odom_x = o.x;
    w.odom_y = o.y;
    w.odom_yaw = o.a;
    w.vx = odom->twist.twist.linear.x;
    w.vy = odom->twist.twist.linear.y;
    w.wz = odom->twist.twist.angular.z;
    w.localized = state == "tracking" && plane_ok && laser_fresh && now - odom_at < .5 &&
                  recent(clock, ns(odom->header.stamp), .5);
    w.wheel_trusted = w.localized && wheel_ok;
    if (w.localized) {
      const Pose projection = w.map && committed_epoch != epoch && committed_from_odom
                                  ? *committed_from_odom : compose(accepted->map, inverse(accepted->odom));
      const Pose p = wheel_ok ? compose(projection, o) : accepted->map;
      w.x = p.x;
      w.y = p.y;
      w.yaw = p.a;
    }
    if (w.wheel_trusted && local_has_free && local_ready(now, clock)) w.local_map = local;
    return w;
  }

  bool local_ready(double now, std::int64_t clock) const {
    const double quarantine_until = local_invalidated_at + fpga_ttl_s;
    return local && local_at >= quarantine_until && now - local_at < 1.5 &&
           recent(clock, ns(local->header.stamp), 1.5) &&
           now - std::max(0., elapsed(clock, ns(local->header.stamp))) >= quarantine_until;
  }

  // The SLAM observation and its wheel motion must describe the same instant.
  // Bracketing samples bound rotational and translational scan distortion.
  std::optional<std::pair<double, double>> motion_at(std::int64_t stamp) const {
    const auto it = std::lower_bound(
        wheel_history.begin(), wheel_history.end(), stamp,
        [](const Odom::ConstSharedPtr &o, std::int64_t t) { return ns(o->header.stamp) < t; });
    if (it == wheel_history.end()) return std::nullopt;
    const auto &after = **it;
    if (ns(after.header.stamp) != stamp && it == wheel_history.begin()) return std::nullopt;
    const auto &before = ns(after.header.stamp) == stamp ? after : **std::prev(it);
    if (elapsed(ns(after.header.stamp), ns(before.header.stamp)) > .25) return std::nullopt;
    const auto &a = before.twist.twist;
    const auto &b = after.twist.twist;
    return std::make_pair(std::max(std::hypot(a.linear.x, a.linear.y), std::hypot(b.linear.x, b.linear.y)),
                          std::max(std::abs(a.angular.z), std::abs(b.angular.z)));
  }

  void publish_current(bool publish_observation = true) {
    // Outgoing frames from a completed fusion and a concurrent controller-mode
    // change must leave in state order. Never serialize DDS under the telemetry lock.
    std::lock_guard<std::mutex> send(publish_mutex);
    WorldSnapshot w;
    {
      std::lock_guard<std::mutex> lock(mutex);
      w = read_locked(steady_now(), node.now().nanoseconds());
    }
    Grid empty;
    empty.header.frame_id = "map";
    if (publish_observation) observation_pub->publish(w.observation_map ? *w.observation_map : empty);
    map_pub->publish(w.map ? *w.map : empty);
  }

  void fuse() {
    const double now = steady_now();
    const auto clock = node.now().nanoseconds();
    Grid::ConstSharedPtr raw, local_grid, other, previous_view;
    Scan::ConstSharedPtr matched, latest;
    std::optional<Sample> sample;
    double speed = 0, turn = 0, matched_speed = 0, matched_turn = 0;
    std::uint64_t frame_epoch, revision, view, sequence;
    double base_received{}, odom_received{};
    std::int64_t odom_stamp{};
    bool use_peer = false, can_project_latest = false;
    {
      std::unique_lock<std::mutex> lock(mutex);
      const bool timed_out = state == "tracking" && accepted && !recent(clock, accepted->stamp, 1.);
      if (timed_out) {
        lose("laser_timeout");
        retire_layers();
      }
      if (!base || now - base_at >= 5.) {
        if (timed_out || observation || navigation) {
          observation.reset();
          navigation.reset();
          local_used = laser_clear = false;
          ++map_version;
          lock.unlock();
          publish_current();
        }
        return;
      }
      const auto w = read_locked(now, clock);
      raw = base;
      previous_view = navigation;
      base_received = base_at;
      revision = base_revision;
      sequence = base_sequence;
      frame_epoch = epoch;
      view = view_revision;
      use_peer = cooperative;
      if (use_peer && w.peer_map_ready) other = peer;
      if (w.localized) {
        sample = accepted;
        can_project_latest = w.wheel_trusted;
        if (can_project_latest && local_ready(now, clock)) local_grid = local;
        odom_received = odom_at;
        odom_stamp = ns(odom->header.stamp);
        matched_speed = laser_speed;
        matched_turn = laser_turn;
        for (auto it = scans.rbegin(); it != scans.rend(); ++it)
          if (ns((*it)->header.stamp) == sample->stamp) {
            matched = *it;
            break;
          }
        if (matched && w.wheel_trusted) {
          if (auto v = motion_at(sample->stamp)) {
            matched_speed = std::max(matched_speed, v->first);
            matched_turn = std::max(matched_turn, v->second);
          } else {
            matched.reset();
          }
        }
        if (can_project_latest && !scans.empty() && recent(clock, ns(scans.back()->header.stamp), 1.) &&
            ns(scans.back()->header.stamp) > sample->stamp) {
          if (auto v = motion_at(ns(scans.back()->header.stamp))) {
            latest = scans.back();
            speed = v->first;
            turn = v->second;
          }
        }
      }
    }

    const bool continue_evidence = frame_epoch == evidence_epoch &&
                                   evidence_resolution == raw->info.resolution &&
                                   previous_fusion_time <= clock;
    auto next_evidence = continue_evidence ? cleared : decltype(cleared){};
    auto counted_stamp = continue_evidence ? previous_observation : 0;
    auto own = std::make_shared<Grid>(*raw);
    std::int64_t oldest = ns(raw->header.stamp), scan_stamp = 0;
    auto include_stamp = [&](std::int64_t value) {
      if (value > 0 && oldest > 0) oldest = std::min(oldest, value);
    };
    ScanEvidence scan;
    bool used = false, used_clear = false;
    // Keep sensor hits separate from the static cells that confirmed rays can clear.
    std::vector<std::uint8_t> live(raw->data.size());
    const Pose from_odom = sample ? compose(sample->map, inverse(sample->odom)) : Pose{};
    if (sample) {
      if (matched) {
        if (auto extrinsic = transform("base_link", matched->header.frame_id, sample->stamp)) {
          scan = scan_cells(*raw, *matched, compose(sample->map, *extrinsic), matched_speed, matched_turn,
                            &sample->variance, local_grid.get(), from_odom);
          scan_stamp = sample->stamp;
          include_stamp(scan_stamp);
          used = true;
        }
      }
      if (latest) {
        const auto t = ns(latest->header.stamp);
        if (auto lidar = transform("odom", latest->header.frame_id, t)) {
          // A newer hit vetoes old free evidence even if localization is running slower than the laser.
          auto new_hits = scan_cells(*raw, *latest, compose(from_odom, *lidar), speed, turn, nullptr).hits;
          scan.hits.insert(scan.hits.end(), new_hits.begin(), new_hits.end());
          include_stamp(t);
          used = true;
        }
      }
      if (local_grid) {
        overlay_hits(*own, *local_grid, from_odom, &live);
        include_stamp(ns(local_grid->header.stamp));
        used = true;
      }
      for (int i : scan.hits) live[i] = 1;
    }
    if (sample || !next_evidence.empty()) {
      const bool new_scan = scan_stamp > counted_stamp;
      const int w = raw->info.width, h = raw->info.height;
      const auto origin = planar(raw->info.origin);
      const double r = raw->info.resolution, quantum = r * .25;
      const Pose back = inverse(origin);
      for (auto it = next_evidence.begin(); it != next_evidence.end();) {
        const Pose cell = compose(back, {it->first.first * quantum, it->first.second * quantum, 0});
        const int x = static_cast<int>(std::floor(cell.x / r)), y = static_cast<int>(std::floor(cell.y / r));
        // A pose can arrive before its matching scan. Measure gaps between observations.
        const bool expired = it->second.count < 3 && new_scan && elapsed(scan_stamp, it->second.stamp) > 1.;
        if (x < 0 || x >= w || y < 0 || y >= h || expired) {
          it = next_evidence.erase(it);
          continue;
        }
        const int i = y * w + x;
        if (raw->data[i] <= 20 || live[i] ||
            (new_scan && it->second.count < 3 && (scan.free.empty() || !scan.free[i]))) {
          it = next_evidence.erase(it);
          continue;
        }
        if (new_scan && !scan.free.empty() && scan.free[i]) {
          it->second.count = std::min(3u, it->second.count + 1);
          it->second.stamp = scan_stamp;
          scan.free[i] = 0;
          used_clear = true;
        }
        if (it->second.count >= 3) {
          // The expired observation no longer certifies free space. The old
          // SLAM obstacle was disproved and stays unknown until observed again.
          const bool fresh = sample && elapsed(clock, it->second.stamp) < 4.;
          own->data[i] = fresh ? 0 : -1;
          if (fresh) include_stamp(it->second.stamp);
          used = true;
        }
        ++it;
      }
      for (std::size_t i = 0; i < live.size(); ++i) {
        if (live[i]) {
          own->data[i] = 100;
          continue;
        }
        if (scan.free.empty() || !scan.free[i]) continue;
        used_clear = true;
        if (raw->data[i] <= 20) {
          own->data[i] = 0;
          continue;
        }
        if (!new_scan) continue;
        const Pose p = compose(origin, {(i % w + .5) * r, (i / w + .5) * r, 0});
        next_evidence[{std::llround(p.x / quantum), std::llround(p.y / quantum)}] = {scan_stamp, 1};
      }
      if (new_scan) counted_stamp = scan_stamp;
    }
    if (local_grid) overlay_local_free(*own, *local_grid, from_odom, live);
    own->header.stamp = rclcpp::Time(oldest, node.get_clock()->get_clock_type());
    auto out = std::make_shared<Grid>(*own);
    if (other) overlay_hits(*out, *other, {});
    out->header.stamp = node.now();
    const bool changed_view = !same_cells(previous_view, out);
    {
      std::lock_guard<std::mutex> lock(mutex);
      const auto finish = steady_now();
      const auto finish_stamp = node.now().nanoseconds();
      if (epoch != frame_epoch || base_revision != revision || view_revision != view || finish - now >= 1. ||
          finish - base_received >= 5. || (use_peer && other != peer) ||
          ((local_grid || latest) && !wheel_ok) ||
          (local_grid && (!recent(finish_stamp, ns(local_grid->header.stamp), 1.5) ||
                          finish - std::max(0., elapsed(finish_stamp, ns(local_grid->header.stamp))) <
                              local_invalidated_at + fpga_ttl_s)) ||
          (latest && !recent(finish_stamp, ns(latest->header.stamp), 1.)) ||
          (sample && (!plane_ok || state != "tracking" || !recent(finish_stamp, sample->stamp, 1.) ||
                      !recent(finish_stamp, odom_stamp, .5) || finish - odom_received >= .5)))
        return;
      if (changed_view || committed_epoch != frame_epoch) ++map_version;
      observation = own;
      navigation = out;
      local_used = static_cast<bool>(local_grid);
      laser_clear = used_clear;
      committed_until = std::min(base_received + 5., now + 1.);
      auto limit_by_stamp = [&](std::int64_t source, double ttl) {
        committed_until =
            std::min(committed_until, finish + ttl - std::max(0., elapsed(finish_stamp, source)));
      };
      if (sample && used) limit_by_stamp(sample->stamp, 1.);
      if (latest) limit_by_stamp(ns(latest->header.stamp), 1.);
      if (local_grid) limit_by_stamp(ns(local_grid->header.stamp), 1.5);
      committed_epoch = frame_epoch;
      if (sample) committed_from_odom = from_odom;
      else committed_from_odom.reset();
      committed_sequence = sequence;
    }
    cleared = std::move(next_evidence);
    previous_observation = counted_stamp;
    evidence_epoch = frame_epoch;
    evidence_resolution = raw->info.resolution;
    previous_fusion_time = clock;
    publish_current();
  }
};

NavigationWorld::NavigationWorld(rclcpp::Node &node) : impl_(std::make_unique<Impl>(node)) {}
NavigationWorld::~NavigationWorld() = default;

WorldSnapshot NavigationWorld::snapshot() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->read_locked(steady_now(), impl_->node.now().nanoseconds());
}

void NavigationWorld::set_cooperative(bool enabled) {
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->cooperative == enabled) return;
    impl_->cooperative = enabled;
    impl_->navigation = impl_->observation;
    ++impl_->view_revision;
    ++impl_->map_version;
  }
  impl_->publish_current(false);
}

void NavigationWorld::set_fpga_goal_speed_mm_s(unsigned speed) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  // robot_navigation_set_goal changes the board map's TTL at the 500 mm/s cruise limit.
  impl_->fpga_ttl_s = speed > 500 ? 5. : 4.;
}
}  // namespace car_navigation
