#include "car_navigation/grid_route.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace car_navigation {
namespace {
using Map = nav_msgs::msg::OccupancyGrid;
constexpr int max_side = 2000;
constexpr int max_cells = 500000;
constexpr double start_radius = 0.30;
constexpr double sensing_distance = 0.75;
constexpr double waypoint_clearance = 0.25;
constexpr int occupied_threshold = 20;

bool finite(Point p) { return std::isfinite(p.x) && std::isfinite(p.y); }

struct PeerObstacles {
  const Map &map;
  Point origin;
  double c, s, scale;
  bool occupied(int x, int y) const {
    const Point center{origin.x + scale * (c * (x + 0.5) - s * (y + 0.5)),
                       origin.y + scale * (s * (x + 0.5) + c * (y + 0.5))};
    return visit_overlapping_cells(
        center, scale / 2, c, s, map.info.width, map.info.height,
        [&](int px, int py) { return map.data[py * map.info.width + px] > occupied_threshold; });
  }
};

struct Grid {
  explicit Grid(const Map &input) : map(input), res(input.info.resolution) {
    const auto &p = map.info.origin.position;
    const auto &q = map.info.origin.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    valid = map.info.width > 0 && map.info.width <= max_side && map.info.height > 0 &&
            map.info.height <= max_side && map.info.width * map.info.height <= max_cells &&
            map.data.size() == map.info.width * map.info.height && res > 0 && std::isfinite(res) &&
            finite({p.x, p.y}) && norm > 0 && std::isfinite(norm);
    if (!valid) return;
    width = static_cast<int>(map.info.width);
    height = static_cast<int>(map.info.height);
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), norm - 2 * (q.y * q.y + q.z * q.z));
    c = std::cos(yaw);
    s = std::sin(yaw);
  }

  Point local(Point p) const {
    const double dx = p.x - map.info.origin.position.x, dy = p.y - map.info.origin.position.y;
    return {(c * dx + s * dy) / res, (-s * dx + c * dy) / res};
  }
  Point world(int x, int y) const { return world(Point{x + 0.5, y + 0.5}); }
  Point world(Point p) const {
    const double dx = p.x * res, dy = p.y * res;
    return {map.info.origin.position.x + c * dx - s * dy, map.info.origin.position.y + s * dx + c * dy};
  }
  bool contains(Point p) const {
    return p.x >= left && p.y >= bottom && p.x < left + width && p.y < bottom + height;
  }
  int index(int x, int y) const { return (y - bottom) * width + x - left; }
  int x_of(int i) const { return i % width + left; }
  int y_of(int i) const { return i / width + bottom; }
  int value(int x, int y) const {
    int v = -1;
    if (x >= 0 && y >= 0 && x < static_cast<int>(map.info.width) && y < static_cast<int>(map.info.height))
      v = map.data[y * map.info.width + x];
    if (v <= occupied_threshold && peer && peer->occupied(x, y)) return 100;
    if (v < 0 && start_clear) {
      const double dx = (std::abs(x + 0.5 - start_clear->x) + 0.5) * res;
      const double dy = (std::abs(y + 0.5 - start_clear->y) + 0.5) * res;
      if (dx * dx + dy * dy <= start_radius * start_radius) return 0;
    }
    return v;
  }
  // Extend the planning view with unknown cells. The input message is never copied or modified.
  bool include(Point p, int margin) {
    if (!finite(p) || std::abs(p.x) > 2 * max_side || std::abs(p.y) > 2 * max_side) return false;
    const int x = static_cast<int>(std::floor(p.x)), y = static_cast<int>(std::floor(p.y));
    const int right = std::max(left + width, x + margin + 1), top = std::max(bottom + height, y + margin + 1);
    left = std::min(left, x - margin);
    bottom = std::min(bottom, y - margin);
    width = right - left;
    height = top - bottom;
    return width <= max_side && height <= max_side && width * height <= max_cells;
  }
  bool configure_start(std::optional<Point> point) {
    if (!point) return true;
    start_clear = local(*point);
    return start_radius / res <= max_side &&
           include(*start_clear, static_cast<int>(std::ceil(start_radius / res)) + 1);
  }
  bool configure_peer(const Map *source) {
    if (!source) return true;
    // A new peer snapshot can precede publication of the fused map. Check hits
    // inside and outside the base map, only in cells touched by the caller.
    Grid other(*source);
    if (!other.valid || source->header.frame_id != map.header.frame_id) return false;
    const auto &position = map.info.origin.position;
    const Point origin = other.local({position.x, position.y});
    if (!finite(origin)) return false;
    peer.emplace(PeerObstacles{*source, origin, c * other.c + s * other.s, s * other.c - c * other.s,
                               res / other.res});
    return true;
  }
  const Map &map;
  double res, c = 1, s = 0;
  int left = 0, bottom = 0, width = 0, height = 0;
  bool valid = false;
  std::optional<Point> start_clear;
  std::optional<PeerObstacles> peer;
};

int floor_cell(double coordinate) { return static_cast<int>(std::floor(coordinate)); }
bool representable(Point p) {
  const double limit = std::numeric_limits<int>::max() - 2. * max_side - 4.;
  return finite(p) && std::abs(p.x) < limit && std::abs(p.y) < limit;
}

struct Traversal {
  const Grid &grid;
  double clearance;
  int radius;
  bool unknown_allowed;
  int start = -1;
  const std::vector<int> *obstacle_distance = nullptr;

  bool in_start_region(int x, int y) const {
    return grid.start_clear &&
           std::hypot(x + 0.5 - grid.start_clear->x, y + 0.5 - grid.start_clear->y) * grid.res <=
               start_radius + clearance + 2 * grid.res;
  }

  bool raw_clear(int x, int y, bool ignore_unknown = false) const {
    const int v = grid.value(x, y);
    if (v > occupied_threshold || (v < 0 && !unknown_allowed && !ignore_unknown)) return false;
    // The temporary circle is narrower than normal square inflation; check its actual disk.
    const bool disk = in_start_region(x, y);
    const int reach = disk ? static_cast<int>(std::ceil(clearance / grid.res + 0.5)) : radius;
    for (int yy = y - reach; yy <= y + reach; ++yy) {
      for (int xx = x - reach; xx <= x + reach; ++xx) {
        if (disk) {
          const double dx = std::max(0.0, std::abs(xx - x) - 0.5) * grid.res;
          const double dy = std::max(0.0, std::abs(yy - y) - 0.5) * grid.res;
          if (dx * dx + dy * dy > clearance * clearance) continue;
        }
        const bool inside = xx >= grid.left && yy >= grid.bottom && xx < grid.left + grid.width &&
                            yy < grid.bottom + grid.height;
        if (!inside && !ignore_unknown) return false;
        const int obstacle = grid.value(xx, yy);
        if (obstacle > occupied_threshold || (obstacle < 0 && !ignore_unknown && !unknown_allowed))
          return false;
      }
    }
    return true;
  }
  bool open(int x, int y) const {
    if (x < grid.left || x >= grid.left + grid.width || y < grid.bottom || y >= grid.bottom + grid.height)
      return false;
    const int i = grid.index(x, y);
    if (i == start) return true;
    if (!obstacle_distance) return raw_clear(x, y);
    const int gap = (*obstacle_distance)[i];
    return gap > radius || (gap > 0 && in_start_region(x, y) && raw_clear(x, y));
  }
  bool segment(Point a, Point b) const {
    if (!grid.contains(a) || !grid.contains(b)) return false;
    int x = floor_cell(a.x), y = floor_cell(a.y);
    const int gx = floor_cell(b.x), gy = floor_cell(b.y);
    if (!open(x, y)) return false;
    const int sx = (gx > x) - (gx < x), sy = (gy > y) - (gy < y);
    const double inf = std::numeric_limits<double>::infinity();
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double step_x = sx ? std::abs(1 / dx) : inf, step_y = sy ? std::abs(1 / dy) : inf;
    double tx = sx ? (x + (sx > 0) - a.x) / dx : inf, ty = sy ? (y + (sy > 0) - a.y) / dy : inf;
    while (x != gx || y != gy) {
      if (std::abs(tx - ty) <= 1e-10) {
        if (!open(x + sx, y) || !open(x, y + sy)) return false;
        x += sx;
        y += sy;
        tx = x == gx ? inf : tx + step_x;
        ty = y == gy ? inf : ty + step_y;
      } else if (tx < ty) {
        x += sx;
        tx = x == gx ? inf : tx + step_x;
      } else {
        y += sy;
        ty = y == gy ? inf : ty + step_y;
      }
      if (!open(x, y)) return false;
    }
    return true;
  }
};

bool valid_clearance(const Grid &grid, double clearance) {
  return grid.valid && clearance >= 0 && clearance / grid.res <= max_side;
}
using Check = std::function<bool()>;
bool stopped(const Check &check) { return check && check(); }

double bearing(Point a, Point b) { return std::atan2(b.y - a.y, b.x - a.x); }

// A vertex rotating about the origin can touch an axis-aligned box only on
// one of its four finite edges. Test those circle/line intersections.
bool arc_hits_box(double r, double mid, double half_angle, double xmin, double xmax, double ymin,
                  double ymax) {
  const auto on_arc = [&](double x, double y) {
    return std::abs(std::remainder(std::atan2(y, x) - mid, 2 * std::acos(-1.))) <= half_angle;
  };
  for (double x : {xmin, xmax}) {
    if (std::abs(x) > r) continue;
    const double h = std::sqrt(std::max(0., (r - x) * (r + x)));
    for (double y : {-h, h})
      if (y >= ymin && y <= ymax && on_arc(x, y)) return true;
  }
  for (double y : {ymin, ymax}) {
    if (std::abs(y) > r) continue;
    const double h = std::sqrt(std::max(0., (r - y) * (r + y)));
    for (double x : {-h, h})
      if (x >= xmin && x <= xmax && on_arc(x, y)) return true;
  }
  return false;
}

struct BodyMotion {
  const Grid &g;
  bool unknown;
  const std::vector<int> *field = nullptr;
  double length = body_half_length / g.res, width = body_half_width / g.res;
  double origin_yaw = std::atan2(g.s, g.c);

  bool blocked(int x, int y) const {
    if (field && x >= g.left && y >= g.bottom && x < g.left + g.width && y < g.bottom + g.height)
      return (*field)[g.index(x, y)] == 0;
    const int v = g.value(x, y);
    return v > occupied_threshold || (!unknown && v < 0);
  }

  // In grid coordinates a translating rectangle sweeps its Minkowski sum
  // with the displacement segment. The separating axes are the two grid
  // axes, the two body axes, and the normal to the displacement.
  template <class Visit>
  bool slide_while(Point a, Point b, double yaw, Visit visit) const {
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double bx = length * std::abs(c) + width * std::abs(s),
                 by = length * std::abs(s) + width * std::abs(c);
    const double dx = b.x - a.x, dy = b.y - a.y, mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    const double cell_projection = .5 * (std::abs(c) + std::abs(s));
    const double longitudinal = length + cell_projection + .5 * std::abs(c * dx + s * dy);
    const double lateral = width + cell_projection + .5 * std::abs(c * dy - s * dx);
    const double perpendicular = length * std::abs(c * dy - s * dx) + width * std::abs(s * dy + c * dx) +
                                 .5 * (std::abs(dx) + std::abs(dy));
    for (int y = static_cast<int>(std::ceil(std::min(a.y, b.y) - by)) - 1;
         y <= floor_cell(std::max(a.y, b.y) + by); ++y) {
      // Limit each row to the part of the segment crossing its swept band.
      double u = 0., v = 1.;
      if (dy != 0.) {
        const double p = (y - by - a.y) / dy, q = (y + 1 + by - a.y) / dy;
        u = std::clamp(std::min(p, q), 0., 1.);
        v = std::clamp(std::max(p, q), 0., 1.);
      }
      const double x0 = a.x + u * dx, x1 = a.x + v * dx;
      for (int x = static_cast<int>(std::ceil(std::min(x0, x1) - bx)) - 1;
           x <= floor_cell(std::max(x0, x1) + bx); ++x) {
        const double ex = x + .5 - mx, ey = y + .5 - my;
        if (std::abs(c * ex + s * ey) > longitudinal || std::abs(c * ey - s * ex) > lateral ||
            std::abs(dy * ex - dx * ey) > perpendicular)
          continue;
        if (!visit(x, y)) return false;
      }
    }
    return true;
  }

  bool slide(Point a, Point b, double yaw) const {
    return slide_while(a, b, yaw, [this](int x, int y) { return !blocked(x, y); });
  }

  bool turn(Point p, double from, double to) const {
    const double delta = std::remainder(to - from, 2 * std::acos(-1.));
    if (!slide(p, p, from)) return false;
    if (delta == 0.) return true;
    const double mid = from + delta / 2, half = std::abs(delta) / 2, r = std::hypot(length, width);
    for (int y = static_cast<int>(std::ceil(p.y - r)) - 1; y <= floor_cell(p.y + r); ++y) {
      for (int x = static_cast<int>(std::ceil(p.x - r)) - 1; x <= floor_cell(p.x + r); ++x) {
        const double x0 = x - p.x, x1 = x0 + 1, y0 = y - p.y, y1 = y0 + 1;
        if (std::hypot(std::max({x0, 0., -x1}), std::max({y0, 0., -y1})) > r || !blocked(x, y)) continue;
        // First contact is a moving body corner against a cell edge, or a
        // stationary cell corner crossing a body edge in the rotating frame.
        for (double vx : {-length, length})
          for (double vy : {-width, width})
            if (arc_hits_box(r, mid + std::atan2(vy, vx), half, x0, x1, y0, y1)) return false;
        for (double vx : {x0, x1})
          for (double vy : {y0, y1})
            if (arc_hits_box(std::hypot(vx, vy), std::atan2(vy, vx) - mid, half, -length, length, -width,
                             width))
              return false;
      }
    }
    return true;
  }

  // Positions are in cells; all supplied headings are in the map's world frame.
  bool segment(Point a, Point b, double travel, std::optional<double> start,
               std::optional<double> end) const {
    if (!finite(a) || !finite(b) || !std::isfinite(travel) || (start && !std::isfinite(*start)) ||
        (end && !std::isfinite(*end)))
      return false;
    const double yaw = travel - origin_yaw;
    return (!start || *start == travel || turn(a, *start - origin_yaw, yaw)) && slide(a, b, yaw) &&
           (!end || *end == travel || turn(b, yaw, *end - origin_yaw));
  }
};

// The exterior is one cell from the nearest border. The two sweeps account for
// horizontal, vertical and diagonal neighbours without repeated area scans.
enum class DistanceTo { unknown, blocked };
std::vector<int> distances(const Grid &g, DistanceTo target, bool allow_unknown, const Check &cancelled) {
  std::vector<int> d(g.width * g.height);
  for (int y = 0; y < g.height; ++y) {
    if (stopped(cancelled)) return {};
    for (int x = 0; x < g.width; ++x) {
      const int i = y * g.width + x;
      const int value = g.value(x + g.left, y + g.bottom);
      const bool seed = target == DistanceTo::unknown
                            ? value < 0
                            : value > occupied_threshold || (value < 0 && !allow_unknown);
      d[i] = seed ? 0 : std::min({x + 1, g.width - x, y + 1, g.height - y});
    }
  }
  for (const int sign : {1, -1}) {
    for (int row = 0; row < g.height; ++row) {
      if (stopped(cancelled)) return {};
      const int y = sign == 1 ? row : g.height - 1 - row;
      for (int col = 0; col < g.width; ++col) {
        const int x = sign == 1 ? col : g.width - 1 - col;
        const int i = y * g.width + x;
        if (col) d[i] = std::min(d[i], 1 + d[i - sign]);
        if (row) {
          const int previous = i - sign * g.width;
          d[i] = std::min(d[i], 1 + d[previous]);
          if (x) d[i] = std::min(d[i], 1 + d[previous - 1]);
          if (x + 1 < g.width) d[i] = std::min(d[i], 1 + d[previous + 1]);
        }
      }
    }
  }
  return d;
}
}  // namespace

bool clear_body_segment(const Map &map, Point from, Point to, double travel_yaw, bool allow_unknown,
                        std::optional<Point> start_clear, const Map *peer, std::optional<double> start_yaw,
                        std::optional<double> end_yaw) {
  Grid g(map);
  if (!valid_clearance(g, body_radius) || !g.configure_start(start_clear) || !g.configure_peer(peer))
    return false;
  const auto a = g.local(from), b = g.local(to);
  if (!representable(a) || !representable(b) || std::abs(b.x - a.x) > max_side ||
      std::abs(b.y - a.y) > max_side || (!allow_unknown && (!g.contains(a) || !g.contains(b))))
    return false;
  return BodyMotion{g, allow_unknown}.segment(a, b, travel_yaw, start_yaw, end_yaw);
}

bool clear_observed_departure(const Map &map, Point from, Point to, double yaw) {
  Grid g(map);
  if (!valid_clearance(g, body_radius) || !std::isfinite(yaw)) return false;
  const auto a = g.local(from), b = g.local(to);
  if (!g.contains(a) || !g.contains(b)) return false;
  const BodyMotion body{g, false};
  const double heading = yaw - body.origin_yaw;
  std::vector<std::pair<int, int>> parked;
  body.slide_while(a, a, heading, [&](int x, int y) {
    parked.emplace_back(y, x);
    return true;
  });
  bool observed = false;
  return body.slide_while(a, b, heading, [&](int x, int y) {
           const int value = g.value(x, y);
           if (value > occupied_threshold) return false;
           if (std::binary_search(parked.begin(), parked.end(), std::pair{y, x})) return true;
           observed |= value >= 0;
           return value >= 0;
         }) && observed;
}

bool known_free_square(const Map &map, Point center, double half, double c, double s) {
  const double extent = half * (std::abs(c) + std::abs(s));
  const int w = map.info.width, h = map.info.height;
  if (!(center.x - extent >= -1e-8 && center.y - extent >= -1e-8 &&
        center.x + extent <= w + 1e-8 && center.y + extent <= h + 1e-8))
    return false;
  bool covered = false;
  const bool blocked = visit_overlapping_cells(center, half, c, s, w, h, [&](int x, int y) {
    covered = true;
    const int value = map.data[y * w + x];
    return value < 0 || value > occupied_threshold;
  });
  return covered && !blocked;
}

Map::ConstSharedPtr recovery_local_map(const Map::ConstSharedPtr &local, const Map &fused,
                                       Point odom, Point robot, double odom_yaw, double odom_to_map_yaw) {
  const Grid board(*local), global(fused);
  if (!valid_clearance(board, body_radius) || !valid_clearance(global, body_radius)) return local;
  const Point a = board.local(odom), b = global.local(robot);
  if (!board.contains(a) || !global.contains(b)) return local;
  const BodyMotion body{board, false};
  auto view = std::make_shared<Map>(*local);
  // The caller checked the exact parked footprint against the fused map before entering recovery.
  body.slide_while(a, a, odom_yaw - body.origin_yaw, [&](int x, int y) {
    if (x >= 0 && x < board.width && y >= 0 && y < board.height) {
      auto &v = view->data[board.index(x, y)];
      if (v < 0) v = 0;
    }
    return true;
  });
  const double angle = odom_to_map_yaw + body.origin_yaw - std::atan2(global.s, global.c);
  const double c = std::cos(angle), s = std::sin(angle), scale = board.res / global.res;
  for (int y = 0; y < board.height; ++y) {
    for (int x = 0; x < board.width; ++x) {
      auto &value = view->data[board.index(x, y)];
      if (value >= 0) continue;
      const double dx = (x + .5 - a.x) * scale, dy = (y + .5 - a.y) * scale;
      const Point center{b.x + c * dx - s * dy, b.y + s * dx + c * dy};
      if (known_free_square(fused, center, .5 * scale, c, s)) value = 0;
    }
  }
  return view;
}

namespace {
std::string terminal_obstruction(const Grid &g, Point local_goal, std::optional<double> end_yaw) {
  if (!representable(local_goal)) return {};
  if (g.value(floor_cell(local_goal.x), floor_cell(local_goal.y)) > occupied_threshold)
    return "goal_occupied";
  if (end_yaw && !BodyMotion{g, true}.segment(local_goal, local_goal, *end_yaw, {}, {}))
    return "goal_footprint_blocked";
  return {};
}
// The field belongs to one map, peer, start-clear circle and unknown policy.
// Unknown-space probes use a fresh field because their endpoint can extend the search view.
Route wide_search(const Map &map, Point start, std::optional<Point> goal, bool allow_unknown, bool frontier,
                  const std::vector<Point> &excluded, const Check &cancelled,
                  std::optional<Point> start_clear, double clearance_m, const Map *peer,
                  std::vector<int> &obstacles) {
  if (stopped(cancelled)) return {{}, "cancelled"};
  Grid g(map);
  if (!valid_clearance(g, clearance_m) || !finite(start) || (goal && !finite(*goal)) || (!goal && !frontier))
    return {{}, "invalid_map_or_goal"};
  const int radius = static_cast<int>(std::ceil(clearance_m / g.res));
  const Point a = g.local(start);
  const Point b = goal ? g.local(*goal) : a;
  if (!g.configure_start(start_clear)) return {{}, "start_area_exceeds_map_limits"};
  if (!g.configure_peer(peer)) return {{}, "invalid_peer_map"};
  allow_unknown = allow_unknown && goal.has_value();
  if (allow_unknown && goal && (!g.include(a, radius + 2) || !g.include(b, radius + 2)))
    return {{}, "unknown_route_exceeds_map_limits"};
  if (!g.contains(a)) return {{}, "start_outside_map"};
  const int sx = floor_cell(a.x), sy = floor_cell(a.y), start_id = g.index(sx, sy);
  const int origin_value = g.value(sx, sy);
  if (origin_value > occupied_threshold || (!allow_unknown && origin_value < 0))
    return {{}, "start_unknown_or_occupied"};
  Traversal t{g, clearance_m, radius, allow_unknown, start_id, obstacles.empty() ? nullptr : &obstacles};
  int target = -1;
  if (goal && g.contains(b)) {
    const int gx = floor_cell(b.x), gy = floor_cell(b.y);
    if (allow_unknown || g.value(gx, gy) >= 0) {
      if (t.raw_clear(gx, gy))
        target = g.index(gx, gy);
      else if (!t.raw_clear(gx, gy, true))
        return {{}, "insufficient_wide_clearance"};
    }
  }
  if (target < 0 && !frontier) return {{}, "no_known_goal"};
  if (target >= 0 && !allow_unknown && t.segment(a, b))
    return {target == start_id ? std::vector<Point>{*goal} : std::vector<Point>{start, *goal}, "goal"};

  auto heuristic = [&](int x, int y) {
    if (target < 0) return 0;
    const int dx = std::abs(x - g.x_of(target)), dy = std::abs(y - g.y_of(target));
    return 10 * std::max(dx, dy) + 4 * std::min(dx, dy);
  };
  const int n = g.width * g.height;
  if (obstacles.empty()) obstacles = distances(g, DistanceTo::blocked, allow_unknown, cancelled);
  if (obstacles.empty()) return {{}, "cancelled"};
  t.obstacle_distance = &obstacles;
  std::vector<int> cost(n, std::numeric_limits<int>::max()), parent(n, -1);
  using Entry = std::tuple<int, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  queue.emplace(heuristic(sx, sy), 0, start_id);
  cost[start_id] = 0;
  int selected = -1, pops = 0;
  constexpr int offsets[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  while (!queue.empty()) {
    if ((pops++ % 256) == 0 && stopped(cancelled)) return {{}, "cancelled"};
    const auto [priority, negative_cost, i] = queue.top();
    (void)priority;
    queue.pop();
    if (cost[i] != -negative_cost) continue;
    if (i == target) {
      selected = i;
      break;
    }
    const int x = g.x_of(i), y = g.y_of(i);
    for (const auto &d : offsets) {
      const int nx = x + d[0], ny = y + d[1];
      const bool diagonal = d[0] && d[1];
      if (!t.open(nx, ny) || (diagonal && (!t.open(nx, y) || !t.open(x, ny)))) continue;
      const int j = g.index(nx, ny);
      const int next = cost[i] + (diagonal ? 14 : 10) + (allow_unknown && g.value(nx, ny) < 0 ? 5 : 0);
      if (next >= cost[j]) continue;
      cost[j] = next;
      parent[j] = i;
      queue.emplace(next + heuristic(nx, ny), -next, j);
    }
  }
  const bool reached_goal = selected >= 0;
  if (!reached_goal && frontier) {
    // The exhausted search already contains every reachable known cell, also
    // when the goal is known but lies across an unobserved gap.
    const auto unknown = distances(g, DistanceTo::unknown, false, cancelled);
    if (unknown.empty()) return {{}, "cancelled"};
    const int sensing = std::max(
        radius + 1, static_cast<int>(std::min<double>(max_side, std::ceil(sensing_distance / g.res))));
    double best_score = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
      if (i % 1024 == 0 && stopped(cancelled)) return {{}, "cancelled"};
      if (cost[i] == std::numeric_limits<int>::max() || unknown[i] <= radius || unknown[i] > sensing)
        continue;
      const Point p = g.world(g.x_of(i), g.y_of(i));
      if (std::any_of(excluded.begin(), excluded.end(), [p](Point e) { return distance(p, e) < 0.35; }))
        continue;
      const double score =
          goal ? distance(p, *goal) + cost[i] * g.res / 50 : (cost[i] / 10.0 + 0.2 * unknown[i]) * g.res;
      if (score < best_score) {
        selected = i;
        best_score = score;
      }
    }
  }
  if (selected < 0) return {{}, frontier ? "no_reachable_frontier" : "no_route"};
  std::vector<Point> path;
  for (int i = selected; i != start_id; i = parent[i]) path.push_back(g.world(g.x_of(i), g.y_of(i)));
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  if (!reached_goal) return {std::move(path), "frontier"};
  path.back() = *goal;
  if (!allow_unknown && path.size() > 2) {
    std::vector<Point> corners{start};
    for (std::size_t i = 2; i < path.size(); ++i) {
      if (stopped(cancelled)) return {{}, "cancelled"};
      if (!t.segment(g.local(corners.back()), g.local(path[i]))) corners.push_back(path[i - 1]);
    }
    corners.push_back(*goal);
    return {std::move(corners), "goal"};
  }
  return {std::move(path), "goal"};
}

bool body_path(const BodyMotion &body, Point start, const Route &route, std::optional<double> yaw,
               std::optional<double> end_yaw, std::optional<double> held, const Check &cancel) {
  Point from = start;
  bool moved = false;
  for (std::size_t i = 0; i < route.points.size(); ++i) {
    const Point to = route.points[i];
    if (from.x == to.x && from.y == to.y) continue;
    const double angle = held.value_or(bearing(from, to));
    if (stopped(cancel) || !body.segment(body.g.local(from), body.g.local(to), angle, yaw, std::nullopt))
      return false;
    from = to;
    yaw = angle;
    moved = true;
  }
  if (moved && !end_yaw) return true;
  const double angle = yaw.value_or(end_yaw.value_or(held.value_or(0.)));
  return body.segment(body.g.local(from), body.g.local(from), angle, yaw, end_yaw);
}

// Keep the original lattice. An even number of free cells can leave a usable
// centreline between rows (or columns) while neither adjacent cell centre fits.
// Extra nodes exist only along those centrelines and their approaches.
struct CorridorLattice {
  const Grid &grid;
  const int cells, stride;
  std::vector<int> extra_coordinates;
  std::unordered_map<int, int> extra_indices;
  std::vector<std::uint8_t> near_extra;

  explicit CorridorLattice(const Grid &g) : grid(g), cells(g.width * g.height), stride(2 * g.width) {}
  int size() const { return cells + static_cast<int>(extra_coordinates.size()); }
  std::pair<int, int> coordinates(int vertex) const {
    if (vertex < cells) return {2 * (vertex % grid.width) + 1, 2 * (vertex / grid.width) + 1};
    const int code = extra_coordinates[vertex - cells];
    return {code % stride, code / stride};
  }
  int cell(int vertex) const {
    if (vertex < cells) return vertex;
    const auto [u, v] = coordinates(vertex);
    return (v / 2) * grid.width + u / 2;
  }
  int vertex_at(int u, int v) const {
    if (u < 0 || u >= stride || v < 0 || v >= 2 * grid.height) return -1;
    if ((u & 1) && (v & 1)) return (v / 2) * grid.width + u / 2;
    const auto it = extra_indices.find(v * stride + u);
    return it == extra_indices.end() ? -1 : it->second;
  }
  Point local(int vertex) const {
    const auto [u, v] = coordinates(vertex);
    return {grid.left + .5 * u, grid.bottom + .5 * v};
  }
  bool half_steps(int vertex) const { return vertex >= cells || (!near_extra.empty() && near_extra[vertex]); }
  void add(int u, int v) {
    const int code = v * stride + u;
    if (!extra_indices.emplace(code, size()).second) return;
    extra_coordinates.push_back(code);
    if (near_extra.empty()) near_extra.resize(cells);
    for (int y : {(v - 1) / 2, v / 2})
      for (int x : {(u - 1) / 2, u / 2}) near_extra[y * grid.width + x] = 1;
  }
  bool build(const std::vector<int> &field, std::optional<double> held, bool unknown, const Check &cancel) {
    const double angle = held.value_or(0.) - std::atan2(grid.s, grid.c);
    const int approach = static_cast<int>(std::ceil(body_radius / grid.res)) + 1;
    for (bool horizontal : {true, false}) {
      const int lanes = horizontal ? grid.width : grid.height, span = horizontal ? grid.height : grid.width;
      const double c = std::abs(std::cos(angle)), s = std::abs(std::sin(angle));
      const double half_width = held ? (horizontal ? body_half_length * s + body_half_width * c
                                                   : body_half_length * c + body_half_width * s)
                                     : body_half_width;
      const auto index = [&](int lane, int cross) {
        return horizontal ? cross * grid.width + lane : lane * grid.width + cross;
      };
      // Neighbouring sections share most approach samples on a long straight corridor.
      std::vector<int> covered(span, -1);
      for (int lane = 0; lane < lanes; ++lane) {
        if (stopped(cancel)) return false;
        for (int pos = 0; pos < span;) {
          if (!field[index(lane, pos)]) {
            ++pos;
            continue;
          }
          const int lo = pos;
          while (pos < span && field[index(lane, pos)]) ++pos;
          const int width = pos - lo;
          if ((width & 1) || (unknown && (lo == 0 || pos == span)) || width * grid.res <= 2 * half_width ||
              (width - 1) * grid.res > 2 * half_width)
            continue;
          const int middle = lo + width / 2;
          // Align before the nose reaches a wall end; the rotation happens in the open approach.
          const int end = std::min(lanes - 1, lane + approach);
          for (int k = std::max(covered[middle] + 1, lane - approach); k <= end; ++k) {
            if (!field[index(k, middle)] || !field[index(k, middle - 1)]) continue;
            if (horizontal)
              add(2 * k + 1, 2 * middle);
            else
              add(2 * middle, 2 * k + 1);
          }
          covered[middle] = end;
        }
      }
    }
    const int lines = static_cast<int>(extra_coordinates.size());
    for (int i = 0; i < lines; ++i) {
      if (i % 256 == 0 && stopped(cancel)) return false;
      const auto [u, v] = coordinates(cells + i);
      if (!(u & 1)) continue;
      for (int x : {u - 1, u + 1}) {
        if (x <= 0 || x >= stride || v <= 0 || v >= 2 * grid.height) continue;
        if (vertex_at(x, v - 1) >= 0 && vertex_at(x, v + 1) >= 0) add(x, v);
      }
    }
    return true;
  }
};

// Search exact swept edges only where the wide path did not suffice. A roomy
// cell can turn freely, so only tight cells distinguish incoming directions.
Route oriented_search(const Grid &g, Point start, std::optional<Point> goal, bool unknown, bool frontier,
                      const std::vector<Point> &excluded, const Check &cancel, std::vector<int> &field,
                      std::optional<double> start_yaw, std::optional<double> end_yaw,
                      std::optional<double> held) {
  const Point a = g.local(start);
  int target = -1;
  Point b = a;
  if (goal) {
    b = g.local(*goal);
    if (g.contains(b)) {
      const int x = floor_cell(b.x), y = floor_cell(b.y);
      if (unknown || g.value(x, y) >= 0) target = g.index(x, y);
    }
  }
  if (target < 0 && !frontier) return {{}, "no_known_goal"};
  if (field.empty()) field = distances(g, DistanceTo::blocked, unknown, cancel);
  if (field.empty()) return {{}, "cancelled"};
  const BodyMotion body{g, unknown, &field};
  CorridorLattice lattice(g);
  if (!lattice.build(field, held, unknown, cancel)) return {{}, "cancelled"};
  const auto nearest = [&](Point p) {
    const int x = floor_cell(p.x), y = floor_cell(p.y);
    int closest = g.index(x, y);
    if (!lattice.half_steps(closest)) return closest;
    double gap = distance(p, lattice.local(closest));
    for (int v = 2 * (y - g.bottom); v <= 2 * (y - g.bottom) + 2; ++v) {
      for (int u = 2 * (x - g.left); u <= 2 * (x - g.left) + 2; ++u) {
        const int node = lattice.vertex_at(u, v);
        if (node < 0) continue;
        const double next = distance(p, lattice.local(node));
        if (next < gap) {
          closest = node;
          gap = next;
        }
      }
    }
    return closest;
  };
  const int origin = nearest(a);
  if (target >= 0) target = nearest(b);
  const int nodes = lattice.size(), initial = nodes * 16;
  const int safe_gap = static_cast<int>(std::ceil(body_radius / g.res)) + 1;
  const auto roomy = [&](int node) { return field[lattice.cell(node)] > safe_gap; };
  const auto origin_uv = lattice.coordinates(origin);
  const Point origin_sample = lattice.local(origin);
  const bool offset_start = a.x != origin_sample.x || a.y != origin_sample.y;
  const auto heading_count = [&](int node, int u, int v) {
    if (held || roomy(node)) return 1;
    if (!lattice.half_steps(node)) return 8;
    // Full and half edges have the same bearing, except at substituted endpoints
    // or when the predecessor is the true, off-lattice start.
    const bool near_start =
        offset_start && std::abs(u - origin_uv.first) <= 2 && std::abs(v - origin_uv.second) <= 2;
    return node == origin || node == target || near_start ? 16 : 8;
  };
  const auto position = [&](int state) -> Point {
    if (state == initial) return start;
    const int node = state / 16;
    if (node == target) return *goal;
    if (node == origin) return start;
    return g.world(lattice.local(node));
  };
  struct Label {
    int cost = std::numeric_limits<int>::max(), parent = -1;
  };
  std::vector<int> base(nodes, -1);
  std::vector<std::uint16_t> checked_edges(nodes), free_edges(nodes);
  std::vector<Label> labels;
  const auto label = [&](int key) -> Label & { return labels[base[key / 16] + key % 16]; };
  const auto heuristic = [&](int node) {
    if (target < 0) return 0;
    const auto [u, v] = lattice.coordinates(node);
    const auto [gu, gv] = lattice.coordinates(target);
    const int dx = std::abs(u - gu), dy = std::abs(v - gv);
    return 5 * std::max(dx, dy) + 2 * std::min(dx, dy);
  };
  using Entry = std::tuple<int, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  queue.emplace(heuristic(origin), 0, initial);
  int selected = -1, pops = 0;
  constexpr int moves[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
  while (!queue.empty()) {
    if (pops++ % 64 == 0 && stopped(cancel)) return {{}, "cancelled"};
    const auto [rank, negative, key] = queue.top();
    (void)rank;
    queue.pop();
    if (key != initial && label(key).cost != -negative) continue;
    const int node = key == initial ? origin : key / 16;
    if (key != initial && node == target) {
      selected = key;
      break;
    }
    const auto from = position(key);
    std::optional<double> incoming = start_yaw;
    if (key != initial) incoming = held.value_or(bearing(position(label(key).parent), from));
    const auto [u, v] = lattice.coordinates(node);
    const int directions = lattice.half_steps(node) ? 16 : 8;
    for (int edge = 0; edge < directions; ++edge) {
      const int step = edge < 8 ? 2 : 1, d = edge % 8;
      const int nu = u + step * moves[d][0], nv = v + step * moves[d][1];
      const int next = lattice.vertex_at(nu, nv);
      if (next < 0) continue;
      const int next_cell = lattice.cell(next);
      if (field[next_cell] == 0) continue;
      // Even a diagonal obstacle at this Chebyshev distance intersects the
      // inscribed circle. No incoming heading can fit at this grid centre.
      if (next < lattice.cells && next != target && next != origin &&
          (field[next_cell] - .5) * g.res * std::sqrt(2.) <= std::min(body_half_length, body_half_width)) {
        const int x = next_cell % g.width, y = next_cell / g.width;
        // The distance transform also measures the edge of the search view.
        // An unknown-space probe may sweep outside it; only actual hits can prune it.
        if (!unknown || field[next_cell] < std::min({x + 1, g.width - x, y + 1, g.height - y})) continue;
      }
      const int headings = heading_count(next, nu, nv);
      const int next_key = 16 * next + (headings == 1 ? 0 : headings == 8 ? d : edge);
      const int cost =
          -negative + step * (d < 4 ? 5 : 7) +
          (unknown && g.value(g.x_of(next_cell), g.y_of(next_cell)) < 0 ? (step == 2 ? 5 : 3) : 0);
      if (base[next] >= 0 && cost >= label(next_key).cost) continue;
      if (!roomy(node) || !roomy(next)) {
        const auto to = position(next_key);
        const double travel = held.value_or(bearing(from, to));
        const auto pa = g.local(from), pb = g.local(to);
        const unsigned bit = 1u << edge;
        if (!(checked_edges[node] & bit)) {
          checked_edges[node] |= bit;
          if (body.slide(pa, pb, travel - body.origin_yaw) &&
              (next != target || !end_yaw || roomy(next) ||
               body.turn(pb, travel - body.origin_yaw, *end_yaw - body.origin_yaw)))
            free_edges[node] |= bit;
        }
        if (!(free_edges[node] & bit) ||
            (!roomy(node) && incoming && *incoming != travel &&
             !body.turn(pa, *incoming - body.origin_yaw, travel - body.origin_yaw)))
          continue;
      }
      if (base[next] < 0) {
        base[next] = static_cast<int>(labels.size());
        labels.resize(labels.size() + headings);
      }
      label(next_key) = {cost, key};
      queue.emplace(cost + heuristic(next), -cost, next_key);
    }
  }
  const bool arrived = selected >= 0;
  if (!arrived && frontier) {
    const auto unseen = distances(g, DistanceTo::unknown, false, cancel);
    if (unseen.empty()) return {{}, "cancelled"};
    double best = std::numeric_limits<double>::infinity();
    for (int node = 0; node < nodes; ++node) {
      if (node % 256 == 0 && stopped(cancel)) return {{}, "cancelled"};
      if (node != origin && base[node] < 0) continue;
      const int cell = lattice.cell(node);
      if (!unseen[cell] || (unseen[cell] - .5) * g.res > sensing_distance) continue;
      const bool first = node == origin;
      const auto [u, v] = lattice.coordinates(node);
      const int count = first ? 1 : heading_count(node, u, v);
      for (int d = 0; d < count; ++d) {
        const int key = first ? initial : 16 * node + d;
        const int cost = first ? 0 : label(key).cost;
        if (cost == std::numeric_limits<int>::max()) continue;
        const auto p = position(key);
        const double score =
            (goal ? distance(p, *goal) : .2 * unseen[cell] * g.res) + cost * g.res / (goal ? 50. : 10.);
        if (score >= best ||
            std::any_of(excluded.begin(), excluded.end(), [p](Point e) { return distance(p, e) < .35; }))
          continue;
        if (!roomy(node)) {
          const double heading = frontier_heading(g.map, p, start_yaw.value_or(0.));
          const auto previous =
              first ? start_yaw
                    : std::optional<double>(held.value_or(bearing(position(label(key).parent), p)));
          if (!body.segment(g.local(p), g.local(p), heading, previous, heading)) continue;
        }
        selected = key;
        best = score;
      }
    }
  }
  if (selected < 0) return {{}, frontier ? "no_reachable_frontier" : "no_route"};
  std::vector<Point> path;
  for (int key = selected; key != initial; key = label(key).parent) path.push_back(position(key));
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  // Remove straight-through lattice vertices without changing any turn or swept area.
  std::vector<Point> corners;
  for (auto p : path) {
    if (corners.size() >= 2) {
      const auto u = corners[corners.size() - 2], v = corners.back();
      if ((v.x - u.x) * (p.x - v.x) + (v.y - u.y) * (p.y - v.y) > 0. && segment_distance(v, u, p) < 1e-10)
        corners.pop_back();
    }
    corners.push_back(p);
  }
  return {std::move(corners), arrived ? "goal" : "frontier"};
}

Route search_route(const Map &map, Point start, std::optional<Point> goal, bool unknown, bool frontier,
                   const std::vector<Point> &excluded, const Check &cancel, std::optional<Point> start_clear,
                   double clearance, const Map *peer, std::vector<int> &field,
                   std::optional<double> start_yaw = {}, std::optional<double> end_yaw = {},
                   std::optional<double> held = {}) {
  Grid g(map);
  if (!valid_clearance(g, clearance) || !valid_clearance(g, body_radius) || !finite(start) ||
      (goal && !finite(*goal)) || (start_yaw && !std::isfinite(*start_yaw)) ||
      (end_yaw && !std::isfinite(*end_yaw)) || (held && !std::isfinite(*held)) || (!goal && !frontier))
    return {{}, "invalid_map_or_goal"};
  if (!g.configure_start(start_clear)) return {{}, "start_area_exceeds_map_limits"};
  if (!g.configure_peer(peer)) return {{}, "invalid_peer_map"};
  const auto a = g.local(start);
  const auto b = goal ? g.local(*goal) : a;
  if (goal) {
    auto obstruction = terminal_obstruction(g, b, end_yaw);
    if (!obstruction.empty()) return {{}, std::move(obstruction)};
  }
  unknown = unknown && goal.has_value();
  const int radius = static_cast<int>(std::ceil(clearance / g.res));
  if (unknown && (!g.include(a, radius + 2) || !g.include(b, radius + 2)))
    return {{}, "unknown_route_exceeds_map_limits"};
  if (!g.contains(a)) return {{}, "start_outside_map"};
  const int origin_value = g.value(floor_cell(a.x), floor_cell(a.y));
  if (origin_value > occupied_threshold || (!unknown && origin_value < 0))
    return {{}, "start_unknown_or_occupied"};
  const BodyMotion body{g, unknown};
  const auto direct = [&] {
    if (!goal || !g.contains(b)) return false;
    const double tangent =
        distance(start, *goal) > 0. ? bearing(start, *goal) : start_yaw.value_or(end_yaw.value_or(0.));
    return body.segment(a, b, held.value_or(tangent), start_yaw, end_yaw) ||
           (held && *held != tangent && body.segment(a, b, tangent, start_yaw, end_yaw));
  };
  // Within the existing local reference horizon, a clear continuous sweep avoids a full-map search.
  const bool short_leg = goal && distance(start, *goal) <= (unknown ? 0.7 : 1.2);
  if (stopped(cancel)) return {{}, "cancelled"};
  if (short_leg && direct()) return {{start, *goal}, "goal"};
  auto wide =
      wide_search(map, start, goal, unknown, frontier, excluded, cancel, start_clear, clearance, peer, field);
  if (stopped(cancel)) return {{}, "cancelled"};
  if (!wide.points.empty()) {
    const bool at_frontier = wide.reason == "frontier";
    auto end = at_frontier
                   ? std::optional<double>(frontier_heading(map, wide.points.back(), start_yaw.value_or(0.)))
                   : end_yaw;
    if (!body_path(body, start, wide, start_yaw, end, held, cancel) &&
        !(held && body_path(body, start, wide, start_yaw, end, {}, cancel)))
      wide.points.clear();
    else if (!at_frontier || !goal || !g.contains(b) || g.value(floor_cell(b.x), floor_cell(b.y)) < 0)
      return wide;
  }
  if (!short_leg && direct()) return {{start, *goal}, "goal"};
  // A verified wide frontier is already available if the tight search cannot reach the goal.
  auto narrow = oriented_search(g, start, goal, unknown, frontier && wide.points.empty(), excluded, cancel,
                                field, start_yaw, end_yaw, held);
  if (held && !stopped(cancel) &&
      (narrow.points.empty() || (goal && narrow.reason != "goal" && g.contains(b) &&
                                 (unknown || g.value(floor_cell(b.x), floor_cell(b.y)) >= 0)))) {
    auto turning =
        oriented_search(g, start, goal, unknown, frontier && narrow.points.empty() && wide.points.empty(),
                        excluded, cancel, field, start_yaw, end_yaw, {});
    if (!turning.points.empty() || stopped(cancel)) return turning;
  }
  if (!narrow.points.empty() || wide.points.empty() || stopped(cancel)) return narrow;
  return wide;
}
}  // namespace

double frontier_heading(const Map &map, Point point, double fallback_yaw) {
  Grid g(map);
  if (!g.valid) return fallback_yaw;
  const Point local = g.local(point);
  if (!g.contains(local)) return fallback_yaw;
  const int cx = floor_cell(local.x), cy = floor_cell(local.y);
  const int limit = static_cast<int>(std::min<double>(max_side, std::ceil(sensing_distance / g.res))) + 1;
  double best = std::numeric_limits<double>::infinity();
  double heading = fallback_yaw;
  for (int y = std::max(-1, cy - limit); y <= std::min(g.height, cy + limit); ++y) {
    for (int x = std::max(-1, cx - limit); x <= std::min(g.width, cx + limit); ++x) {
      if (g.value(x, y) >= 0) continue;
      const Point unknown = g.world(x, y);
      const double dx = unknown.x - point.x, dy = unknown.y - point.y;
      const double distance = std::hypot(dx, dy);
      if (distance >= 0.20 && distance <= sensing_distance && distance < best) {
        best = distance;
        heading = std::atan2(dy, dx);
      }
    }
  }
  return heading;
}

double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }

double segment_distance(Point p, Point a, Point b) {
  const double dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
  if (!length2) return distance(p, a);
  const double f = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length2, 0., 1.);
  return distance(p, {a.x + f * dx, a.y + f * dy});
}

RouteCursor tracking_vertex(const Map &map, const std::vector<Point> &points, std::size_t next,
                            Point robot, std::optional<double> yaw, bool allow_unknown,
                            std::optional<Point> start_clear, const Map *peer,
                            std::optional<double> end_yaw, bool can_strafe, std::optional<Point> previous) {
  while (next + 1 < points.size()) {
    const Point near = points[next], after = points[next + 1];
    const double range = distance(robot, near);
    if (range > 1e-8) {
      const double outgoing = segment_distance(robot, near, after);
      const bool passed = next && (robot.x - near.x) * (after.x - near.x) +
                                      (robot.y - near.y) * (after.y - near.y) > 0 &&
                          outgoing <= 0.10 && outgoing < segment_distance(robot, points[next - 1], near);
      if (range > 0.10 && !passed && !(previous && segment_distance(near, *previous, robot) <= 0.10))
        break;
      const auto joins = [&](std::optional<double> held) {
        const auto departure = held.value_or(bearing(robot, after));
        const auto arrival = next + 2 == points.size() ? end_yaw :
                             std::optional<double>{held.value_or(bearing(after, points[next + 2]))};
        return clear_body_segment(map, robot, after, departure, allow_unknown, start_clear, peer, yaw, arrival);
      };
      if (!(can_strafe && yaw && joins(yaw)) && !joins({})) return {next, true};
    }
    ++next;
  }
  return {next, false};
}

bool repeats_blocked_leg(const Map &map, const std::vector<Point> &points, std::size_t next,
                         Point start, Point blocked_target, std::optional<double> yaw, bool allow_unknown,
                         std::optional<Point> start_clear, const Map *peer, std::optional<double> end_yaw,
                         bool can_strafe) {
  if (next >= points.size() || distance(start, blocked_target) <= 1e-8) return false;
  const auto cursor = tracking_vertex(map, points, next, start, yaw, allow_unknown,
                                      start_clear, peer, end_yaw, can_strafe);
  const Point target = points[cursor.next];
  const double pi = std::acos(-1.);
  return distance(start, target) > 1e-8 &&
         std::abs(std::remainder(bearing(start, target) - bearing(start, blocked_target), 2 * pi)) <= pi / 8;
}

Route local_detour(const Map &map, const Map &local, Point start, Point odom, double map_to_odom_yaw,
                   Point goal, const std::vector<Point> &excluded, std::optional<Point> start_clear,
                   const Check &cancel, const Map *peer, std::optional<double> start_yaw,
                   std::optional<double> goal_yaw, std::optional<Point> blocked_target) {
  if (stopped(cancel)) return {};
  const double range = distance(start, goal);
  Grid global(map);
  if (!valid_clearance(global, body_radius) || !global.configure_start(start_clear) ||
      !global.configure_peer(peer))
    return {};
  const auto last = global.local(goal);
  if (!finite(last) || !finite(start) || range == 0) return {};
  // A distant unobserved goal need not expand the entire local probing view.
  if (!terminal_obstruction(global, last, goal_yaw).empty()) return {};
  Grid local_grid(local);
  if (!valid_clearance(local_grid, body_radius)) return {};
  const double c = std::cos(map_to_odom_yaw), s = std::sin(map_to_odom_yaw);
  std::vector<int> local_obstacles;
  const auto probe = [&](Point target, bool final) -> Route {
    if (std::any_of(excluded.begin(), excluded.end(), [&](Point p) { return distance(target, p) < 0.35; }))
      return {};
    const double dx = target.x - start.x, dy = target.y - start.y;
    const Point odom_target{odom.x + c * dx - s * dy, odom.y + s * dx + c * dy};
    const auto local_yaw = start_yaw ? std::optional<double>(*start_yaw + map_to_odom_yaw) : std::nullopt;
    const auto terminal =
        final && goal_yaw ? std::optional<double>(*goal_yaw + map_to_odom_yaw) : std::nullopt;
    const auto odom_leg = search_route(local, odom, odom_target, false, false, {}, cancel, std::nullopt,
                                       body_radius, nullptr, local_obstacles, local_yaw, terminal, local_yaw);
    if (odom_leg.points.empty()) return {};
    Route leg = odom_leg;
    for (Point &p : leg.points) {
      const double x = p.x - odom.x, y = p.y - odom.y;
      p = {start.x + c * x + s * y, start.y - s * x + c * y};
    }
    leg.points.back() = target;
    if (blocked_target && repeats_blocked_leg(map, leg.points, 0, start, *blocked_target, start_yaw, true,
                                             start_clear, peer, final ? goal_yaw : std::nullopt, true))
      return {};
    // The same heading model must pass both the FPGA observation and the map checks.
    const auto clear_in_both = [&](std::optional<double> map_held, std::optional<double> odom_held) {
      return body_path(BodyMotion{global, true}, start, leg, start_yaw, final ? goal_yaw : std::nullopt,
                       map_held, cancel) &&
             body_path(BodyMotion{local_grid, false}, odom, odom_leg, local_yaw, terminal, odom_held, cancel);
    };
    if (!clear_in_both(start_yaw, local_yaw) && !(start_yaw && clear_in_both({}, {}))) return {};
    if (stopped(cancel)) return {};
    leg.reason = final ? "unknown_goal" : "local_fallback";
    return leg;
  };

  constexpr double horizon = 0.7;
  if (range <= horizon) {
    auto leg = probe(goal, true);
    if (!leg.points.empty()) return leg;
  }
  const double heading = bearing(start, goal), pi = std::acos(-1.);
  double previous = -1.;
  for (double limit : {horizon, 0.5, 0.3, 0.15}) {
    const double length = std::min(range, limit);
    if (length == previous) continue;
    previous = length;
    for (double offset : {0., pi / 4, -pi / 4, pi / 2, -pi / 2}) {
      if (stopped(cancel)) return {};
      if (offset == 0. && length == range) continue;
      const double angle = heading + offset;
      const Point target{start.x + length * std::cos(angle), start.y + length * std::sin(angle)};
      auto leg = probe(target, false);
      if (!leg.points.empty()) return leg;
    }
  }
  return {};
}

MissionRoute build_route(const nav_msgs::msg::OccupancyGrid &map, Point start, double yaw,
                         std::optional<Point> goal, std::optional<double> goal_yaw, bool use_mppi,
                         std::optional<Point> start_clear, const std::vector<Point> &excluded,
                         const std::function<bool()> &cancel, const nav_msgs::msg::OccupancyGrid *peer) {
  MissionRoute result;
  std::vector<int> obstacles;
  const auto held = goal && use_mppi ? std::nullopt : std::optional{yaw};
  result.route = search_route(map, start, goal, false, true, excluded, cancel, start_clear,
                              goal ? waypoint_clearance : body_radius, peer, obstacles, yaw, goal_yaw, held);
  if (result.route.reason == "goal") {
    result.known = true;
    return result;
  }
  if (cancel() ||
      (goal && result.route.reason != "frontier" && result.route.reason != "no_reachable_frontier" &&
       result.route.reason != "start_outside_map" && result.route.reason != "start_unknown_or_occupied"))
    return result;
  if (!goal && ((!result.route.points.empty() && distance(start, result.route.points.back()) > 0.35) ||
                (result.route.points.empty() && !excluded.empty())))
    return result;

  const double angle =
      goal ? std::atan2(goal->y - start.y, goal->x - start.x)
           : frontier_heading(map, result.route.points.empty() ? start : result.route.points.back(), yaw);
  double last_length = -1.0;
  for (double length : {0.7, 0.5, 0.3, 0.15}) {
    if (cancel()) break;
    if (goal) length = std::min(length, distance(start, *goal));
    if (length == last_length) continue;
    last_length = length;
    const bool reaches_goal = goal && length == distance(start, *goal);
    const Point target =
        reaches_goal ? *goal : Point{start.x + length * std::cos(angle), start.y + length * std::sin(angle)};
    if (std::any_of(excluded.begin(), excluded.end(), [&](Point p) { return distance(target, p) < 0.35; }))
      continue;
    obstacles.clear();  // Each probe can extend the grid by a different number of cells.
    auto probe = search_route(map, start, target, true, false, excluded, cancel, start_clear, body_radius,
                              peer, obstacles, yaw, reaches_goal ? goal_yaw : std::nullopt, yaw);
    if (!probe.points.empty()) {
      probe.reason = reaches_goal ? "unknown_goal" : "unknown";
      result.route = std::move(probe);
      return result;
    }
    if (reaches_goal && (probe.reason == "goal_occupied" || probe.reason == "goal_footprint_blocked")) {
      return {std::move(probe), false};
    }
  }
  return result;
}

nav_msgs::msg::Path controller_path(const std::vector<Point> &points, double final_yaw,
                                    const std_msgs::msg::Header &header, double resolution) {
  nav_msgs::msg::Path path;
  path.header = header;
  auto append = [&](Point p, double angle) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.orientation.z = std::sin(angle / 2);
    pose.pose.orientation.w = std::cos(angle / 2);
    path.poses.push_back(pose);
  };
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto a = points[i - 1], b = points[i];
    if (distance(a, b) < 1e-9) continue;
    const int steps = static_cast<int>(std::ceil(distance(a, b) / resolution));
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    for (int j = 0; j < steps; ++j) {
      const double f = static_cast<double>(j) / steps;
      append({a.x + f * (b.x - a.x), a.y + f * (b.y - a.y)}, heading);
    }
  }
  if (!points.empty()) append(points.back(), final_yaw);
  return path;
}

bool clear_ahead(const nav_msgs::msg::OccupancyGrid &map, const std::vector<Point> &points, std::size_t next,
                 Point robot, bool allow_unknown, std::optional<Point> start_clear,
                 const nav_msgs::msg::OccupancyGrid *peer, std::optional<double> start_yaw,
                 std::optional<double> end_yaw, std::optional<double> held_yaw) {
  double remaining = obstacle_check_distance;
  for (; next < points.size() && remaining > 0; ++next) {
    const auto target = points[next];
    const double length = distance(robot, target);
    const auto end = next + 1 == points.size() && length <= remaining ? end_yaw : std::nullopt;
    if (length == 0. && !end) continue;
    const double f = length > remaining ? remaining / length : 1.;
    const double travel =
        held_yaw.value_or(length > 0. ? bearing(robot, target) : start_yaw.value_or(end.value_or(0.)));
    if (!clear_body_segment(map, robot,
                            {robot.x + f * (target.x - robot.x), robot.y + f * (target.y - robot.y)}, travel,
                            allow_unknown, start_clear, peer, start_yaw, end))
      return false;
    start_yaw = travel;
    remaining -= length;
    robot = target;
  }
  return true;
}
}  // namespace car_navigation
