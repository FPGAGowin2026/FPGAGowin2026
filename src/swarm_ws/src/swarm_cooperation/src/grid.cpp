// Planar map fusion and frontier selection used by the cooperative coordinator.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;
namespace {
using Point = std::pair<double, double>;
using Triple = std::tuple<double, double, double>;
using Bounds = std::array<double, 4>;
using Cells = std::vector<std::int8_t>;
using Mask = std::vector<std::uint8_t>;
using Target = std::optional<std::pair<Point, Point>>;

struct Pose {
  double x, y, a, c, s;
  Pose(double x0 = 0., double y0 = 0., double angle = 0.)
      : x(x0), y(y0), a(angle), c(std::cos(a)), s(std::sin(a)) {}
  explicit Pose(Triple t) : Pose(std::get<0>(t), std::get<1>(t), std::get<2>(t)) {}
  Point apply(double u, double v) const { return {x + c * u - s * v, y + s * u + c * v}; }
  Point inverse(double u, double v) const {
    const double dx = u - x, dy = v - y;
    return {c * dx + s * dy, -s * dx + c * dy};
  }
  Pose compose(const Pose &child) const {
    const auto [u, v] = apply(child.x, child.y);
    return {u, v, a + child.a};
  }
  Triple tuple() const { return {x, y, a}; }
};

template <class T>
using Array = py::array_t<T, py::array::c_style | py::array::forcecast>;

struct Grid {
  int w = 0, h = 0;
  double r = 1.;
  Pose origin;
  Cells data;
  Bounds bounds(const Pose &alignment) const {
    const Pose p = alignment.compose(origin);
    Bounds b{p.x, p.y, p.x, p.y};
    for (double x : {0., w * r})
      for (double y : {0., h * r}) {
        const auto [u, v] = p.apply(x, y);
        b = {std::min(b[0], u), std::min(b[1], v), std::max(b[2], u), std::max(b[3], v)};
      }
    return b;
  }
};

int area(int w, int h) {
  if (w < 0 || h < 0 || static_cast<std::int64_t>(w) * h > std::numeric_limits<int>::max())
    throw std::invalid_argument("invalid map dimensions");
  return w * h;
}

Grid read_grid(const py::dict &d, bool with_cells = true) {
  Grid g;
  g.w = d["width"].cast<int>();
  g.h = d["height"].cast<int>();
  g.r = d["resolution"].cast<double>();
  g.origin = Pose(d["origin"].cast<Triple>());
  const int size = area(g.w, g.h);
  if (!(g.r > 0.) || !std::isfinite(g.r) || !std::isfinite(g.origin.x) || !std::isfinite(g.origin.y) ||
      !std::isfinite(g.origin.a))
    throw std::invalid_argument("invalid grid resolution or origin");
  if (with_cells) {
    auto a = Array<std::int8_t>::ensure(d["data"]);
    if (!a || a.size() != size) throw std::invalid_argument("grid data does not match its dimensions");
    if (size) g.data.assign(a.data(), a.data() + size);
  }
  return g;
}

std::optional<Grid> optional_grid(const py::object &d) {
  if (d.is_none()) return std::nullopt;
  return read_grid(d.cast<py::dict>());
}

int clipped(double edge, int limit) {
  return static_cast<int>(std::clamp(edge, 0., static_cast<double>(limit)));
}

using Placed = std::pair<Grid, Pose>;
Grid merge(const std::vector<Placed> &sources, std::int64_t maximum) {
  if (sources.empty()) throw std::invalid_argument("没有可合并的地图");
  auto bounds = sources.front().first.bounds(sources.front().second);
  double r = sources.front().first.r;
  for (const auto &[g, t] : sources) {
    if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.a))
      throw std::invalid_argument("invalid map alignment");
    const auto b = g.bounds(t);
    r = std::min(r, g.r);
    bounds = {std::min(bounds[0], b[0]), std::min(bounds[1], b[1]), std::max(bounds[2], b[2]),
              std::max(bounds[3], b[3])};
  }
  const double ox = std::floor(bounds[0] / r) * r, oy = std::floor(bounds[1] / r) * r;
  const double width = std::ceil((bounds[2] - ox) / r), height = std::ceil((bounds[3] - oy) / r);
  if (!(width > 0. && height > 0. && width * height <= maximum &&
        width * height <= std::numeric_limits<int>::max()))
    throw std::invalid_argument("共同地图范围异常或过大");
  Grid out{static_cast<int>(width), static_cast<int>(height), r, {ox, oy, 0.}, {}};
  out.data.assign(area(out.w, out.h), -1);
  for (const auto &[g, t] : sources) {
    const auto b = g.bounds(t);
    const Pose p = t.compose(g.origin);
    // All cells use the containing source sample. Additionally project the
    // occupied squares so that rotated, fine obstacles survive resampling.
    for (int y = clipped(std::floor((b[1] - oy) / r), out.h); y < clipped(std::ceil((b[3] - oy) / r), out.h);
         ++y) {
      for (int x = clipped(std::floor((b[0] - ox) / r), out.w);
           x < clipped(std::ceil((b[2] - ox) / r), out.w); ++x) {
        const auto [u, v] = p.inverse(ox + (x + .5) * r, oy + (y + .5) * r);
        if (u < 0 || v < 0 || u / g.r >= g.w || v / g.r >= g.h) continue;
        auto &dest = out.data[y * out.w + x];
        dest = std::max(dest, g.data[static_cast<int>(v / g.r) * g.w + static_cast<int>(u / g.r)]);
      }
    }
    const double span = std::abs(p.c) + std::abs(p.s), extent = g.r * span / 2.;
    const double tolerance = std::min(g.r, r) * 1e-9;
    const double source_limit = (g.r + r * span) / 2. - tolerance;
    const double output_limit = (r + g.r * span) / 2. - tolerance;
    for (int i = 0; i < static_cast<int>(g.data.size()); ++i) {
      if (g.data[i] <= 20) continue;
      const auto [u, v] = p.apply((i % g.w + .5) * g.r, (i / g.w + .5) * g.r);
      const int x0 = clipped(std::floor((u - extent - ox) / r), out.w),
                x1 = clipped(std::ceil((u + extent - ox) / r), out.w);
      const int y0 = clipped(std::floor((v - extent - oy) / r), out.h),
                y1 = clipped(std::ceil((v + extent - oy) / r), out.h);
      for (int y = y0; y < y1; ++y)
        for (int x = x0; x < x1; ++x) {
          const double dx = ox + (x + .5) * r - u, dy = oy + (y + .5) * r - v;
          if (std::abs(dx) >= output_limit || std::abs(dy) >= output_limit ||
              std::abs(p.c * dx + p.s * dy) >= source_limit || std::abs(p.c * dy - p.s * dx) >= source_limit)
            continue;
          auto &dest = out.data[y * out.w + x];
          dest = std::max(dest, g.data[i]);
        }
    }
  }
  return out;
}

template <class T, class Predicate>
Mask mask_of(const std::vector<T> &data, Predicate predicate) {
  Mask result(data.size());
  std::transform(data.begin(), data.end(), result.begin(), predicate);
  return result;
}

// Prefix sums give one rectangle query per footprint row or transformed map cell.
template <class Total>
struct SummedArea {
  std::size_t stride;
  std::vector<Total> sums;
  template <class T>
  SummedArea(const T *values, int w, int h)
      : stride(static_cast<std::size_t>(w) + 1), sums(stride * (static_cast<std::size_t>(h) + 1)) {
    for (int y = 0; y < h; ++y) {
      Total row = 0;
      for (int x = 0; x < w; ++x) {
        row += values[y * w + x];
        sums[(y + 1) * stride + x + 1] = sums[y * stride + x + 1] + row;
      }
    }
  }
  SummedArea(const Mask &mask, int w, int h) : SummedArea(mask.data(), w, h) {}
  bool positive(int x0, int y0, int x1, int y1) const {
    return sums[y1 * stride + x1] - sums[y0 * stride + x1] - sums[y1 * stride + x0] + sums[y0 * stride + x0] >
           0;
  }
};

template <class Total, class T>
Mask positive_neighborhood(const T *values, int w, int h, int reach) {
  reach = std::min(reach, std::max(w, h));
  const SummedArea<Total> summed(values, w, h);
  Mask result(area(w, h));
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      result[y * w + x] = summed.positive(std::max(0, x - reach), std::max(0, y - reach),
                                          std::min(w, x + reach + 1), std::min(h, y + reach + 1));
  return result;
}

Mask neighborhood(const Mask &mask, int w, int h, int reach) {
  if (reach <= 0) return mask;
  return positive_neighborhood<int>(mask.data(), w, h, reach);
}

Mask traversable(const Mask &free, int w, int h, double radius, double r) {
  Mask result(free.size(), 0);
  if (w == 0 || h == 0 || 2 * radius > w * r || 2 * radius > h * r) return result;
  const int border = std::max(0, static_cast<int>(std::ceil(radius / r - .5)));
  const int reach = static_cast<int>(std::ceil(radius / r + .5));
  const SummedArea<int> blocked(mask_of(free, [](auto value) { return !value; }), w, h);
  std::vector<std::pair<int, int>> spans;
  for (int dy = -reach; dy <= reach; ++dy) {
    const double ydist = std::max(0., (std::abs(dy) - .5) * r);
    if (ydist * ydist > radius * radius) continue;
    int dx = reach;
    while (dx > 0) {
      const double xdist = (dx - .5) * r;
      if (xdist * xdist + ydist * ydist <= radius * radius) break;
      --dx;
    }
    spans.emplace_back(dy, dx);
  }
  for (int y = border; y < h - border; ++y)
    for (int x = border; x < w - border; ++x) {
      if (!free[y * w + x]) continue;
      bool clear = true;
      for (const auto &[dy, dx] : spans) {
        const int row = y + dy;
        if (row >= 0 && row < h &&
            blocked.positive(std::max(0, x - dx), row, std::min(w, x + dx + 1), row + 1)) {
          clear = false;
          break;
        }
      }
      result[y * w + x] = clear;
    }
  return result;
}

bool pose_clear(const Mask &free, int w, int h, double px, double py, double radius, double r) {
  if (px - radius < 0 || py - radius < 0 || px + radius > w * r || py + radius > h * r) return false;
  const int x0 = clipped(std::ceil((px - radius) / r) - 1, w),
            x1 = clipped(std::floor((px + radius) / r) + 1, w);
  const int y0 = clipped(std::ceil((py - radius) / r) - 1, h),
            y1 = clipped(std::floor((py + radius) / r) + 1, h);
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) {
      if (free[y * w + x]) continue;
      const double dx = std::max({x * r - px, 0., px - (x + 1) * r});
      const double dy = std::max({y * r - py, 0., py - (y + 1) * r});
      if (dx * dx + dy * dy <= radius * radius) return false;
    }
  return true;
}

void project_peer(Cells &cells, const Grid &grid, const std::optional<Grid> &peer, const Pose &alignment) {
  if (!peer) return;
  const auto &p = *peer;
  const SummedArea<int> hits(mask_of(p.data, [](auto v) { return v > 20; }), p.w, p.h);
  const Pose common = alignment.compose(grid.origin);
  const auto [tx, ty] = p.origin.inverse(common.x, common.y);
  const Pose relative{tx, ty, common.a - p.origin.a};
  // Frontier reachability uses the bounding box of each projected cell,
  // matching the coordinator's conservative peer-obstacle rule.
  const double extent = grid.r * (std::abs(relative.c) + std::abs(relative.s)) / 2.;
  for (int y = 0; y < grid.h; ++y)
    for (int x = 0; x < grid.w; ++x) {
      const auto [u, v] = relative.apply((x + .5) * grid.r, (y + .5) * grid.r);
      const int x0 = clipped(std::floor((u - extent) / p.r + 1e-9), p.w);
      const int x1 = clipped(std::ceil((u + extent) / p.r - 1e-9), p.w);
      const int y0 = clipped(std::floor((v - extent) / p.r + 1e-9), p.h);
      const int y1 = clipped(std::ceil((v + extent) / p.r - 1e-9), p.h);
      if (x0 < x1 && y0 < y1 && hits.positive(x0, y0, x1, y1)) cells[y * grid.w + x] = 100;
    }
}

struct Reachability {
  Mask passable, known;
  int sx = 0, sy = 0;
};

bool prepare_start(Grid &g, Point robot, double radius, const Pose &alignment,
                   const std::optional<Grid> &peer, bool allow_unknown, bool allow_transition,
                   Reachability &reach) {
  auto [px, py] = g.origin.inverse(robot.first, robot.second);
  Cells planning = g.data;
  project_peer(planning, g, peer, alignment);
  Mask free =
      mask_of(planning, [allow_unknown](auto value) { return value <= 20 && (allow_unknown || value >= 0); });
  reach.passable = traversable(free, g.w, g.h, radius, g.r);
  if (px >= 0 && py >= 0 && px / g.r < g.w && py / g.r < g.h) {
    reach.sx = static_cast<int>(px / g.r);
    reach.sy = static_cast<int>(py / g.r);
    if (reach.passable[reach.sy * g.w + reach.sx] && pose_clear(free, g.w, g.h, px, py, radius, g.r)) {
      reach.known = reach.passable;
      for (std::size_t i = 0; i < planning.size(); ++i)
        if (planning[i] < 0) reach.known[i] = 0;
      return true;
    }
  }
  if (!allow_transition ||
      std::hypot(std::max({-px, 0., px - g.w * g.r}), std::max({-py, 0., py - g.h * g.r})) > .30)
    return false;
  const double padding = std::ceil(.30 / g.r) + 1;
  if (padding > std::numeric_limits<int>::max() / 4) throw std::invalid_argument("startup area too large");
  const int margin = static_cast<int>(padding);
  int sx = static_cast<int>(std::floor(px / g.r)), sy = static_cast<int>(std::floor(py / g.r));
  const int left = std::max(0, margin - sx), bottom = std::max(0, margin - sy);
  const int right = std::max(0, sx + margin + 1 - g.w), top = std::max(0, sy + margin + 1 - g.h);
  const int w = g.w + left + right, h = g.h + bottom + top;
  Cells padded(area(w, h), -1);
  for (int row = 0; row < g.h; ++row)
    std::copy_n(g.data.begin() + row * g.w, g.w, padded.begin() + (row + bottom) * w + left);
  const auto [ox, oy] = g.origin.apply(-left * g.r, -bottom * g.r);
  g.origin = Pose(ox, oy, g.origin.a);
  g.w = w;
  g.h = h;
  g.data = std::move(padded);
  px += left * g.r;
  py += bottom * g.r;
  reach.sx = sx + left;
  reach.sy = sy + bottom;
  planning = g.data;
  project_peer(planning, g, peer, alignment);
  free = mask_of(planning, [](auto value) { return value >= 0 && value <= 20; });
  reach.known = traversable(free, w, h, radius, g.r);
  for (int y = std::max(0, reach.sy - margin); y < std::min(h, reach.sy + margin + 1); ++y)
    for (int x = std::max(0, reach.sx - margin); x < std::min(w, reach.sx + margin + 1); ++x) {
      const double dx = std::max(std::abs(x * g.r - px), std::abs((x + 1) * g.r - px));
      const double dy = std::max(std::abs(y * g.r - py), std::abs((y + 1) * g.r - py));
      if (planning[y * w + x] < 0 && dx * dx + dy * dy <= .30 * .30) free[y * w + x] = 1;
    }
  const int start = reach.sy * w + reach.sx;
  if (!free[start]) return false;
  // The real starting pose may touch unknown beyond the assumed circle. A
  // known obstacle in the physical footprint still blocks the transition.
  const int x0 = clipped(std::floor((px - radius) / g.r), w),
            x1 = clipped(std::floor((px + radius) / g.r) + 1, w);
  const int y0 = clipped(std::floor((py - radius) / g.r), h),
            y1 = clipped(std::floor((py + radius) / g.r) + 1, h);
  for (int y = y0; y < y1; ++y)
    for (int x = x0; x < x1; ++x) {
      const double dx = std::max({x * g.r - px, 0., px - (x + 1) * g.r}),
                   dy = std::max({y * g.r - py, 0., py - (y + 1) * g.r});
      if (planning[y * w + x] > 20 && dx * dx + dy * dy <= radius * radius) return false;
    }
  reach.passable = traversable(free, w, h, radius, g.r);
  reach.passable[start] = 1;
  return true;
}

int sensing_cells(const Grid &g, double sensing) {
  return static_cast<int>(std::clamp(std::ceil(sensing / g.r), 0., double(std::max(g.w, g.h))));
}

Target choose(Grid g, Point robot, double radius, double sensing, const std::vector<Point> &reservations,
              double separation, const Pose &alignment, const std::optional<Grid> &common,
              bool allow_transition, const std::optional<Grid> &peer, bool allow_unknown, bool soft) {
  Reachability reach;
  if (!prepare_start(g, robot, radius, alignment, peer, allow_unknown, allow_transition, reach))
    return std::nullopt;
  const auto unknown = [](auto v) { return v < 0; };
  const auto near = neighborhood(mask_of(g.data, unknown), g.w, g.h, sensing_cells(g, sensing));
  Mask common_near;
  if (common)
    common_near =
        neighborhood(mask_of(common->data, unknown), common->w, common->h, sensing_cells(*common, sensing));
  const Pose common_pose = alignment.compose(g.origin);
  Mask seen(g.data.size(), 0);
  std::vector<int> queue{reach.sy * g.w + reach.sx};
  seen[queue.front()] = 1;
  Target fallback;
  double best = -1;
  const double min_steps = std::max(1., std::ceil(radius / g.r));
  for (std::size_t head = 0; head < queue.size(); ++head) {
    const int i = queue[head], x = i % g.w, y = i / g.w;
    const double dx = x - reach.sx, dy = y - reach.sy;
    if (reach.known[i] && near[i] && dx * dx + dy * dy >= min_steps * min_steps) {
      const Point p = g.origin.apply((x + .5) * g.r, (y + .5) * g.r);
      const Point anchor = alignment.apply(p.first, p.second);
      bool visible = true;
      if (common) {
        const auto projected = common_pose.apply((x + .5) * g.r, (y + .5) * g.r);
        const auto [cx, cy] = common->origin.inverse(projected.first, projected.second);
        visible =
            cx >= 0 && cy >= 0 && cx / common->r < common->w && cy / common->r < common->h &&
            common_near[static_cast<int>(cy / common->r) * common->w + static_cast<int>(cx / common->r)];
      }
      if (visible) {
        double gap = std::numeric_limits<double>::infinity();
        for (const auto &[rx, ry] : reservations)
          gap = std::min(gap, std::hypot(anchor.first - rx, anchor.second - ry));
        if (gap >= separation) return std::make_pair(p, anchor);
        if (soft && gap > best) {
          best = gap;
          fallback = std::make_pair(p, anchor);
        }
      }
    }
    constexpr int steps[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    for (const auto &step : steps) {
      const int nx = x + step[0], ny = y + step[1];
      if (nx < 0 || ny < 0 || nx >= g.w || ny >= g.h) continue;
      const int neighbor = ny * g.w + nx;
      if (reach.passable[neighbor] && !seen[neighbor]) {
        seen[neighbor] = 1;
        queue.push_back(neighbor);
      }
    }
  }
  return fallback;
}

py::dict merge_result(const std::vector<Placed> &sources, std::int64_t max_cells) {
  Grid result;
  {
    py::gil_scoped_release release;
    result = merge(sources, max_cells);
  }
  return py::dict("width"_a = result.w, "height"_a = result.h, "resolution"_a = result.r,
                  "origin"_a = result.origin.tuple(), "data"_a = result.data);
}

struct Image {
  int w, h;
  Mask bits;
  explicit Image(const Array<bool> &a) {
    if (a.ndim() != 2 || a.shape(0) > std::numeric_limits<int>::max() ||
        a.shape(1) > std::numeric_limits<int>::max())
      throw std::invalid_argument("expected a 2-D mask");
    h = static_cast<int>(a.shape(0));
    w = static_cast<int>(a.shape(1));
    if (area(w, h)) bits.assign(a.data(), a.data() + a.size());
  }
  py::array_t<bool> array(const Mask &result) const {
    py::array_t<bool> out({h, w});
    std::copy(result.begin(), result.end(), out.mutable_data());
    return out;
  }
};

// These public transforms historically also accepted NumPy broadcast operands.
py::tuple transform_values(Triple pose, py::object x, py::object y, bool inverse) {
  const Pose p(pose);
  const py::float_ c(p.c), s(p.s), ox(p.x), oy(p.y);
  if (inverse) {
    auto dx = x - ox, dy = y - oy;
    return py::make_tuple(c * dx + s * dy, py::float_(-p.s) * dx + c * dy);
  }
  return py::make_tuple(ox + c * x - s * y, oy + s * x + c * y);
}
}  // namespace

PYBIND11_MODULE(grid, m) {
  m.doc() = "Planar grid transforms, map fusion and cooperative frontier selection";
  m.def(
      "transform_point",
      [](Triple p, py::object x, py::object y) { return transform_values(p, x, y, false); }, "pose"_a, "x"_a,
      "y"_a);
  m.def(
      "inverse_point", [](Triple p, py::object x, py::object y) { return transform_values(p, x, y, true); },
      "pose"_a, "x"_a, "y"_a);
  m.def(
      "compose_pose", [](Triple parent, Triple child) { return Pose(parent).compose(Pose(child)).tuple(); },
      "parent"_a, "child"_a);
  m.def(
      "inverse_pose",
      [](Triple value) {
        const Pose p(value);
        auto [x, y] = p.inverse(0., 0.);
        return Triple{x, y, -p.a};
      },
      "pose"_a);
  m.def(
      "world_pose",
      [](py::dict g, Triple t) { return Pose(t).compose(Pose(g["origin"].cast<Triple>())).tuple(); },
      "grid"_a, "alignment"_a);
  m.def(
      "grid_bounds",
      [](py::dict g, Triple t) {
        const auto b = read_grid(g, false).bounds(Pose(t));
        return py::make_tuple(b[0], b[1], b[2], b[3]);
      },
      "grid"_a, "alignment"_a);
  m.def(
      "merge_grids",
      [](py::dict car, py::dict hexapod, Triple tf, std::int64_t max_cells) {
        std::vector<Placed> sources;
        sources.emplace_back(read_grid(car), Pose());
        sources.emplace_back(read_grid(hexapod), Pose(tf));
        return merge_result(sources, max_cells);
      },
      "car"_a, "hexapod"_a, "alignment"_a, "max_cells"_a = 4000000);
  m.def(
      "merge_transformed_grids",
      [](py::iterable grids, std::int64_t max_cells) {
        std::vector<Placed> sources;
        for (const auto item : grids) {
          const auto entry = item.cast<std::pair<py::dict, Triple>>();
          sources.emplace_back(read_grid(entry.first), Pose(entry.second));
        }
        return merge_result(sources, max_cells);
      },
      "grids"_a, "max_cells"_a = 4000000);
  m.def(
      "neighborhood",
      [](const py::object &mask, int radius) -> py::object {
        if (radius <= 0) return mask.attr("copy")();
        auto values = Array<std::int32_t>::ensure(mask);
        if (!values || values.ndim() != 2 || values.shape(0) > std::numeric_limits<int>::max() ||
            values.shape(1) > std::numeric_limits<int>::max())
          throw std::invalid_argument("expected a 2-D mask");
        const int h = static_cast<int>(values.shape(0)), w = static_cast<int>(values.shape(1));
        area(w, h);
        py::array_t<bool> output({h, w});
        Mask result;
        const auto *data = values.data();
        auto *destination = output.mutable_data();
        {
          py::gil_scoped_release release;
          result = positive_neighborhood<std::int64_t>(data, w, h, radius);
          std::copy(result.begin(), result.end(), destination);
        }
        return output;
      },
      "mask"_a, "radius"_a);
  m.def(
      "_traversable",
      [](const Array<bool> &mask, double radius, double resolution) {
        const Image image(mask);
        if (!(resolution > 0) || !std::isfinite(radius)) throw std::invalid_argument("invalid geometry");
        Mask result;
        {
          py::gil_scoped_release release;
          result = traversable(image.bits, image.w, image.h, radius, resolution);
        }
        return image.array(result);
      },
      "free"_a, "radius"_a, "resolution"_a);
  m.def(
      "_pose_clear",
      [](const Array<bool> &mask, double x, double y, double radius, double resolution) {
        const Image image(mask);
        if (!(resolution > 0) || !std::isfinite(radius) || !std::isfinite(x) || !std::isfinite(y))
          throw std::invalid_argument("invalid geometry");
        py::gil_scoped_release release;
        return pose_clear(image.bits, image.w, image.h, x, y, radius, resolution);
      },
      "free"_a, "px"_a, "py"_a, "radius"_a, "resolution"_a);
  m.def(
      "exclude_receiver_footprint",
      [](py::dict input, Point receiver, double radius) {
        Grid g = read_grid(input);
        {
          py::gil_scoped_release release;
          const auto [px, py] = g.origin.inverse(receiver.first, receiver.second);
          for (int i = 0; i < static_cast<int>(g.data.size()); ++i) {
            if (g.data[i] <= 20) continue;
            const double dx = std::abs((i % g.w + .5) * g.r - px) + g.r / 2.,
                         dy = std::abs((i / g.w + .5) * g.r - py) + g.r / 2.;
            if (dx * dx + dy * dy <= radius * radius) g.data[i] = -1;
          }
        }
        auto output = input.attr("copy")().cast<py::dict>();
        output["data"] = g.data;
        return output;
      },
      "grid"_a, "receiver"_a, "radius"_a);
  m.def(
      "choose_frontier",
      [](py::dict input, Point robot, double radius, double sensing, const std::vector<Point> &reservations,
         double separation, Triple tf, py::object common, bool transition, py::object peer,
         bool allow_unknown, bool soft) {
        auto g = read_grid(input);
        const auto shared = optional_grid(common), obstacles = optional_grid(peer);
        if (!std::isfinite(robot.first) || !std::isfinite(robot.second) || !std::isfinite(radius) ||
            !std::isfinite(sensing) || radius < 0 || !std::isfinite(std::get<0>(tf)) ||
            !std::isfinite(std::get<1>(tf)) || !std::isfinite(std::get<2>(tf)))
          throw std::invalid_argument("invalid frontier geometry");
        py::gil_scoped_release release;
        return choose(std::move(g), robot, radius, sensing, reservations, separation, Pose(tf), shared,
                      transition, obstacles, allow_unknown, soft);
      },
      "grid"_a, "robot"_a, "radius"_a, "sensing"_a, "reservations"_a, "separation"_a, "alignment"_a,
      "common_grid"_a = py::none(), "allow_start_transition"_a = false, "peer_grid"_a = py::none(),
      "allow_unknown"_a = false, "soft_separation"_a = false);
}
