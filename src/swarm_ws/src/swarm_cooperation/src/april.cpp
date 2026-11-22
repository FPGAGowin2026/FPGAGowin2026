#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#if defined(SWARM_APRIL_ARUCO_MODERN)
#include <opencv2/objdetect/aruco_detector.hpp>
#elif defined(SWARM_APRIL_ARUCO_LEGACY)
#include <opencv2/aruco.hpp>
#endif

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;
namespace {
using XY = std::vector<cv::Point2f>;
using XYZ = std::vector<cv::Point3f>;
using Transform = cv::Matx44d;
using BoardCorners = std::array<std::array<double, 3>, 4>;
using Tag = std::pair<int, XY>;
struct AlignmentUnavailable : std::runtime_error {
  using std::runtime_error::runtime_error;
};

template <class T>
using Array = py::array_t<T, py::array::c_style | py::array::forcecast>;

template <class T>
Array<T> as_array(py::handle value) {
  auto a = Array<T>::ensure(value);
  if (!a) throw py::value_error("Expected a numeric array");
  return a;
}

// These matrices outlive conversion of a Python list or non-contiguous array.
cv::Mat doubles(py::handle value, int rows, int cols = -1) {
  auto a = as_array<double>(value);
  if (cols < 0) cols = static_cast<int>(a.size());
  if (a.size() != static_cast<py::ssize_t>(rows) * cols) throw py::value_error("Invalid matrix shape");
  if (!cols) return {};
  return cv::Mat(rows, cols, CV_64F, const_cast<double *>(a.data())).clone();
}

template <class T>
py::array_t<T> array_from(const cv::Mat &matrix) {
  py::array_t<T> out({matrix.rows, matrix.cols});
  if (matrix.total()) {
    cv::Mat dst(matrix.size(), cv::traits::Type<T>::value, out.mutable_data());
    matrix.copyTo(dst);
  }
  return out;
}

XY xy_from(py::handle value) {
  auto a = as_array<float>(value);
  if (a.size() % 2) throw py::value_error("Expected pairs of coordinates");
  XY result(a.size() / 2);
  for (std::size_t i = 0; i < result.size(); ++i) result[i] = {a.data()[2 * i], a.data()[2 * i + 1]};
  return result;
}

py::array_t<float> xy_array(const XY &points) {
  return array_from<float>(cv::Mat(points).reshape(1, static_cast<int>(points.size())));
}

BoardCorners board_points(int tag, double size, double spacing) {
  // Kalibr's rotated tags on the 6x6 board, in decoded corner order.
  const int col = static_cast<int>(std::floor(tag / 6.));
  const double step = size * (1. + spacing), x = col * step, y = (tag - col * 6) * step;
  return {{{x, y + size, 0.}, {x, y, 0.}, {x + size, y, 0.}, {x + size, y + size, 0.}}};
}

struct Intrinsics {
  cv::Mat k, d;
  explicit Intrinsics(const py::dict &model)
      : k(doubles(model["matrix"], 3, 3)), d(doubles(model["distortion"], 1)) {}
  Intrinsics(py::handle matrix, py::handle distortion)
      : k(doubles(matrix, 3, 3)), d(doubles(distortion, 1)) {}
};

cv::Mat image_view(const py::array &a) {
  if (a.ndim() != 2 && a.ndim() != 3) CV_Error(cv::Error::StsBadArg, "Expected a BGR image");
  int depth;
  const char kind = a.dtype().kind();
  if (kind == 'u' && a.itemsize() == 1)
    depth = CV_8U;
  else if (kind == 'u' && a.itemsize() == 2)
    depth = CV_16U;
  else if (kind == 'f' && a.itemsize() == 4)
    depth = CV_32F;
  else
    CV_Error(cv::Error::StsUnsupportedFormat, "Unsupported BGR image depth");
  const int channels = a.ndim() == 2 ? 1 : static_cast<int>(a.shape(2));
  if (channels < 1 || channels > CV_CN_MAX) CV_Error(cv::Error::StsBadArg, "Invalid image channels");
  return cv::Mat(static_cast<int>(a.shape(0)), static_cast<int>(a.shape(1)), CV_MAKETYPE(depth, channels),
                 const_cast<void *>(a.data()));
}

class MarkerDetector {
 public:
  explicit MarkerDetector(py::object fallback) : fallback_(std::move(fallback)) {
#if defined(SWARM_APRIL_ARUCO_MODERN)
    cv::aruco::DetectorParameters parameters;
    parameters.markerBorderBits = 2;
    detector_.emplace(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), parameters);
#elif defined(SWARM_APRIL_ARUCO_LEGACY)
    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    parameters_ = cv::aruco::DetectorParameters::create();
    parameters_->markerBorderBits = 2;
#endif
  }

  void detect(const cv::Mat &gray, std::vector<XY> &corners, std::vector<int> &ids) {
#if defined(SWARM_APRIL_ARUCO_MODERN)
    detector_->detectMarkers(gray, corners, ids);
#elif defined(SWARM_APRIL_ARUCO_LEGACY)
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, parameters_);
#else
    py::gil_scoped_acquire acquire;
    // Some installations provide ArUco only through their Python OpenCV build.
    const auto type = gray.depth() == CV_8U    ? py::dtype::of<std::uint8_t>()
                      : gray.depth() == CV_16U ? py::dtype::of<std::uint16_t>()
                                               : py::dtype::of<float>();
    py::capsule owner(new cv::Mat(gray), [](void *p) { delete static_cast<cv::Mat *>(p); });
    const std::array<py::ssize_t, 2> shape{gray.rows, gray.cols};
    const std::array<py::ssize_t, 2> strides{static_cast<py::ssize_t>(gray.step[0]),
                                             static_cast<py::ssize_t>(gray.elemSize())};
    const py::array image(type, shape, strides, gray.data, owner);
    py::sequence detected = fallback_(image);
    if (detected[1].is_none()) return;
    auto labels = as_array<int>(detected[1]);
    for (auto corner : py::reinterpret_borrow<py::iterable>(detected[0])) corners.push_back(xy_from(corner));
    ids.assign(labels.data(), labels.data() + labels.size());
#endif
  }

 private:
  py::object fallback_;
#if defined(SWARM_APRIL_ARUCO_MODERN)
  std::optional<cv::aruco::ArucoDetector> detector_;
#elif defined(SWARM_APRIL_ARUCO_LEGACY)
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
#endif
};

py::dict detect_tags(const py::array &image, py::object fallback) {
  auto contiguous = py::array::ensure(image, py::array::c_style);
  if (!contiguous) throw py::value_error("Expected an image array");
  const auto bgr = image_view(contiguous);
  MarkerDetector detector(std::move(fallback));
  std::vector<Tag> best;
  {
    py::gil_scoped_release release;
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    for (int scale = 1; scale <= 4; ++scale) {
      cv::Mat enlarged;
      if (scale == 1)
        enlarged = gray;
      else
        cv::resize(gray, enlarged, {}, scale, scale, cv::INTER_CUBIC);
      std::vector<XY> corners;
      std::vector<int> ids;
      detector.detect(enlarged, corners, ids);
      std::vector<Tag> found;
      std::array<int, 36> index;
      index.fill(-1);
      for (std::size_t i = 0; i < std::min(ids.size(), corners.size()); ++i) {
        const auto id = ids[i];
        if (id < 0 || id >= 36 || corners[i].size() != 4) continue;
        for (auto &p : corners[i]) p *= 1.f / scale;
        if (index[id] < 0) {
          index[id] = static_cast<int>(found.size());
          found.emplace_back(id, std::move(corners[i]));
        } else
          found[index[id]].second = std::move(corners[i]);
      }
      if (found.size() > best.size()) best = std::move(found);
      if (best.size() == 36) break;
    }
    for (auto &entry : best) {
      auto &corners = entry.second;
      XY initial;
      std::vector<int> indices;
      for (int i = 0; i < 4; ++i) {
        const auto &p = corners[i];
        if (p.x >= 3 && p.x < gray.cols - 3 && p.y >= 3 && p.y < gray.rows - 3) {
          indices.push_back(i);
          initial.push_back(p);
        }
      }
      if (initial.empty()) continue;
      auto refined = initial;
      cv::cornerSubPix(gray, refined, {2, 2}, {-1, -1},
                       {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, .1});
      for (std::size_t n = 0; n < indices.size(); ++n)
        if (cv::norm(refined[n] - initial[n]) < 1.5) corners[indices[n]] = refined[n];
    }
  }
  py::dict result;
  for (const auto &item : best) result[py::int_(item.first)] = xy_array(item.second);
  return result;
}

struct Correspondences {
  XYZ object;
  XY pixels;
};
Correspondences board_data(const py::dict &tags, const std::vector<int> &shared, double size,
                           double spacing) {
  Correspondences p;
  for (int tag : shared) {
    for (auto [x, y, z] : board_points(tag, size, spacing))
      p.object.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    auto corners = xy_from(tags[py::int_(tag)]);
    p.pixels.insert(p.pixels.end(), corners.begin(), corners.end());
  }
  return p;
}

double reprojection_rms(const Correspondences &p, const Intrinsics &cam, const cv::Mat &r, const cv::Mat &t) {
  XY projected;
  cv::projectPoints(p.object, r, t, cam.k, cam.d, projected);
  double sum = 0.;
  for (std::size_t i = 0; i < projected.size(); ++i) {
    const auto delta = projected[i] - p.pixels[i];
    sum += delta.dot(delta);
  }
  return std::sqrt(sum / projected.size());
}

struct PoseCandidate {
  Transform transform;
  double rms;
  int inliers;
};
std::vector<PoseCandidate> solve_board(const Correspondences &all, const Intrinsics &cam) {
  std::vector<cv::Mat> rotations, translations;
  Correspondences inliers;
  if (all.object.size() == 4) {
    cv::solvePnPGeneric(all.object, all.pixels, cam.k, cam.d, rotations, translations, false,
                        cv::SOLVEPNP_IPPE);
    inliers = all;
  } else {
    cv::Mat r, t;
    std::vector<int> indices;
    if (cv::solvePnPRansac(all.object, all.pixels, cam.k, cam.d, r, t, false, 200, 2.5, .999, indices,
                           cv::SOLVEPNP_ITERATIVE) &&
        indices.size() >= 6) {
      for (int i : indices) {
        inliers.object.push_back(all.object[i]);
        inliers.pixels.push_back(all.pixels[i]);
      }
      cv::solvePnPRefineLM(inliers.object, inliers.pixels, cam.k, cam.d, r, t);
      rotations.push_back(r);
      translations.push_back(t);
    }
  }
  std::vector<PoseCandidate> out;
  for (std::size_t i = 0; i < rotations.size(); ++i) {
    auto T = Transform::eye();
    cv::Mat rotation;
    cv::Rodrigues(rotations[i], rotation);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) T(row, col) = rotation.at<double>(row, col);
      T(row, 3) = translations[i].at<double>(row);
    }
    const double rms = reprojection_rms(inliers, cam, rotations[i], translations[i]);
    if (!cv::checkRange(cv::Mat(T)) || !(rms <= 2.5)) continue;
    if (std::all_of(all.object.begin(), all.object.end(), [&](const cv::Point3f &p) {
          return T(2, 0) * p.x + T(2, 1) * p.y + T(2, 2) * p.z + T(2, 3) > 0;
        }))
      out.push_back({T, rms, static_cast<int>(inliers.object.size())});
  }
  if (out.empty()) throw AlignmentUnavailable("无法计算标签板位姿或重投影误差过大");
  return out;
}

py::list board_pose(const py::dict &tags, const std::vector<int> &shared, const py::dict &model,
                    double tag_size, double tag_spacing) {
  const auto data = board_data(tags, shared, tag_size, tag_spacing);
  const Intrinsics cam(model);
  std::vector<PoseCandidate> poses;
  {
    py::gil_scoped_release release;
    poses = solve_board(data, cam);
  }
  py::list results;
  for (const auto &p : poses)
    results.append(py::make_tuple(array_from<double>(cv::Mat(p.transform)), p.rms, p.inliers));
  return results;
}

std::vector<int> tag_ids(const py::dict &tags) {
  std::vector<int> ids;
  for (auto item : tags) ids.push_back(py::cast<int>(item.first));
  std::sort(ids.begin(), ids.end());
  return ids;
}

py::dict poses_from_tags(const py::dict &car, const py::dict &hex, const py::dict &car_model,
                         const py::dict &hex_model, double size, double spacing) {
  std::vector<int> shared;
  for (int tag : tag_ids(car))
    if (hex.contains(py::int_(tag))) shared.push_back(tag);
  if (shared.empty()) throw AlignmentUnavailable("两台相机尚未识别到同一 AprilTag");
  if (shared.size() == 1) {
    for (auto tags : {car, hex}) {
      const auto corners = xy_from(tags[py::int_(shared.front())]);
      double perimeter = 0.;
      for (int i = 0; i < 4; ++i) perimeter += cv::norm(corners.at(i) - corners.at((i + 1) % 4));
      if (perimeter / 4. < 25.) throw AlignmentUnavailable("单个共同标签在画面中太小，请将标定板靠近相机");
    }
  }
  py::dict result;
  result["car_candidates"] = board_pose(car, shared, car_model, size, spacing);
  result["hex_candidates"] = board_pose(hex, shared, hex_model, size, spacing);
  result["tags"] = shared;
  return result;
}

py::array_t<double> transform_matrix(const std::array<double, 3> &t, const std::array<double, 4> &q) {
  const double length = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!std::isfinite(length) || length < .5) throw AlignmentUnavailable("坐标变换的旋转无效");
  const double x = q[0] / length, y = q[1] / length, z = q[2] / length, w = q[3] / length;
  Transform m(1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t[0], 2 * (x * y + z * w),
              1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t[1], 2 * (x * z - y * w), 2 * (y * z + x * w),
              1 - 2 * (x * x + y * y), t[2], 0., 0., 0., 1.);
  if (!cv::checkRange(cv::Mat(m))) throw AlignmentUnavailable("坐标变换包含无效数值");
  return array_from<double>(cv::Mat(m));
}

Transform inverse(const Transform &m) {
  bool ok = false;
  const auto result = m.inv(cv::DECOMP_LU, &ok);
  if (!ok) {
    const auto exception = py::module_::import("numpy.linalg").attr("LinAlgError");
    PyErr_SetString(exception.ptr(), "Singular matrix");
    throw py::error_already_set();
  }
  return result;
}

std::vector<PoseCandidate> candidates(py::handle sequence) {
  std::vector<PoseCandidate> out;
  for (auto entry : py::reinterpret_borrow<py::iterable>(sequence)) {
    const auto item = py::reinterpret_borrow<py::sequence>(entry);
    out.push_back(
        {static_cast<Transform>(doubles(item[0], 4, 4)), item[1].cast<double>(), item[2].cast<int>()});
  }
  return out;
}

py::tuple map_alignment(py::handle car_map_camera, py::handle hex_map_camera, py::dict observation) {
  const auto cars = candidates(observation["car_candidates"]),
             hexes = candidates(observation["hex_candidates"]);
  if (cars.empty() || hexes.empty()) throw py::value_error("min() arg is an empty sequence");
  const Transform car_world = static_cast<Transform>(doubles(car_map_camera, 4, 4));
  const auto hex_world_inv = inverse(static_cast<Transform>(doubles(hex_map_camera, 4, 4)));
  Transform best;
  double best_tilt{}, best_error{};
  const PoseCandidate *car_best = nullptr, *hex_best = nullptr;
  for (const auto &car : cars)
    for (const auto &hex : hexes) {
      const auto m = car_world * car.transform * inverse(hex.transform) * hex_world_inv;
      const double tilt = std::acos(std::clamp(m(2, 2), -1., 1.)), error = car.rms + hex.rms;
      if (!car_best || tilt < best_tilt || (tilt == best_tilt && error < best_error)) {
        best = m;
        best_tilt = tilt;
        best_error = error;
        car_best = &car;
        hex_best = &hex;
      }
    }
  const double degrees = best_tilt * 180. / CV_PI;
  if (best_tilt > 15. * CV_PI / 180.) {
    std::ostringstream msg;
    msg << "两张地图倾角差 " << std::fixed << std::setprecision(1) << degrees
        << "°，请核对相机安装参数和地图姿态";
    throw AlignmentUnavailable(msg.str());
  }
  observation["car_reprojection_px"] = car_best->rms;
  observation["hex_reprojection_px"] = hex_best->rms;
  observation["car_inliers"] = car_best->inliers;
  observation["hex_inliers"] = hex_best->inliers;
  observation["map_tilt_deg"] = degrees;
  return py::make_tuple(best(0, 3), best(1, 3), std::atan2(best(1, 0), best(0, 0)));
}

py::object observe_grid(const py::dict &tags, double tag_size, double tag_spacing, py::object filename) {
  if (tags.size() < 6) return py::none();
  const auto ids = tag_ids(tags);
  const auto p = board_data(tags, ids, tag_size, tag_spacing);
  XY object, pixels, kept_object;
  cv::Mat homography;
  {
    py::gil_scoped_release release;
    for (const auto &v : p.object) object.emplace_back(v.x, v.y);
    std::vector<std::uint8_t> mask;
    homography = cv::findHomography(object, p.pixels, cv::RANSAC, 2., mask);
    for (std::size_t i = 0; i < mask.size(); ++i) {
      if (!mask[i]) continue;
      kept_object.push_back(object[i]);
      pixels.push_back(p.pixels[i]);
    }
  }
  if (pixels.size() < 24 || pixels.size() < .8 * object.size()) return py::none();
  auto low = pixels.front(), high = low;
  for (const auto &v : pixels) {
    low.x = std::min(low.x, v.x);
    low.y = std::min(low.y, v.y);
    high.x = std::max(high.x, v.x);
    high.y = std::max(high.y, v.y);
  }
  py::dict out;
  out["file"] = filename;
  out["tag_ids"] = ids;
  out["object_points_xy_m"] = xy_array(kept_object).attr("tolist")();
  out["image_points_px"] = xy_array(pixels).attr("tolist")();
  out["homography"] = array_from<double>(homography).attr("tolist")();
  out["center"] = std::array<float, 2>{(low.x + high.x) / 2, (low.y + high.y) / 2};
  out["size"] = std::array<float, 2>{high.x - low.x, high.y - low.y};
  return out;
}

py::list distinct_views(py::iterable observations, double tag_size, double tag_spacing) {
  const float side = static_cast<float>(tag_size * (6. + 5. * tag_spacing));
  const XY corners{{0.f, 0.f}, {side, 0.f}, {side, side}, {0.f, side}};
  std::vector<XY> projections;
  py::list selected;
  for (auto entry : observations) {
    const auto item = py::reinterpret_borrow<py::dict>(entry);
    XY projected;
    cv::perspectiveTransform(corners, projected, doubles(item["homography"], 3, 3));
    const bool repeated = std::any_of(projections.begin(), projections.end(), [&](const XY &old) {
      float sum = 0.f;
      for (std::size_t i = 0; i < old.size(); ++i) {
        const auto d = projected[i] - old[i];
        sum += d.dot(d);
      }
      return std::sqrt(sum / 4.f) < 8.f;
    });
    if (!repeated) {
      selected.append(item);
      projections.push_back(std::move(projected));
    }
  }
  return selected;
}

Correspondences calibration_data(const py::dict &item) {
  const auto planar = xy_from(item["object_points_xy_m"]);
  Correspondences p;
  p.object.reserve(planar.size());
  for (const auto &v : planar) p.object.emplace_back(v.x, v.y, 0.f);
  p.pixels = xy_from(item["image_points_px"]);
  return p;
}

py::tuple points(const py::dict &observation) {
  const auto xy = as_array<float>(observation["object_points_xy_m"]);
  if (xy.ndim() != 1 && xy.ndim() != 2) throw py::value_error("Expected a table of planar coordinates");
  const auto rows = xy.shape(0), cols = xy.ndim() == 1 ? py::ssize_t{1} : xy.shape(1);
  py::array_t<float> object({rows, cols + 1});
  auto out = object.mutable_unchecked<2>();
  for (py::ssize_t i = 0; i < rows; ++i) {
    for (py::ssize_t j = 0; j < cols; ++j) out(i, j) = xy.data()[i * cols + j];
    out(i, cols) = 0.f;
  }
  // Keep the caller's contiguous Nx2 (or Nx1x2) image point representation.
  return py::make_tuple(std::move(object), as_array<float>(observation["image_points_px"]));
}

py::tuple fit(py::iterable observations, const std::array<int, 2> &size) {
  std::vector<XYZ> objects;
  std::vector<XY> pixels;
  for (auto item : observations) {
    auto p = calibration_data(py::reinterpret_borrow<py::dict>(item));
    objects.push_back(std::move(p.object));
    pixels.push_back(std::move(p.pixels));
  }
  double rms;
  cv::Mat matrix, distortion;
  {
    py::gil_scoped_release release;
    rms = cv::calibrateCamera(objects, pixels, {size[0], size[1]}, matrix, distortion, cv::noArray(),
                              cv::noArray(), cv::CALIB_FIX_K3);
  }
  if (!std::isfinite(rms) || !cv::checkRange(matrix) || !cv::checkRange(distortion) ||
      matrix.at<double>(0, 0) <= 0 || matrix.at<double>(1, 1) <= 0)
    throw py::value_error("Camera fit is not finite with positive focal lengths");
  // Python calibrateCamera's default coefficients are a 1x5 row.
  return py::make_tuple(rms, array_from<double>(matrix), array_from<double>(distortion.reshape(1, 1)));
}

py::tuple evaluate(py::iterable observations, py::handle matrix, py::handle distortion) {
  const Intrinsics cam(matrix, distortion);
  py::list errors, poses;
  for (auto item : observations) {
    const auto p = calibration_data(py::reinterpret_borrow<py::dict>(item));
    cv::Mat r, t;
    double error;
    {
      py::gil_scoped_release release;
      if (!cv::solvePnP(p.object, p.pixels, cam.k, cam.d, r, t)) continue;
      error = reprojection_rms(p, cam, r, t);
    }
    errors.append(error);
    poses.append(py::make_tuple(array_from<double>(r), array_from<double>(t)));
  }
  return py::make_tuple(errors, poses);
}
}  // namespace

PYBIND11_MODULE(_april, m) {
  using py::arg;
  py::register_local_exception<AlignmentUnavailable>(m, "AlignmentUnavailable", PyExc_ValueError);
  py::register_local_exception_translator([](std::exception_ptr p) {
    try {
      if (p) std::rethrow_exception(p);
    } catch (const cv::Exception &e) {
      const auto cls = py::module_::import("cv2").attr("error");
      auto ex = cls(e.what());
      ex.attr("code") = e.code;
      ex.attr("file") = e.file;
      ex.attr("func") = e.func;
      ex.attr("line") = e.line;
      ex.attr("err") = e.err;
      ex.attr("msg") = e.msg;
      PyErr_SetObject(cls.ptr(), ex.ptr());
    }
  });
#if defined(SWARM_APRIL_ARUCO_MODERN) || defined(SWARM_APRIL_ARUCO_LEGACY)
  m.attr("native_detector") = true;
#else
  m.attr("native_detector") = false;
#endif
  m.def("detect_tags", &detect_tags, arg("image"), arg("fallback") = py::none());
  m.def("board_points", &board_points, arg("tag"), arg("tag_size"), arg("tag_spacing"));
  m.def("board_pose", &board_pose, arg("tags"), arg("shared"), arg("model"), arg("tag_size"),
        arg("tag_spacing"));
  m.def("poses_from_tags", &poses_from_tags);
  m.def("transform_matrix", &transform_matrix);
  m.def("map_alignment", &map_alignment, arg("car_map_camera"), arg("hex_map_camera"), arg("observation"));
  m.def("observe_grid", &observe_grid);
  m.def("distinct_views", &distinct_views, arg("observations"), arg("tag_size"), arg("tag_spacing"));
  m.def("points", &points, arg("observation"));
  m.def("fit", &fit, arg("observations"), arg("size"));
  m.def("evaluate", &evaluate, arg("observations"), arg("matrix"), arg("distortion"));
}
