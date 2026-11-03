#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace bx72 {

// 仅显示FPGA报告的两点方向预览，不表示完整曲线路径或碰撞验收。
// camera_from_base是车体系到相机光学坐标系的行优先3x4变换。
class RouteOverlay {
 public:
  bool enabled() const { return enabled_; }
  void configure(const std::array<double, 9>& k, const std::vector<double>& distortion,
                 const std::vector<double>& camera_from_base) {
    if (camera_from_base.size() != 12 || distortion.size() != 5)
      throw std::invalid_argument("route overlay requires camera_from_base[12] and plumb_bob D[5]");
    for (double v : camera_from_base) if (!std::isfinite(v))
      throw std::invalid_argument("route overlay transform must be finite");
    // Reject scaled/reflected transforms: calibration must describe a rigid rotation.
    cv::Matx33d r(camera_from_base[0],camera_from_base[1],camera_from_base[2],
                  camera_from_base[4],camera_from_base[5],camera_from_base[6],
                  camera_from_base[8],camera_from_base[9],camera_from_base[10]);
    if (cv::norm(cv::Mat(r*r.t()-cv::Matx33d::eye())) > 1e-3 ||
        std::abs(cv::determinant(cv::Mat(r))-1) > 1e-3)
      throw std::invalid_argument("route overlay transform rotation must be orthonormal");
    k_ = k; d_ = distortion; t_ = camera_from_base; enabled_ = true;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    nav_ = pose_ = plan_ = nullptr;
  }

  void observe(const nlohmann::json& packet, double now) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    const auto type = packet.value("type", std::string{});
    if (type == "navigation") {
      if (nav_.is_object() && (packet.value("boot_id",0U) != nav_.value("boot_id",0U) ||
          packet.value("odom_domain",0U) != nav_.value("odom_domain",0U))) {
        pose_ = plan_ = nullptr;
      }
      nav_ = packet; nav_at_ = now;
    } else if (type == "fusion") { pose_ = packet; pose_at_ = now; }
    else if (type == "local_plan") { plan_ = packet; plan_at_ = now; }
  }

  bool draw(cv::Mat& image, double now, double frame_at) {
    if (!enabled_) return false;
    nlohmann::json nav, pose, plan;
    double nav_elapsed=0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (now-frame_at > .2 || now-pose_at_ > .2 || now-plan_at_ >= .5 || now-nav_at_ >= .5)
        return false;
      nav = nav_; pose = pose_; plan = plan_;
      nav_elapsed=now-nav_at_;
    }
    if (!nav.is_object() || !pose.is_object() || !plan.is_object() ||
        !plan.value("valid",false) || !(pose.value("flags",0U)&1U)) return false;
    const auto protocol=nav.value("protocol",std::string{});
    const bool jetson=protocol == "jetson_nav_v1";
    const bool grid=protocol == "nav_grid_v1" || jetson;
    if (grid != plan.value("grid_direction_preview",false)) return false;
    const int status = nav.value("status",0);
    if (status != 1 && status != 2 && status != 3 && status != 5) return false;
    if (plan.value("boot_id",0U) != nav.value("boot_id",0U) ||
        plan.value("odom_domain",0U) != nav.value("odom_domain",0U) ||
        plan.value("task_id",0U) != nav.value("task_id",0U)) return false;
    const auto age = static_cast<std::int32_t>(nav.value("now_us",0ULL) -
                                             plan.value("plan_stamp_us",0U));
    const auto pose_delta = static_cast<std::int32_t>(pose.value("pose_stamp_us",0U) -
                                                    plan.value("plan_stamp_us",0U));
    if (age < 0 || age+nav_elapsed*1e6 >= 500000 || std::abs(static_cast<double>(pose_delta)) >= 500000)
      return false;
    const auto& points = plan.at("points");
    if (!points.is_array() || points.size()!=2) return false;
    const auto& state = pose.at("state");
    const double px=state.at("x_m"), py=state.at("y_m"), yaw=state.at("yaw_rad");
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(yaw)) return false;
    std::optional<cv::Point> previous;
    std::vector<std::pair<cv::Point,cv::Point>> lines;
    // Subdivision preserves lens distortion and clips the part below/behind the camera.
    for (int i=0;i<=64;++i) {
      const double a=i/64.0;
      const double x=(1-a)*points[0].at("x").get<double>()+a*points[1].at("x").get<double>();
      const double y=(1-a)*points[0].at("y").get<double>()+a*points[1].at("y").get<double>();
      const double dx=x-px, dy=y-py;
      const auto pixel=project(std::cos(yaw)*dx+std::sin(yaw)*dy,
                               -std::sin(yaw)*dx+std::cos(yaw)*dy);
      if (pixel && previous) {
        auto start=*previous, end=*pixel;
        if (cv::clipLine(image.size(),start,end)) {
          lines.emplace_back(start,end);
        }
      }
      previous=pixel;
    }
    for (const auto& line:lines)
      cv::line(image,line.first,line.second,cv::Scalar(30,38,45),7,cv::LINE_AA);
    for (const auto& line:lines)
      cv::line(image,line.first,line.second,cv::Scalar(30,170,255),3,cv::LINE_AA);
    if (!lines.empty()) cv::putText(image,jetson ? "JETSON DIRECTION / PREVIEW" : grid ? "FPGA DIRECTION / PREVIEW" : "FPGA PLAN / APPROX.",cv::Point(16,65),
        cv::FONT_HERSHEY_SIMPLEX,.45,cv::Scalar(30,170,255),1,cv::LINE_AA);
    return !lines.empty();
  }

 private:
  std::optional<cv::Point> project(double x, double y) const {
    const double cx=t_[0]*x+t_[1]*y+t_[3], cy=t_[4]*x+t_[5]*y+t_[7];
    const double z=t_[8]*x+t_[9]*y+t_[11];
    if (z<=.05) return std::nullopt;
    const double u=cx/z,v=cy/z,r2=u*u+v*v;
    const double radial=1+d_[0]*r2+d_[1]*r2*r2+d_[4]*r2*r2*r2;
    const double xd=u*radial+2*d_[2]*u*v+d_[3]*(r2+2*u*u);
    const double yd=v*radial+d_[2]*(r2+2*v*v)+2*d_[3]*u*v;
    const double ix=k_[0]*xd+k_[1]*yd+k_[2], iy=k_[4]*yd+k_[5];
    if (!std::isfinite(ix) || !std::isfinite(iy) || std::abs(ix)>1e6 || std::abs(iy)>1e6)
      return std::nullopt;
    return cv::Point(cvRound(ix),cvRound(iy));
  }
  bool enabled_=false;
  std::array<double,9> k_{};
  std::vector<double> d_,t_;
  std::mutex mutex_;
  nlohmann::json nav_,pose_,plan_;
  double nav_at_=-1e100,pose_at_=-1e100,plan_at_=-1e100;
};
}  // namespace bx72
