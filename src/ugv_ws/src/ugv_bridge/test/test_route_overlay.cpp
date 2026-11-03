#include <gtest/gtest.h>
#include "ugv_bridge/route_overlay.hpp"

using Json=nlohmann::json;
namespace {
const std::array<double,9> k{420,0,320,0,420,180,0,0,1};
const std::vector<double> transform{0,-1,0,0, 0,0,-1,.24, 1,0,0,0};
Json nav() { return {{"type","navigation"},{"boot_id",7},{"odom_domain",1},
  {"task_id",5},{"now_us",1000000},{"status",3}}; }
Json pose() { return {{"type","fusion"},{"flags",1},{"pose_stamp_us",1000000},
  {"state",{{"x_m",0},{"y_m",0},{"yaw_rad",0}}}}; }
Json plan() { return {{"type","local_plan"},{"boot_id",7},{"odom_domain",1},
  {"task_id",5},{"valid",true},{"plan_stamp_us",1000000},
  {"points",Json::array({{{"x",0},{"y",0}},{{"x",.9},{"y",.1}}})}}; }
void seed(bx72::RouteOverlay& overlay) {
  overlay.configure(k,{0,0,0,0,0},transform);
  overlay.observe(nav(),10); overlay.observe(pose(),10); overlay.observe(plan(),10);
}
cv::Mat image() {return cv::Mat::zeros(360,640,CV_8UC3);}
}

TEST(RouteOverlay, DrawsOnlyReportedSegmentInFrontOfCamera) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  ASSERT_TRUE(overlay.draw(frame,10.05,10.02));
  // Reported endpoint projects to (273,292), not an invented full-goal path.
  EXPECT_GT(frame.at<cv::Vec3b>(292,273)[2],150);
  EXPECT_EQ(frame.at<cv::Vec3b>(180,320),cv::Vec3b(0,0,0));
}
TEST(RouteOverlay, DisabledAndMissingDataLeaveVideoUntouched) {
  bx72::RouteOverlay overlay; auto frame=image();
  EXPECT_FALSE(overlay.draw(frame,10,10));
  overlay.configure(k,{0,0,0,0,0},transform);
  EXPECT_FALSE(overlay.draw(frame,10,10));
  EXPECT_EQ(cv::countNonZero(frame.reshape(1)),0);
}
TEST(RouteOverlay, RejectsOldVideoPoseAndSourcePlan) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  EXPECT_FALSE(overlay.draw(frame,10.1,9.8));
  EXPECT_FALSE(overlay.draw(frame,10.3,10.3));
  auto old=plan(); old["plan_stamp_us"]=400000; overlay.observe(old,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
}
TEST(RouteOverlay, GridDirectionRequiresMatchingGridMarker) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  auto n=nav();n["protocol"]="nav_grid_v1";overlay.observe(n,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
  auto p=plan();p["grid_direction_preview"]=true;overlay.observe(p,10);
  EXPECT_TRUE(overlay.draw(frame,10,10));
  n["protocol"]="nav_polar_v2";overlay.observe(n,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
}
TEST(RouteOverlay, JetsonDirectionUsesTheExistingProjection) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  auto n=nav(); n["protocol"]="jetson_nav_v1"; overlay.observe(n,10);
  auto p=plan(); p["protocol"]="jetson_local_segment_v1";
  p["grid_direction_preview"]=true; overlay.observe(p,10);
  EXPECT_TRUE(overlay.draw(frame,10,10));
}
TEST(RouteOverlay, ClearsOnTerminalTaskDomainChangeAndMismatch) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  auto n=nav(); n["status"]=4; overlay.observe(n,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
  n=nav(); n["odom_domain"]=2; overlay.observe(n,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
  seed(overlay); auto p=plan(); p["task_id"]=6; overlay.observe(p,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
  seed(overlay); overlay.clear(); EXPECT_FALSE(overlay.draw(frame,10,10));
}
TEST(RouteOverlay, BehindCameraAndInvalidPoseDoNotDraw) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  auto p=plan(); p["points"][1]["x"]=-.9; overlay.observe(p,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
  overlay.observe(plan(),10); auto s=pose(); s["flags"]=0; overlay.observe(s,10);
  EXPECT_FALSE(overlay.draw(frame,10,10));
}
TEST(RouteOverlay, RotationAndTranslationAreApplied) {
  bx72::RouteOverlay overlay; seed(overlay); auto frame=image();
  auto s=pose(); s["state"]={{"x_m",2},{"y_m",3},{"yaw_rad",std::acos(-1)/2}};
  overlay.observe(s,10);
  auto p=plan(); p["points"]=Json::array({{{"x",2},{"y",3}},{{"x",1.9},{"y",3.9}}});
  overlay.observe(p,10); EXPECT_TRUE(overlay.draw(frame,10,10));
  EXPECT_GT(frame.at<cv::Vec3b>(292,273)[2],150);
}
TEST(RouteOverlay, RejectsInvalidCalibrationTransform) {
  bx72::RouteOverlay overlay;
  EXPECT_THROW(overlay.configure(k,{0,0,0,0,0},{}),std::invalid_argument);
  auto bad=transform; bad[0]=2;
  EXPECT_THROW(overlay.configure(k,{0,0,0,0,0},bad),std::invalid_argument);
}
