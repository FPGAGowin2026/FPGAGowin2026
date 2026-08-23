#include "vision_bridge/detector.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace vision_bridge {

class RedTargetNode : public rclcpp::Node {
 public:
  RedTargetNode()
  : Node("red_target_node")
  {
    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    const auto observation_topic = declare_parameter<std::string>(
      "observation_topic", "/vision/target_observation");
    min_red_ = declare_parameter<int>("min_red", 120);
    red_ratio_ = declare_parameter<double>("red_ratio", 1.5);
    min_fraction_ = declare_parameter<double>("min_area_fraction", 0.002);
    const auto sample_stride = declare_parameter<int64_t>("sample_stride", 2);
    if (min_red_ < 0 || min_red_ > 255 || red_ratio_ <= 1.0 ||
      min_fraction_ <= 0.0 || min_fraction_ >= 1.0 || sample_stride <= 0)
    {
      throw std::invalid_argument("红色目标阈值参数无效");
    }
    sample_stride_ = static_cast<std::size_t>(sample_stride);

    publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(observation_topic, 10);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&RedTargetNode::on_image, this, std::placeholders::_1));
  }

 private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr image)
  {
    const auto observation = detect_red_target(
      image->data, image->width, image->height, image->step, image->encoding,
      min_red_, red_ratio_, min_fraction_, sample_stride_);
    if (!observation) {
      return;
    }

    geometry_msgs::msg::Vector3Stamped message;
    message.header = image->header;
    // x/y 为图像中心归一化偏差，z 为目标像素占比；消息新鲜度即“目标可见”。
    message.vector.x = observation->horizontal_error;
    message.vector.y = observation->vertical_error;
    message.vector.z = observation->area_fraction;
    publisher_->publish(message);
  }

  int min_red_{};
  double red_ratio_{};
  double min_fraction_{};
  std::size_t sample_stride_{};
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

}  // namespace vision_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vision_bridge::RedTargetNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("red_target_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
