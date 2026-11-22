#include "udp_input.hpp"
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

using ugv_bridge::u16;
using ugv_bridge::u32;

std::int64_t steady_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::uint8_t clip_709(int value)
{
  if (value <= 0) return 0;
  if (value >= 255 * 256) return 255;
  return static_cast<std::uint8_t>((value + 128) >> 8);
}

double parse_finite_number(const std::string &text)
{
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) throw std::invalid_argument("invalid number");
  return value;
}

std::string json_quote(const std::string &value)
{
  static const char *hex = "0123456789abcdef";
  std::string result = "\"";
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') {
      result += '\\';
      result += static_cast<char>(c);
    } else if (c < 0x20) {
      result += "\\u00";
      result += hex[c >> 4];
      result += hex[c & 0x0f];
    } else {
      result += static_cast<char>(c);
    }
  }
  result += '"';
  return result;
}

void yuyv_709_to_bgr(const std::vector<std::uint8_t> &input, cv::Mat &output)
{
  const int width = output.cols;
  for (int row = 0; row < output.rows; ++row) {
    const auto *src = input.data() + static_cast<std::size_t>(row) * width * 2;
    auto *dst = output.ptr<std::uint8_t>(row);
    for (int x = 0; x < width; x += 2, src += 4, dst += 6) {
      const int y0 = (static_cast<int>(src[0]) - 16) * 298;
      const int y1 = (static_cast<int>(src[2]) - 16) * 298;
      const int cb = static_cast<int>(src[1]) - 128;
      const int cr = static_cast<int>(src[3]) - 128;
      const int blue = 541 * cb;
      const int green = -55 * cb - 136 * cr;
      const int red = 459 * cr;
      dst[0] = clip_709(y0 + blue);
      dst[1] = clip_709(y0 + green);
      dst[2] = clip_709(y0 + red);
      dst[3] = clip_709(y1 + blue);
      dst[4] = clip_709(y1 + green);
      dst[5] = clip_709(y1 + red);
    }
  }
}

struct RawFrame {
  std::vector<std::uint8_t> data;
  rclcpp::Time received_at;
  std::int64_t received_ms = 0;
  std::once_flag conversion;
  sensor_msgs::msg::Image bgr;
};

bool has_subscribers(const rclcpp::PublisherBase &publisher)
{
  return publisher.get_intra_process_subscription_count() || publisher.get_subscription_count();
}

class VideoNode : public rclcpp::Node {
public:
  explicit VideoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("robot_video", options)
  {
    const auto bind_address = declare_parameter<std::string>("bind_address", "0.0.0.0");
    const auto source_address = declare_parameter<std::string>("source_address", "");
    const auto port = declare_parameter<int>("video_port", 6102);
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 360);
    const auto timeout_ms = declare_parameter<int>("frame_timeout_ms", 250);
    const auto receive_buffer = declare_parameter<int>("receive_buffer_bytes", 4 * 1024 * 1024);
    const auto http_port = declare_parameter<int>("http_port", 8081);
    jpeg_fps_ = declare_parameter<double>("jpeg_fps", 10.0);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 60);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera");

    if (port < 1 || port > 65535 || width_ <= 0 || height_ <= 0 ||
        width_ > 4096 || height_ > 4096 || width_ % 2 ||
        timeout_ms < 1 || receive_buffer < 1 || http_port < 0 || http_port > 65535 ||
        !std::isfinite(jpeg_fps_) || jpeg_fps_ <= 0.0 ||
        jpeg_fps_ > 60.0 || jpeg_quality_ < 1 || jpeg_quality_ > 100) {
      throw std::invalid_argument("invalid video parameters");
    }
    expected_frame_bytes_ = static_cast<std::size_t>(width_) * height_ * 2;
    timeout_ = std::chrono::milliseconds(timeout_ms);

    const auto bind_addr = ugv_bridge::ipv4_address(bind_address, "bind_address");
    const auto video_port = static_cast<std::uint16_t>(port);
    source_addr_ = ugv_bridge::udp_source(source_address);
    socket_ = open_video_socket(bind_addr, video_port, receive_buffer);

    auto sensor_qos = rclcpp::SensorDataQoS();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", sensor_qos);
    jpeg_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", sensor_qos);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>("camera/video_diagnostics", 10);
    health_sub_ = create_subscription<std_msgs::msg::String>(
      "fpga/video_health", 10, [this](const std_msgs::msg::String &message) {
        if (message.data.size() > 4096 || message.data.size() < 2 ||
            message.data.front() != '{' || message.data.back() != '}') return;
        std::lock_guard<std::mutex> lock(health_mutex_);
        fpga_health_ = message.data;
        fpga_health_ms_ = steady_ms();
      });
    // Humble intra-process subscriptions do not support transient-local durability.
    rclcpp::SubscriptionOptions map_options;
    map_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) { update_map(std::move(map), false); },
      map_options);
    local_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "fpga/local_map", 1,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) { update_map(std::move(map), true); });
    mission_sub_ = create_subscription<std_msgs::msg::String>(
      "robot/mission/status", 10, [this](const std_msgs::msg::String &message) {
        if (message.data.size() > 65536 || message.data.size() < 2 ||
            message.data.front() != '{' || message.data.back() != '}') return;
        std::lock_guard<std::mutex> lock(mission_mutex_);
        mission_status_ = message.data;
        mission_status_ms_ = steady_ms();
      });
    vision_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "vision/detections", 10, [this](const vision_msgs::msg::Detection2DArray &message) {
        std::ostringstream boxes;
        boxes << '[';
        std::size_t count = 0;
        for (const auto &detection : message.detections) {
          if (count >= 16) break;
          if (detection.results.empty()) continue;
          const auto &box = detection.bbox;
          const auto &result = detection.results.front().hypothesis;
          if (!std::isfinite(box.center.position.x) || !std::isfinite(box.center.position.y) ||
              !std::isfinite(box.size_x) || !std::isfinite(box.size_y) ||
              !std::isfinite(result.score) || box.size_x <= 0 || box.size_y <= 0 ||
              result.class_id.size() > 64 || detection.id.size() > 128) continue;
          if (count++) boxes << ',';
          boxes << "{\"x\":" << box.center.position.x << ",\"y\":" << box.center.position.y
                << ",\"w\":" << box.size_x << ",\"h\":" << box.size_y
                << ",\"class\":" << json_quote(result.class_id)
                << ",\"score\":" << result.score << ",\"id\":" << json_quote(detection.id) << '}';
        }
        boxes << ']';
        std::lock_guard<std::mutex> lock(vision_mutex_);
        vision_boxes_ = boxes.str();
        vision_ms_ = steady_ms();
      });
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("robot/mission/goal", 10);
    waypoints_pub_ = create_publisher<nav_msgs::msg::Path>("robot/mission/waypoints", 10);
    explore_pub_ = create_publisher<std_msgs::msg::Empty>("robot/mission/explore", 10);
    stop_pub_ = create_publisher<std_msgs::msg::Empty>("robot/mission/stop", 10);
    direct_stop_pub_ = create_publisher<std_msgs::msg::Empty>("fpga/stop", 10);
    manual_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    diagnostics_timer_ = create_wall_timer(1s, [this] { publish_diagnostics(); });

    if (http_port) setup_http(static_cast<int>(http_port));
    receiver_ = std::thread([this, bind_addr, video_port, receive_buffer] {
      receive_loop(bind_addr, video_port, receive_buffer);
    });
    converter_ = std::thread([this] { convert_loop(); });
    encoder_ = std::thread([this] { encode_loop(); });
    if (http_socket_ >= 0) http_server_ = std::thread([this] { http_loop(); });
    RCLCPP_INFO(get_logger(),
      "Receiving BX72 YUYV RTP on %s:%d, %dx%d, UDP buffer reported as %d bytes",
      bind_address.c_str(), static_cast<int>(port), width_, height_, receive_buffer_actual_.load());
  }

  ~VideoNode() override
  {
    {
      std::lock_guard<std::mutex> lock(raw_mutex_);
      running_ = false;
    }
    raw_ready_.notify_all();
    jpeg_ready_.notify_all();
    if (receiver_.joinable()) receiver_.join();
    if (converter_.joinable()) converter_.join();
    if (encoder_.joinable()) encoder_.join();
    if (http_server_.joinable()) http_server_.join();
    if (socket_ >= 0) close(socket_);
    if (http_socket_ >= 0) close(http_socket_);
  }

private:
  int open_video_socket(in_addr address, std::uint16_t port, int receive_buffer)
  {
    const int fd = ugv_bridge::open_udp_input(address, port, receive_buffer,
      ugv_bridge::ReceiveBuffer::required, "video");
    int actual_buffer = 0;
    socklen_t opt_size = sizeof(actual_buffer);
    receive_buffer_actual_ = getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &actual_buffer, &opt_size) == 0 ?
      actual_buffer : 0;
    return fd;
  }

  void receive_loop(in_addr address, std::uint16_t port, int receive_buffer)
  {
    std::array<std::uint8_t, 65536> datagram{};
    std::vector<std::uint8_t> frame;
    frame.reserve(expected_frame_bytes_);
    bool active = false;
    bool bad = false;
    bool have_sequence = false;
    std::uint16_t next_sequence = 0;
    std::uint32_t timestamp = 0;
    auto started = std::chrono::steady_clock::now();
    const auto discard_frame = [&] {
      if (active) ++incomplete_frames_;
      active = bad = have_sequence = false;
      frame.clear();
    };
    const auto invalidate_socket = [&] {
      ++receive_errors_;
      discard_frame();
      close(socket_);
      socket_ = -1;
      receiver_running_ = false;
    };

    while (running_) {
      if (socket_ < 0) {
        std::this_thread::sleep_for(250ms);
        if (!running_) break;
        try {
          socket_ = open_video_socket(address, port, receive_buffer);
          receiver_running_ = true;
          RCLCPP_INFO(get_logger(), "Reopened FPGA video on UDP %u", static_cast<unsigned>(port));
        } catch (const std::system_error &error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Reopening FPGA video: %s", error.what());
          continue;
        }
      }
      if (active && std::chrono::steady_clock::now() - started >= timeout_) discard_frame();
      pollfd poll_fd{socket_, POLLIN, 0};
      const int result = poll(&poll_fd, 1, 50);
      if (result < 0 && errno == EINTR) continue;
      if (const int error = result > 0 ? ugv_bridge::clear_udp_async_error(poll_fd) : 0)
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Video UDP error: %s", std::strerror(error));
      if (result < 0 || (poll_fd.revents & ugv_bridge::UDP_ERROR_EVENTS)) {
        if (result < 0)
          RCLCPP_ERROR(get_logger(), "Video poll failed: %s", std::strerror(errno));
        else
          RCLCPP_ERROR(get_logger(), "Video poll revents=0x%x, reopening socket",
            static_cast<unsigned>(poll_fd.revents));
        invalidate_socket();
        continue;
      }
      if (!(poll_fd.revents & POLLIN)) continue;

      sockaddr_in peer{};
      socklen_t peer_len = sizeof(peer);
      const auto size = recvfrom(socket_, datagram.data(), datagram.size(), MSG_DONTWAIT,
        reinterpret_cast<sockaddr *>(&peer), &peer_len);
      if (size < 0) {
        if (ugv_bridge::retry_udp_receive(errno)) continue;
        RCLCPP_ERROR(get_logger(), "Video receive failed: %s", std::strerror(errno));
        invalidate_socket();
        continue;
      }
      ++packets_;
      const auto n = static_cast<std::size_t>(size);
      if (source_addr_ && peer.sin_addr.s_addr != source_addr_->s_addr) {
        ++invalid_packets_;
        continue;
      }
      // The FPGA also sends a 32-byte status datagram on the video port.
      if (n == 32 && u32(datagram.data()) == 0x41434736 && datagram[4] == 1 &&
          (datagram[5] == 1 || datagram[5] == 2) &&
          std::memcmp(datagram.data() + 8, "UDP-TEST", 8) == 0) {
        {
          std::lock_guard<std::mutex> lock(rx_status_mutex_);
          rx_status_mode_ = datagram[5];
          for (std::size_t i = 0; i < rx_status_words_.size(); ++i)
            rx_status_words_[i] = u16(datagram.data() + 16 + i * 2);
          rx_status_ms_ = steady_ms();
        }
        ++status_packets_;
        continue;
      }
      if (n < 16 || n > 1500 || datagram[0] != 0x80 ||
          (datagram[1] & 0x7f) != 97 || u32(datagram.data() + 8) != 0x41434736 ||
          (n - 12) % 4 != 0) {
        ++invalid_packets_;
        continue;
      }

      const auto sequence = u16(datagram.data() + 2);
      const auto incoming_timestamp = u32(datagram.data() + 4);
      const bool gap = have_sequence && sequence != next_sequence;
      if (gap) ++sequence_gaps_;

      if (!active || incoming_timestamp != timestamp) {
        if (active) ++incomplete_frames_;
        active = true;
        bad = false;
        frame.clear();
        timestamp = incoming_timestamp;
        started = std::chrono::steady_clock::now();
      } else if (gap) {
        bad = true;
      }
      have_sequence = true;
      next_sequence = static_cast<std::uint16_t>(sequence + 1);

      if (!bad) {
        if (frame.size() + n - 12 > expected_frame_bytes_) {
          bad = true;
        } else {
          frame.insert(frame.end(), datagram.begin() + 12, datagram.begin() + n);
        }
      }
      if (datagram[1] & 0x80) {
        if (!bad && frame.size() == expected_frame_bytes_) {
          auto completed = std::make_shared<RawFrame>();
          completed->data = std::move(frame);
          completed->received_at = now();
          completed->received_ms = steady_ms();
          {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            latest_raw_ = std::move(completed);
            ++raw_generation_;
          }
          if (++complete_frames_ == 1)
            RCLCPP_INFO(get_logger(), "Received the first complete video frame");
          last_complete_ms_ = steady_ms();
          last_rtp_timestamp_ = timestamp;
          raw_ready_.notify_all();
          frame = {};
          frame.reserve(expected_frame_bytes_);
        } else {
          ++incomplete_frames_;
        }
        active = false;
        frame.clear();
      }
    }
    receiver_running_ = false;
  }

  bool convert_frame(RawFrame &frame)
  {
    try {
      // The ROS publisher and the JPEG encoder both read these pixels after conversion.
      std::call_once(frame.conversion, [&] {
        auto &image = frame.bgr;
        image.header.stamp = frame.received_at;
        image.header.frame_id = frame_id_;
        image.height = static_cast<std::uint32_t>(height_);
        image.width = static_cast<std::uint32_t>(width_);
        image.encoding = "bgr8";
        image.step = image.width * 3;
        image.data.resize(static_cast<std::size_t>(image.step) * image.height);
        cv::Mat bgr(height_, width_, CV_8UC3, image.data.data(), image.step);
        yuyv_709_to_bgr(frame.data, bgr);
        if (++converted_frames_ == 1)
          RCLCPP_INFO(get_logger(), "Converted the first complete video frame");
      });
      return true;
    } catch (const std::exception &error) {
      ++conversion_errors_;
      RCLCPP_ERROR(get_logger(), "Video conversion failed: %s", error.what());
      return false;
    }
  }

  void convert_loop()
  {
    std::uint64_t seen = 0;
    while (running_) {
      std::shared_ptr<RawFrame> frame;
      {
        std::unique_lock<std::mutex> lock(raw_mutex_);
        raw_ready_.wait(lock, [&] { return !running_ || raw_generation_ != seen; });
        if (!running_) break;
        frame = latest_raw_;
        seen = raw_generation_;
      }
      try {
        if (has_subscribers(*image_pub_) && convert_frame(*frame))
          image_pub_->publish(frame->bgr);
      } catch (const std::exception &error) {
        ++conversion_errors_;
        RCLCPP_ERROR(get_logger(), "Raw image publication failed: %s", error.what());
      }
    }
  }

  void encode_loop()
  {
    std::uint64_t seen = 0;
    auto next_encode = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / jpeg_fps_));
    while (running_) {
      std::shared_ptr<RawFrame> frame;
      {
        std::unique_lock<std::mutex> lock(raw_mutex_);
        raw_ready_.wait(lock, [&] { return !running_ || raw_generation_ != seen; });
        if (!running_) break;
        raw_ready_.wait_until(lock, next_encode, [&] { return !running_; });
        if (!running_) break;
        frame = latest_raw_;
        seen = raw_generation_;
      }
      try {
        const bool publish = has_subscribers(*jpeg_pub_);
        if (!http_running_ && !publish) continue;
        next_encode = std::chrono::steady_clock::now() + period;
        if (!convert_frame(*frame)) continue;
        const cv::Mat bgr(height_, width_, CV_8UC3, frame->bgr.data.data(), frame->bgr.step);
        sensor_msgs::msg::CompressedImage image;
        image.header = frame->bgr.header;
        image.format = "jpeg";
        if (cv::imencode(".jpg", bgr, image.data,
            {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
          if (publish) jpeg_pub_->publish(image);
          {
            std::lock_guard<std::mutex> lock(jpeg_mutex_);
            latest_jpeg_ = std::make_shared<std::vector<std::uint8_t>>(std::move(image.data));
            latest_jpeg_ms_ = frame->received_ms;
            ++jpeg_generation_;
          }
          jpeg_ready_.notify_all();
          if (++encoded_frames_ == 1)
            RCLCPP_INFO(get_logger(), "Published the first JPEG video frame");
        } else {
          ++encode_errors_;
        }
      } catch (const std::exception &error) {
        ++encode_errors_;
        RCLCPP_ERROR(get_logger(), "JPEG encoding failed: %s", error.what());
      }
    }
  }

  static bool send_all(int fd, const void *buffer, std::size_t size)
  {
    const auto *data = static_cast<const std::uint8_t *>(buffer);
    while (size) {
      const auto sent = send(fd, data, size, MSG_NOSIGNAL);
      if (sent < 0 && errno == EINTR) continue;
      if (sent <= 0) return false;
      data += sent;
      size -= static_cast<std::size_t>(sent);
    }
    return true;
  }

  static bool send_response(int fd, const char *status, const char *content_type,
    const std::string &body, const std::string &extra_headers = "")
  {
    std::ostringstream header;
    header << "HTTP/1.1 " << status << "\r\nContent-Type: " << content_type
           << "\r\n" << extra_headers << "Content-Length: " << body.size()
           << "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
    const auto text = header.str();
    return send_all(fd, text.data(), text.size()) && send_all(fd, body.data(), body.size());
  }

  void update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr map, bool local)
  {
    const auto &info = map->info;
    const auto width = static_cast<std::size_t>(info.width);
    const auto height = static_cast<std::size_t>(info.height);
    if (!width || !height || width > 8192 || height > 8192 ||
        width * height > 16 * 1024 * 1024 || map->data.size() != width * height) return;
    if (!local) {
      std::lock_guard<std::mutex> lock(map_mutex_);
      const auto &previous = latest_map_grid_;
      if (previous && previous->info.width == info.width && previous->info.height == info.height &&
          previous->info.resolution == info.resolution && previous->info.origin == info.origin &&
          previous->data == map->data) {
        latest_map_ms_ = steady_ms();
        return;
      }
    }
    try {
      cv::Mat pixels(static_cast<int>(height), static_cast<int>(width), CV_8UC1);
      for (std::size_t y = 0; y < height; ++y) {
        auto *row = pixels.ptr<std::uint8_t>(static_cast<int>(height - 1 - y));
        for (std::size_t x = 0; x < width; ++x) {
          const int value = map->data[y * width + x];
          row[x] = value < 0 ? 127 : static_cast<std::uint8_t>(255 - std::min(value, 100) * 255 / 100);
        }
      }
      auto png = std::make_shared<std::vector<std::uint8_t>>();
      if (!cv::imencode(".png", pixels, *png)) return;
      if (local) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_local_map_ = std::move(png);
        latest_local_map_ms_ = steady_ms();
        return;
      }
      const auto &origin = info.origin;
      const auto &q = origin.orientation;
      const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      std::ostringstream geometry;
      geometry << std::setprecision(17) << width << ',' << height << ','
               << info.resolution << ',' << origin.position.x << ','
               << origin.position.y << ',' << yaw;
      std::string header = geometry.str();
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_map_ = std::move(png);
      latest_map_grid_ = std::move(map);
      latest_map_geometry_ = std::move(header);
      latest_map_ms_ = steady_ms();
    } catch (const cv::Exception &error) {
      RCLCPP_WARN(get_logger(), "Cannot encode map preview: %s", error.what());
    }
  }

  void setup_http(int port)
  {
    http_socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (http_socket_ < 0) {
      RCLCPP_WARN(get_logger(), "Video HTTP socket unavailable: %s", std::strerror(errno));
      return;
    }
    int reuse = 1;
    setsockopt(http_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (bind(http_socket_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0 ||
        listen(http_socket_, 8) < 0) {
      RCLCPP_WARN(get_logger(), "Video HTTP port %d unavailable: %s", port, std::strerror(errno));
      close(http_socket_);
      http_socket_ = -1;
      return;
    }
    http_running_ = true;
    RCLCPP_INFO(get_logger(), "Video preview available on HTTP port %d", port);
  }

  std::string diagnostics_json()
  {
    const auto complete_ms = last_complete_ms_.load();
    const auto age_ms = complete_ms ? steady_ms() - complete_ms : -1;
    std::int64_t jpeg_age = -1;
    std::int64_t health_age = -1;
    std::int64_t map_age = -1;
    std::int64_t local_map_age = -1;
    std::int64_t vision_age = -1;
    std::string vision = "[]";
    std::string health = "null";
    std::array<std::uint16_t, 8> rx_words{};
    std::uint8_t rx_mode = 0;
    std::int64_t rx_received_ms = 0;
    {
      std::lock_guard<std::mutex> lock(rx_status_mutex_);
      rx_words = rx_status_words_;
      rx_mode = rx_status_mode_;
      rx_received_ms = rx_status_ms_;
    }
    {
      std::lock_guard<std::mutex> lock(jpeg_mutex_);
      if (latest_jpeg_ms_) jpeg_age = steady_ms() - latest_jpeg_ms_;
    }
    {
      std::lock_guard<std::mutex> lock(health_mutex_);
      if (fpga_health_ms_) {
        health_age = steady_ms() - fpga_health_ms_;
        health = json_quote(fpga_health_);
      }
    }
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (latest_map_ms_) map_age = steady_ms() - latest_map_ms_;
      if (latest_local_map_ms_) local_map_age = steady_ms() - latest_local_map_ms_;
    }
    {
      std::lock_guard<std::mutex> lock(vision_mutex_);
      if (vision_ms_) {
        vision_age = steady_ms() - vision_ms_;
        vision = vision_boxes_;
      }
    }
    std::ostringstream data;
    data << "{\"video_instance\":" << json_quote(video_instance_)
         << ",\"receiver_running\":" << (receiver_running_.load() ? "true" : "false")
         << ",\"http_running\":" << (http_running_.load() ? "true" : "false")
         << ",\"http_clients\":" << http_clients_.load()
         << ",\"packets\":" << packets_.load()
         << ",\"status_packets\":" << status_packets_.load()
         << ",\"fpga_rx_status\":";
    if (!rx_mode) {
      data << "null";
    } else {
      data << "{\"mode\":" << json_quote(rx_mode == 1 ? "ready" : "training")
           << ",\"mode_code\":" << static_cast<unsigned>(rx_mode)
           << ",\"age_ms\":" << steady_ms() - rx_received_ms << ",\"words\":[";
      for (std::size_t i = 0; i < rx_words.size(); ++i) {
        if (i) data << ',';
        data << rx_words[i];
      }
      data << "],\"counters\":";
      // Training reuses these words for a raw prefix, phase mask and tap.
      if (rx_mode == 1) {
        static const char *names[] = {
          "clock", "frame", "sfd", "payload", "done", "commit", "rx_er", "network_error"};
        data << '{';
        for (std::size_t i = 0; i < rx_words.size(); ++i) {
          if (i) data << ',';
          data << json_quote(names[i]) << ':' << rx_words[i];
        }
        data << '}';
      } else {
        data << "null";
      }
      data << '}';
    }
    data << ",\"invalid_packets\":" << invalid_packets_.load()
         << ",\"sequence_gaps\":" << sequence_gaps_.load()
         << ",\"incomplete_frames\":" << incomplete_frames_.load()
         << ",\"complete_frames\":" << complete_frames_.load()
         << ",\"converted_frames\":" << converted_frames_.load()
         << ",\"encoded_frames\":" << encoded_frames_.load()
         << ",\"receive_errors\":" << receive_errors_.load()
         << ",\"conversion_errors\":" << conversion_errors_.load()
         << ",\"encode_errors\":" << encode_errors_.load()
         << ",\"last_frame_age_ms\":" << age_ms
         << ",\"last_jpeg_age_ms\":" << jpeg_age
         << ",\"fpga_health_age_ms\":" << health_age
         << ",\"fpga_health_json\":" << health
         << ",\"map_age_ms\":" << map_age
         << ",\"local_map_age_ms\":" << local_map_age
         << ",\"vision_age_ms\":" << vision_age << ",\"detections\":" << vision
         << ",\"last_rtp_timestamp\":" << last_rtp_timestamp_.load()
         << ",\"receive_buffer_actual\":" << receive_buffer_actual_.load() << "}";
    return data.str();
  }

  void publish_diagnostics()
  {
    std_msgs::msg::String msg;
    msg.data = diagnostics_json();
    diagnostics_pub_->publish(msg);
  }

  void http_client(int fd)
  {
    const timeval timeout{1, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    std::string request;
    std::array<char, 1024> buffer{};
    while (request.find("\r\n") == std::string::npos && request.size() < 4096) {
      const auto received = recv(fd, buffer.data(), buffer.size(), 0);
      if (received <= 0) return;
      request.append(buffer.data(), static_cast<std::size_t>(received));
    }
    if (request.rfind("POST /stop ", 0) == 0) {
      direct_stop_pub_->publish(std_msgs::msg::Empty{});
      stop_pub_->publish(std_msgs::msg::Empty{});
      send_response(fd, "200 OK", "text/plain; charset=utf-8", "已请求停车\n");
    } else if (request.rfind("POST /explore ", 0) == 0 || request.rfind("POST /goal?x=", 0) == 0 ||
               request.rfind("POST /goal?waypoints=", 0) == 0 ||
               request.rfind("POST /manual?x=", 0) == 0) {
      const bool manual = request.rfind("POST /manual?x=", 0) == 0;
      bool ready;
      {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        ready = mission_status_ms_ && steady_ms() - mission_status_ms_ < 2000;
      }
      if (!ready && !manual) {
        send_response(fd, "503 Service Unavailable", "text/plain; charset=utf-8", "任务节点未连接\n");
      } else if (request.rfind("POST /explore ", 0) == 0) {
        explore_pub_->publish(std_msgs::msg::Empty{});
        send_response(fd, "200 OK", "text/plain; charset=utf-8", "已请求探索\n");
      } else if (request.rfind("POST /goal?waypoints=", 0) == 0) {
        try {
          const auto begin = std::string("POST /goal?waypoints=").size();
          const auto angle = request.find("&yaw=", begin);
          const auto end = angle == std::string::npos ? angle : request.find(' ', angle + 5);
          if (angle == std::string::npos || end == std::string::npos)
            throw std::invalid_argument("invalid waypoints");
          const double yaw = parse_finite_number(request.substr(angle + 5, end - angle - 5));
          if (std::abs(yaw) > 3.142) throw std::invalid_argument("yaw out of range");
          const auto hex = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
          };
          std::string coordinates;
          for (auto i = begin; i < angle; ++i) {
            if (request[i] != '%') {
              coordinates += request[i];
            } else {
              if (i + 2 >= angle || hex(request[i + 1]) < 0 || hex(request[i + 2]) < 0)
                throw std::invalid_argument("invalid URL encoding");
              coordinates += static_cast<char>(hex(request[i + 1]) * 16 + hex(request[i + 2]));
              i += 2;
            }
          }
          nav_msgs::msg::Path waypoints;
          waypoints.header.frame_id = "map";
          waypoints.header.stamp = now();
          std::size_t start = 0;
          while (start < coordinates.size()) {
            const auto separator = coordinates.find(';', start);
            const auto finish = separator == std::string::npos ? coordinates.size() : separator;
            const auto comma = coordinates.find(',', start);
            if (comma == std::string::npos || comma >= finish || waypoints.poses.size() >= 32)
              throw std::invalid_argument("invalid waypoint");
            const double x = parse_finite_number(coordinates.substr(start, comma - start));
            const double y = parse_finite_number(coordinates.substr(comma + 1, finish - comma - 1));
            if (std::abs(x) > 10000 || std::abs(y) > 10000)
              throw std::invalid_argument("waypoint out of range");
            geometry_msgs::msg::PoseStamped point;
            point.header = waypoints.header;
            point.pose.position.x = x;
            point.pose.position.y = y;
            point.pose.orientation.z = std::sin(yaw / 2);
            point.pose.orientation.w = std::cos(yaw / 2);
            waypoints.poses.push_back(point);
            if (separator == std::string::npos) break;
            start = separator + 1;
            if (start == coordinates.size()) throw std::invalid_argument("empty waypoint");
          }
          if (waypoints.poses.empty()) throw std::invalid_argument("empty waypoints");
          waypoints_pub_->publish(waypoints);
          send_response(fd, "200 OK", "text/plain; charset=utf-8", "多航点已提交，执行结果见任务状态\n");
        } catch (const std::exception &) {
          send_response(fd, "400 Bad Request", "text/plain; charset=utf-8", "多航点格式无效\n");
        }
      } else {
        try {
          const std::size_t begin = manual ? 15 : 13;
          const auto middle = request.find("&y=", begin), angle = request.find("&yaw=", middle);
          const auto end = request.find(' ', angle);
          if (middle == std::string::npos || angle == std::string::npos || end == std::string::npos)
            throw std::invalid_argument("invalid goal");
          const double x = parse_finite_number(request.substr(begin, middle - begin));
          const double y = parse_finite_number(request.substr(middle + 3, angle - middle - 3));
          const double yaw = parse_finite_number(request.substr(angle + 5, end - angle - 5));
          if (manual) {
            if (std::abs(x) > 0.4 || std::abs(y) > 0.4 || std::abs(yaw) > 1.0)
              throw std::invalid_argument("manual command out of range");
            geometry_msgs::msg::Twist velocity;
            velocity.linear.x = x;
            velocity.linear.y = y;
            velocity.angular.z = yaw;
            manual_pub_->publish(velocity);
            send_response(fd, "200 OK", "text/plain; charset=utf-8", "已发送手动指令\n");
          } else {
            if (std::abs(x) > 10000 || std::abs(y) > 10000 || std::abs(yaw) > 3.142)
              throw std::invalid_argument("goal out of range");
            geometry_msgs::msg::PoseStamped goal;
            goal.header.frame_id = "map";
            goal.header.stamp = now();
            goal.pose.position.x = x;
            goal.pose.position.y = y;
            goal.pose.orientation.z = std::sin(yaw / 2);
            goal.pose.orientation.w = std::cos(yaw / 2);
            goal_pub_->publish(goal);
            send_response(fd, "200 OK", "text/plain; charset=utf-8", "航点已提交，执行结果见任务状态\n");
          }
        } catch (const std::exception &) {
          send_response(fd, "400 Bad Request", "text/plain; charset=utf-8", "航点格式无效\n");
        }
      }
    } else if (request.rfind("GET /mission.json ", 0) == 0) {
      std::string status;
      {
        std::lock_guard<std::mutex> lock(mission_mutex_);
        if (mission_status_ms_ && steady_ms() - mission_status_ms_ < 2000) status = mission_status_;
      }
      if (status.empty()) send_response(fd, "503 Service Unavailable", "application/json", "{}");
      else send_response(fd, "200 OK", "application/json", status);
    } else if (request.compare(0, 14, "GET /frame.jpg") == 0 && request.size() > 14 &&
               (request[14] == ' ' || request[14] == '?')) {
      std::shared_ptr<const std::vector<std::uint8_t>> jpeg;
      std::int64_t source_ms;
      std::uint64_t generation;
      {
        std::lock_guard<std::mutex> lock(jpeg_mutex_);
        jpeg = latest_jpeg_;
        source_ms = latest_jpeg_ms_;
        generation = jpeg_generation_;
      }
      if (!jpeg || steady_ms() - source_ms > 1000) {
        send_response(fd, "503 Service Unavailable", "text/plain", "No fresh video frame");
      } else {
        const std::string body(reinterpret_cast<const char *>(jpeg->data()), jpeg->size());
        send_response(fd, "200 OK", "image/jpeg", body,
          "X-BX72-Frame: " + video_instance_ + ":" + std::to_string(generation) + "\r\n");
      }
    } else if (request.compare(0, 15, "GET /video.mjpg") == 0 && request.size() > 15 &&
               (request[15] == ' ' || request[15] == '?')) {
      const std::string header =
        "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
      if (!send_all(fd, header.data(), header.size())) return;
      std::uint64_t seen = 0;
      while (running_) {
        std::shared_ptr<const std::vector<std::uint8_t>> jpeg;
        std::int64_t source_ms;
        {
          std::unique_lock<std::mutex> lock(jpeg_mutex_);
          jpeg_ready_.wait_for(lock, 500ms, [&] {
            return !running_ || !http_running_ || jpeg_generation_ != seen;
          });
          if (!running_ || !http_running_) break;
          if (jpeg_generation_ == seen) {
            lock.unlock();
            pollfd peer{fd, POLLIN, 0};
            if (poll(&peer, 1, 0) > 0 &&
                ((peer.revents & (POLLERR | POLLHUP | POLLNVAL)) ||
                ((peer.revents & POLLIN) && recv(fd, buffer.data(), 1, MSG_PEEK | MSG_DONTWAIT) == 0)))
              break;
            continue;
          }
          jpeg = latest_jpeg_;
          source_ms = latest_jpeg_ms_;
          seen = jpeg_generation_;
        }
        if (!jpeg || steady_ms() - source_ms > 1000) continue;
        std::ostringstream part;
        part << "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
             << jpeg->size() << "\r\n\r\n";
        const auto part_header = part.str();
        if (!send_all(fd, part_header.data(), part_header.size()) ||
            !send_all(fd, jpeg->data(), jpeg->size()) || !send_all(fd, "\r\n", 2))
          break;
      }
    } else if (request.compare(0, 18, "GET /local_map.png") == 0 && request.size() > 18 &&
               (request[18] == ' ' || request[18] == '?')) {
      std::shared_ptr<const std::vector<std::uint8_t>> map;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map = latest_local_map_;
      }
      if (map) {
        send_response(fd, "200 OK", "image/png",
          std::string(reinterpret_cast<const char *>(map->data()), map->size()));
      } else {
        send_response(fd, "503 Service Unavailable", "text/plain; charset=utf-8", "Local map not available\n");
      }
    } else if (request.compare(0, 12, "GET /map.png") == 0 && request.size() > 12 &&
               (request[12] == ' ' || request[12] == '?')) {
      std::shared_ptr<const std::vector<std::uint8_t>> map;
      std::string geometry;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map = latest_map_;
        geometry = latest_map_geometry_;
      }
      if (map) {
        send_response(fd, "200 OK", "image/png",
          std::string(reinterpret_cast<const char *>(map->data()), map->size()),
          "X-BX72-Map: " + geometry + "\r\n");
      } else {
        send_response(fd, "503 Service Unavailable", "text/plain; charset=utf-8", "Map not available\n");
      }
    } else if (request.compare(0, 17, "GET /status.json ") == 0) {
      send_response(fd, "200 OK", "application/json", diagnostics_json());
    } else if (request.compare(0, 6, "GET / ") == 0) {
      static const std::string page = R"HTML(<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><title>机器人数据预览</title>
<style>body{font:16px sans-serif;background:#111827;color:#eee;margin:2rem}img,canvas{max-width:100%;background:#222}canvas,#local_map{image-rendering:pixelated;min-width:320px}canvas{cursor:crosshair}button,input{font:inherit;margin:.25rem;padding:.4rem}input{width:6rem}pre{white-space:pre-wrap}</style>
<h1>机器人控制与数据</h1><p id="state">等待视频状态…</p><img id="video" hidden alt="实时视频">
<p id="vision">视觉识别未启用或暂无新结果</p>
<h2>全局地图</h2><p id="map_state">等待地图…</p><canvas id="map" hidden></canvas>
<p>点击地图选择航点，灰色边界外也可选；或直接输入地图坐标（米）。</p>
<label>X <input id="x" type="number" step="0.05" value="0"></label><label>Y <input id="y" type="number" step="0.05" value="0"></label><label>朝向（弧度）<input id="yaw" type="number" step="0.1" value="0"></label>
<button id="go">前往航点</button><button id="explore">自主探索</button><button id="stop">停止</button><p id="mission">等待任务状态…</p>
<h2>手动控制</h2><p>按住按钮移动，松开停车。</p>
<button data-motion="0.3,0,0">前进</button><button data-motion="-0.3,0,0">后退</button><button data-motion="0,0.3,0">左移</button><button data-motion="0,-0.3,0">右移</button><button data-motion="0,0,0.4">左转</button><button data-motion="0,0,-0.4">右转</button>
<h2>FPGA 局部地图</h2><p id="local_map_state">等待地图…</p><img id="local_map" hidden alt="局部栅格地图">
<p>白色：空闲；黑色：占据；灰色：未知。</p><pre id="detail"></pre><script>
let mission=null,pad=0;
let manualPress = null, manualTimer = null;
let manualInFlight = Promise.resolve(), manualStopPending = Promise.resolve();
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d'),mapImage=new Image();
const video=document.getElementById('video');
let videoInstance=null,videoNeedsReconnect=true,videoAttempt=0;
video.onerror=()=>{videoNeedsReconnect=true};
function pixel(p,m){let dx=p[0]-m.origin_x,dy=p[1]-m.origin_y,c=Math.cos(m.origin_yaw),s=Math.sin(m.origin_yaw);return [pad+(dx*c+dy*s)/m.resolution,pad+m.height-(-dx*s+dy*c)/m.resolution]}
function draw(){if(!mission||!mission.map)return;const m=mission.map;pad=Math.ceil(2/m.resolution);canvas.width=m.width+2*pad;canvas.height=m.height+2*pad;canvas.hidden=false;ctx.fillStyle='#7f7f7f';ctx.fillRect(0,0,canvas.width,canvas.height);if(mapImage.complete&&mapImage.naturalWidth===m.width&&mapImage.naturalHeight===m.height)ctx.drawImage(mapImage,pad,pad);ctx.lineWidth=2;ctx.strokeStyle='#22d3ee';ctx.beginPath();(mission.route||[]).forEach((p,i)=>{const q=pixel(p,m);if(i)ctx.lineTo(q[0],q[1]);else ctx.moveTo(q[0],q[1])});ctx.stroke();if(mission.robot){const q=pixel([mission.robot.x,mission.robot.y],m);ctx.fillStyle='#f59e0b';ctx.beginPath();ctx.arc(q[0],q[1],4,0,7);ctx.fill()}if(mission.goal){const q=pixel(mission.goal,m);ctx.fillStyle='#ef4444';ctx.beginPath();ctx.arc(q[0],q[1],4,0,7);ctx.fill()}}
mapImage.onload=draw;
canvas.onclick=e=>{if(!mission||!mission.map)return;const m=mission.map,r=canvas.getBoundingClientRect(),px=(e.clientX-r.left)*canvas.width/r.width,py=(e.clientY-r.top)*canvas.height/r.height,lx=(px-pad)*m.resolution,ly=(m.height-(py-pad))*m.resolution,c=Math.cos(m.origin_yaw),s=Math.sin(m.origin_yaw);document.getElementById('x').value=(m.origin_x+lx*c-ly*s).toFixed(2);document.getElementById('y').value=(m.origin_y+lx*s+ly*c).toFixed(2)};
async function post(path){const r=await fetch(path,{method:'POST'});document.getElementById('mission').textContent=await r.text()}
document.getElementById('go').onclick=()=>post('/goal?x='+encodeURIComponent(document.getElementById('x').value)+'&y='+encodeURIComponent(document.getElementById('y').value)+'&yaw='+encodeURIComponent(document.getElementById('yaw').value));
document.getElementById('explore').onclick=()=>post('/explore');
function stopManual() {
  if (manualTimer !== null) clearTimeout(manualTimer);
  manualTimer = null;
  const firstStop = post('/stop');
  // An earlier velocity request can reach the server after the immediate stop.
  manualStopPending = Promise.allSettled([manualStopPending, manualInFlight, firstStop])
    .then(() => post('/stop'));
  manualStopPending.catch(() => {});
  return manualStopPending;
}
function manualEnd() {
  manualPress = null;
  stopManual();
}
async function manualStart(button, event) {
  event.preventDefault();
  const replacing = manualPress !== null;
  const press = {button, pointerId: event.pointerId};
  manualPress = press;
  try {
    // The command bridge handles takeover when it accepts a fresh cmd_vel.
    await (replacing ? stopManual() : manualStopPending.catch(() => {}));
  } catch {
    if (manualPress === press) manualPress = null;
    return;
  }
  if (manualPress !== press) return;
  const [x, y, yaw] = button.dataset.motion.split(',');
  const path = `/manual?x=${x}&y=${y}&yaw=${yaw}`;
  function send() {
    if (manualPress !== press) return;
    manualTimer = null;
    const started = performance.now();
    manualInFlight = fetch(path, {method: 'POST'}).catch(() => {}).finally(() => {
      if (manualPress === press)
        manualTimer = setTimeout(send, Math.max(0, 100 - (performance.now() - started)));
    });
  }
  send();
}
document.getElementById('stop').onclick=manualEnd;
document.querySelectorAll('[data-motion]').forEach(button => {
  button.onpointerdown = event => manualStart(button, event);
  const release = event => {
    if (manualPress?.button === button && manualPress.pointerId === event.pointerId) manualEnd();
  };
  button.onpointerup = button.onpointercancel = button.onpointerleave = release;
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden && manualPress) manualEnd();
});
setInterval(async()=>{try{const s=await(await fetch('/status.json',{cache:'no-store'})).json();
if(s.fpga_health_json){try{s.fpga_health=JSON.parse(s.fpga_health_json)}catch(e){} delete s.fpga_health_json}
const fresh=s.last_jpeg_age_ms>=0&&s.last_jpeg_age_ms<1000;
if(videoInstance!==null&&videoInstance!==s.video_instance)videoNeedsReconnect=true;
videoInstance=s.video_instance;
if(fresh&&videoNeedsReconnect){videoNeedsReconnect=false;video.src='/video.mjpg?attempt='+(++videoAttempt)}
video.hidden=!fresh;
document.getElementById('state').textContent=fresh?'视频正在更新':'暂无新画面';
document.getElementById('vision').textContent=s.vision_age_ms>=0&&s.vision_age_ms<1000?
  '识别结果：'+(s.detections.length?s.detections.map(d=>'类别 '+d.class+' ('+d.score.toFixed(2)+')').join('，'):'当前未检出目标'):
  '视觉识别未启用或暂无新结果';
if(s.map_age_ms>=0)mapImage.src='/map.png?t='+Date.now();
document.getElementById('map_state').textContent=s.map_age_ms<0?'等待地图…':
  s.map_age_ms<5000?'地图正在更新':'地图数据已超过 5 秒未更新';
const localMap=document.getElementById('local_map');
if(s.local_map_age_ms>=0){localMap.hidden=false;localMap.src='/local_map.png?t='+Date.now();}
document.getElementById('local_map_state').textContent=s.local_map_age_ms<0?'等待地图…':
  s.local_map_age_ms<1000?'地图正在更新':'地图数据已超过 1 秒未更新';
document.getElementById('detail').textContent=JSON.stringify(s,null,2);
try{mission=await(await fetch('/mission.json',{cache:'no-store'})).json();document.getElementById('mission').textContent=mission.mode+'：'+(mission.reason||'');draw()}catch(e){mission=null;document.getElementById('mission').textContent='任务节点未连接'}
}catch(e){videoNeedsReconnect=true;video.hidden=true;document.getElementById('state').textContent='视频服务未连接';}},1000);
</script></html>)HTML";
      send_response(fd, "200 OK", "text/html; charset=utf-8", page);
    } else {
      send_response(fd, "404 Not Found", "text/plain; charset=utf-8", "Not found\n");
    }
  }

  void http_loop()
  {
    struct Client {
      std::thread worker;
      std::shared_ptr<std::atomic<bool>> done;
    };
    std::vector<Client> clients;
    while (running_) {
      for (auto it = clients.begin(); it != clients.end();) {
        if (it->done->load()) {
          it->worker.join();
          it = clients.erase(it);
        } else {
          ++it;
        }
      }
      pollfd descriptor{http_socket_, POLLIN, 0};
      const int result = poll(&descriptor, 1, 100);
      if (result < 0 && errno == EINTR) continue;
      if (result < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        RCLCPP_ERROR(get_logger(), "Video HTTP listener failed");
        break;
      }
      if (!(descriptor.revents & POLLIN)) continue;
      const int client_fd = accept(http_socket_, nullptr, nullptr);
      if (client_fd < 0) continue;
      auto done = std::make_shared<std::atomic<bool>>(false);
      ++http_clients_;
      clients.push_back(Client{std::thread([this, client_fd, done] {
        http_client(client_fd);
        close(client_fd);
        --http_clients_;
        done->store(true);
      }), done});
    }
    http_running_ = false;
    jpeg_ready_.notify_all();
    for (auto &client : clients) client.worker.join();
  }

  int socket_ = -1;
  int http_socket_ = -1;
  int width_ = 0;
  int height_ = 0;
  int jpeg_quality_ = 0;
  std::atomic<int> receive_buffer_actual_{0};
  double jpeg_fps_ = 0;
  std::size_t expected_frame_bytes_ = 0;
  std::chrono::milliseconds timeout_{250};
  std::string frame_id_;
  const std::string video_instance_ = std::to_string(steady_ms()) + "-" + std::to_string(getpid());
  std::optional<in_addr> source_addr_;
  std::atomic<bool> running_{true};
  std::atomic<bool> receiver_running_{true};
  std::atomic<bool> http_running_{false};
  std::atomic<std::uint32_t> http_clients_{0};
  std::thread receiver_, converter_, encoder_, http_server_;

  std::mutex raw_mutex_;
  std::condition_variable raw_ready_;
  std::shared_ptr<RawFrame> latest_raw_;
  std::uint64_t raw_generation_ = 0;
  std::mutex jpeg_mutex_;
  std::condition_variable jpeg_ready_;
  std::shared_ptr<const std::vector<std::uint8_t>> latest_jpeg_;
  std::int64_t latest_jpeg_ms_ = 0;
  std::uint64_t jpeg_generation_ = 0;
  std::mutex health_mutex_;
  std::string fpga_health_;
  std::int64_t fpga_health_ms_ = 0;
  std::mutex rx_status_mutex_;
  std::array<std::uint16_t, 8> rx_status_words_{};
  std::uint8_t rx_status_mode_ = 0;
  std::int64_t rx_status_ms_ = 0;
  std::mutex map_mutex_;
  std::shared_ptr<const std::vector<std::uint8_t>> latest_map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_map_grid_;
  std::string latest_map_geometry_;
  std::int64_t latest_map_ms_ = 0;
  std::shared_ptr<const std::vector<std::uint8_t>> latest_local_map_;
  std::int64_t latest_local_map_ms_ = 0;
  std::mutex mission_mutex_;
  std::string mission_status_;
  std::int64_t mission_status_ms_ = 0;
  std::mutex vision_mutex_;
  std::string vision_boxes_ = "[]";
  std::int64_t vision_ms_ = 0;

  std::atomic<std::uint64_t> packets_{0}, status_packets_{0}, invalid_packets_{0}, sequence_gaps_{0};
  std::atomic<std::uint64_t> incomplete_frames_{0}, complete_frames_{0};
  std::atomic<std::uint64_t> converted_frames_{0}, encoded_frames_{0};
  std::atomic<std::uint64_t> receive_errors_{0}, conversion_errors_{0}, encode_errors_{0};
  std::atomic<std::int64_t> last_complete_ms_{0};
  std::atomic<std::uint32_t> last_rtp_timestamp_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr vision_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr explore_pub_, stop_pub_, direct_stop_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<VideoNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_video"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
