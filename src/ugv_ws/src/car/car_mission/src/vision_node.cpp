#include <opencv2/dnn/dnn.hpp>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "BYTETracker.h"
#include "frame_processing_schedule.hpp"


namespace car_mission {
namespace {

constexpr int kInputWidth = 416;
constexpr int kInputHeight = 416;
constexpr std::size_t kInputValues = 3U * kInputWidth * kInputHeight;
using SteadyClock = std::chrono::steady_clock;

class TrtLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, char const * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "TensorRT: " << message << '\n';
    }
  }
};

void check_cuda(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
            std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

struct CudaBuffer {
  void * data{nullptr};

  ~CudaBuffer()
  {
    if (data != nullptr) {
      cudaFree(data);
    }
  }

  void allocate(std::size_t bytes)
  {
    check_cuda(cudaMalloc(&data, bytes), "cudaMalloc");
  }
};

struct CudaStream {
  cudaStream_t value{nullptr};

  ~CudaStream()
  {
    if (value != nullptr) {
      cudaStreamDestroy(value);
    }
  }
};

struct Letterbox {
  int left;
  int top;
  float scale_x;
  float scale_y;
};

Letterbox preprocess_to_nchw(const cv::Mat & bgr, std::vector<float> & input)
{
  const float scale = std::min(
    static_cast<float>(kInputWidth) / static_cast<float>(bgr.cols),
    static_cast<float>(kInputHeight) / static_cast<float>(bgr.rows));
  const int resized_width = std::max(1, static_cast<int>(std::lround(bgr.cols * scale)));
  const int resized_height = std::max(1, static_cast<int>(std::lround(bgr.rows * scale)));
  const int left = (kInputWidth - resized_width) / 2;
  const int top = (kInputHeight - resized_height) / 2;

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(resized_width, resized_height));
  cv::Mat padded(kInputHeight, kInputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(left, top, resized_width, resized_height)));
  cv::Mat rgb;
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
  for (int y = 0; y < kInputHeight; ++y) {
    const auto * row = rgb.ptr<cv::Vec3b>(y);
    for (int x = 0; x < kInputWidth; ++x) {
      for (int c = 0; c < 3; ++c) {
        input[static_cast<std::size_t>(c) * kInputWidth * kInputHeight +
          static_cast<std::size_t>(y) * kInputWidth + x] =
          static_cast<float>(row[x][c]) / 255.0F;
      }
    }
  }
  return Letterbox{
    left, top, static_cast<float>(resized_width) / bgr.cols,
    static_cast<float>(resized_height) / bgr.rows};
}

std::string dims_string(const nvinfer1::Dims & dims);

cv::Rect2f restore_xyxy(
  float x1, float y1, float x2, float y2, const Letterbox & letterbox,
  int image_width, int image_height)
{
  x1 = std::clamp((x1 - letterbox.left) / letterbox.scale_x, 0.0F,
      static_cast<float>(image_width));
  y1 = std::clamp((y1 - letterbox.top) / letterbox.scale_y, 0.0F,
      static_cast<float>(image_height));
  x2 = std::clamp((x2 - letterbox.left) / letterbox.scale_x, 0.0F,
      static_cast<float>(image_width));
  y2 = std::clamp((y2 - letterbox.top) / letterbox.scale_y, 0.0F,
      static_cast<float>(image_height));
  return cv::Rect2f(x1, y1, std::max(0.0F, x2 - x1), std::max(0.0F, y2 - y1));
}

std::size_t output_rows_for_format(
  const std::string & format, const nvinfer1::Dims & dims)
{
  if (dims.nbDims == 3 && dims.d[0] == 1 && dims.d[1] == 84 && dims.d[2] > 0) {
    return static_cast<std::size_t>(dims.d[2]);
  }
  if (format != "rows_xyxy_score_class") {
    throw std::runtime_error("unsupported output_format='" + format + "'");
  }
  if (dims.nbDims == 3 && dims.d[0] == 1 && dims.d[1] > 0 && dims.d[2] == 6) {
    return static_cast<std::size_t>(dims.d[1]);
  }
  if (dims.nbDims == 2 && dims.d[0] > 0 && dims.d[1] == 6) {
    return static_cast<std::size_t>(dims.d[0]);
  }
  throw std::runtime_error(
          "rows_xyxy_score_class 只接受输出 [1,N,6] 或 [N,6]，实际为 " +
          dims_string(dims));
}

std::int64_t stamp_nanoseconds(std::int32_t sec, std::uint32_t nanosec)
{
  if (sec < 0 || nanosec >= 1000000000U) {
    return 0;
  }
  return static_cast<std::int64_t>(sec) * 1000000000LL + nanosec;
}

bool source_stamp_is_fresh(
  std::int64_t stamp_ns, std::int64_t now_ns, double timeout_s)
{
  return stamp_ns > 0 && now_ns > 0 && stamp_ns <= now_ns &&
         static_cast<double>(now_ns - stamp_ns) / 1.0e9 <= timeout_s;
}

bool source_stamp_is_newer(std::int64_t stamp_ns, std::int64_t watermark_ns)
{
  return stamp_ns > 0 && stamp_ns > watermark_ns;
}

bool steady_receipt_is_fresh(
  SteadyClock::time_point received, SteadyClock::time_point now, double timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0 || now < received) {
    return false;
  }
  return std::chrono::duration<double>(now - received).count() <= timeout_s;
}

bool image_message_is_valid(const sensor_msgs::msg::Image & image)
{
  if (image.header.frame_id.empty() || image.width == 0 || image.height == 0 ||
    image.width > INT_MAX || image.height > INT_MAX)
  {
    return false;
  }
  const auto & encoding = image.encoding;
  if (encoding != sensor_msgs::image_encodings::BGR8 &&
    encoding != sensor_msgs::image_encodings::RGB8 &&
    encoding != sensor_msgs::image_encodings::MONO8 &&
    encoding != sensor_msgs::image_encodings::BGRA8 &&
    encoding != sensor_msgs::image_encodings::RGBA8)
  {
    return false;
  }
  try {
    const int cv_type = cv_bridge::getCvType(image.encoding);
    const std::uint64_t row_bytes =
      static_cast<std::uint64_t>(image.width) * CV_ELEM_SIZE(cv_type);
    const std::uint64_t image_bytes =
      static_cast<std::uint64_t>(image.step) * image.height;
    return image.step >= row_bytes && image.data.size() >= image_bytes;
  } catch (const cv_bridge::Exception &) {
    return false;
  }
}

std::string dims_string(const nvinfer1::Dims & dims)
{
  std::ostringstream out;
  out << '[';
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i != 0) {
      out << ',';
    }
    out << dims.d[i];
  }
  out << ']';
  return out.str();
}

const char * data_type_name(nvinfer1::DataType type)
{
  switch (type) {
    case nvinfer1::DataType::kFLOAT:
      return "FP32";
    case nvinfer1::DataType::kHALF:
      return "FP16";
    default:
      return "unsupported";
  }
}

std::size_t data_type_size(nvinfer1::DataType type)
{
  if (type == nvinfer1::DataType::kFLOAT) {
    return sizeof(float);
  }
  if (type == nvinfer1::DataType::kHALF) {
    return sizeof(__half);
  }
  throw std::runtime_error("TensorRT tensor type must be FP32 or FP16");
}

std::string new_session_id()
{
  std::random_device random_device;
  std::seed_seq seed{
    random_device(), random_device(), random_device(), random_device(),
    random_device(), random_device(), random_device(), random_device()};
  std::mt19937_64 random(seed);
  std::ostringstream out;
  out << std::hex << std::setfill('0') << std::setw(16) << random() <<
    std::setw(16) << random();
  return out.str();
}

class VisionNode final : public rclcpp::Node {
 public:
  VisionNode()
  : Node("vision_node")
  {
    engine_path_ = declare_parameter<std::string>(
      "engine_path", "/home/aa/yolo/yolo26n_fp16.engine");
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    class_id_ = declare_parameter<int>("class_id", -1);
    process_hz_ = declare_parameter<double>("process_hz", 10.0);
    image_timeout_s_ = declare_parameter<double>("image_timeout_s", 0.5);
    score_threshold_ = declare_parameter<double>("score_threshold", 0.1);
    tracker_threshold_ = declare_parameter<double>("tracker_threshold", 0.5);
    new_track_threshold_ = declare_parameter<double>("new_track_threshold", 0.6);
    output_format_ = declare_parameter<std::string>(
      "output_format", "rows_xyxy_score_class");
    debug_image_enabled_ = declare_parameter<bool>("debug_image", false);

    if (image_topic_.empty() || class_id_ < -1 || !std::isfinite(process_hz_) ||
      process_hz_ <= 0.0 ||
      process_hz_ > 1000.0 ||
      !std::isfinite(image_timeout_s_) || image_timeout_s_ <= 0.0 ||
      !std::isfinite(score_threshold_) || score_threshold_ < 0.0 || score_threshold_ > 1.0 ||
      !std::isfinite(tracker_threshold_) || tracker_threshold_ < score_threshold_ || tracker_threshold_ > 1.0 ||
      !std::isfinite(new_track_threshold_) || new_track_threshold_ < tracker_threshold_ || new_track_threshold_ > 1.0)
    {
      throw std::invalid_argument("process_hz, image_timeout_s, score_threshold 参数无效");
    }
    if (output_format_ != "rows_xyxy_score_class") {
      throw std::invalid_argument(
              "不支持 output_format='" + output_format_ +
              "'；当前只接受 rows_xyxy_score_class");
    }

    load_engine();
    reset_tracker(false);

    detections_publisher_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "/vision/detections", rclcpp::QoS(10));
    if (debug_image_enabled_) {
      debug_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "/vision/debug_image", rclcpp::QoS(1));
    }
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VisionNode::on_image, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / process_hz_));
    frame_schedule_.emplace(period);
    timer_ = create_wall_timer(period, std::bind(&VisionNode::process_latest_image, this));
    RCLCPP_INFO(
      get_logger(), "TensorRT 输入 %s %s %s，输出 %s %s %s，格式 %s",
      input_name_.c_str(), dims_string(input_dims_).c_str(),
      data_type_name(input_type_), output_name_.c_str(), dims_string(output_dims_).c_str(),
      data_type_name(output_type_), raw_output_ ? "raw_xywh_80_scores" : output_format_.c_str());
  }

 private:
  void load_engine()
  {
    if (!std::filesystem::is_regular_file(engine_path_)) {
      throw std::runtime_error("TensorRT engine 文件不存在: " + engine_path_);
    }
    std::ifstream file(engine_path_, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("无法打开 TensorRT engine: " + engine_path_);
    }
    const auto file_position = file.tellg();
    if (file_position <= std::streampos(0)) {
      throw std::runtime_error("TensorRT engine 文件为空: " + engine_path_);
    }
    const auto file_size = static_cast<std::size_t>(
      static_cast<std::streamoff>(file_position));
    std::vector<char> bytes(file_size);
    file.seekg(0);
    file.read(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!file) {
      throw std::runtime_error("读取 TensorRT engine 失败: " + engine_path_);
    }

    runtime_.reset(nvinfer1::createInferRuntime(trt_logger_));
    if (!runtime_) {
      throw std::runtime_error("TensorRT createInferRuntime 失败");
    }
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) {
      throw std::runtime_error("TensorRT engine 反序列化失败: " + engine_path_);
    }

    std::vector<std::string> inputs;
    std::vector<std::string> outputs;
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * tensor_name = engine_->getIOTensorName(i);
      if (tensor_name == nullptr) {
        throw std::runtime_error("TensorRT 返回空 I/O tensor 名称");
      }
      const auto mode = engine_->getTensorIOMode(tensor_name);
      if (mode != nvinfer1::TensorIOMode::kINPUT &&
        mode != nvinfer1::TensorIOMode::kOUTPUT)
      {
        throw std::runtime_error(
                std::string("TensorRT 返回不支持的 I/O mode: ") + tensor_name);
      }
      const auto shape = engine_->getTensorShape(tensor_name);
      const auto type = engine_->getTensorDataType(tensor_name);
      RCLCPP_INFO(
        get_logger(), "发现 TensorRT tensor name=%s mode=%s shape=%s type=%s",
        tensor_name, mode == nvinfer1::TensorIOMode::kINPUT ? "input" : "output",
        dims_string(shape).c_str(), data_type_name(type));
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        inputs.emplace_back(tensor_name);
      } else {
        outputs.emplace_back(tensor_name);
      }
    }
    if (inputs.size() != 1 || outputs.size() != 1) {
      throw std::runtime_error(
              "引擎必须恰有一个输入和一个输出；发现输入=" +
              std::to_string(inputs.size()) + " 输出=" + std::to_string(outputs.size()));
    }
    input_name_ = inputs.front();
    output_name_ = outputs.front();
    if (engine_->getTensorLocation(input_name_.c_str()) != nvinfer1::TensorLocation::kDEVICE ||
      engine_->getTensorLocation(output_name_.c_str()) != nvinfer1::TensorLocation::kDEVICE)
    {
      throw std::runtime_error("当前节点只支持 GPU 上的 TensorRT 输入和输出 tensor");
    }
    if (engine_->getTensorFormat(input_name_.c_str()) != nvinfer1::TensorFormat::kLINEAR ||
      engine_->getTensorFormat(output_name_.c_str()) != nvinfer1::TensorFormat::kLINEAR)
    {
      throw std::runtime_error("当前节点只支持 TensorRT LINEAR tensor 格式");
    }

    input_type_ = engine_->getTensorDataType(input_name_.c_str());
    output_type_ = engine_->getTensorDataType(output_name_.c_str());
    data_type_size(input_type_);
    data_type_size(output_type_);
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("TensorRT createExecutionContext 失败");
    }

    const auto engine_input_dims = engine_->getTensorShape(input_name_.c_str());
    bool dynamic_input = false;
    for (int i = 0; i < engine_input_dims.nbDims; ++i) {
      dynamic_input = dynamic_input || engine_input_dims.d[i] < 0;
    }
    if (dynamic_input) {
      nvinfer1::Dims fixed_input_dims{};
      fixed_input_dims.nbDims = 4;
      fixed_input_dims.d[0] = 1;
      fixed_input_dims.d[1] = 3;
      fixed_input_dims.d[2] = kInputHeight;
      fixed_input_dims.d[3] = kInputWidth;
      if (!context_->setInputShape(input_name_.c_str(), fixed_input_dims)) {
        throw std::runtime_error("引擎不能接受输入 shape [1,3,416,416]");
      }
    }
    input_dims_ = context_->getTensorShape(input_name_.c_str());
    if (input_dims_.nbDims != 4 || input_dims_.d[0] != 1 || input_dims_.d[1] != 3 ||
      input_dims_.d[2] != kInputHeight || input_dims_.d[3] != kInputWidth)
    {
      throw std::runtime_error(
              "输入 tensor 必须为 [1,3,416,416]，实际为 " + dims_string(input_dims_));
    }
    output_dims_ = context_->getTensorShape(output_name_.c_str());
    raw_output_ = output_dims_.nbDims == 3 && output_dims_.d[0] == 1 && output_dims_.d[1] == 84;
    if (raw_output_ && class_id_ >= 80) throw std::runtime_error("raw output class_id out of range");
    output_rows_ = output_rows_for_format(output_format_, output_dims_);
    if (output_rows_ > 1000000U) {
      throw std::runtime_error("输出行数超过节点支持上限: " + std::to_string(output_rows_));
    }

    input_float_.resize(kInputValues);
    input_half_.resize(kInputValues);
    output_host_.resize(output_rows_ * (raw_output_ ? 84U : 6U) * data_type_size(output_type_));
    input_device_.allocate(kInputValues * data_type_size(input_type_));
    output_device_.allocate(output_host_.size());
    check_cuda(cudaStreamCreate(&stream_.value), "cudaStreamCreate");
    if (!context_->setTensorAddress(input_name_.c_str(), input_device_.data) ||
      !context_->setTensorAddress(output_name_.c_str(), output_device_.data))
    {
      throw std::runtime_error("TensorRT setTensorAddress 失败");
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    const std::int64_t ros_now_ns = get_clock()->now().nanoseconds();
    observe_ros_clock(ros_now_ns);
    const std::int64_t stamp_ns = stamp_nanoseconds(
      message->header.stamp.sec, message->header.stamp.nanosec);
    if (!source_stamp_is_newer(stamp_ns, last_source_stamp_ns_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "丢弃旧或重复图像时间戳，不刷新视觉输入");
      return;
    }

    if (!source_stamp_is_fresh(stamp_ns, ros_now_ns, image_timeout_s_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "丢弃 ROS 时间戳无效、未来或过期的图像");
      return;
    }
    if (!image_message_is_valid(*message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "丢弃 frame_id、编码、步长或数据长度无效的图像");
      return;
    }
    last_source_stamp_ns_ = stamp_ns;
    if (latest_image_ && !image_is_fresh(
        *latest_image_, latest_image_received_, ros_now_ns))
    {
      discard_cached_image("新图像到达前缓存已过期", true);
    }
    latest_image_ = message;
    latest_image_received_ = SteadyClock::now();
    frame_schedule_->image_received();
    timeout_reset_done_ = false;
    // Do not wait for the next periodic tick when the inference rate limit allows it.
    process_latest_image();
  }

  void discard_cached_image(const char * reason, bool reset)
  {
    latest_image_.reset();
    frame_schedule_->discard_pending();
    timeout_reset_done_ = true;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason);
    if (reset) {
      reset_tracker();
    }
  }

  bool observe_ros_clock(std::int64_t now_ns)
  {
    const bool rolled_back = last_ros_now_ns_ > 0 && now_ns < last_ros_now_ns_;
    last_ros_now_ns_ = now_ns;
    if (rolled_back) {
      last_source_stamp_ns_ = 0;
      discard_cached_image("检测到 ROS 时钟回退", false);
      reset_tracker();
    }
    return rolled_back;
  }

  bool image_is_fresh(
    const sensor_msgs::msg::Image & image, SteadyClock::time_point received,
    std::int64_t ros_now_ns) const
  {
    if (!steady_receipt_is_fresh(received, SteadyClock::now(), image_timeout_s_)) {
      return false;
    }
    const std::int64_t stamp_ns = stamp_nanoseconds(
      image.header.stamp.sec, image.header.stamp.nanosec);
    return source_stamp_is_fresh(stamp_ns, ros_now_ns, image_timeout_s_);
  }

  void reset_tracker(bool warn = true)
  {
    tracker_ = std::make_unique<BYTETracker>(
      std::max(1, static_cast<int>(std::lround(process_hz_))), 30,
      static_cast<float>(tracker_threshold_), static_cast<float>(new_track_threshold_));
    session_id_ = new_session_id();
    if (warn) {
      RCLCPP_WARN(get_logger(), "重置 ByteTrack，开始新 session=%s", session_id_.c_str());
    }
  }

  void infer()
  {
    const std::size_t input_bytes = kInputValues * data_type_size(input_type_);
    const void * input = input_float_.data();
    if (input_type_ == nvinfer1::DataType::kHALF) {
      for (std::size_t i = 0; i < kInputValues; ++i) {
        input_half_[i] = __float2half_rn(input_float_[i]);
      }
      input = input_half_.data();
    }
    check_cuda(
      cudaMemcpyAsync(input_device_.data, input, input_bytes, cudaMemcpyHostToDevice, stream_.value),
      "cudaMemcpyAsync H2D");
    if (!context_->enqueueV3(stream_.value)) {
      throw std::runtime_error("TensorRT enqueueV3 失败");
    }
    check_cuda(
      cudaMemcpyAsync(
        output_host_.data(), output_device_.data, output_host_.size(),
        cudaMemcpyDeviceToHost, stream_.value), "cudaMemcpyAsync D2H");
    check_cuda(cudaStreamSynchronize(stream_.value), "cudaStreamSynchronize");
  }

  float output_value(std::size_t index) const
  {
    if (output_type_ == nvinfer1::DataType::kFLOAT) {
      float value;
      std::memcpy(&value, output_host_.data() + index * sizeof(value), sizeof(value));
      return value;
    }
    __half value;
    std::memcpy(&value, output_host_.data() + index * sizeof(value), sizeof(value));
    return __half2float(value);
  }

  std::vector<Object> detections_from_output(
    const cv::Mat & bgr, const Letterbox & letterbox)
  {
    std::vector<Object> detections;
    if (raw_output_) {
      std::vector<cv::Rect> boxes;
      std::vector<float> scores;
      std::vector<int> labels;
      for (std::size_t i = 0; i < output_rows_; ++i) {
        float best = -1.0F; int label = -1;
        for (int c = 0; c < 80; ++c) {
          const float v = output_value((4U + c) * output_rows_ + i);
          if (std::isfinite(v) && v > best) { best = v; label = c; }
        }
        if ((class_id_ >= 0 && label != class_id_) || best < score_threshold_ || best > 1.0F) continue;
        const float x = output_value(i), y = output_value(output_rows_ + i);
        const float w = output_value(2U * output_rows_ + i), h = output_value(3U * output_rows_ + i);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) || !std::isfinite(h) || w <= 0 || h <= 0) continue;
        auto r = restore_xyxy(x-w/2, y-h/2, x+w/2, y+h/2, letterbox, bgr.cols, bgr.rows);
        if (r.width <= 0 || r.height <= 0) continue;
        boxes.emplace_back(cvRound(r.x), cvRound(r.y), cvRound(r.width), cvRound(r.height)); scores.push_back(best); labels.push_back(label);
      }
      std::vector<int> kept;
      for (int label = 0; label < 80; ++label) {
        std::vector<cv::Rect> class_boxes;
        std::vector<float> class_scores;
        std::vector<int> indices, selected;
        for (std::size_t j = 0; j < labels.size(); ++j) {
          if (labels[j] == label) { class_boxes.push_back(boxes[j]); class_scores.push_back(scores[j]); indices.push_back(static_cast<int>(j)); }
        }
        if (!class_boxes.empty()) {
          cv::dnn::NMSBoxes(class_boxes, class_scores, static_cast<float>(score_threshold_), 0.45F, selected);
          for (int j : selected) kept.push_back(indices[j]);
        }
      }
      for (int i : kept) {
        Object d; d.rect = boxes[i]; d.label = labels[i]; d.prob = scores[i];
        d.detection_index = static_cast<int>(detections.size()); detections.push_back(d);
      }
      return detections;
    }
    for (std::size_t row = 0; row < output_rows_; ++row) {
      const std::size_t offset = row * 6U;
      const float x1_model = output_value(offset);
      const float y1_model = output_value(offset + 1U);
      const float x2_model = output_value(offset + 2U);
      const float y2_model = output_value(offset + 3U);
      const float score = output_value(offset + 4U);
      const float label_value = output_value(offset + 5U);
      if (!std::isfinite(x1_model) || !std::isfinite(y1_model) ||
        !std::isfinite(x2_model) || !std::isfinite(y2_model) || !std::isfinite(score) ||
        !std::isfinite(label_value) || score < score_threshold_ || score > 1.0F)
      {
        continue;
      }
      const double rounded_label = std::round(static_cast<double>(label_value));
      if (std::abs(static_cast<double>(label_value) - rounded_label) > 1.0e-3 ||
        rounded_label < 0.0 ||
        rounded_label > static_cast<double>(std::numeric_limits<int>::max()) ||
        (class_id_ >= 0 && static_cast<int>(rounded_label) != class_id_))
      {
        continue;
      }

      const auto rect = restore_xyxy(
        x1_model, y1_model, x2_model, y2_model, letterbox, bgr.cols, bgr.rows);
      if (rect.width <= 0.0F || rect.height <= 0.0F) {
        continue;
      }
      Object detection;
      detection.rect = rect;
      detection.label = static_cast<int>(rounded_label);
      detection.prob = score;
      detection.detection_index = static_cast<int>(detections.size());
      detections.push_back(detection);
    }
    return detections;
  }

  vision_msgs::msg::Detection2DArray make_message(
    const sensor_msgs::msg::Image & image, const std::vector<Object> & detections,
    const std::vector<STrack> & tracks)
  {
    vision_msgs::msg::Detection2DArray message;
    message.header = image.header;
    std::vector<bool> published(detections.size(), false);
    const int current_frame = tracker_->current_frame_id();
    for (const auto & track : tracks) {
      if (track.frame_id != current_frame || track.detection_index < 0 ||
        static_cast<std::size_t>(track.detection_index) >= detections.size() ||
        published[static_cast<std::size_t>(track.detection_index)])
      {
        continue;
      }
      const auto & observation = detections[static_cast<std::size_t>(track.detection_index)];
      if (track.observed_tlwh.size() != 4 || track.score != observation.prob) {
        continue;
      }
      const float x = track.observed_tlwh[0];
      const float y = track.observed_tlwh[1];
      const float width = track.observed_tlwh[2];
      const float height = track.observed_tlwh[3];
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(width) ||
        !std::isfinite(height) || width <= 0.0F || height <= 0.0F)
      {
        continue;
      }
      published[static_cast<std::size_t>(track.detection_index)] = true;

      vision_msgs::msg::Detection2D detection;
      detection.header = image.header;
      detection.id = session_id_ + ":" + std::to_string(track.track_id);
      detection.bbox.center.position.x = x + width * 0.5F;
      detection.bbox.center.position.y = y + height * 0.5F;
      detection.bbox.center.theta = 0.0;
      detection.bbox.size_x = width;
      detection.bbox.size_y = height;
      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.hypothesis.class_id = std::to_string(observation.label);
      result.hypothesis.score = observation.prob;
      detection.results.push_back(result);
      message.detections.push_back(std::move(detection));
    }
    return message;
  }

  void process_latest_image()
  {
    const std::int64_t ros_now_ns = get_clock()->now().nanoseconds();
    if (observe_ros_clock(ros_now_ns)) {
      return;
    }
    if (!inference_healthy_) {
      return;
    }
    if (!latest_image_) {
      return;
    }
    if (!frame_schedule_->has_pending()) {
      if (!timeout_reset_done_ && !image_is_fresh(
          *latest_image_, latest_image_received_, ros_now_ns))
      {
        discard_cached_image("缓存图像已过期", true);
      }
      return;
    }
    const auto image = latest_image_;
    const auto received = latest_image_received_;
    if (!image_is_fresh(*image, received, ros_now_ns)) {
      discard_cached_image("图像在推理前已过期", true);
      return;
    }
    if (!frame_schedule_->try_start(SteadyClock::now())) {
      return;
    }

    try {
      const auto cv_image = cv_bridge::toCvCopy(
        image, sensor_msgs::image_encodings::BGR8);
      if (cv_image->image.empty() || cv_image->image.cols <= 0 || cv_image->image.rows <= 0) {
        throw std::runtime_error("收到空图像");
      }
      const auto letterbox = preprocess_to_nchw(cv_image->image, input_float_);
      auto fresh_now_ns = get_clock()->now().nanoseconds();
      if (observe_ros_clock(fresh_now_ns)) {
        return;
      }
      if (!image_is_fresh(*image, received, fresh_now_ns)) {
        discard_cached_image("图像在推理开始前已过期", true);
        return;
      }
      infer();
      fresh_now_ns = get_clock()->now().nanoseconds();
      if (observe_ros_clock(fresh_now_ns)) {
        return;
      }
      if (!image_is_fresh(*image, received, fresh_now_ns)) {
        discard_cached_image("图像在推理后已过期", true);
        return;
      }
      const auto detections = detections_from_output(cv_image->image, letterbox);
      if (debug_image_enabled_ && !diagnostic_logged_) {
        for (const auto & d : detections) {
          RCLCPP_INFO(get_logger(), "decoded class=%d score=%.6f xywh=%.2f,%.2f,%.2f,%.2f", d.label, d.prob, d.rect.x, d.rect.y, d.rect.width, d.rect.height);
        }
        RCLCPP_INFO(get_logger(), "decoded count=%zu", detections.size());
        diagnostic_logged_ = true;
      }
      const auto tracks = tracker_->update(detections);
      auto message = make_message(*image, detections, tracks);
      fresh_now_ns = get_clock()->now().nanoseconds();
      if (observe_ros_clock(fresh_now_ns)) {
        return;
      }
      if (!image_is_fresh(*image, received, fresh_now_ns)) {
        discard_cached_image("图像在发布前已过期", true);
        return;
      }
      detections_publisher_->publish(message);
      if (debug_image_enabled_) {
        cv::Mat debug = cv_image->image.clone();
        for (const auto & detection : message.detections) {
          const auto & box = detection.bbox;
          const cv::Point top_left(
            static_cast<int>(box.center.position.x - box.size_x * 0.5),
            static_cast<int>(box.center.position.y - box.size_y * 0.5));
          const cv::Point bottom_right(
            static_cast<int>(box.center.position.x + box.size_x * 0.5),
            static_cast<int>(box.center.position.y + box.size_y * 0.5));
          cv::rectangle(debug, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
          cv::putText(
            debug, detection.id, top_left, cv::FONT_HERSHEY_SIMPLEX, 0.5,
            cv::Scalar(0, 255, 0), 1);
        }
        debug_publisher_->publish(
          *cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::BGR8, debug)
          .toImageMsg());
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "视觉帧处理失败，不发布该帧: %s", error.what());
      const cudaError_t sync_result = cudaStreamSynchronize(stream_.value);
      if (sync_result != cudaSuccess) {
        inference_healthy_ = false;
        RCLCPP_FATAL(
          get_logger(), "CUDA stream 无法同步，停止视觉处理: %s",
          cudaGetErrorString(sync_result));
        rclcpp::shutdown();
      }
      reset_tracker();
    }
  }

  TrtLogger trt_logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  CudaBuffer input_device_;
  CudaBuffer output_device_;
  CudaStream stream_;
  std::vector<float> input_float_;
  std::vector<__half> input_half_;
  std::vector<std::uint8_t> output_host_;
  std::string engine_path_;
  std::string image_topic_;
  std::string output_format_;
  std::string input_name_;
  std::string output_name_;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  nvinfer1::DataType input_type_{nvinfer1::DataType::kFLOAT};
  nvinfer1::DataType output_type_{nvinfer1::DataType::kFLOAT};
  std::size_t output_rows_{0};
  bool raw_output_{false};
  bool diagnostic_logged_{false};
  int class_id_{0};
  double process_hz_{10.0};
  double image_timeout_s_{0.5};
  double tracker_threshold_{0.5}, new_track_threshold_{0.6};
  double score_threshold_{0.1};
  bool debug_image_enabled_{false};
  std::unique_ptr<BYTETracker> tracker_;
  std::string session_id_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  SteadyClock::time_point latest_image_received_{};
  std::optional<FrameProcessingSchedule> frame_schedule_;
  std::int64_t last_source_stamp_ns_{0};
  std::int64_t last_ros_now_ns_{0};
  bool timeout_reset_done_{false};
  bool inference_healthy_{true};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace car_mission

#ifndef CAR_MISSION_VISION_NODE_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<car_mission::VisionNode>());
  } catch (const std::exception & error) {
    std::cerr << "vision_node 启动失败: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
