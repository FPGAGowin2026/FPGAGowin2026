#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "ugv_bridge/bx72_protocol.hpp"

namespace bx72 {

inline cv::Mat yuv422_to_bgr(const Bytes& raw, const std::string& pixel_format)
{
  constexpr int kWidth = video_width;
  constexpr int kHeight = video_height;
  constexpr std::size_t kFrameBytes =
    static_cast<std::size_t>(kWidth) * kHeight * 2U;

  const bool yuyv = pixel_format == "yuyv";
  if (!yuyv && pixel_format != "uyvy") {
    throw std::invalid_argument("pixel_format must be yuyv or uyvy");
  }
  if (raw.size() != kFrameBytes) {
    throw std::invalid_argument("YUV422 frame size does not match the BX72 video contract");
  }

  // The BX72 stream is BT.709 limited-range; generic YUYV cvtColor uses BT.601.
  cv::Mat bgr(kHeight, kWidth, CV_8UC3);
  const auto to_byte = [](double value) {
      const double rounded = std::floor(value * 255.0 + 0.5);
      return static_cast<std::uint8_t>(std::clamp(rounded, 0.0, 255.0));
    };
  const auto convert_pixel = [&to_byte](std::uint8_t y, double cb, double cr) {
      const double normalized_y = (static_cast<int>(y) - 16) / 219.0;
      const double red = normalized_y + 1.5748 * cr;
      const double green = normalized_y - 0.187324 * cb - 0.468124 * cr;
      const double blue = normalized_y + 1.8556 * cb;
      return cv::Vec3b(to_byte(blue), to_byte(green), to_byte(red));
    };

  for (int row = 0; row < kHeight; ++row) {
    const auto* src = raw.data() + static_cast<std::size_t>(row) * kWidth * 2U;
    auto* dst = bgr.ptr<cv::Vec3b>(row);
    for (int x = 0; x < kWidth; x += 2) {
      const std::size_t i = static_cast<std::size_t>(x) * 2U;
      const auto y0 = src[i + (yuyv ? 0 : 1)];
      const auto u = src[i + (yuyv ? 1 : 0)];
      const auto y1 = src[i + (yuyv ? 2 : 3)];
      const auto v = src[i + (yuyv ? 3 : 2)];
      const double cb = (static_cast<int>(u) - 128) / 224.0;
      const double cr = (static_cast<int>(v) - 128) / 224.0;
      dst[x] = convert_pixel(y0, cb, cr);
      dst[x + 1] = convert_pixel(y1, cb, cr);
    }
  }
  return bgr;
}

}  // namespace bx72
