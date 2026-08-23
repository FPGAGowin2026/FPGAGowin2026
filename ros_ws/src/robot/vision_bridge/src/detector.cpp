#include "vision_bridge/detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace vision_bridge {
namespace {

struct PixelLayout {
  std::size_t channels;
  std::size_t red;
  std::size_t green;
  std::size_t blue;
};

std::optional<PixelLayout> pixel_layout(const std::string & encoding)
{
  if (encoding == "rgb8") {
    return PixelLayout{3, 0, 1, 2};
  }
  if (encoding == "bgr8") {
    return PixelLayout{3, 2, 1, 0};
  }
  if (encoding == "rgba8") {
    return PixelLayout{4, 0, 1, 2};
  }
  if (encoding == "bgra8") {
    return PixelLayout{4, 2, 1, 0};
  }
  return std::nullopt;
}

}  // namespace

std::optional<TargetObservation> detect_red_target(
  const std::vector<std::uint8_t> & data, std::size_t width, std::size_t height,
  std::size_t row_step, const std::string & encoding, int min_red,
  double red_ratio, double min_fraction, std::size_t sample_stride)
{
  const auto layout = pixel_layout(encoding);
  if (!layout || width == 0 || height == 0 || sample_stride == 0 ||
    row_step < width * layout->channels || data.size() < row_step * height)
  {
    return std::nullopt;
  }

  std::size_t red_count = 0;
  std::size_t sampled_count = 0;
  std::size_t sum_x = 0;
  std::size_t sum_y = 0;
  for (std::size_t y = 0; y < height; y += sample_stride) {
    const std::size_t row = y * row_step;
    for (std::size_t x = 0; x < width; x += sample_stride) {
      const std::size_t offset = row + x * layout->channels;
      const int red = data[offset + layout->red];
      const int green = data[offset + layout->green];
      const int blue = data[offset + layout->blue];
      ++sampled_count;
      if (red >= min_red && red >= green * red_ratio && red >= blue * red_ratio) {
        ++red_count;
        sum_x += x;
        sum_y += y;
      }
    }
  }

  const double fraction = sampled_count == 0 ? 0.0 :
    static_cast<double>(red_count) / static_cast<double>(sampled_count);
  if (red_count == 0 || fraction < min_fraction) {
    return std::nullopt;
  }

  const double center_x = static_cast<double>(sum_x) / static_cast<double>(red_count);
  const double center_y = static_cast<double>(sum_y) / static_cast<double>(red_count);
  return TargetObservation{
    (center_x - static_cast<double>(width - 1U) * 0.5) /
      std::max(static_cast<double>(width) * 0.5, 1.0),
    (center_y - static_cast<double>(height - 1U) * 0.5) /
      std::max(static_cast<double>(height) * 0.5, 1.0),
    fraction,
  };
}

std::vector<std::uint8_t> render_target(
  std::size_t width, std::size_t height, double distance, double bearing,
  double field_of_view, double size_scale)
{
  std::vector<std::uint8_t> image(width * height * 3U, 0U);
  if (distance <= 0.0 || std::abs(bearing) > field_of_view * 0.5) {
    return image;
  }

  const int center_x = static_cast<int>(
    (0.5 - bearing / field_of_view) * static_cast<double>(width));
  const int center_y = static_cast<int>(height / 2U);
  const int half_size = std::max(
    2, std::min(
      static_cast<int>(width / 4U),
      static_cast<int>(size_scale / std::max(distance, 0.2))));
  const int x_begin = std::max(0, center_x - half_size);
  const int x_end = std::min(static_cast<int>(width), center_x + half_size + 1);
  const int y_begin = std::max(0, center_y - half_size);
  const int y_end = std::min(static_cast<int>(height), center_y + half_size + 1);
  for (int y = y_begin; y < y_end; ++y) {
    for (int x = x_begin; x < x_end; ++x) {
      image[(static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)) * 3U] =
        255U;
    }
  }
  return image;
}

}  // namespace vision_bridge
