#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vision_bridge {

struct TargetObservation {
  double horizontal_error;
  double vertical_error;
  double area_fraction;
};

std::optional<TargetObservation> detect_red_target(
  const std::vector<std::uint8_t> & data, std::size_t width, std::size_t height,
  std::size_t row_step, const std::string & encoding, int min_red = 120,
  double red_ratio = 1.5, double min_fraction = 0.002,
  std::size_t sample_stride = 2);

std::vector<std::uint8_t> render_target(
  std::size_t width, std::size_t height, double distance, double bearing,
  double field_of_view, double size_scale);

}  // namespace vision_bridge
