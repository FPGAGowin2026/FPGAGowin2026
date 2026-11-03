#include <array>
#include <cmath>
#include <stdexcept>
#include "uav_control/px4_dds_interface.hpp"

int main()
{
  using namespace uav_control::dds_frames;
  const auto check = [](bool ok) {if (!ok) {throw std::runtime_error("frame conversion failed");}};
  check(position({1, 0, 0}) == (std::array<float, 3>{0, 1, 0}));
  check(position({0, 1, 0}) == (std::array<float, 3>{1, 0, 0}));
  check(position({0, 0, 1}) == (std::array<float, 3>{0, 0, -1}));
  check(position(position({2, -3, 4})) == (std::array<float, 3>{2, -3, 4}));
  for (double input : {0.0, pi / 2, -pi / 2, pi, -pi, 2*pi}) {
    check(std::abs(std::remainder(yaw(yaw(input)) - input, 2*pi)) < 1e-12);
    const auto q = attitude({static_cast<float>(std::cos(input/2)), 0, 0,
      static_cast<float>(std::sin(input/2))});
    check(std::abs(std::remainder(2*std::atan2(q[3], q[0]) - yaw(input), 2*pi)) < 1e-6);
  }
  // Combined roll/pitch/yaw: check basis vectors against world/body axis transforms.
  const std::array<float, 4> q{0.5, 0.5, 0.5, 0.5};
  const auto out = attitude(q);
  check(std::abs(out[0] - std::sqrt(0.5)) < 1e-12);
  check(std::abs(out[1] - std::sqrt(0.5)) < 1e-12);
  check(std::abs(out[2]) < 1e-12 && std::abs(out[3]) < 1e-12);
  const auto back = attitude({static_cast<float>(out[0]), static_cast<float>(out[1]),
    static_cast<float>(out[2]), static_cast<float>(out[3])});
  for (size_t i = 0; i < q.size(); ++i) {check(std::abs(back[i] - q[i]) < 1e-6);}
}
