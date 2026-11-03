#pragma once

#include <cstdint>
#include <string>

#include "ugv_bridge/msg/authorized_twist.hpp"

namespace bx72 {

inline bool valid_mission_command(
    const ugv_bridge::msg::AuthorizedTwist& command, const std::string& bridge_id,
    std::uint64_t epoch, std::int64_t authorized_at_ns, std::int64_t now_ns, double timeout_s) {
  const auto& stamp = command.header.stamp;
  if (command.bridge_id != bridge_id || command.authorization_epoch != epoch ||
      command.header.frame_id != "base_link" || stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
      (stamp.sec == 0 && stamp.nanosec == 0)) return false;
  const auto stamp_ns = static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  if (stamp_ns <= authorized_at_ns || stamp_ns > now_ns) return false;
  return static_cast<double>(now_ns - stamp_ns) / 1.0e9 <= timeout_s;
}

}  // namespace bx72
