#pragma once

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>

namespace ugv_bridge {
inline std::uint16_t u16(const std::uint8_t *p) { return (static_cast<std::uint16_t>(p[0]) << 8) | p[1]; }

inline std::uint32_t u32(const std::uint8_t *p) {
  return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
         (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}

inline in_addr ipv4_address(const std::string &value, const char *parameter) {
  in_addr address{};
  if (inet_pton(AF_INET, value.c_str(), &address) != 1)
    throw std::invalid_argument(std::string(parameter) + " must be an IPv4 address");
  return address;
}

inline std::optional<in_addr> udp_source(const std::string &value) {
  if (value.empty()) return std::nullopt;
  return ipv4_address(value, "source_address");
}

enum class ReceiveBuffer { best_effort, required };

constexpr short UDP_ERROR_EVENTS = POLLERR | POLLHUP | POLLNVAL;

inline int clear_udp_async_error(pollfd &descriptor) {
  if ((descriptor.revents & UDP_ERROR_EVENTS) != POLLERR) return 0;
  int error = 0;
  socklen_t length = sizeof(error);
  if (getsockopt(descriptor.fd, SOL_SOCKET, SO_ERROR, &error, &length) < 0 || !error) return 0;
  descriptor.revents &= ~POLLERR;
  return error;
}

inline bool retry_udp_receive(int error) {
  return error == EINTR || error == EAGAIN || error == EWOULDBLOCK;
}

inline int open_udp_input(in_addr address, std::uint16_t port, int buffer_size, ReceiveBuffer buffer_policy,
                          const char *name) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) throw std::system_error(errno, std::generic_category(), std::string(name) + " socket");
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr = address;
  local.sin_port = htons(port);
  const char *operation = nullptr;
  if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size)) < 0 &&
      buffer_policy == ReceiveBuffer::required)
    operation = " receive buffer";
  else if (::bind(fd, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) < 0)
    operation = " bind";
  if (operation) {
    const int error = errno;
    close(fd);
    throw std::system_error(error, std::generic_category(), std::string(name) + operation);
  }
  return fd;
}
}  // namespace ugv_bridge
