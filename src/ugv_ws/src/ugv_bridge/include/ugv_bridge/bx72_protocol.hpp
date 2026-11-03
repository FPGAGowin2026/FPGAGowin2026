#pragma once
#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <netinet/in.h>
#include <nlohmann/json.hpp>

namespace bx72 {
using Bytes = std::vector<std::uint8_t>;
using Json = nlohmann::json;
using Velocity = std::array<int, 3>;
constexpr std::size_t max_jpeg = 512 * 1024;
// BX72 ISP YUYV422 contract; PT96 denotes the retired RGB565 stream.
constexpr std::uint8_t video_payload_type = 97;
constexpr int video_width = 640;
constexpr int video_height = 360;
std::uint16_t crc16(const Bytes& data, std::size_t length);
Bytes fpga_packet(std::uint32_t seq, std::uint64_t token, Velocity velocity = {});
Bytes fpga_auto_packet(std::uint32_t seq, std::uint64_t token, bool enabled);
// nav_full_v1：1接收、2规划、3跟踪、5受阻、9等待；4到达、6拒绝、7取消、8失效。
constexpr bool navigation_status_active(std::uint8_t status) {
  return (status >= 1 && status <= 3) || status == 5 || status == 9;
}
constexpr bool navigation_status_terminal(std::uint8_t status) {
  return status == 4 || status == 6 || status == 7 || status == 8;
}
// Service acceptance does not imply task completion.
struct NavigationTask {
  // flags: bit0允许协同断链，bit1仅要求位置到达并停稳；未设bit1时保留终点航向。
  std::uint8_t action=1, mode=2, flags=0;
  std::uint32_t task_id=0, odom_domain=0, boot_id=0;
  std::int32_t goal_x_q16=0, goal_y_q16=0, hold_yaw_q16=0, final_yaw_q16=0;
  std::uint16_t max_speed_mm_s=400, max_accel_mm_s2=200;
  std::uint16_t tolerance_mm=100, yaw_tolerance_mrad=100;
  std::uint64_t expiry_us=0, sent_at_us=0;
  std::uint16_t horizon_ms=0;
};
Bytes encode_navigation_task(const NavigationTask& task);
NavigationTask decode_navigation_task(const Bytes& bytes);
std::vector<Bytes> fpga_task_packets(std::uint32_t first_seq, std::uint64_t token,
                                    const NavigationTask& task);
// Validate source time, domain, and task IDs before submitting to Jetson.
// 无上一个任务时传nullopt；返回0表示合格，1身份，2时间/字段，3编号。
std::uint8_t navigation_task_error(const NavigationTask& task, std::uint64_t now_us,
                                  std::uint32_t odom_domain, std::uint32_t boot_id,
                                  std::optional<std::uint32_t> previous_task_id=std::nullopt);
bool valid_ack(const Bytes& data, const Bytes& request);
bool newer(std::uint32_t seq, std::uint32_t previous);
Json decode_telemetry(const Bytes& data);
// 输入已经过二进制入口校验；完整快照才进入通用遥测序号和重启过滤。
class LocalGridAssembler {
 public:
  std::optional<Json> feed(const Json& chunk, double received_at);
  void set_domain(std::uint32_t boot_id, std::uint32_t odom_domain);
  void reset();

 private:
  std::optional<Json> header_;
  std::array<Bytes,8> chunks_;
  std::optional<std::uint64_t> context_;
  std::uint8_t seen_ = 0;
  bool invalid_ = false;
  bool completed_ = false;
  double started_at_ = 0;
};
std::vector<Json> telemetry_envelopes(const std::string& bridge_id, std::uint32_t epoch,
                                      const Json& data, double rx_mono_s, double tx_mono_s,
                                      std::size_t max_bytes=1400);
Json bounded_status(Json status, std::size_t max_bytes=1400);
std::vector<Bytes> jpeg_packets(std::uint32_t frame_id, const Bytes& jpeg);

// Shared UDP/runtime helpers used by the ROS gateway and the standalone tool.
double monotonic();
sockaddr_in endpoint(const std::string& ip, int port, bool allow_zero = false);
std::string peer_key(const sockaddr_in& address);
bool same_peer(const sockaddr_in& a, const sockaddr_in& b);

struct Datagram {
  Bytes data;
  sockaddr_in peer{};
};

class Udp {
 public:
  int fd = -1;

  Udp(const std::string& ip, int port, int buffer = 0);
  ~Udp();
  Udp(const Udp&) = delete;
  Udp& operator=(const Udp&) = delete;

  std::optional<Datagram> receive() const;
  bool send(const Bytes& data, const sockaddr_in& peer) const;
  bool send(const Json& json, const sockaddr_in& peer) const;
  int buffer() const;
};

class Args {
 public:
  std::map<std::string, std::string> values;

  Args(int argc, char** argv, std::map<std::string, std::string> defaults,
       const std::string& description);
  std::string get(const std::string& key) const;
  int integer(const std::string& key) const;
  double number(const std::string& key) const;
};

struct ProbePacket {
  std::uint8_t kind = 0;
  std::uint32_t sequence = 0;
  std::uint64_t token = 0;
  std::array<std::uint8_t,10> data{};
};
Bytes encode_probe(const ProbePacket& packet);
ProbePacket decode_probe(const Bytes& data);

struct PcControlFrame {
  std::string session;
  std::string claim_nonce;
  std::uint32_t seq = 0;
  Velocity velocity{};
  bool auto_requested = false;
  bool take_control = true;
  bool claim = false;
};
std::optional<PcControlFrame> parse_pc_control(const Json& msg);

struct PcLease {
  struct RetiredOwner {
    std::string session, peer;
    std::uint32_t seq = 0;
    double at = 0;
  };
  std::string session, peer;
  std::uint32_t seq = 0;
  double last = -1e100;
  Velocity velocity{};
  bool armed = false;
  bool auto_requested = false;
  std::deque<RetiredOwner> retired;
  void expire(double now);
  void disarm();
  bool accept(const Json& msg, const std::string& source, double now);
  bool accept(const PcControlFrame& msg, const std::string& source, double now,
              bool resume_retired = false);
};

bool cooperative_navigation_owner_lost(const PcLease& lease, bool owner_is_ros,
    std::uint8_t mode, bool flags_known, std::uint8_t flags,
    const std::string& owner_session, const std::string& owner_peer, double now);

struct PcObserver {
  std::string session;
  sockaddr_in endpoint{};
  std::uint32_t seq = 0;
  double last = 0;
  std::string claim_nonce;
  std::deque<std::string> retired_sessions;
};

// Single-threaded registry: read-only observers do not own the motion lease.
class PcObservers {
 public:
  bool would_accept(const PcControlFrame& frame, const sockaddr_in& peer) const;
  bool record(const PcControlFrame& frame, const sockaddr_in& peer, double now,
              bool prefer_owner = false, bool* changed = nullptr);
  bool expire(double now);
  void renew_claim_nonces();
  const PcObserver* find(const sockaddr_in& peer) const;
  const std::vector<PcObserver>& entries() const { return entries_; }

 private:
  static constexpr std::size_t max_observers_ = 16;
  static constexpr double timeout_s_ = 1.0;
  std::vector<PcObserver> entries_;
};

enum class PcInputResult { Rejected, Observer, Owner };
PcInputResult accept_pc_input(PcLease& lease, PcObservers& observers,
                              const PcControlFrame& frame, const sockaddr_in& peer,
                              double now, bool explicit_command = false,
                              bool safety_only = false, bool* changed = nullptr);

// 每个流独立按模序过滤；重启须由多条连续前进的流共同确认。
struct TelemetryTracker {
  struct Stamp {
    std::uint32_t sequence=0, time_us=0;
    double received_at=0;
    bool seen=false;
    std::uint32_t boot_id=0;
    bool boot_known=false;
  };
  struct ResetVote { Stamp latest; bool advancing=false; };
  std::map<std::string,Stamp> streams;
  std::map<std::string,ResetVote> reset_votes;
  Stamp epoch_anchor;
  std::deque<std::uint32_t> retired_boot_ids;
  std::uint32_t epoch=1;
  bool observe(const Json& data, double received_at, bool* restarted=nullptr);
};

struct RtpFrames {
  explicit RtpFrames(int width = video_width, int height = video_height);
  std::size_t size;
  std::optional<std::uint32_t> stamp;
  std::optional<std::uint16_t> expected;
  Bytes frame;
  bool bad = false;
  std::uint64_t packets = 0, complete = 0, dropped = 0, gaps = 0;
  std::optional<Bytes> feed(const Bytes& data);
  void expire_incomplete();
};

}  // namespace bx72
