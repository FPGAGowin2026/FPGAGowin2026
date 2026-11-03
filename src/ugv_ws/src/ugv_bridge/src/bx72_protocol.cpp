#include "ugv_bridge/bx72_protocol.hpp"
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <utility>

namespace bx72 {
namespace {
std::uint16_t u16(const Bytes& b, std::size_t p) { return (b.at(p) << 8) | b.at(p+1); }
std::uint32_t u32(const Bytes& b, std::size_t p) {
  return (std::uint32_t(u16(b,p)) << 16) | u16(b,p+2);
}
std::uint64_t u64(const Bytes& b, std::size_t p) {
  return (std::uint64_t(u32(b,p))<<32)|u32(b,p+4);
}
std::int32_t i32(const Bytes& b, std::size_t p) {
  auto v = u32(b,p);
  return v <= 0x7fffffffU ? static_cast<std::int32_t>(v) : static_cast<std::int32_t>(std::int64_t(v)-0x100000000LL);
}
int i16(const Bytes& b, std::size_t p) { auto v=u16(b,p); return v<32768 ? v : int(v)-65536; }
float f32(const Bytes& b, std::size_t p) {
  const auto bits = u32(b,p);
  float value;
  static_assert(sizeof(value) == sizeof(bits), "BX72 telemetry float must be IEEE-754 binary32");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
void put(Bytes& b, std::uint64_t value, int count) {
  for (int i=count-1; i>=0; --i) b.push_back((value >> (i*8)) & 255);
}
std::uint32_t gray(std::uint32_t v) { for (int shift : {16,8,4,2,1}) v ^= v >> shift; return v; }
std::uint16_t gray16_to_binary(std::uint16_t v) {
  v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
  return v;
}
Json wrap_path(const std::vector<std::string>& path, Json value) {
  for (auto it=path.rbegin();it!=path.rend();++it) value=Json{{*it,std::move(value)}};
  return value;
}
void merge_json(Json& into, const Json& from) {
  for (auto it=from.begin();it!=from.end();++it) {
    auto key=it.key();
    if (into.contains(key) && into[key].is_object() && it.value().is_object()) merge_json(into[key],it.value());
    else if (into.contains(key) && into[key].is_array() && it.value().is_array())
      for (const auto& item : it.value()) into[key].push_back(item);
    else into[key]=it.value();
  }
}
Json telemetry_base(const std::string& bridge_id, std::uint32_t epoch, double rx, double tx) {
  return Json{{"v",1},{"type","telemetry"},{"source_protocol","BX72"},{"source_version",1},
              {"bridge_id",bridge_id},{"epoch",epoch},
              {"valid",true},{"jetson_rx_mono_s",rx},{"jetson_tx_mono_s",tx}};
}
bool envelope_fits(const std::string& bridge_id, std::uint32_t epoch, double rx, double tx,
                   const Json& common, const Json& fragment, std::size_t maximum) {
  auto data=common; merge_json(data,fragment);
  auto packet=telemetry_base(bridge_id,epoch,rx,tx);
  packet["stream"]=common.at("type"); packet["data"]=std::move(data);
  // 预留五位分片序号空间，避免打包后补充分片字段导致超限。
  packet["part"]=99999; packet["parts"]=99999;
  return packet.dump().size()<=maximum;
}
std::vector<Json> split_value(const std::string& bridge_id, std::uint32_t epoch,
                              double rx, double tx, const Json& common,
                              const std::vector<std::string>& path, const Json& value,
                              std::size_t maximum) {
  auto wrapped=wrap_path(path,value);
  if (envelope_fits(bridge_id,epoch,rx,tx,common,wrapped,maximum)) return {std::move(wrapped)};
  std::vector<Json> result;
  if (value.is_object() && !value.empty()) {
    for (auto it=value.begin();it!=value.end();++it) {
      auto child_path=path; child_path.push_back(it.key());
      auto pieces=split_value(bridge_id,epoch,rx,tx,common,child_path,it.value(),maximum);
      result.insert(result.end(),std::make_move_iterator(pieces.begin()),std::make_move_iterator(pieces.end()));
    }
    return result;
  }
  if (value.is_array() && !value.empty()) {
    std::size_t offset=0;
    while (offset<value.size()) {
      // 每个数组元素至少占一个字节；包长也是候选元素数的安全上界。
      std::size_t low=1, high=std::min(value.size()-offset,maximum), best=0;
      auto slice=[&](std::size_t count) {
        return Json(value.cbegin()+offset,value.cbegin()+offset+count);
      };
      while (low<=high) {
        const auto middle=low+(high-low)/2;
        if (envelope_fits(bridge_id,epoch,rx,tx,common,wrap_path(path,slice(middle)),maximum)) {
          best=middle; low=middle+1;
        } else high=middle-1;
      }
      if (!best) throw std::length_error("one telemetry array item exceeds UDP envelope limit");
      result.push_back(wrap_path(path,slice(best)));
      offset+=best;
    }
    return result;
  }
  if (value.is_string()) {
    // String fragments cannot be reassembled by the receiver, and slicing
    // arbitrary bytes can split a UTF-8 character. Report an oversized field.
    throw std::length_error("telemetry string cannot fit UDP envelope");
  }
  throw std::length_error("telemetry scalar cannot fit UDP envelope");
}
}
double monotonic() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
sockaddr_in endpoint(const std::string& ip, int port, bool allow_zero) {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  if (port < (allow_zero ? 0 : 1) || port > 65535 ||
      inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1)
    throw std::invalid_argument("invalid IPv4 endpoint: " + ip + ":" + std::to_string(port));
  return address;
}
std::string peer_key(const sockaddr_in& address) {
  char ip[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &address.sin_addr, ip, sizeof(ip));
  return std::string(ip) + ":" + std::to_string(ntohs(address.sin_port));
}
bool same_peer(const sockaddr_in& a, const sockaddr_in& b) {
  return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}
Udp::Udp(const std::string& ip, int port, int buffer) {
  const auto address = endpoint(ip, port, true);
  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) throw std::runtime_error(std::strerror(errno));
  if ((buffer && setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &buffer, sizeof(buffer)) < 0) ||
      bind(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0 ||
      fcntl(fd, F_SETFL, O_NONBLOCK) < 0) {
    const std::string error = std::strerror(errno);
    close(fd);
    fd = -1;
    throw std::runtime_error("UDP " + ip + ":" + std::to_string(port) + ": " + error);
  }
}
Udp::~Udp() {
  if (fd >= 0) close(fd);
}
std::optional<Datagram> Udp::receive() const {
  Datagram datagram;
  datagram.data.resize(65535);
  socklen_t size = sizeof(datagram.peer);
  const auto count = recvfrom(fd, datagram.data.data(), datagram.data.size(), 0,
                              reinterpret_cast<sockaddr*>(&datagram.peer), &size);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return {};
    throw std::runtime_error(std::strerror(errno));
  }
  datagram.data.resize(static_cast<std::size_t>(count));
  return datagram;
}
bool Udp::send(const Bytes& data, const sockaddr_in& peer) const {
  return sendto(fd, data.data(), data.size(), 0,
                reinterpret_cast<const sockaddr*>(&peer), sizeof(peer)) ==
         static_cast<ssize_t>(data.size());
}
bool Udp::send(const Json& json, const sockaddr_in& peer) const {
  const auto text = json.dump();
  return send(Bytes(text.begin(), text.end()), peer);
}
int Udp::buffer() const {
  int value = 0;
  socklen_t size = sizeof(value);
  getsockopt(fd, SOL_SOCKET, SO_RCVBUF, &value, &size);
  return value;
}
Args::Args(int argc, char** argv, std::map<std::string, std::string> defaults,
           const std::string& description) : values(std::move(defaults)) {
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    const auto equals = key.find('=');
    if (equals != std::string::npos) {
      const auto option = key.substr(0, equals);
      if (!values.count(option)) throw std::invalid_argument("unknown option: " + option);
      values[option] = key.substr(equals + 1);
      continue;
    }
    if (key == "--help" || key == "-h") {
      std::cout << description << "\n";
      for (const auto& entry : values)
        std::cout << "  " << entry.first << " (default: " << entry.second << ")\n";
      std::exit(0);
    }
    if (!values.count(key) || i + 1 >= argc)
      throw std::invalid_argument("unknown option or missing value: " + key);
    values[key] = argv[++i];
  }
}
std::string Args::get(const std::string& key) const { return values.at(key); }
int Args::integer(const std::string& key) const {
  const auto value = get(key);
  std::size_t used = 0;
  const int result = std::stoi(value, &used);
  if (used != value.size()) throw std::invalid_argument("invalid integer: " + key);
  return result;
}
double Args::number(const std::string& key) const {
  const auto value = get(key);
  std::size_t used = 0;
  const double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result))
    throw std::invalid_argument("invalid number: " + key);
  return result;
}
std::uint16_t crc16(const Bytes& data, std::size_t length) {
  std::uint16_t crc=0xffff;
  for (std::size_t i=0;i<length;++i) {
    crc ^= std::uint16_t(data.at(i)) << 8;
    for (int j=0;j<8;++j) crc = (crc & 0x8000) ? (crc<<1)^0x1021 : crc<<1;
  }
  return crc;
}
Bytes encode_probe(const ProbePacket& packet) {
  if (packet.kind!=1 && packet.kind!=2) throw std::invalid_argument("probe type must be 1/2");
  Bytes bytes{'B','X','7','2',1,packet.kind,0,32};
  put(bytes,packet.sequence,4); put(bytes,packet.token,8);
  bytes.insert(bytes.end(),packet.data.begin(),packet.data.end());
  put(bytes,crc16(bytes,bytes.size()),2);
  return bytes;
}
ProbePacket decode_probe(const Bytes& bytes) {
  if (bytes.size()!=32) throw std::invalid_argument("probe length");
  if (!std::equal(bytes.begin(),bytes.begin()+4,"BX72") || bytes[4]!=1 ||
      (bytes[5]!=1 && bytes[5]!=2) || bytes[6]!=0 || bytes[7]!=32)
    throw std::invalid_argument("probe header");
  if (crc16(bytes,30)!=u16(bytes,30)) throw std::invalid_argument("probe CRC");
  ProbePacket packet{}; packet.kind=bytes[5];
  packet.sequence=u32(bytes,8);
  packet.token=(std::uint64_t(u32(bytes,12))<<32)|u32(bytes,16);
  std::copy(bytes.begin()+20,bytes.begin()+30,packet.data.begin());
  return packet;
}
Bytes fpga_packet(std::uint32_t seq, std::uint64_t token, Velocity velocity) {
  Bytes b{'B','X','7','2',1,3}; put(b,32,2); put(b,seq,4); put(b,token,8);
  for (int v : velocity) put(b,static_cast<std::uint16_t>(v),2);
  put(b,0,4); put(b,crc16(b,b.size()),2); return b;
}
Bytes fpga_auto_packet(std::uint32_t seq, std::uint64_t token, bool enabled) {
  Bytes b{'B','X','7','2',1,5}; put(b,32,2); put(b,seq,4); put(b,token,8);
  put(b,enabled ? 1 : 0,1); for (int i=0;i<9;++i) put(b,0,1);
  put(b,crc16(b,b.size()),2); return b;
}
bool valid_ack(const Bytes& b, const Bytes& request) {
  return b.size()==32 && request.size()==32 &&
    (request[5]==1 || request[5]==3 || request[5]==5 || request[5]==7) &&
    b[5]==request[5]+1 &&
    std::equal(b.begin(),b.begin()+5,request.begin()) &&
    std::equal(b.begin()+6,b.begin()+30,request.begin()+6) && crc16(b,30)==u16(b,30);
}
namespace {
void check_task(const NavigationTask& t) {
  if (t.action<1 || t.action>3 || t.mode>3 || (t.flags&~3U) ||
      !t.task_id || !t.odom_domain || !t.boot_id || t.horizon_ms!=0)
    throw std::invalid_argument("navigation identity/action/reserved fields");
  if ((t.sent_at_us >> 32) || (t.expiry_us >> 32))
    throw std::invalid_argument("navigation timestamps must be low-32-bit microseconds");
  const auto duration = static_cast<std::uint32_t>(t.expiry_us) - static_cast<std::uint32_t>(t.sent_at_us);
  if (!duration || duration>120000000U) throw std::invalid_argument("navigation source time/expiry");
  if (t.action==1 && (t.mode<2 || t.max_speed_mm_s<1 || t.max_speed_mm_s>400 ||
      t.max_accel_mm_s2<1 || t.max_accel_mm_s2>1000 || t.tolerance_mm<1 || t.tolerance_mm>500 ||
      t.yaw_tolerance_mrad<1 || t.yaw_tolerance_mrad>1000 ||
      t.hold_yaw_q16 < -205887 || t.hold_yaw_q16>205887 ||
      t.final_yaw_q16 < -205887 || t.final_yaw_q16>205887))
    throw std::invalid_argument("navigation bounds/expiry");
}
}
Bytes encode_navigation_task(const NavigationTask& t) {
  Bytes b{1,t.action,t.mode,t.flags};
  for (auto v:{t.task_id,t.odom_domain,t.boot_id}) put(b,v,4);
  for (auto v:{t.goal_x_q16,t.goal_y_q16,t.hold_yaw_q16,t.final_yaw_q16})
    put(b,static_cast<std::uint32_t>(v),4);
  for (auto v:{t.max_speed_mm_s,t.max_accel_mm_s2,t.tolerance_mm,t.yaw_tolerance_mrad}) put(b,v,2);
  put(b,t.expiry_us,8); put(b,0,8); put(b,t.sent_at_us,8); put(b,t.horizon_ms,2); put(b,0,6);
  return b;
}
NavigationTask decode_navigation_task(const Bytes& b) {
  if (b.size()!=72 || b[0]!=1 || u64(b,48)!=0 || u16(b,66)!=0 || u32(b,68)!=0)
    throw std::invalid_argument("navigation length/version/reserved");
  NavigationTask t;
  t.action=b[1];t.mode=b[2];t.flags=b[3];t.task_id=u32(b,4);t.odom_domain=u32(b,8);t.boot_id=u32(b,12);
  t.goal_x_q16=i32(b,16);t.goal_y_q16=i32(b,20);t.hold_yaw_q16=i32(b,24);t.final_yaw_q16=i32(b,28);
  t.max_speed_mm_s=u16(b,32);t.max_accel_mm_s2=u16(b,34);t.tolerance_mm=u16(b,36);t.yaw_tolerance_mrad=u16(b,38);
  t.expiry_us=u64(b,40);t.sent_at_us=u64(b,56);t.horizon_ms=u16(b,64);
  check_task(t);return t;
}
std::vector<Bytes> fpga_task_packets(std::uint32_t first_seq,std::uint64_t token,const NavigationTask& t) {
  const auto payload=encode_navigation_task(t);
  std::vector<Bytes> result;
  for (unsigned index=0;index<8;++index) {
    Bytes b{'B','X','7','2',1,7,0,32};put(b,first_seq+index,4);put(b,token,8);
    b.push_back(static_cast<std::uint8_t>(index));
    b.insert(b.end(),payload.begin()+index*9,payload.begin()+(index+1)*9);
    put(b,crc16(b,30),2);result.push_back(std::move(b));
  }
  return result;
}
std::uint8_t navigation_task_error(const NavigationTask& t,std::uint64_t now_us,
                                  std::uint32_t domain,std::uint32_t boot,
                                  std::optional<std::uint32_t> previous) {
  if (!boot || !domain || t.boot_id!=boot || t.odom_domain!=domain || !t.task_id)
    return 1;
  const auto now32=static_cast<std::uint32_t>(now_us);
  const auto sent32=static_cast<std::uint32_t>(t.sent_at_us);
  const auto expiry32=static_cast<std::uint32_t>(t.expiry_us);
  const auto age_us=now32-sent32;
  const auto duration_us=expiry32-sent32;
  if (age_us>=0x80000000U || age_us>500000U || age_us>=duration_us)
    return 2;
  if (previous && !newer(t.task_id,*previous)) return 3;
  return 0;
}
bool newer(std::uint32_t seq, std::uint32_t previous) { auto d=seq-previous; return d>0 && d<0x80000000U; }
void PcLease::expire(double now) {
  if (armed && now-last>=0.2) { armed=false; velocity={}; auto_requested=false; }
}
void PcLease::disarm() {
  armed=false; velocity={}; auto_requested=false;
}
std::optional<PcControlFrame> parse_pc_control(const Json& msg) {
  try {
    if (!msg.is_object() || msg.at("v")!=1 || msg.at("type")!="control" || !msg.at("session").is_string()) return {};
    PcControlFrame result;
    result.session=msg.at("session").get<std::string>();
    if (msg.contains("claim_nonce")) {
      if (!msg["claim_nonce"].is_string()) return {};
      result.claim_nonce=msg["claim_nonce"].get<std::string>();
      if (result.claim_nonce.size()>64) return {};
    }
    auto integer=[](const Json& v) { return v.is_number_integer() && !v.is_boolean(); };
    if (result.session.empty() || result.session.size()>64 || !integer(msg.at("seq")) ||
        msg.at("seq")<0 || msg.at("seq")>0xffffffffULL) return {};
    result.seq=msg.at("seq").get<std::uint32_t>();
    int j=0;
    for (auto key : {"vx","vy","wz"}) {
      const auto& value=msg.at(key); int limit=j==2 ? 1000 : 400;
      if (!integer(value) || value < -limit || value > limit) return {};
      result.velocity[j++]=value.get<int>();
    }
    if (msg.contains("auto")) {
      if (!msg["auto"].is_boolean()) return {};
      result.auto_requested=msg["auto"].get<bool>();
    }
    if (result.auto_requested && result.velocity!=Velocity{}) return {};
    // Legacy clients explicitly own their zero-speed lease.
    if (msg.contains("take_control")) {
      if (!msg["take_control"].is_boolean()) return {};
      result.take_control=msg["take_control"].get<bool>();
    }
    if (msg.contains("claim")) {
      if (!msg["claim"].is_boolean()) return {};
      result.claim=msg["claim"].get<bool>();
    }
    if (result.claim && (!result.take_control || result.auto_requested || result.velocity!=Velocity{})) return {};
    if (!result.take_control && (result.auto_requested || result.velocity!=Velocity{})) return {};
    return result;
  } catch (const Json::exception&) { return {}; }
}
bool PcLease::accept(const Json& msg, const std::string& source, double now) {
  const auto parsed=parse_pc_control(msg);
  if (!parsed) { expire(now); return false; }
  return accept(*parsed,source,now);
}
bool PcLease::accept(const PcControlFrame& msg, const std::string& source, double now,
                     bool resume_retired) {
  expire(now);
  const bool moving=msg.velocity!=Velocity{};
  const bool changed=msg.session!=session || source!=peer;
  const auto retired_match=std::find_if(retired.begin(),retired.end(),[&](const auto& owner) {
    return owner.session==msg.session && owner.peer==source;
  });
  if (changed && (moving || msg.auto_requested ||
      (retired_match!=retired.end() && (!resume_retired || !newer(msg.seq,retired_match->seq))))) return false;
  if (!changed && !newer(msg.seq,seq)) return false;
  if (!changed && msg.auto_requested!=auto_requested && moving) return false;
  if (!armed && msg.auto_requested) return false;
  if (!armed && moving) return false;
  if (!session.empty() && changed) {
    auto previous=std::find_if(retired.begin(),retired.end(),[&](const auto& owner) {
      return owner.session==session && owner.peer==peer;
    });
    if (previous==retired.end()) retired.push_back(RetiredOwner{session,peer,seq,now});
    else { previous->seq=seq; previous->at=now; }
    while (retired.size()>64) retired.pop_front();
  }
  session=msg.session; peer=source; seq=msg.seq; last=now; velocity=msg.velocity;
  auto_requested=msg.auto_requested; armed=msg.take_control;
  return true;
}

bool cooperative_navigation_owner_lost(const PcLease& lease, bool owner_is_ros,
    std::uint8_t mode, bool flags_known, std::uint8_t flags,
    const std::string& owner_session, const std::string& owner_peer, double now) {
  if (owner_is_ros || mode!=2) return false;
  if (!flags_known || owner_session.empty() || owner_peer.empty()) return true;
  return (flags & 1U)==0 && (lease.session!=owner_session || lease.peer!=owner_peer ||
                             now-lease.last>=.2);
}

static std::string new_pc_claim_nonce() {
  static std::random_device random;
  static constexpr char hex[]="0123456789abcdef";
  std::string result(32,'0');
  for (std::size_t i=0;i<4;++i) {
    const auto value=random();
    for (unsigned digit=0;digit<8;++digit)
      result[i*8+digit]=hex[(value >> (28-digit*4)) & 15u];
  }
  return result;
}

const PcObserver* PcObservers::find(const sockaddr_in& peer) const {
  const auto found=std::find_if(entries_.begin(),entries_.end(),[&](const auto& observer) {
    return same_peer(observer.endpoint,peer);
  });
  return found==entries_.end() ? nullptr : &*found;
}
bool PcObservers::would_accept(const PcControlFrame& frame, const sockaddr_in& peer) const {
  const auto found=std::find_if(entries_.begin(),entries_.end(),[&](const auto& observer) {
    return same_peer(observer.endpoint,peer);
  });
  if (found==entries_.end()) return true;
  if (found->session==frame.session) return newer(frame.seq,found->seq);
  return std::find(found->retired_sessions.begin(),found->retired_sessions.end(),frame.session)==
         found->retired_sessions.end();
}
bool PcObservers::record(const PcControlFrame& frame, const sockaddr_in& peer, double now,
                         bool prefer_owner, bool* changed) {
  if (changed) *changed=false;
  auto found=std::find_if(entries_.begin(),entries_.end(),[&](const auto& observer) {
    return same_peer(observer.endpoint,peer);
  });
  if (found!=entries_.end()) {
    if (!would_accept(frame,peer)) return false;
    if (found->session!=frame.session) {
      found->retired_sessions.push_back(found->session);
      while (found->retired_sessions.size()>16) found->retired_sessions.pop_front();
      found->session=frame.session;
      found->claim_nonce=new_pc_claim_nonce();
      if (changed) *changed=true;
    }
    found->seq=frame.seq;
    found->last=now;
    return true;
  }
  if (entries_.size()>=max_observers_) {
    if (!prefer_owner) return false;
    entries_.erase(std::min_element(entries_.begin(),entries_.end(),
        [](const auto& a,const auto& b) { return a.last<b.last; }));
  }
  entries_.push_back(PcObserver{frame.session,peer,frame.seq,now,new_pc_claim_nonce(),{}});
  if (changed) *changed=true;
  return true;
}
bool PcObservers::expire(double now) {
  const auto old_size=entries_.size();
  entries_.erase(std::remove_if(entries_.begin(),entries_.end(),[now](const auto& observer) {
    return now-observer.last>timeout_s_;
  }),entries_.end());
  return entries_.size()!=old_size;
}
void PcObservers::renew_claim_nonces() {
  for (auto& observer : entries_) observer.claim_nonce=new_pc_claim_nonce();
}
PcInputResult accept_pc_input(PcLease& lease, PcObservers& observers,
                              const PcControlFrame& frame, const sockaddr_in& peer,
                              double now, bool explicit_command, bool safety_only, bool* changed) {
  if (changed) *changed=false;
  lease.expire(now);
  if (safety_only) {
    if (frame.take_control || frame.auto_requested || frame.velocity!=Velocity{}) return PcInputResult::Rejected;
    // A retired or reordered session may still request a stop; it must not
    // replace the current observer or consume its sequence watermark.
    if (observers.would_accept(frame,peer) && !observers.record(frame,peer,now,true,changed))
      return PcInputResult::Rejected;
    // A stop invalidates commands issued before it, including commands still
    // in flight from another observer with a newer sequence number.
    observers.renew_claim_nonces();
    return PcInputResult::Observer;
  }
  if (!observers.would_accept(frame,peer)) return PcInputResult::Rejected;
  const auto source=peer_key(peer);
  const bool owner=explicit_command || frame.take_control ||
                   (frame.session==lease.session && source==lease.peer);
  const bool changed_owner=owner && (frame.session!=lease.session || source!=lease.peer);
  // Only a user-initiated zero-speed claim may acquire or re-arm control.
  if (frame.take_control && (changed_owner || !lease.armed) && !frame.claim)
    return PcInputResult::Rejected;
  bool resume_retired=false;
  if (owner) {
    const auto retired=std::find_if(lease.retired.begin(),lease.retired.end(),[&](const auto& item) {
      return item.session==frame.session && item.peer==source;
    });
    const auto* observer=observers.find(peer);
    // Every active claim or task must echo the challenge of an already
    // registered observer, including the first owner and after expiry.
    const bool challenge_required = frame.take_control || explicit_command;
    if (challenge_required &&
        (!observer || observer->session!=frame.session || frame.claim_nonce.empty() ||
         frame.claim_nonce!=observer->claim_nonce)) return PcInputResult::Rejected;
    resume_retired=retired!=lease.retired.end() && observer && observer->session==frame.session &&
                   frame.claim_nonce==observer->claim_nonce && newer(frame.seq,retired->seq);
    if (!lease.accept(frame,source,now,resume_retired)) return PcInputResult::Rejected;
  }
  if (!observers.record(frame,peer,now,owner,changed)) return PcInputResult::Rejected;
  if (changed_owner && !lease.retired.empty()) observers.renew_claim_nonces();
  return owner ? PcInputResult::Owner : PcInputResult::Observer;
}

std::vector<Json> telemetry_envelopes(const std::string& bridge_id, std::uint32_t epoch,
                                      const Json& data, double rx, double tx, std::size_t max_bytes) {
  if (!data.is_object() || !data.contains("type") || !data["type"].is_string())
    throw std::invalid_argument("telemetry data has no stream type");
  if (!data.contains("sequence") || !data.contains("time_us"))
    throw std::invalid_argument("telemetry data misses source sequence/time");
  // Most IMU, wheel and N10 packets already fit one datagram. Avoid copying
  // and serializing every field just to rediscover that fact at sensor rate.
  auto whole=telemetry_base(bridge_id,epoch,rx,tx);
  whole["stream"]=data.at("type"); whole["data"]=data;
  if (whole.dump().size()<=max_bytes) return {std::move(whole)};
  Json common=Json::object(); std::vector<std::pair<std::string,Json>> other;
  for (auto it=data.begin();it!=data.end();++it) {
    const auto& key=it.key();
    bool required=key=="type" || key=="sequence" || key=="time_us" || key=="flags" || key.find("_flags")!=std::string::npos;
    if (required) common[key]=it.value(); else other.emplace_back(key,it.value());
  }
  if (!common.contains("sequence") || !common.contains("time_us"))
    throw std::invalid_argument("telemetry data misses source sequence/time");
  std::vector<Json> pieces;
  for (const auto& field : other) {
    auto field_pieces=split_value(bridge_id,epoch,rx,tx,common,{field.first},field.second,max_bytes);
    pieces.insert(pieces.end(),std::make_move_iterator(field_pieces.begin()),std::make_move_iterator(field_pieces.end()));
  }
  std::vector<Json> chunks{common};
  for (const auto& piece : pieces) {
    Json candidate=chunks.back(); merge_json(candidate,piece);
    if (envelope_fits(bridge_id,epoch,rx,tx,common,candidate,max_bytes)) chunks.back()=std::move(candidate);
    else {
      if (chunks.size()>=32) throw std::length_error("telemetry exceeds 32 parts");
      chunks.push_back(common); merge_json(chunks.back(),piece);
    }
  }
  if (chunks.size()>1 && pieces.empty()) throw std::length_error("telemetry common fields exceed UDP envelope limit");
  std::vector<Json> result; result.reserve(chunks.size());
  const auto stream=data["type"].get<std::string>();
  for (std::size_t i=0;i<chunks.size();++i) {
    auto packet=telemetry_base(bridge_id,epoch,rx,tx); packet["stream"]=stream; packet["data"]=std::move(chunks[i]);
    if (chunks.size()>1) { packet["part"]=i; packet["parts"]=chunks.size(); }
    if (packet.dump().size()>max_bytes) throw std::length_error("telemetry envelope exceeds UDP limit");
    result.push_back(std::move(packet));
  }
  return result;
}

Json bounded_status(Json status, std::size_t max_bytes) {
  if (status.dump().size() > max_bytes && status.contains("motor") && !status["motor"].is_null())
    status["motor"] = nullptr;
  if (status.dump().size() > max_bytes)
    throw std::length_error("status exceeds UDP envelope limit");
  return status;
}

bool TelemetryTracker::observe(const Json& data, double at, bool* restarted) {
  if (restarted) *restarted=false;
  try {
    auto name=data.at("type").get<std::string>();
    if (name!="imu" && name!="motor" && name!="lidar" && name!="fusion" && name!="health" && name!="navigation" && name!="local_plan" && name!="local_grid") return false;
    Stamp incoming{data.at("sequence").get<std::uint32_t>(),data.at("time_us").get<std::uint32_t>(),at,true};
    if ((name=="navigation" || name=="local_plan" || name=="local_grid") && !data.value("rejected",false) && data.contains("boot_id") &&
        data.at("boot_id").is_number_unsigned()) {
      incoming.boot_id=data.at("boot_id").get<std::uint32_t>();
      incoming.boot_known=true;
      // 复用旧 boot ID 与历史包无法区分；最近16个退役身份一律拒收。
      if (incoming.boot_id!=0 && std::find(retired_boot_ids.begin(),retired_boot_ids.end(),incoming.boot_id)!=retired_boot_ids.end())
        return false;
    }
    // epoch 确认后用源时钟与主机时间的关系隔离旧 epoch 迟到包。
    bool current_epoch_time=true;
    if (epoch_anchor.seen) {
      const double elapsed=at-epoch_anchor.received_at;
      if (!std::isfinite(elapsed) || elapsed<0) return false;
      const auto elapsed_us=static_cast<std::uint32_t>(std::fmod(elapsed*1e6,4294967296.0));
      const auto expected=epoch_anchor.time_us+elapsed_us;
      const auto delta=std::uint32_t(incoming.time_us-expected);
      const auto signed_delta=delta<0x80000000U ? std::int64_t(delta) : std::int64_t(delta)-0x100000000LL;
      // 允许批量传输及长期时钟漂移；仍不能让旧域的巨大时间跳变进入当前缓存。
      const double tolerance_s=5.0+std::min(25.0,elapsed*0.01);
      current_epoch_time=std::abs(static_cast<double>(signed_delta))<=tolerance_s*1e6;
    }
    const auto active_navigation=streams.find("navigation");
    const bool foreign_planning_boot=(name=="local_plan" || name=="local_grid") &&
        active_navigation!=streams.end() && active_navigation->second.boot_known &&
        active_navigation->second.boot_id!=0 &&
        (!incoming.boot_known || incoming.boot_id!=active_navigation->second.boot_id);
    auto& previous=streams[name];
    if (!previous.seen) {
      if (!current_epoch_time || foreign_planning_boot) return false;
      previous=incoming; reset_votes.erase(name); return true;
    }
    bool seq_forward=newer(incoming.sequence,previous.sequence);
    bool time_forward=incoming.time_us==previous.time_us || newer(incoming.time_us,previous.time_us);
    if (seq_forward && time_forward && current_epoch_time) {
      // 附属规划流只发布已确认的导航身份；新域需等待导航更新或重启确认。
      if (foreign_planning_boot) return false;
      if (name=="navigation") {
        if (!incoming.boot_known || incoming.boot_id==0) {
          // 拒绝包或零身份不应覆盖最近一次可信的导航 boot。
          incoming.boot_id=previous.boot_id;
          incoming.boot_known=previous.boot_known;
        } else if (previous.boot_known && previous.boot_id!=0 && incoming.boot_id!=previous.boot_id) {
          // 同一源时钟域内也可能更换 boot，旧身份不能作为未来重启候选。
          retired_boot_ids.push_back(previous.boot_id);
          if (retired_boot_ids.size()>16) retired_boot_ids.pop_front();
        }
      }
      previous=incoming; reset_votes.erase(name); return true;
    }
    bool seq_back=incoming.sequence!=previous.sequence && !seq_forward;
    bool time_back=incoming.time_us!=previous.time_us && !newer(incoming.time_us,previous.time_us);
    const double silence=at-previous.received_at;
    const auto time_jump=std::uint32_t(incoming.time_us-previous.time_us);
    // 源时钟数值回退也可能被模序判为向前。
    const bool wrapped_forward=incoming.time_us<previous.time_us && time_forward;
    const bool startup_jump=wrapped_forward && incoming.time_us<=60000000U &&
        static_cast<double>(time_jump)>(silence+1.0)*1e6;
    const bool large_time_back=time_back && std::uint32_t(previous.time_us-incoming.time_us)>1000000U;
    const bool changed_navigation_boot=name=="navigation" && previous.boot_known && previous.boot_id!=0 &&
        incoming.boot_known && incoming.boot_id!=0 && incoming.boot_id!=previous.boot_id;
    bool follows_new_navigation=false;
    const auto navigation=streams.find("navigation");
    const auto navigation_vote=reset_votes.find("navigation");
    if (name!="navigation" && navigation!=streams.end() && navigation->second.boot_known &&
        navigation->second.boot_id!=0 && navigation_vote!=reset_votes.end() &&
        navigation_vote->second.advancing && navigation_vote->second.latest.boot_known &&
        navigation_vote->second.latest.boot_id!=0 &&
        navigation_vote->second.latest.boot_id!=navigation->second.boot_id) {
      const auto& candidate=navigation_vote->second.latest;
      const double host_step=at-candidate.received_at;
      const auto delta=std::uint32_t(incoming.time_us-candidate.time_us);
      const auto signed_delta=delta<0x80000000U ? std::int64_t(delta) : std::int64_t(delta)-0x100000000LL;
      follows_new_navigation=host_step>=0 && host_step<=2.0 &&
          std::abs(signed_delta/1e6-host_step)<=1.0;
    }
    // 长时间断流后，模向前差可能小于主机间隔；新boot和同步的独立流共同判定。
    const bool changed_boot_wrap=wrapped_forward && (changed_navigation_boot || follows_new_navigation);
    if (seq_back && (large_time_back || startup_jump || changed_boot_wrap) && silence>=0.5) {
      for (auto it=reset_votes.begin();it!=reset_votes.end();) {
        if (at-it->second.latest.received_at>2.0) it=reset_votes.erase(it); else ++it;
      }
      auto& vote=reset_votes[name];
      if (!vote.latest.seen) vote.latest=incoming;
      else if (incoming.boot_known && (!vote.latest.boot_known || incoming.boot_id!=vote.latest.boot_id))
        vote={incoming,false};
      else if (newer(incoming.sequence,vote.latest.sequence) && newer(incoming.time_us,vote.latest.time_us)) {
        const double host_step=at-vote.latest.received_at;
        const double source_step=std::uint32_t(incoming.time_us-vote.latest.time_us)/1e6;
        if (host_step>=0 && source_step<=host_step+0.5) { vote.latest=incoming; vote.advancing=true; }
      }
      if (vote.advancing && vote.latest.sequence==incoming.sequence && vote.latest.time_us==incoming.time_us) {
        const auto old_navigation=streams.find("navigation");
        const auto new_navigation=reset_votes.find("navigation");
        const bool old_boot_known=old_navigation!=streams.end() && old_navigation->second.boot_known &&
            old_navigation->second.boot_id!=0;
        const bool nav_advancing=new_navigation!=reset_votes.end() && new_navigation->second.advancing;
        const bool new_boot_known=nav_advancing && new_navigation->second.latest.boot_known &&
            new_navigation->second.latest.boot_id!=0;
        if (old_boot_known && (!new_boot_known ||
            new_navigation->second.latest.boot_id==old_navigation->second.boot_id)) return false;
        const auto related=[](const std::string& stream) {
          return stream=="local_plan" || stream=="local_grid";
        };
        const auto matches_boot=[&](const std::string& stream,const Stamp& stamp) {
          return !related(stream) || !new_boot_known ||
              (stamp.boot_known && stamp.boot_id==new_navigation->second.latest.boot_id);
        };
        const auto aligned=[](const Stamp& a,const Stamp& b) {
          const auto delta=std::uint32_t(a.time_us-b.time_us);
          const auto signed_delta=delta<0x80000000U ? std::int64_t(delta) : std::int64_t(delta)-0x100000000LL;
          return std::abs(signed_delta/1e6-(a.received_at-b.received_at))<=1.0;
        };
        // 触发包本身必须属于新导航 boot；所有计票流之间也须两两时钟吻合。
        if (!matches_boot(name,incoming)) return false;
        std::map<std::string,Stamp> confirmed{{name,incoming}};
        if (nav_advancing && name!="navigation") {
          if (aligned(incoming,new_navigation->second.latest))
            confirmed["navigation"]=new_navigation->second.latest;
          else if (old_boot_known) return false;
        }
        // 先选择独立传感器，再加入一致的导航附属流，避免附属流阻塞恢复。
        for (int pass=0;pass<2;++pass) for (const auto& entry:reset_votes) {
          if (entry.first=="navigation" || related(entry.first)!=(pass==1) ||
              confirmed.count(entry.first) || !entry.second.advancing ||
              !matches_boot(entry.first,entry.second.latest)) continue;
          bool consistent=true;
          for (const auto& accepted:confirmed)
            if (!aligned(entry.second.latest,accepted.second)) { consistent=false; break; }
          if (consistent) confirmed[entry.first]=entry.second.latest;
        }
        unsigned groups=0;
        bool navigation_group=false;
        for (const auto& entry:confirmed) {
          const bool nav_group=entry.first=="navigation" || related(entry.first);
          if (!nav_group || !navigation_group) ++groups;
          if (nav_group) navigation_group=true;
        }
        if (groups>=2) {
          if (old_boot_known) {
            retired_boot_ids.push_back(old_navigation->second.boot_id);
            if (retired_boot_ids.size()>16) retired_boot_ids.pop_front();
          }
          ++epoch; if (!epoch) epoch=1;
          streams=std::move(confirmed); reset_votes.clear(); epoch_anchor=incoming;
          if (restarted) *restarted=true;
          return true;
        }
      }
    }
    return false;
  } catch (const Json::exception&) { return false; }
}
RtpFrames::RtpFrames(int width, int height) : size(std::size_t(width)*height*2) {
  if (width<=0 || height<=0 || width%2) throw std::invalid_argument("YUV422 needs positive dimensions and even width");
  frame.reserve(size);
}
std::optional<Bytes> RtpFrames::feed(const Bytes& b) {
  if (b.size()<14 || b.size()>1500 || b[0]!=0x80 || (b[1]&127)!=video_payload_type ||
      !std::equal(b.begin()+8,b.begin()+12,"ACG6") || (b.size()-12)%4) return {};
  ++packets; auto seq=u16(b,2); auto ts=u32(b,4); bool gap=expected && seq!=*expected;
  gaps+=gap;
  if (!stamp || ts!=*stamp) { dropped+=stamp.has_value(); stamp=ts; bad=false; frame.clear(); }
  else if (gap) bad=true;
  expected=static_cast<std::uint16_t>(seq+1);
  if (!bad) { frame.insert(frame.end(),b.begin()+12,b.end()); bad=frame.size()>size; }
  std::optional<Bytes> result;
  if (b[1]&128) {
    if (!bad && frame.size()==size) { ++complete; result=std::move(frame); frame=Bytes{}; frame.reserve(size); }
    else ++dropped;
    stamp.reset(); frame.clear();
  }
  return result;
}
void RtpFrames::expire_incomplete() {
  if (stamp) ++dropped;
  stamp.reset(); expected.reset(); frame.clear(); bad=false;
}
std::vector<Bytes> jpeg_packets(std::uint32_t id, const Bytes& jpeg) {
  std::vector<Bytes> result;
  if (jpeg.empty() || jpeg.size()>max_jpeg) return result;
  auto count=(jpeg.size()+1199)/1200;
  for (std::size_t i=0;i<count;++i) {
    Bytes b{'B','X','J','V'}; put(b,id,4); put(b,i,2); put(b,count,2);
    auto start=jpeg.begin()+i*1200;
    b.insert(b.end(),start,jpeg.begin()+std::min(jpeg.size(),(i+1)*1200)); result.push_back(std::move(b));
  }
  return result;
}
Json decode_telemetry(const Bytes& b) {
  const std::array<const char*,8> names{"imu","motor","lidar","fusion","health","navigation","local_plan","local_grid_chunk"};
  const std::array<int,8> lengths{48,104,160,120,128,84,56,224};
  if (b.size()<16 || !std::equal(b.begin(),b.begin()+4,"BX72") || b[4]!=1 || b[5]<3 || b[5]>10)
    throw std::invalid_argument("unknown BX72 telemetry");
  int kind=b[5];
  const bool map_packet=kind==8 && b.size()==160;
  if (u16(b,6)!=b.size() ||
      (!map_packet && b.size()!=std::size_t(lengths[kind-3])))
    throw std::invalid_argument("telemetry length");
  Json r={{"type",map_packet ? "map" : names[kind-3]},
          {"sequence",u32(b,8)},{"time_us",u32(b,12)}};
  if (map_packet) {
    const auto le16=[&b](std::size_t p) {
      return static_cast<std::uint16_t>(
          static_cast<std::uint16_t>(b[p]) |
          (static_cast<std::uint16_t>(b[p+1]) << 8));
    };
    if (b[16]!=0x4d || b[17]!=0x02 || b[24]!=10 || b[25]!=10 ||
        b[26]!=64 || b[27]!=0x03)
      throw std::invalid_argument("global map metadata");
    const auto chunk_index=le16(20);
    const auto chunk_count=le16(22);
    const auto robot_cell_x=le16(28);
    const auto robot_cell_y=le16(30);
    if (chunk_count!=8192 || chunk_index>=chunk_count ||
        robot_cell_x>=1024 || robot_cell_y>=1024)
      throw std::invalid_argument("global map bounds");
    r["protocol"]="occupancy_global_v1";
    r["frame_id"]="map";
    r["source"]="fpga";
    r["snapshot_sequence"]=le16(18);
    r["chunk_index"]=chunk_index;
    r["chunk_count"]=chunk_count;
    r["width"]=1024;
    r["height"]=1024;
    r["resolution_mm"]=64;
    r["flags"]=0x03;
    r["robot_cell_x"]=robot_cell_x;
    r["robot_cell_y"]=robot_cell_y;
    r["cells"]=Json::array();
    for (std::size_t i=32;i<160;++i) {
      if (b[i]&0xc0U)
        throw std::invalid_argument("global map reserved cell bits");
      r["cells"].push_back(b[i]&0x3fU);
    }
    return r;
  }
  if (kind==10) {
    const auto part_info=u32(b,92);
    const auto flags=u32(b,84);
    if ((part_info&0xfffffff8U)!=0x01320800U || flags&~31U ||
        std::uint32_t(u32(b,88)-u32(b,28))>100000U ||
        i32(b,72)<-3142 || i32(b,72)>3142)
      throw std::invalid_argument("local grid snapshot metadata");
    for (std::size_t i=96;i<b.size();++i)
      for (unsigned shift=0;shift<8;shift+=2)
        if (((b[i]>>shift)&3U)==3U)
          throw std::invalid_argument("local grid reserved cell state");
    r["protocol"]="nav_grid_chunk_v1";
    r["frame_id"]="local_grid";
    r["source"]="fpga";
    r["width"]=64; r["height"]=64; r["resolution_mm"]=50;
    r["boot_id"]=u32(b,16); r["odom_domain"]=u32(b,20);
    r["snapshot_id"]=u32(b,24); r["capture_begin_us"]=u32(b,28);
    r["grid_generation"]=u32(b,32); r["scan_generation"]=u32(b,36);
    r["origin_x_cell"]=i32(b,40); r["origin_y_cell"]=i32(b,44);
    r["complete_scan_stamp_us"]=u32(b,48); r["last_ray_stamp_us"]=u32(b,52);
    r["grid_from_odom"]={{"map_anchor_x_mm",i32(b,56)},{"map_anchor_y_mm",i32(b,60)},
      {"odom_anchor_x_mm",i32(b,64)},{"odom_anchor_y_mm",i32(b,68)},
      {"yaw_offset_mrad",i32(b,72)}};
    r["grid_frame_epoch"]=u32(b,76); r["transform_revision"]=u32(b,80);
    r["flags"]=flags; r["capture_end_us"]=u32(b,88);
    r["chunk_index"]=part_info&7U;
    r["chunk_cells"]=Bytes(b.begin()+96,b.end());
    return r;
  }
  if (kind==9) {
    const auto flags=u32(b,48);
    if (flags & ~15U) throw std::invalid_argument("local plan flags");
    r["protocol"]="nav_local_segment_v1";
    r["boot_id"]=u32(b,16); r["odom_domain"]=u32(b,20); r["task_id"]=u32(b,24);
    r["plan_stamp_us"]=u32(b,28);
    r["start_x_mm"]=i32(b,32); r["start_y_mm"]=i32(b,36);
    r["end_x_mm"]=i32(b,40); r["end_y_mm"]=i32(b,44);
    r["flags"]=flags; r["valid"]=(flags&1U)!=0;
    r["guard_stop"]=(flags&4U)!=0;
    const bool grid_preview=(flags&8U)!=0;
    r["grid_direction_preview"]=grid_preview;
    if (grid_preview) r["map_version"]=u32(b,52);
    else r["polar_generation"]=u32(b,52);
    r["frame_id"]="odom"; r["source"]="fpga";
    r["points"]=Json::array({{{"x",i32(b,32)/1000.0},{"y",i32(b,36)/1000.0}},
                              {{"x",i32(b,40)/1000.0},{"y",i32(b,44)/1000.0}}});
    return r;
  }
  if (kind==8) {
    const auto nav_flags=b[51];
    const bool velocity_only=(nav_flags&(1U<<4))!=0;
    if ((nav_flags&0x80U)==0 || (nav_flags&0x08U) ||
        (nav_flags&0x60U)==0x60U || (velocity_only && (nav_flags&0x60U)) || b[48]>9 || b[50]>3)
      throw std::invalid_argument("navigation telemetry fields");
    const bool polar_v2=(nav_flags&(1U<<6))!=0;
    const bool grid_v1=(nav_flags&(1U<<5))!=0;
    r["protocol"]=velocity_only ? "velocity_only_v1" : grid_v1 ? "nav_grid_v1" : polar_v2 ? "nav_polar_v2" : "nav_full_v1";
    r["source"]="fpga";
    r["boot_id"]=u32(b,16);r["odom_domain"]=u32(b,20);r["task_id"]=u32(b,24);
    r["last_task_id"]=u32(b,28);r["task_counter_valid"]=true;
    const auto expiry_us=u32(b,32);
    r["expiry_us"]=expiry_us;
    // 活动任务及暂时失效但仍保留的任务都有期限，回绕到0仍然有效。
    r["deadline_known"]=expiry_us!=0 ||
        ((navigation_status_active(b[48]) || b[48]==8) && (b[50]==2 || b[50]==3));
    r["now_us"]=u32(b,36);r["session_token"]=u64(b,40);
    r["status"]=b[48];r["error"]=b[49];r["mode"]=b[50];r["flags"]=nav_flags;
    r["pose_valid"]=(nav_flags&2U)!=0;
    if (velocity_only) r["guard_stop"]=(nav_flags&1U)!=0;
    r["startup_algorithm_selftest_passed"]=(nav_flags&4U)!=0;
    r["rejected"]=(b[49]&0x80U)!=0;r["rejection_reason"]=(b[49]&0x80U) ? (b[49]&0x7fU) : 0;
    r["guard_error"]=(b[49]&0xc0U)==0x40U ? (b[49]&0x3fU) : 0;
    r["vx_mm_s"]=i16(b,52);r["vy_mm_s"]=i16(b,54);r["wz_mrad_s"]=i16(b,56);
    if (grid_v1) {
      r["candidates_evaluated"]=u16(b,58);
      r["map_age_us"]=u32(b,72);
      r["map_version"]=u32(b,80);
    } else if (polar_v2) {
      r["polar_candidates_evaluated"]=u16(b,58);
      r["polar_age_us"]=u32(b,72);
      r["polar_generation"]=u32(b,80);
    } else {
      r["planner_expansions"]=u16(b,58);
      r["map_age_us"]=u32(b,72);
      r["map_version"]=u32(b,80);
    }
    r["goal_x_q16"]=i32(b,60);r["goal_y_q16"]=i32(b,64);
    r["progress"]=u32(b,68);r["pose_age_us"]=u32(b,76);
    return r;
  }
  const std::array<const char*,4> wheels{"A","B","C","D"};
  if (kind==3) {
    if (u32(b,40)) throw std::invalid_argument("IMU reserved field");
    r["flags"]=b[16]; r["who_am_i"]=b[17]; r["sample_sequence"]=u16(b,18);
    int index=0;
    for (auto axis : {"x","y","z"}) {
      int v=i16(b,20+index*2); r["gyro_raw"][axis]=v; r["accel_raw"][axis]=i16(b,26+index*2);
      r["gyro_rad_s_nominal"][axis]=v*.0175*std::acos(-1)/180; ++index;
    }
    r["samples"]=u32(b,32); r["identity_errors"]=u32(b,36); r["queue_dropped"]=u32(b,44);
  } else if (kind==4) {
    int index=16;
    for (auto key : {"motor_channel_map","encoder_channel_map","motor_invert","encoder_invert"}) r[key]=b[index++];
    // 按 Unicode 码点编码命令字节，兼容现有 JSON 字段的 0..255 取值。
    r["command"]=b[20]<128 ? std::string(1,char(b[20])) : std::string{char(0xc0|(b[20]>>6)),char(0x80|(b[20]&63))};
    r["pwm_percent"]=b[21]; r["active_physical_mask"]=b[22]; r["encoder_invalid_mask"]=b[23];
    for (int i=0;i<4;++i) {
      r["count_ticks"][wheels[i]]=i32(b,24+i*4);
      r["physical_count_ticks"]["MOTOR"+std::to_string(i+1)]=i32(b,40+i*4);
      r["wheel_mm_s"][wheels[i]]=i32(b,56+i*4); r["target_mm_s"][wheels[i]]=i32(b,72+i*4);
    }
    index=88; for (auto key : {"vx_mm_s","vy_mm_s","wz_mrad_s"}) { r["odometry"][key]=i32(b,index); index+=4; }
    r["queue_dropped"]=u32(b,100);
  } else if (kind==5) {
    int index=16; for (auto key : {"packets","crc_errors","length_errors","queue_dropped"}) { r[key]=u32(b,index); index+=4; }
    r["points"]=Json::array();
    for (int i=0;i<16;++i) {
      int p=32+i*8;
      if (b[p+5]!=i || u16(b,p+6) || u16(b,p)>=36000) throw std::invalid_argument("lidar point index/angle");
      r["points"].push_back({{"angle_cdeg",u16(b,p)},{"distance_mm",u16(b,p+2)},{"strength",b[p+4]}});
    }
  } else if (kind==6) {
    const auto flags=u32(b,72);
    if ((flags&0xc0000000U)!=0xc0000000U)
      throw std::invalid_argument("fusion telemetry layout marker");
    const std::array<const char*,9> state_keys{
      "x_m","y_m","yaw_rad","vx_m_s","vy_m_s","wz_rad_s",
      "gyro_bias_rad_s","roll_rad","pitch_rad"};
    r["state_mm_mrad"]=Json::array();
    for (std::size_t i=0;i<state_keys.size();++i) {
      const auto value=i32(b,16+i*4);
      r["state_mm_mrad"].push_back(value);
      r["state"][state_keys[i]]=static_cast<double>(value)/1000.0;
    }
    r["roll_mrad"]=i32(b,44);r["pitch_mrad"]=i32(b,48);
    r["roll_rad"]=static_cast<double>(i32(b,44))/1000.0;
    r["pitch_rad"]=static_cast<double>(i32(b,48))/1000.0;
    const bool estimate_layout=(flags&(1U<<14))!=0;
    const bool velocity_only=(flags&(1U<<13))!=0;
    const bool grid_v1=estimate_layout && (flags&(1U<<15))!=0;
    if (velocity_only && (!estimate_layout || grid_v1))
      throw std::invalid_argument("velocity-only fusion layout");
    const bool polar_v2=estimate_layout && !grid_v1 && !velocity_only;
    if (estimate_layout) {
      Json estimate=Json::array();
      for (std::size_t i=0;i<3;++i) {
        const auto value=f32(b,52+i*4);
        if (!std::isfinite(value)) throw std::invalid_argument("polar error estimate is not finite");
        estimate.push_back(static_cast<double>(value));
      }
      r["architecture"]=velocity_only ? "velocity_only_v1" : grid_v1 ? "local_grid_v1" : "polar_v2";
      r["polar_navigation_architecture"]=polar_v2;
      r["error_estimate"]=std::move(estimate);
      r["error_estimate_units"]={{"x","mm"},{"y","mm"},{"yaw","mrad"}};
      r["ros_odometry_covariance_model"]=velocity_only ? "velocity_error_estimate_squared_heuristic" : grid_v1 ? "local_grid_error_estimate_squared_heuristic" : "polar_error_estimate_squared_heuristic";
      r["lidar_raw_samples"]=u32(b,64);
      r["lidar_reduced_samples"]=u32(b,68);
    } else {
      r["polar_navigation_architecture"]=false;
      r["covariance_diag"]=Json::array();
      const std::array<double,5> covariance_scale{1e-6,1e-6,1.0,1e-6,1e-6};
      for (std::size_t i=0;i<5;++i) {
        const auto value=f32(b,52+i*4);
        if (!std::isfinite(value)) throw std::invalid_argument("fusion covariance is not finite");
        r["covariance_diag"].push_back(static_cast<double>(value)*covariance_scale[i]);
      }
    }
    r["flags"]=flags;r["pose_valid"]=(flags&1U)!=0;
    r["complete_navigation_architecture"]=(flags&(1U<<30))!=0;
    r["startup_algorithm_selftest_passed"]=(flags&(1U<<28))!=0;
    r["startup_selftest_failure_mask"]=(flags>>16)&0xfffU;
    r["updates"]=u32(b,76);r["fault_code"]=u32(b,80);
    r["max_control_gap_us"]=u32(b,84);r["pose_stamp_us"]=u32(b,88);
    r["lidar_drops"]=u32(b,92);r["radar_errors"]=u32(b,96);
    r["imu_age_us"]=u32(b,104);r["radar_age_us"]=u32(b,108);
    r["nav_status"]=u32(b,112);
    const auto nav_status=u32(b,112);
    if (velocity_only) {
      r["scan_match_accepted"]=u32(b,100);
      r["scan_match_rejected"]=u32(b,116);
      r["software_motion_stop"]=(nav_status&(1U<<31))!=0;
    } else if (grid_v1) {
      if (nav_status&0x60000000U)
        throw std::invalid_argument("grid navigation diagnostic reserved bits");
      r["map_update_mode"]="local_grid";
      r["scan_match_accepted"]=u32(b,100);
      r["scan_match_rejected"]=u32(b,116);
      r["map_fresh"]=(nav_status&(1U<<8))!=0;
      r["map_ready"]=(nav_status&(1U<<9))!=0;
      r["map_gap_latched"]=(nav_status&(1U<<10))!=0;
      r["candidate_planning"]=(nav_status&(1U<<11))!=0;
      r["progress_blocked"]=(nav_status&(1U<<12))!=0;
      r["unknown_path"]=(nav_status&(1U<<13))!=0;
      r["map_recovery_required"]=(nav_status&(1U<<14))!=0;
      r["grid_v1_diagnostic"]=(nav_status&(1U<<15))!=0;
      const bool map_fifo_count=(nav_status&(1U<<20))!=0;
      const bool shared_reduced=(nav_status&(1U<<22))!=0;
      r["lidar_drops_scope"]=shared_reduced ? "shared_reduced" : map_fifo_count ? "map_raw" : "csm_reduced";
      r["map_input_mode"]=shared_reduced ? "shared_nearest_5deg" : "raw_fifo";
      r["map_lidar_drops"]=map_fifo_count ? Json(u32(b,92)) : Json(nullptr);
      const auto gap_reason=(nav_status>>16)&15U;
      static constexpr std::array<const char*,8> gap_names{
        "none","fifo_overflow","upstream_error","sample_stale",
        "pose_unavailable","angle_discontinuity","invalid_sample","coordinate_reset"};
      r["map_gap_reason"]=gap_reason;
      r["map_gap_reason_name"]=gap_reason<gap_names.size() ? gap_names[gap_reason] : "unknown";
      r["start_confirmation_lost"]=(nav_status&(1U<<21))!=0;
      r["software_motion_stop"]=(nav_status&(1U<<31))!=0;
    } else if (polar_v2) {
      if (nav_status&0x60008000U)
        throw std::invalid_argument("polar navigation diagnostic reserved bits");
      r["map_update_mode"]="polar_gap";
      r["scan_match_accepted"]=u32(b,100);
      r["scan_match_rejected"]=u32(b,116);
      r["polar_scan_fresh"]=(nav_status&(1U<<8))!=0;
      r["polar_complete_coverage"]=(nav_status&(1U<<9))!=0;
      r["polar_gap_latched"]=(nav_status&(1U<<10))!=0;
      r["boundary_following"]=(nav_status&(1U<<11))!=0;
      r["progress_timeout_or_loop"]=(nav_status&(1U<<12))!=0;
      r["unknown_in_swept_corridor"]=(nav_status&(1U<<13))!=0;
      r["polar_recovery_required"]=(nav_status&(1U<<14))!=0;
      r["software_motion_stop"]=(nav_status&(1U<<31))!=0;
    } else {
      r["csm_accepted"]=u32(b,100);r["csm_rejected"]=u32(b,116);
      r["map_shift_pending"]=(nav_status&(1U<<29))!=0;
      const bool map_raster=(flags&(1U<<15))!=0;
      r["map_update_mode"]=map_raster ? "visibility_raster" : "ray_queue";
      if(map_raster) {
        r["map_raster_row"]=(nav_status>>23)&0x3fU;
        r["map_input_gaps"]=(nav_status>>8)&0x7fffU;
        r["map_input_gap_latched"]=(nav_status&(1U<<30))!=0;
        r["map_ray_overflow"]=nullptr;
        r["map_ray_queue_depth"]=nullptr;
        r["map_ray_drops"]=nullptr;
      } else {
        r["map_ray_overflow"]=(nav_status&(1U<<30))!=0;
        r["map_ray_queue_depth"]=(nav_status>>23)&0x3fU;
        r["map_ray_drops"]=(nav_status>>8)&0x7fffU;
        r["map_raster_row"]=nullptr;
        r["map_input_gaps"]=nullptr;
        r["map_input_gap_latched"]=nullptr;
      }
    }
  } else {
    std::vector<std::string> fields{"video_flags","camera_frames","ethernet_packets","video_packets","video_errors","video_dropped_frames","video_skipped_frames"};
    for (int i=0;i<10;++i) fields.push_back("ddr_gray_"+std::to_string(i));
    for (auto key : {"radar_uart_bytes","radar_framing_errors","radar_packets","radar_crc_errors","radar_length_errors","radar_points","imu_flags","imu_who_am_i","imu_samples","imu_identity_errors","queue_dropped"}) fields.push_back(key);
    for (std::size_t i=0;i<fields.size();++i) r[fields[i]]=u32(b,16+i*4);
    // 当前单软核版末字是带4752签名的CPU状态，不再是旧版队列丢弃数。
    const auto cpu_health=u32(b,124);
    const bool cpu_layout=(cpu_health>>16)==0x4752U;
    r["cpu_health_valid"]=cpu_layout;
    if(cpu_layout) {
      // 位2声明新的相机诊断布局；无能力标记的旧格式仍严格拒绝非零保留位。
      const bool camera_failure_supported=(cpu_health&4U)!=0;
      const auto camera_failure=(cpu_health>>12)&15U;
      if(!camera_failure_supported && camera_failure)
        throw std::invalid_argument("reserved CPU health bits");
      r["camera_guard_failure_diagnostics_valid"]=camera_failure_supported;
      if(camera_failure_supported) {
        r["camera_guard_failure_sticky"]={
          {"source_incomplete",(camera_failure&1U)!=0},
          {"source_complete_pixel_geometry",(camera_failure&2U)!=0},
          {"end_timeout",(camera_failure&4U)!=0},
          {"no_fe_timeout",(camera_failure&8U)!=0}};
      }
      r["cpu_health_word"]=cpu_health;r["queue_dropped"]=nullptr;
      r["cpu_ready"]=(cpu_health&1U)!=0;r["cpu_fault"]=(cpu_health&2U)!=0;
      r["cpu_busy"]=(cpu_health&8U)!=0;r["cpu_fault_code"]=(cpu_health>>4)&0xffU;
      // 当前签名布局保留了相机校准状态；初始化完成不等于采样眼图已就绪。
      const auto calibration=u32(b,32);
      r["camera_calibration"]={{"state",(calibration>>28)&15U},
          {"ready",(calibration&(1U<<27))!=0}, {"fault",(calibration&(1U<<26))!=0},
          {"scan_row",(calibration>>21)&31U}, {"scan_col",(calibration>>16)&31U},
          {"verified_bursts",(calibration>>3)&8191U}};
      const auto video_flags=u32(b,16);
      r["video_rejected_mod256"]={{"camera_bad",(video_flags>>8)&255U},
          {"pixel_count",(video_flags>>16)&255U}, {"fifo_full",(video_flags>>24)&255U}};
    }
    const auto camera_diag=u32(b,72);
    if ((camera_diag>>28)==0xCU) {
      static const std::array<const char*,7> reasons{
        "unknown_or_none","no_eye","bad_csi_burst","bad_raw_eof",
        "hs_burst_timeout","good_eof_timeout","tap_load_timeout"};
      const auto reason=(camera_diag>>24)&0xFU;
      r["ddr_gray_7"]=nullptr;
      r["ddr_gray_8"]=nullptr;
      r["camera_autocal_diag_valid"]=true;
      r["camera_autocal_diag"]={{"last_failure_reason",reason},
          {"reason_name",reason<reasons.size() ? reasons[reason] : "reserved"},
          {"tap0",(camera_diag>>16)&0xffU}, {"tap1",(camera_diag>>8)&0xffU},
          {"retry_count",camera_diag&0xffU}};
      const auto guard=u32(b,76);
      r["camera_guard_frame_counts"]={{"bad_frames",gray16_to_binary(static_cast<std::uint16_t>(guard>>16))},
          {"good_frames",gray16_to_binary(static_cast<std::uint16_t>(guard))}};
    }
    int mask=1; for (auto key : {"pll_locked","camera_initialized","ddr_calibrated","camera_frame_seen","capture_enabled"}) { r["video_ready"][key]=(u32(b,16)&mask)!=0; mask<<=1; }
    r["ddr_counters"]=Json::array();
    for (int i=0;i<10;++i) {
      if ((i==7 || i==8) && r.value("camera_autocal_diag_valid",false))
        r["ddr_counters"].push_back(nullptr);
      else r["ddr_counters"].push_back(gray(u32(b,44+i*4)));
    }
  }
  return r;
}

void LocalGridAssembler::reset() {
  header_.reset();
  context_.reset();
  seen_=0; invalid_=false; completed_=false;
}

void LocalGridAssembler::set_domain(std::uint32_t boot, std::uint32_t domain) {
  const auto context=(std::uint64_t(boot)<<32)|domain;
  if (context_ && *context_==context) return;
  reset(); context_=context;
}

std::optional<Json> LocalGridAssembler::feed(const Json& chunk, double at) {
  const auto context=(std::uint64_t(chunk.at("boot_id").get<std::uint32_t>())<<32)|
      chunk.at("odom_domain").get<std::uint32_t>();
  if (!context_ || *context_!=context) return std::nullopt;
  auto header=chunk;
  for (const auto* key : {"sequence","time_us","chunk_index","chunk_cells"}) header.erase(key);
  const auto id=chunk.at("snapshot_id").get<std::uint32_t>();
  if (!header_ || id!=header_->at("snapshot_id").get<std::uint32_t>()) {
    if (header_ && !newer(id,header_->at("snapshot_id").get<std::uint32_t>())) return std::nullopt;
    header_=header; seen_=0; invalid_=false; completed_=false; started_at_=at;
  }
  // 重复片、冲突片及后续迟到片都不能重新开始250ms接收期限。
  if (at-started_at_>0.25 || header!=*header_) invalid_=true;
  if (invalid_ || completed_) return std::nullopt;
  const auto index=chunk.at("chunk_index").get<unsigned>();
  auto cells=chunk.at("chunk_cells").get<Bytes>();
  const auto bit=static_cast<std::uint8_t>(1U<<index);
  if (seen_&bit) {
    if (chunks_[index]!=cells) invalid_=true;
    return std::nullopt;
  }
  chunks_[index]=std::move(cells); seen_|=bit;
  if (seen_!=255U) return std::nullopt;
  auto result=*header_;
  result["type"]="local_grid"; result["protocol"]="nav_grid_snapshot_v1";
  result["sequence"]=id; result["time_us"]=result.at("capture_end_us");
  Bytes packed; packed.reserve(1024);
  for (const auto& part : chunks_) packed.insert(packed.end(),part.begin(),part.end());
  result["packed_cells"]=std::move(packed);
  completed_=true;
  return result;
}
}  // namespace bx72
