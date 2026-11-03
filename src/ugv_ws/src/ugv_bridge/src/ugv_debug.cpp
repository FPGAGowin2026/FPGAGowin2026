#include "ugv_bridge/bx72_protocol.hpp"
#include "ugv_bridge/video.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <poll.h>
#include <random>
#include <stdexcept>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace bx72 {
namespace fs = std::filesystem;
namespace {

volatile std::sig_atomic_t stopping = 0;

void stop_signal(int) { stopping = 1; }

void signals() {
  std::signal(SIGINT, stop_signal);
  std::signal(SIGTERM, stop_signal);
}

double wall_time() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void atomic_write(const fs::path& path, const char* data, std::size_t size) {
  auto temporary = path;
  temporary += ".tmp";
  {
    std::ofstream file(temporary, std::ios::binary);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file.write(data, size);
  }
  fs::rename(temporary, path);
}

void atomic_json(const fs::path& path, const Json& value) {
  const auto data = value.dump(2);
  atomic_write(path, data.data(), data.size());
}

class ProbeClient {
  std::mt19937_64 random_{std::random_device{}()};
 public:
  Udp socket;
  sockaddr_in peer;
  double timeout;
  Bytes pending;
  double pending_at = 0;
  std::uint32_t sequence = 0;
  std::uint64_t sent = 0, received = 0, timeouts = 0, invalid = 0, send_errors = 0;
  std::optional<double> last_ack, last_rtt_ms;

  ProbeClient(const std::string& fpga_ip, const std::string& bind_ip, int control_port,
              int reply_port, double timeout_s)
      : socket(bind_ip, reply_port), peer(endpoint(fpga_ip, control_port)), timeout(timeout_s) {
    if (timeout <= 0) throw std::invalid_argument("reply timeout must be positive");
  }

  void poll() {
    for (int i = 0; i < 64; ++i) {
      auto datagram = socket.receive();
      if (!datagram) break;
      try {
        decode_probe(datagram->data);
      } catch (const std::invalid_argument&) {
        ++invalid;
        continue;
      }
      const auto now = monotonic();
      if (!same_peer(datagram->peer, peer) || pending.empty() || datagram->data[5] != 2 ||
          now - pending_at > timeout ||
          !std::equal(datagram->data.begin() + 8, datagram->data.begin() + 30, pending.begin() + 8)) {
        ++invalid;
        continue;
      }
      ++received;
      last_ack = now;
      last_rtt_ms = (now - pending_at) * 1000;
      pending.clear();
    }
    if (!pending.empty() && monotonic() - pending_at > timeout) {
      ++timeouts;
      pending.clear();
    }
  }

  void send() {
    if (!pending.empty()) return;
    ProbePacket packet{};
    packet.kind = 1;
    packet.sequence = sequence;
    packet.token = random_();
    for (auto& value : packet.data) value = static_cast<std::uint8_t>(random_() & 255);
    auto data = encode_probe(packet);
    if (!socket.send(data, peer)) {
      ++send_errors;
      return;
    }
    pending = std::move(data);
    pending_at = monotonic();
    ++sequence;
    ++sent;
  }

  Json snapshot() const {
    return {{"tx", sent}, {"ack", received}, {"timeouts", timeouts},
            {"invalid", invalid}, {"send_errors", send_errors}};
  }
  std::string summary() const {
    std::ostringstream out;
    out << "TX=" << sent << " ACK=" << received << " timeouts=" << timeouts
        << " invalid=" << invalid << " send_errors=" << send_errors << " RTT=";
    if (last_rtt_ms) out << std::fixed << std::setprecision(2) << *last_rtt_ms;
    else out << '-';
    out << " ms";
    return out.str();
  }
};

class CheckVideo {
  RtpFrames frames_;
  std::uint64_t invalid_ = 0, status_ = 0;
 public:
  std::optional<double> last_packet;

  CheckVideo(int width, int height) : frames_(width, height) {}

  std::optional<Bytes> feed(const Bytes& data, double now) {
    if (data.size() == 32 && std::equal(data.begin(), data.begin() + 4, "ACG6") &&
        data[4] == 1 && data[5] == 1) {
      ++status_;
      return {};
    }
    // RtpFrames counts accepted RTP packets only; count invalid datagrams here and status packets separately.
    const auto packets = frames_.packets;
    auto complete = frames_.feed(data);
    if (frames_.packets == packets) {
      ++invalid_;
    } else {
      last_packet = now;
    }
    return complete;
  }

  Json snapshot() const {
    return {{"frames", frames_.complete}, {"dropped", frames_.dropped},
            {"gaps", frames_.gaps}, {"invalid", invalid_}, {"status", status_}};
  }
};

Json difference(const Json& end, const Json& start) {
  Json result = Json::object();
  for (auto it = start.begin(); it != start.end(); ++it)
    result[it.key()] = end.at(it.key()).get<std::uint64_t>() - it.value().get<std::uint64_t>();
  return result;
}

Json assess(const Json& probe, const Json& video, double elapsed, double minimum_fps) {
  Json reasons = Json::array();
  if (probe.at("tx") == 0 || probe.at("ack") != probe.at("tx"))
    reasons.push_back("请求与匹配应答数量不一致或没有请求");
  if (probe.at("timeouts") != 0 || probe.at("invalid") != 0 || probe.at("send_errors") != 0)
    reasons.push_back("稳定阶段出现超时、无效应答或发送失败");
  const double fps = video.at("frames").get<double>() / std::max(elapsed, .001);
  if (video.at("frames") == 0 || fps < minimum_fps) reasons.push_back("完整视频帧率不足");
  if (video.at("dropped") != 0 || video.at("gaps") != 0 || video.at("invalid") != 0)
    reasons.push_back("稳定阶段出现不完整视频帧、序号缺口或无效包");
  return {{"passed", reasons.empty()}, {"probe", probe}, {"video", video},
          {"seconds", elapsed}, {"complete_fps", fps}, {"minimum_fps", minimum_fps},
          {"reasons", reasons}};
}

void validate_port(int port, const std::string& option) {
  if (port < 1 || port > 65535) throw std::invalid_argument(option + " must be in 1..65535");
}

int run_probe(int argc, char** argv) {
  Args args(argc, argv,
      {{"--fpga-ip", "192.168.0.2"}, {"--bind-ip", "0.0.0.0"}, {"--control-port", "5000"},
       {"--reply-port", "6103"}, {"--duration", "30"}, {"--rate", "10"}, {"--timeout", "0.3"}},
      "Usage: ugv_debug probe [options]\nSend BX72 diagnostic requests and verify matching replies. No motion command is sent.");
  const double duration = args.number("--duration");
  const double rate = args.number("--rate");
  const double timeout = args.number("--timeout");
  if (duration <= 0 || rate <= 0 || timeout <= 0)
    throw std::invalid_argument("duration, rate, and timeout must be positive");
  validate_port(args.integer("--control-port"), "--control-port");
  validate_port(args.integer("--reply-port"), "--reply-port");

  signals();
  ProbeClient client(args.get("--fpga-ip"), args.get("--bind-ip"), args.integer("--control-port"),
                     args.integer("--reply-port"), timeout);
  const auto started = monotonic();
  double next = started, report = started;
  while (!stopping && monotonic() - started < duration) {
    client.poll();
    const auto now = monotonic();
    if (now >= next) {
      client.send();
      next = now + 1 / rate;
    }
    if (now - report >= 1) {
      std::cout << client.summary() << std::endl;
      report = now;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(std::min(.005, 1 / rate)));
  }
  while (!stopping && !client.pending.empty()) {
    client.poll();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << client.summary() << std::endl;
  const bool passed = !stopping && client.received > 0 && client.received == client.sent && !client.send_errors;
  std::cout << (passed ? "PASS: all requests acknowledged" : "FAIL: diagnostic probe incomplete") << std::endl;
  return passed ? 0 : 1;
}

int run_check(int argc, char** argv) {
  Args args(argc, argv,
      {{"--mode", "coexist"}, {"--bind-ip", "192.168.0.4"}, {"--fpga-ip", "192.168.0.2"},
       {"--control-port", "5000"}, {"--video-port", "6102"}, {"--reply-port", "6103"},
       {"--duration", "30"}, {"--minimum-fps", "1"}, {"--output", ""}},
      "Usage: ugv_debug check [--mode coexist|recovery] [options]\nCheck full video frames and matching BX72 replies. Binds exclusive UDP video/reply ports; recovery requires operator cable disconnect/reconnect.");
  const auto mode = args.get("--mode");
  const double duration = args.number("--duration");
  const double minimum = args.number("--minimum-fps");
  const int video_port = args.integer("--video-port");
  const int reply_port = args.integer("--reply-port");
  const int control_port = args.integer("--control-port");
  if ((mode != "coexist" && mode != "recovery") || duration <= 0 || minimum <= 0 ||
      (mode == "recovery" && duration < 15))
    throw std::invalid_argument("invalid mode/duration/minimum-fps (recovery duration must be at least 15 seconds)");
  validate_port(video_port, "--video-port");
  validate_port(reply_port, "--reply-port");
  validate_port(control_port, "--control-port");
  if (video_port == reply_port) throw std::invalid_argument("--video-port and --reply-port must differ");

  signals();
  ProbeClient client(args.get("--fpga-ip"), args.get("--bind-ip"), control_port, reply_port, .3);
  Udp video_socket(args.get("--bind-ip"), video_port, 8 * 1024 * 1024);
  CheckVideo video(video_width, video_height);
  const double started = monotonic();
  double next_send = started, report = started, stage_at = started, stable_at = 0, baseline_fps = 0;
  std::string stage = "baseline";
  Json video_start = video.snapshot(), probe_start = client.snapshot(), result = nullptr;
  double outage_at = 0;
  std::uint64_t outage_frames = 0;
  bool outage_active = false, disconnected = false, sending = false;
  std::cout << "先采集 5 秒视频基线；独占 UDP " << video_port << '/' << reply_port << "。" << std::endl;
  try {
    while (result.is_null() && !stopping) {
      pollfd descriptors[] = {{video_socket.fd, POLLIN, 0}, {client.socket.fd, POLLIN, 0}};
      if (::poll(descriptors, 2, 5) < 0 && errno != EINTR)
        throw std::runtime_error(std::strerror(errno));
      for (int i = 0; i < 256; ++i) {
        auto datagram = video_socket.receive();
        if (!datagram) break;
        if (same_peer(datagram->peer, client.peer)) video.feed(datagram->data, monotonic());
      }
      client.poll();
      const auto now = monotonic();
      if (stage == "baseline" && now - stage_at >= 5) {
        baseline_fps = video.snapshot().at("frames").get<double>() / (now - stage_at);
        if (baseline_fps < minimum) {
          result = {{"passed", false}, {"reasons", {"视频基线没有足够完整帧"}}, {"baseline_fps", baseline_fps}};
          continue;
        }
        stage = mode == "coexist" ? "coexist" : "warmup";
        stage_at = now;
        video_start = video.snapshot();
        probe_start = client.snapshot();
        next_send = now;
        sending = true;
        std::cout << "基线=" << baseline_fps << " fps; 开始 " << mode << std::endl;
      }
      if (stage == "warmup" && now - stage_at >= 3) {
        auto probe = difference(client.snapshot(), probe_start);
        if (!client.pending.empty()) probe["tx"] = probe["tx"].get<std::uint64_t>() - 1;
        auto warmup = assess(probe, difference(video.snapshot(), video_start), now - stage_at, minimum);
        if (!warmup["passed"].get<bool>()) {
          result = std::move(warmup);
          continue;
        }
        stage = "await_disconnect";
        std::cout << "初始视频和应答正常。现在拔开 FPGA 网线，等确认断链后再接回。" << std::endl;
      }
      const bool ack_fresh = client.last_ack && now - *client.last_ack < 1;
      const bool video_fresh = video.last_packet && now - *video.last_packet < 1;
      if (stage == "await_disconnect") {
        if (!ack_fresh && !video_fresh) {
          if (!outage_active) {
            outage_at = now;
            outage_active = true;
          } else if (now - outage_at >= 2) {
            outage_frames = video.snapshot().at("frames").get<std::uint64_t>();
            disconnected = true;
            stage = "await_reconnect";
            std::cout << "已确认断链。请接回网线；必要时重新设置静态邻居。" << std::endl;
          }
        } else {
          outage_active = false;
        }
      }
      if (stage == "await_reconnect" && ack_fresh && video_fresh &&
          video.snapshot().at("frames").get<std::uint64_t>() > outage_frames) {
        stage = "settle";
        stable_at = now;
        std::cout << "恢复后的应答和完整帧已收到，等待 2 秒。" << std::endl;
      }
      if (stage == "settle" && now - stable_at >= 2) {
        if (!ack_fresh || !video_fresh) {
          result = {{"passed", false}, {"reasons", {"恢复后再次失联"}}};
          continue;
        }
        sending = false;
        if (client.pending.empty()) {
          stage = "stable";
          stage_at = now;
          video_start = video.snapshot();
          probe_start = client.snapshot();
          next_send = now;
          sending = true;
        }
      }
      const bool finished = (stage == "coexist" && now - stage_at >= duration) ||
                            (stage == "stable" && now - stage_at >= 5);
      if (finished) {
        sending = false;
        if (client.pending.empty()) {
          result = assess(difference(client.snapshot(), probe_start),
                          difference(video.snapshot(), video_start), now - stage_at,
                          std::max(minimum, baseline_fps * .85));
          continue;
        }
      }
      if (mode == "recovery" && now - started >= duration && stage != "baseline") {
        result = {{"passed", false}, {"reasons", {"恢复测试超时或未观察到完整断开/恢复过程"}}, {"stage", stage}};
        continue;
      }
      if (sending && now >= next_send) {
        client.send();
        next_send = now + .1;
      }
      if (now - report >= 1) {
        const auto stats = video.snapshot();
        std::cout << "阶段=" << stage << " 完整帧=" << stats["frames"] << " 丢弃=" << stats["dropped"]
                  << " 缺口=" << stats["gaps"] << ' ' << client.summary() << std::endl;
        report = now;
      }
    }
  } catch (const std::exception& e) {
    result = {{"passed", false}, {"reasons", {e.what()}}, {"stage", stage}};
  }
  if (result.is_null()) result = {{"passed", false}, {"reasons", {"用户提前结束测试"}}, {"stage", stage}};
  result.update({{"mode", mode}, {"baseline_fps", baseline_fps}, {"disconnect_observed", disconnected},
                 {"recovery_stable_window_completed", stage == "stable" && monotonic() - stage_at >= 5},
                 {"total_probe", client.snapshot()}, {"total_video", video.snapshot()},
                 {"measurement_source", "runtime UDP sockets"}, {"fpga_ip", args.get("--fpga-ip")},
                 {"bind_ip", args.get("--bind-ip")},
                 {"physical_disconnect_procedure", "operator unplugs/replugs cable in recovery mode"}});
  if (!args.get("--output").empty()) {
    const fs::path path(args.get("--output"));
    if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
    atomic_json(path, result);
  }
  std::cout << result.dump() << std::endl;
  const bool passed = result["passed"].get<bool>();
  std::cout << (passed ? "PASS: 实板验收通过" : "FAIL: 实板验收未通过") << std::endl;
  return passed ? 0 : 1;
}

struct Frame {
  std::uint32_t stamp;
  Bytes raw;
};

class VideoWriter {
  fs::path output_;
  int width_, height_;
  std::string pixel_format_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::optional<Frame> job_;
  bool done_ = false, first_ = true;
  std::string error_;
  std::thread worker_;

  static void write_bytes(const fs::path& path, const Bytes& bytes) {
    atomic_write(path, reinterpret_cast<const char*>(bytes.data()), bytes.size());
  }

  void run() {
    while (true) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [this] { return done_ || job_.has_value(); });
        if (!job_) return;
        frame = std::move(*job_);
        job_.reset();
      }
      try {
        const auto bgr = yuv422_to_bgr(frame.raw, pixel_format_);
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        const auto header = "P6\n" + std::to_string(width_) + " " + std::to_string(height_) + "\n255\n";
        Bytes ppm(header.begin(), header.end());
        const auto rgb_bytes = rgb.total() * rgb.elemSize();
        ppm.reserve(ppm.size() + rgb_bytes);
        ppm.insert(ppm.end(), rgb.data, rgb.data + rgb_bytes);
        if (first_) {
          write_bytes(output_ / "first_frame.yuv422", frame.raw);
          write_bytes(output_ / "first_frame.ppm", ppm);
          first_ = false;
        }
        write_bytes(output_ / "latest_frame.yuv422", frame.raw);
        write_bytes(output_ / "latest_frame.ppm", ppm);
        atomic_json(output_ / "latest_frame.json",
                    {{"timestamp_90khz", frame.stamp}, {"width", width_}, {"height", height_},
                     {"format", "YUV422"}, {"pixel_format", pixel_format_},
                     {"color_space", "BT.709 limited-range"},
                     {"raw_file", "latest_frame.yuv422"}, {"ppm_file", "latest_frame.ppm"},
                     {"host_time", wall_time()}});
      } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = e.what();
      }
    }
  }

 public:
  VideoWriter(fs::path path, int width, int height, std::string pixel_format)
      : output_(std::move(path)), width_(width), height_(height),
        pixel_format_(std::move(pixel_format)), worker_(&VideoWriter::run, this) {}
  ~VideoWriter() { finish(); }

  bool submit(Frame frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (job_ || done_) return false;
    job_ = std::move(frame);
    wake_.notify_one();
    return true;
  }

  void finish() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      done_ = true;
      wake_.notify_one();
    }
    if (worker_.joinable()) worker_.join();
  }

  Json last_error() {
    std::lock_guard<std::mutex> lock(mutex_);
    return error_.empty() ? Json(nullptr) : Json(error_);
  }
};

void run_record(const Args& args) {
  const int width = args.integer("--width"), height = args.integer("--height");
  const auto pixel_format = args.get("--pixel-format");
  if (width != video_width || height != video_height) {
    throw std::invalid_argument("record currently supports only the BX72 640x360 YUV422 contract");
  }
  if (pixel_format != "yuyv" && pixel_format != "uyvy") {
    throw std::invalid_argument("--pixel-format must be yuyv or uyvy");
  }
  const fs::path output(args.get("--output"));
  fs::create_directories(output);
  const auto source = endpoint(args.get("--source"), 1);
  const int video_port = args.integer("--video-port"), telemetry_port = args.integer("--telemetry-port");
  Udp video(args.get("--bind"), video_port, 16 * 1024 * 1024);
  Udp telemetry(args.get("--bind"), telemetry_port, 16 * 1024 * 1024);
  RtpFrames assembler(width, height);
  std::uint64_t invalid_video = 0;
  VideoWriter writer(output, width, height, pixel_format);
  std::ofstream records(output / "telemetry.jsonl", std::ios::app);
  std::ofstream frames(output / "video_frames.jsonl", std::ios::app);
  std::ofstream raw_packets(output / "telemetry_packets.bin", std::ios::binary | std::ios::app);
  records.exceptions(std::ios::failbit | std::ios::badbit);
  frames.exceptions(std::ios::failbit | std::ios::badbit);
  raw_packets.exceptions(std::ios::failbit | std::ios::badbit);
  std::map<std::string, std::uint64_t> counts, gaps;
  std::map<std::string, std::uint32_t> previous;
  Json latest = Json::object();
  const double started = monotonic();
  double report = started, last_image = started - 2;

  auto summary = [&](bool running) {
    const double elapsed = monotonic() - started;
    return Json{{"host_time", wall_time()}, {"elapsed_seconds", elapsed}, {"source", args.get("--source")},
                {"ports", {{"video", video_port}, {"telemetry", telemetry_port}}},
                {"socket_receive_buffers", {{"video", video.buffer()}, {"telemetry", telemetry.buffer()}}},
                {"telemetry", counts}, {"sequence_gaps", gaps},
                {"video", {{"format", "YUV422"}, {"pixel_format", pixel_format},
                            {"color_space", "BT.709 limited-range"},
                            {"width", width}, {"height", height},
                            {"packets", assembler.packets}, {"complete_frames", assembler.complete},
                            {"discarded_frames", assembler.dropped}, {"sequence_gaps", assembler.gaps},
                            {"invalid_packets", invalid_video}}},
                {"average_video_fps", assembler.complete / std::max(elapsed, .001)}, {"latest", latest},
                {"image_writer_error", writer.last_error()}, {"running", running}};
  };
  std::cout << "UGV debug recorder: UDP " << video_port << '/' << telemetry_port << "; " << output << std::endl;
  std::exception_ptr error;
  try {
    while (!stopping && (args.number("--duration") == 0 || monotonic() - started < args.number("--duration"))) {
      pollfd descriptors[] = {{telemetry.fd, POLLIN, 0}, {video.fd, POLLIN, 0}};
      if (::poll(descriptors, 2, 100) < 0 && errno != EINTR)
        throw std::runtime_error(std::strerror(errno));
      for (int index = 0; index < 2; ++index) {
        if (!(descriptors[index].revents & POLLIN)) continue;
        for (int batch = 0; batch < 512; ++batch) {
          auto datagram = (index == 0 ? telemetry : video).receive();
          if (!datagram) break;
          if (datagram->peer.sin_addr.s_addr != source.sin_addr.s_addr) {
            ++counts["unexpected_source"];
            continue;
          }
          if (index == 0) {
            // Preserve source-filtered telemetry even when the decoder rejects
            // a packet, so a protocol bug does not erase its own raw evidence.
            const auto size = static_cast<std::uint32_t>(datagram->data.size());
            const char prefix[] = {static_cast<char>(size >> 24), static_cast<char>(size >> 16),
                                   static_cast<char>(size >> 8), static_cast<char>(size)};
            raw_packets.write(prefix, 4);
            raw_packets.write(reinterpret_cast<const char*>(datagram->data.data()), datagram->data.size());
          }
          try {
            if (index == 0) {
              auto record = decode_telemetry(datagram->data);
              const auto name = record["type"].get<std::string>();
              const auto sequence = record["sequence"].get<std::uint32_t>();
              if (previous.count(name) && sequence != std::uint32_t(previous[name] + 1)) ++gaps[name];
              previous[name] = sequence;
              ++counts[name];
              if (name == "lidar") counts["lidar_points"] += record["points"].size();
              record["host_time"] = wall_time();
              records << record.dump() << '\n';
              latest[name] = std::move(record);
            } else if (datagram->data.size() == 32 &&
                       std::equal(datagram->data.begin(), datagram->data.begin() + 4, "ACG6")) {
              ++counts["legacy_status"];
            } else {
              // RtpFrames has no invalid counter, so an unchanged accepted-packet count marks bad input.
              const auto packets = assembler.packets;
              auto complete = assembler.feed(datagram->data);
              if (assembler.packets == packets) {
                ++invalid_video;
                if (++counts["decode_errors"] <= 5) std::cerr << "decode: unknown video format\n";
              }
              if (complete) {
                const auto& data = datagram->data;
                const auto stamp = (std::uint32_t(data[4]) << 24) | (std::uint32_t(data[5]) << 16) |
                                   (std::uint32_t(data[6]) << 8) | data[7];
                frames << Json({{"timestamp_90khz", stamp}, {"bytes", complete->size()},
                                {"width", width}, {"height", height},
                                {"format", "YUV422"}, {"pixel_format", pixel_format},
                                {"color_space", "BT.709 limited-range"},
                                {"host_time", wall_time()}}).dump() << '\n';
                if (monotonic() - last_image >= 1 && writer.submit({stamp, std::move(*complete)}))
                  last_image = monotonic();
              }
            }
          } catch (const std::invalid_argument& e) {
            if (++counts["decode_errors"] <= 5) std::cerr << "decode: " << e.what() << '\n';
          }
        }
      }
      if (monotonic() - report >= 5) {
        records.flush();
        frames.flush();
        raw_packets.flush();
        auto info = summary(true);
        atomic_json(output / "summary.json", info);
        atomic_json(output / "latest.json", latest);
        std::cout << Json({{"elapsed", info["elapsed_seconds"]}, {"telemetry", counts},
                           {"video", info["video"]}, {"gaps", gaps}}).dump() << std::endl;
        report = monotonic();
      }
    }
  } catch (...) {
    error = std::current_exception();
  }
  records.close();
  frames.close();
  raw_packets.close();
  writer.finish();
  auto info = summary(false);
  atomic_json(output / "summary.json", info);
  atomic_json(output / "latest.json", latest);
  std::cout << info.dump() << std::endl;
  if (error) std::rethrow_exception(error);
}

}  // namespace
}  // namespace bx72

int main(int argc, char** argv) {
  if (argc < 2 || std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h") {
    std::cout << "Usage: ugv_debug <probe|check|record> [options]\n"
                 "  probe   Send BX72 diagnostic requests; no motion commands are sent.\n"
                 "  check   Check video and diagnostic replies; optionally run coexist/recovery acceptance.\n"
                 "  record  Save filtered telemetry, raw packets, and YUV422 video frame evidence.\n"
                 "Use 'ugv_debug <command> --help' for command options and defaults.\n";
    return 0;
  }
  try {
    const std::string command(argv[1]);
    if (command == "probe") return bx72::run_probe(argc - 1, argv + 1);
    if (command == "check") return bx72::run_check(argc - 1, argv + 1);
    if (command == "record") {
      bx72::Args args(argc - 1, argv + 1,
          {{"--bind", "0.0.0.0"}, {"--source", "192.168.0.2"}, {"--video-port", "6102"},
           {"--telemetry-port", "6104"}, {"--width", std::to_string(bx72::video_width)},
           {"--height", std::to_string(bx72::video_height)},
           {"--pixel-format", "yuyv"},
           {"--duration", "0"}, {"--output", std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/bx72-telemetry/data"}},
          "Usage: ugv_debug record [options]\nRecord source-filtered telemetry and video evidence. Writes telemetry.jsonl, latest.json, summary.json, telemetry_packets.bin, video_frames.jsonl, first/latest YUV422 and PPM frames. --pixel-format accepts yuyv or uyvy.");
      const int video_port = args.integer("--video-port"), telemetry_port = args.integer("--telemetry-port");
      if (args.number("--duration") < 0 || args.integer("--width") <= 0 || args.integer("--height") <= 0)
        throw std::invalid_argument("duration must be nonnegative and dimensions must be positive");
      bx72::validate_port(video_port, "--video-port");
      bx72::validate_port(telemetry_port, "--telemetry-port");
      if (video_port == telemetry_port) throw std::invalid_argument("--video-port and --telemetry-port must differ");
      bx72::signals();
      bx72::run_record(args);
      return 0;
    }
    throw std::invalid_argument("unknown command: " + command + " (use --help)");
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
