#include "Vnav_fp_gate_top.h"
#include "verilated.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <deque>

struct LidarSample {
    uint16_t angle_cdeg;
    uint16_t distance_mm;
    uint8_t strength;
    uint32_t stamp_us;
};

struct TaskFragment {
    uint32_t words[8];
};

static void put_u16_be(uint8_t *bytes, unsigned offset, uint16_t value)
{
    bytes[offset] = static_cast<uint8_t>(value >> 8);
    bytes[offset + 1u] = static_cast<uint8_t>(value);
}

static void put_u32_be(uint8_t *bytes, unsigned offset, uint32_t value)
{
    bytes[offset] = static_cast<uint8_t>(value >> 24);
    bytes[offset + 1u] = static_cast<uint8_t>(value >> 16);
    bytes[offset + 2u] = static_cast<uint8_t>(value >> 8);
    bytes[offset + 3u] = static_cast<uint8_t>(value);
}

static void put_u64_be(uint8_t *bytes, unsigned offset, uint64_t value)
{
    put_u32_be(bytes, offset, static_cast<uint32_t>(value >> 32));
    put_u32_be(bytes, offset + 4u, static_cast<uint32_t>(value));
}

static uint16_t crc16_ccitt(const uint8_t *bytes, unsigned length)
{
    uint16_t crc = 0xffffu;
    for (unsigned i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (unsigned bit = 0; bit < 8u; ++bit)
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

// CPU周期为25ns；以皮秒整数推进，采集时标用分数周期累加到微秒。
static const uint64_t CPU_CLOCK_HZ = 40000000u;
static const uint32_t CPU_CYCLE_PS = 25000u;
static const uint32_t US_PERIOD_PS = 1000000u;
static const uint64_t MAX_COMMIT_GAP_CYCLES = CPU_CLOCK_HZ * 30000u / 1000000u;
static const uint64_t MIN_COMMIT_GAP_CYCLES =
    (CPU_CLOCK_HZ * 18000u + 999999u) / 1000000u;
static uint64_t sim_time_ps = 0;
static uint32_t us_phase_ps = 0;
double sc_time_stamp()
{
    return static_cast<double>(sim_time_ps) / 1000.0;
}

struct TestEnvironment {
    uint32_t now_us = 1;
    uint32_t snapshot_us = 1;
    uint32_t snapshot_status = 0x00000017u;
    uint32_t imu_seq = 1;
    uint32_t wheel_seq = 1;
    uint32_t imu_stamp_us = 1;
    uint32_t wheel_stamp_us = 1;
    uint32_t snapshot_imu_seq = 1;
    uint32_t snapshot_wheel_seq = 1;
    uint32_t snapshot_imu_stamp_us = 1;
    uint32_t snapshot_wheel_stamp_us = 1;
    uint32_t next_imu_us = 8334;
    uint32_t next_wheel_us = 20001;
    bool sensors_frozen = false;
    bool navigation_scene = false;
    bool lidar_started = false;
    bool task_schedule_started = false;
    uint32_t next_lidar_us = 0;
    uint32_t lidar_sample_index = 0;
    uint32_t lidar_popped = 0;
    uint32_t lidar_drops = 0;
    uint32_t lidar_errors = 0;
    uint32_t lidar_last_stamp_us = 0;
    bool cpu_autonomous = false;
    std::deque<LidarSample> lidar_queue;
    std::deque<TaskFragment> task_queue;
    bool task_queue_overflow = false;
    uint8_t task_schedule_index = 0;
    uint32_t next_task_fragment_us = 0;
    uint32_t task_fragments_enqueued = 0;
    uint32_t task_fragments_popped = 0;
    uint8_t task_payload[72] = {0};
    uint32_t nav_boot_id = 0x13572469u;
    uint32_t nav_domain = 1u;
    uint64_t nav_token = 0x0102030405060708ull;

    static uint16_t wall_distance(uint16_t angle_cdeg)
    {
        const double radians = static_cast<double>(angle_cdeg) *
                               (3.14159265358979323846 / 18000.0);
        const double dx = std::cos(radians);
        const double dy = std::sin(radians);
        const double tx = std::fabs(dx) < 1.0e-9 ? 1.0e9 : 1200.0 / std::fabs(dx);
        const double ty = std::fabs(dy) < 1.0e-9 ? 1.0e9 : 1200.0 / std::fabs(dy);
        const double range = tx < ty ? tx : ty;
        return static_cast<uint16_t>(range + 0.5);
    }

    void start_lidar_scene()
    {
        lidar_started = true;
        next_lidar_us = now_us + 167u;
        lidar_sample_index = 0u;
    }

    void begin_navigation_task()
    {
        task_schedule_started = true;
        task_schedule_index = 0u;
        next_task_fragment_us = now_us + 1u;

        for (unsigned i = 0; i < 72u; ++i) task_payload[i] = 0u;
        task_payload[0] = 1u;                       // 版本
        task_payload[1] = 1u;                       // 新建任务
        task_payload[2] = 3u;                       // 自主模式
        task_payload[3] = 1u;                       // 链路断开时允许本地任务继续
        put_u32_be(task_payload, 4u, 1u);            // task_id
        put_u32_be(task_payload, 8u, nav_domain);
        put_u32_be(task_payload, 12u, nav_boot_id);
        put_u32_be(task_payload, 16u, 32768u);       // 0.5m，Q16米
        put_u16_be(task_payload, 32u, 200u);          // 最大速度mm/s
        put_u16_be(task_payload, 34u, 500u);          // 最大加速度mm/s^2
        put_u16_be(task_payload, 36u, 100u);          // 到达容差mm
        put_u16_be(task_payload, 38u, 100u);          // 航向容差mrad
        put_u64_be(task_payload, 40u,
                   static_cast<uint64_t>(now_us) + 5000000ull);
        put_u64_be(task_payload, 56u, now_us);         // 低32位发送采集时刻
    }

    void enqueue_lidar_sample()
    {
        const uint16_t angle = static_cast<uint16_t>((lidar_sample_index % 600u) * 60u);
        ++lidar_sample_index;
        lidar_last_stamp_us = now_us;
        if (lidar_queue.size() >= 256u) {
            ++lidar_drops;
            return;
        }
        LidarSample sample = {angle, wall_distance(angle), 200u, now_us};
        lidar_queue.push_back(sample);
    }

    void enqueue_task_fragment()
    {
        uint8_t frame[32] = {0};
        const uint8_t index = task_schedule_index;

        frame[0] = 'B'; frame[1] = 'X'; frame[2] = '7'; frame[3] = '2';
        frame[4] = 1u;
        frame[5] = 8u;                              // 已校验任务分片ACK形式
        frame[6] = 0u; frame[7] = 32u;
        put_u32_be(frame, 8u, 100u + index);
        put_u64_be(frame, 12u, nav_token);
        frame[20] = index;
        for (unsigned i = 0; i < 9u; ++i)
            frame[21u + i] = task_payload[static_cast<unsigned>(index) * 9u + i];
        put_u16_be(frame, 30u, crc16_ccitt(frame, 30u));

        if (task_queue.size() >= 16u) {
            task_queue_overflow = true;
        } else {
            TaskFragment fragment = {{0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u}};
            for (unsigned word = 0; word < 8u; ++word) {
                const unsigned offset = word * 4u;
                fragment.words[word] = (static_cast<uint32_t>(frame[offset]) << 24) |
                    (static_cast<uint32_t>(frame[offset + 1u]) << 16) |
                    (static_cast<uint32_t>(frame[offset + 2u]) << 8) |
                    static_cast<uint32_t>(frame[offset + 3u]);
            }
            task_queue.push_back(fragment);
        }
        ++task_fragments_enqueued;
        ++task_schedule_index;
        next_task_fragment_us += 1000u;
    }

    void pop_lidar()
    {
        if (!lidar_queue.empty()) {
            lidar_queue.pop_front();
            ++lidar_popped;
        }
    }

    void pop_task()
    {
        if (!task_queue.empty()) {
            task_queue.pop_front();
            ++task_fragments_popped;
        }
    }

    uint32_t read(uint16_t address) const
    {
        if (address >= 0x164u && address <= 0x180u && (address & 3u) == 0u) {
            const unsigned word = (address - 0x164u) >> 2;
            return task_queue.empty() ? 0u : task_queue.front().words[word];
        }
        switch (address) {
        case 0x100: return snapshot_us;
        case 0x104: return snapshot_status;
        case 0x108: return now_us;
        case 0x110: return snapshot_imu_seq;
        case 0x114: return 0u;                    // gx高半字、gy低半字
        case 0x118: return 0u;                    // gz高半字、ax低半字
        case 0x11c: return 0x00002000u;            // 静止1g，±4g量程
        case 0x120: return snapshot_wheel_seq;
        case 0x124: case 0x128: case 0x12c: return 0u;
        case 0x130: return snapshot_wheel_stamp_us;
        case 0x134: return snapshot_imu_stamp_us;
        case 0x140:
            return lidar_queue.empty() ? 0u :
                (static_cast<uint32_t>(lidar_queue.front().angle_cdeg) << 16) |
                 lidar_queue.front().distance_mm;
        case 0x144:
            return lidar_queue.empty() ? 0u :
                (static_cast<uint32_t>(lidar_queue.front().strength) << 8) | 3u;
        case 0x14c: return lidar_queue.empty() ? 0u : lidar_queue.front().stamp_us;
        case 0x150: return lidar_drops;
        case 0x154: return lidar_errors;
        case 0x160: return (task_queue.empty() ? 0u : 1u) |
                            (task_queue_overflow ? 2u : 0u);
        case 0x190: return 0u;
        case 0x194: case 0x198: case 0x19c: return 0u;
        case 0x1a0: return navigation_scene ? nav_boot_id : 0u;
        case 0x1a4: return 1u;
        case 0x1a8: return navigation_scene ? static_cast<uint32_t>(nav_token) : 0u;
        case 0x1ac: return navigation_scene ? static_cast<uint32_t>(nav_token >> 32) : 0u;
        case 0x280: return 0u;                    // 电机硬件锁保持关闭
        default: return 0u;
        }
    }

    void snapshot()
    {
        snapshot_us = now_us;
        if (navigation_scene) {
            snapshot_status = 0x00000013u;         // LINK_DOWN、IMU、WHEEL有效
            if (lidar_started &&
                static_cast<uint32_t>(now_us - lidar_last_stamp_us) <= 100000u)
                snapshot_status |= 1u << 2;        // 雷达最近采样有效
            if (!task_queue.empty()) snapshot_status |= 1u << 5;
            if (!lidar_queue.empty()) snapshot_status |= 1u << 6;
            if (cpu_autonomous) snapshot_status |= 1u << 7;
        } else {
            snapshot_status = 0x00000017u;         // 静止基础场景沿用健康状态
        }
        snapshot_imu_seq = imu_seq;
        snapshot_wheel_seq = wheel_seq;
        snapshot_imu_stamp_us = imu_stamp_us;
        snapshot_wheel_stamp_us = wheel_stamp_us;
    }

    void advance_one_us()
    {
        ++now_us;
        if (!sensors_frozen) {
            if (now_us >= next_imu_us) {
                ++imu_seq;
                imu_stamp_us = now_us;
                next_imu_us += 8333u;               // IMU 120Hz，间隔约8.333ms
            }
            if (now_us >= next_wheel_us) {
                ++wheel_seq;
                wheel_stamp_us = now_us;
                next_wheel_us += 20000u;             // 轮速采样50Hz
            }
            if (navigation_scene && lidar_started && now_us >= next_lidar_us) {
                enqueue_lidar_sample();
                next_lidar_us += 167u;               // 每16点约2672us，约6000点/s
            }
        }
        if (navigation_scene && task_schedule_started && task_schedule_index < 8u &&
            now_us >= next_task_fragment_us)
            enqueue_task_fragment();
    }
};

struct Observations {
    uint64_t cycles = 0;
    uint64_t ready_cycle = 0;
    uint32_t ready_time_us = 0;
    uint32_t last_heartbeat = 0;
    uint32_t heartbeat_changes = 0;
    uint32_t control_commits = 0;
    uint32_t valid_pose_commits = 0;
    uint32_t invalid_pose_commits = 0;
    uint32_t diagnostic_words = 0;
    uint64_t last_commit_cycle = 0;
    uint64_t max_commit_gap_cycles = 0;
    uint64_t last_ready_commit_cycle = 0;
    uint64_t min_ready_commit_gap_cycles = 0;
    uint32_t ready_control_commits = 0;
    int32_t out_vx = 0;
    int32_t out_vy = 0;
    int32_t out_wz = 0;
    int32_t pose_roll = 0;
    int32_t pose_pitch = 0;
    int32_t pose_yaw = 0;
    uint32_t pose_flags = 0;
    uint32_t max_control_gap_us = 0;
    uint32_t firmware_selftest = 0;
    uint32_t csm_accepted = 0;
    uint32_t nav_status = 0;
    uint32_t nav_task_id = 0;
    uint32_t planner_expansions = 0;
    uint32_t output_autonomous = 0;
    uint32_t nav_commits = 0;
    uint32_t active_task_commits = 0;
    uint32_t forward_request_commits = 0;
    uint32_t zero_after_stale_commits = 0;
    uint32_t covariance_diag[7] = {0, 0, 0, 0, 0, 0, 0};
};

static void fail(const char *reason, const Observations &obs,
                 const TestEnvironment &env, const Vnav_fp_gate_top &top)
{
    std::fprintf(stderr,
        "FAIL nav_fp_gate cycle=%llu now_us=%u ready=%u fault=%u code=%u heartbeat=%u lidar_q=%u lidar_drop=%u csm=%u nav_status=%u task=%u expansions=%u autonomous=%u vx=%d gap_us=%u reason=%s\n",
        static_cast<unsigned long long>(obs.cycles), env.now_us,
        static_cast<unsigned>(top.cpu_ready_o), static_cast<unsigned>(top.cpu_fault_o),
        static_cast<unsigned>(top.fault_code_o), static_cast<unsigned>(top.heartbeat_o),
        static_cast<unsigned>(env.lidar_queue.size()), env.lidar_drops,
        obs.csm_accepted, obs.nav_status, obs.nav_task_id,
        obs.planner_expansions, obs.output_autonomous, obs.out_vx,
        obs.max_control_gap_us, reason);
    std::fflush(stderr);
    std::exit(1);
}

static int32_t signed_word(uint32_t value)
{
    return static_cast<int32_t>(value);
}

static int32_t abs_i32(int32_t value)
{
    if (value == (-2147483647 - 1)) return 2147483647;
    return value < 0 ? -value : value;
}

static void capture_write(uint16_t address, uint32_t data, uint8_t strobe,
                          TestEnvironment &env, Observations &obs,
                          Vnav_fp_gate_top &top)
{
    if (strobe != 0x0fu) return;
    switch (address) {
    case 0x108:
        env.snapshot();
        break;
    case 0x148:
        env.pop_lidar();
        break;
    case 0x184:
        env.pop_task();
        break;
    case 0x280:
        fail("固件不得写电机ARM只读寄存器", obs, env, top);
        break;
    case 0x200: obs.out_vx = signed_word(data); break;
    case 0x204: obs.out_vy = signed_word(data); break;
    case 0x208: obs.out_wz = signed_word(data); break;
    case 0x218: obs.pose_yaw = signed_word(data); break;
    case 0x21c: obs.pose_roll = signed_word(data); break;
    case 0x220: obs.pose_pitch = signed_word(data); break;
    case 0x224: obs.pose_flags = data; break;
    case 0x20c:
        if (data == 0x42583732u) {
            if (!env.navigation_scene &&
                (obs.out_vx != 0 || obs.out_vy != 0 || obs.out_wz != 0))
                fail("链路断开时CPU提交了非零速度", obs, env, top);
            if (env.navigation_scene &&
                (abs_i32(obs.out_vx) > 300 || abs_i32(obs.out_vy) > 300 ||
                 abs_i32(obs.out_wz) > 300))
                fail("导航场景输出速度超过安全限幅", obs, env, top);
            if (env.navigation_scene && obs.output_autonomous != 0u &&
                (abs_i32(obs.out_vx) > 200 || abs_i32(obs.out_vy) > 200 ||
                 abs_i32(obs.out_wz) > 300))
                fail("自主任务速度超过任务限幅", obs, env, top);
            ++obs.control_commits;
            if (obs.last_commit_cycle != 0) {
                const uint64_t gap = obs.cycles - obs.last_commit_cycle;
                if (gap > obs.max_commit_gap_cycles) obs.max_commit_gap_cycles = gap;
            }
            obs.last_commit_cycle = obs.cycles;
            if (top.cpu_ready_o) {
                if (obs.last_ready_commit_cycle != 0u) {
                    const uint64_t gap = obs.cycles - obs.last_ready_commit_cycle;
                    if (obs.min_ready_commit_gap_cycles == 0u ||
                        gap < obs.min_ready_commit_gap_cycles)
                        obs.min_ready_commit_gap_cycles = gap;
                }
                obs.last_ready_commit_cycle = obs.cycles;
                ++obs.ready_control_commits;
            }
            if (env.navigation_scene && obs.out_vx > 0 &&
                obs.out_vx <= 200 && abs_i32(obs.out_vy) <= 200 &&
                abs_i32(obs.out_wz) <= 300 && obs.output_autonomous != 0u)
                ++obs.forward_request_commits;
            if (env.navigation_scene && env.sensors_frozen &&
                obs.invalid_pose_commits != 0u && obs.out_vx == 0 &&
                obs.out_vy == 0 && obs.out_wz == 0)
                ++obs.zero_after_stale_commits;
        }
        break;
    case 0x22c:
        if (data == 0x42583732u) {
            if ((obs.pose_flags & 1u) != 0u)
                ++obs.valid_pose_commits;
            else if (env.sensors_frozen)
                ++obs.invalid_pose_commits;
        }
        break;
    case 0x26c: obs.max_control_gap_us = data; break;
    case 0x230: obs.nav_status = data & 0x7fffffffu; break;
    case 0x234: obs.nav_task_id = data; break;
    case 0x270: obs.csm_accepted = data; break;
    case 0x27c: obs.firmware_selftest = data; break;
    case 0x294: obs.planner_expansions = data; break;
    case 0x2a0:
        obs.output_autonomous = data;
        env.cpu_autonomous = data != 0u;
        break;
    case 0x23c:
        if (data == 0x42583732u) {
            ++obs.nav_commits;
            if (obs.nav_task_id == 1u && obs.output_autonomous != 0u &&
                (obs.nav_status == 2u || obs.nav_status == 3u))
                ++obs.active_task_commits;
        }
        break;
    default:
        if (address >= 0x240u && address <= 0x258u && (address & 3u) == 0u) {
            const unsigned index = (address - 0x240u) >> 2;
            obs.covariance_diag[index] = data;
            ++obs.diagnostic_words;
        }
        break;
    }
}

static void evaluate_half_cycle(Vnav_fp_gate_top &top, TestEnvironment &env,
                                bool high)
{
    top.clk = high ? 1u : 0u;
    top.io_ready_i = 1u;
    top.io_rdata_i = env.read(static_cast<uint16_t>(top.io_addr_o));
    top.eval();
    // 读地址为组合输出；再驱动一次可保证采样沿前rdata稳定。
    top.io_rdata_i = env.read(static_cast<uint16_t>(top.io_addr_o));
    top.eval();
}

static void reset_cycle(Vnav_fp_gate_top &top)
{
    top.clk = 0u;
    top.io_ready_i = 1u;
    top.io_rdata_i = 0u;
    top.eval();
    sim_time_ps += CPU_CYCLE_PS / 2u;
    top.clk = 1u;
    top.eval();
    sim_time_ps += CPU_CYCLE_PS / 2u;
    top.clk = 0u;
    top.eval();
}

static void run_cycle(Vnav_fp_gate_top &top, TestEnvironment &env,
                      Observations &obs)
{
    // 下降沿先稳定总线与rdata，记录将在随后的上升沿完成的MMIO事务。
    evaluate_half_cycle(top, env, false);
    const bool write_fire = top.io_valid_o && top.io_write_o &&
                            top.io_ready_i && top.io_wstrb_o == 0x0fu;
    const uint16_t address = static_cast<uint16_t>(top.io_addr_o);
    const uint32_t data = top.io_wdata_o;
    const uint8_t strobe = static_cast<uint8_t>(top.io_wstrb_o);

    sim_time_ps += CPU_CYCLE_PS / 2u;
    top.clk = 1u;
    top.eval();
    ++obs.cycles;
    if (write_fire) capture_write(address, data, strobe, env, obs, top);
    us_phase_ps += CPU_CYCLE_PS;
    if (us_phase_ps >= US_PERIOD_PS) {
        us_phase_ps -= US_PERIOD_PS;
        env.advance_one_us();
    }

    sim_time_ps += CPU_CYCLE_PS / 2u;
    top.clk = 0u;
    top.eval();
    if (top.heartbeat_o != obs.last_heartbeat) {
        obs.last_heartbeat = top.heartbeat_o;
        ++obs.heartbeat_changes;
    }
}

static bool positive_finite_float(uint32_t bits)
{
    return bits != 0u && (bits & 0x80000000u) == 0u &&
           (bits & 0x7f800000u) != 0x7f800000u;
}

int main(int argc, char **argv)
{
    bool navigation_scene = false;
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "--navigation-scene") == 0)
            navigation_scene = true;

    Verilated::commandArgs(argc, argv);
    Vnav_fp_gate_top top;
    TestEnvironment env;
    Observations obs;
    env.navigation_scene = navigation_scene;
    bool ready_seen = false;
    bool navigation_scene_succeeded = false;
    uint32_t navigation_success_us = 0u;
    const uint64_t startup_timeout_cycles = (CPU_CLOCK_HZ * 3u + 1u) / 2u; // 1.5秒
    const uint64_t total_timeout_cycles = navigation_scene ?
        CPU_CLOCK_HZ * 7u : CPU_CLOCK_HZ * 2u;           // 导航场景最多7秒仿真时间

    top.clk = 0u;
    top.rst = 1u;
    top.gsri = 0u;
    top.io_ready_i = 1u;
    top.io_rdata_i = 0u;
    for (unsigned i = 0; i < 8u; ++i) reset_cycle(top);
    top.gsri = 1u;
    for (unsigned i = 0; i < 2u; ++i) reset_cycle(top);
    top.rst = 0u;
    top.eval();

    while (obs.cycles < total_timeout_cycles) {
        run_cycle(top, env, obs);

        if (top.cpu_fault_o)
            fail("真实门级CPU置位sticky fault", obs, env, top);

        if (!ready_seen && top.cpu_ready_o) {
            ready_seen = true;
            obs.ready_cycle = obs.cycles;
            obs.ready_time_us = env.now_us;
            if (navigation_scene) env.start_lidar_scene();
        }
        if (!ready_seen && obs.cycles >= startup_timeout_cycles)
            fail("GoRV32 FP未在启动预算内通过固件自检并ready", obs, env, top);

        if (!navigation_scene && ready_seen && !env.sensors_frozen &&
            static_cast<uint32_t>(env.now_us - obs.ready_time_us) >= 300000u)
            env.sensors_frozen = true;

        if (!navigation_scene && ready_seen &&
            static_cast<uint32_t>(env.now_us - obs.ready_time_us) >= 300000u) {
            if (obs.firmware_selftest != 0x80000000u)
                fail("固件自检状态不是成功值", obs, env, top);
            if (obs.control_commits < 10u || obs.valid_pose_commits == 0u)
                fail("没有持续的50Hz控制提交和有效姿态", obs, env, top);
            if (obs.heartbeat_changes == 0u)
                fail("CPU ready后没有心跳进展", obs, env, top);
            if (obs.pose_roll < -30 || obs.pose_roll > 30 ||
                obs.pose_pitch < -30 || obs.pose_pitch > 30 ||
                obs.pose_yaw < -30 || obs.pose_yaw > 30)
                fail("静止IMU姿态超出零姿态容限", obs, env, top);
            if (obs.diagnostic_words < 7u)
                fail("没有发布7个EKF协方差对角", obs, env, top);
            for (unsigned i = 0; i < 7u; ++i) {
                if (!positive_finite_float(obs.covariance_diag[i]))
                    fail("EKF协方差对角为零、负数或非有限值", obs, env, top);
            }
            if (obs.max_control_gap_us == 0u || obs.max_control_gap_us > 30000u)
                fail("最大控制间隔超出30ms", obs, env, top);
            if (obs.max_commit_gap_cycles > MAX_COMMIT_GAP_CYCLES)
                fail("电机提交间隔超出30ms", obs, env, top);
        }

        if (!navigation_scene && ready_seen && env.sensors_frozen &&
            static_cast<uint32_t>(env.now_us - obs.ready_time_us) >= 450000u) {
            if (obs.invalid_pose_commits == 0u)
                fail("冻结采集时间戳后位姿没有失效", obs, env, top);
            std::printf(
                "PASS nav_fp_gate_top cycles=%llu now_us=%u ready_cycle=%llu heartbeat=%u commits=%u valid=%u invalid=%u max_gap_us=%u max_commit_cycles=%llu selftest=%08x\n",
                static_cast<unsigned long long>(obs.cycles), env.now_us,
                static_cast<unsigned long long>(obs.ready_cycle),
                static_cast<unsigned>(top.heartbeat_o), obs.control_commits,
                obs.valid_pose_commits, obs.invalid_pose_commits,
                obs.max_control_gap_us,
                static_cast<unsigned long long>(obs.max_commit_gap_cycles),
                obs.firmware_selftest);
            return 0;
        }

        if (navigation_scene && ready_seen) {
            const uint32_t ready_elapsed_us =
                static_cast<uint32_t>(env.now_us - obs.ready_time_us);
            if (ready_elapsed_us >= 500000u && !env.task_schedule_started)
                env.begin_navigation_task();

            if (!navigation_scene_succeeded) {
                const bool active_task = obs.nav_task_id == 1u &&
                    obs.output_autonomous != 0u && obs.active_task_commits != 0u;
                const bool integrated_motion = active_task &&
                    obs.csm_accepted != 0u && obs.planner_expansions != 0u &&
                    obs.forward_request_commits != 0u && obs.nav_commits != 0u;
                if (integrated_motion) {
                    if (env.task_fragments_enqueued != 8u)
                        fail("任务分片没有全部入队", obs, env, top);
                    if (env.task_fragments_popped != 8u)
                        fail("8个任务分片没有经MMIO pop全部交给固件", obs, env, top);
                    if (env.task_queue_overflow)
                        fail("任务RX队列发生溢出", obs, env, top);
                    if (env.lidar_sample_index < 600u || env.lidar_popped < 600u)
                        fail("雷达输入没有生成并消费完整360度扫描", obs, env, top);
                    if (env.lidar_drops != 0u)
                        fail("256点雷达队列出现丢点", obs, env, top);
                    if (obs.firmware_selftest != 0x80000000u ||
                        obs.heartbeat_changes == 0u || obs.diagnostic_words < 7u)
                        fail("固件自检、心跳或EKF诊断未完成", obs, env, top);
                    for (unsigned i = 0; i < 7u; ++i) {
                        if (!positive_finite_float(obs.covariance_diag[i]))
                            fail("导航运行时EKF协方差诊断无效", obs, env, top);
                    }
                    if (obs.max_control_gap_us == 0u || obs.max_control_gap_us > 30000u ||
                        obs.max_commit_gap_cycles > MAX_COMMIT_GAP_CYCLES ||
                        obs.ready_control_commits < 10u ||
                        obs.min_ready_commit_gap_cycles < MIN_COMMIT_GAP_CYCLES)
                        fail("导航控制提交间隔超过30ms", obs, env, top);
                    navigation_scene_succeeded = true;
                    navigation_success_us = env.now_us;
                    env.sensors_frozen = true;
                } else if (ready_elapsed_us >= 2000000u) {
                    fail("2秒内没有形成CSM、任务、A*与自主速度闭环", obs, env, top);
                }
            } else if (static_cast<uint32_t>(env.now_us - navigation_success_us) >= 250000u) {
                if (env.lidar_drops != 0u || env.task_queue_overflow)
                    fail("雷达或任务输入队列发生溢出", obs, env, top);
                if (obs.invalid_pose_commits == 0u || obs.zero_after_stale_commits == 0u)
                    fail("冻结传感器后没有提交失效姿态和零速输出", obs, env, top);
                if (obs.max_control_gap_us == 0u || obs.max_control_gap_us > 30000u ||
                    obs.max_commit_gap_cycles > MAX_COMMIT_GAP_CYCLES)
                    fail("冻结前后控制提交间隔超过30ms", obs, env, top);
                std::printf(
                    "PASS nav_fp_gate_navigation_scene cycles=%llu now_us=%u ready_cycle=%llu csm_accepted=%u lidar_samples=%u lidar_popped=%u lidar_drops=%u task_fragments=%u task_popped=%u nav_commits=%u nav_status=%u task=%u expansions=%u autonomous=%u forward_commits=%u control_commits=%u min_interval_cycles=%llu max_gap_us=%u stale_pose=%u stale_zero=%u selftest=%08x\n",
                    static_cast<unsigned long long>(obs.cycles), env.now_us,
                    static_cast<unsigned long long>(obs.ready_cycle), obs.csm_accepted,
                    env.lidar_sample_index, env.lidar_popped, env.lidar_drops,
                    env.task_fragments_enqueued, env.task_fragments_popped,
                    obs.nav_commits, obs.nav_status, obs.nav_task_id, obs.planner_expansions,
                    obs.output_autonomous, obs.forward_request_commits,
                    obs.ready_control_commits,
                    static_cast<unsigned long long>(obs.min_ready_commit_gap_cycles),
                    obs.max_control_gap_us, obs.invalid_pose_commits,
                    obs.zero_after_stale_commits, obs.firmware_selftest);
                return 0;
            }

            if (ready_elapsed_us >= 5000000u)
                fail("导航场景超过ready后5秒总时间预算", obs, env, top);
        }
    }

    fail("仿真超过全局周期预算", obs, env, top);
    return 1;
}
