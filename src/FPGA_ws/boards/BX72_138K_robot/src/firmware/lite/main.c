#include "bx72_lite.h"
#include "csm.h"
#include "fusion.h"
#include "navigation.h"

#define BX72_LOOP_TICK_US 20000u
#define BX72_LIDAR_BUDGET 16u
#define BX72_LIDAR_SLICE_US 1000u
#define BX72_CSM_CANDIDATES_PER_SLICE 12u
#define BX72_FUSION_PREDICT_STEP_US 5000u
#define BX72_CPU_READY_VALUE 1u
#define BX72_GRID_SNAPSHOT_INTERVAL_US 200000u
#define BX72_GRID_PARTS 8u
#define BX72_GRID_PART_BYTES 128u

static volatile uint32_t *const mmio = (volatile uint32_t *)BX72_IO_BASE;
static bx72_pose_t pose;
static bx72_nav_output_t nav_output;
static bx72_lite_fusion_state_t fusion_state;
static bx72_csm_state_t csm_state;
static uint32_t last_control_us;
static uint32_t next_control_us;
static uint32_t max_control_gap_us;
static uint32_t heartbeat_value;
static uint32_t last_drop_count;
static uint32_t last_error_count;
static uint32_t map_gap_cutoff_us;
static uint8_t map_gap_cutoff_valid;
/* 发送期间冻结整帧；新扫描/CSM仅影响下一帧。 */
static uint32_t grid_snapshot_header[20], grid_snapshot_id, grid_snapshot_last_us;
static uint8_t grid_snapshot_cells[BX72_GRID_EXPORT_BYTES], grid_snapshot_part;

static uint32_t io_read(uint32_t address)
{
    return mmio[address >> 2];
}

static void io_write(uint32_t address, uint32_t value)
{
    mmio[address >> 2] = value;
}

static void grid_snapshot_init(void)
{
    grid_snapshot_id = grid_snapshot_last_us = 0u;
    grid_snapshot_part = BX72_GRID_PARTS;
}

/* 控制命令提交后单次复制，主循环不会在复制中更新地图或变换。 */
static void grid_snapshot_capture(uint32_t now_us, uint32_t deadline_us)
{
    bx72_nav_grid_snapshot_t snapshot;
    uint32_t boot, domain, begin, end;
    if (grid_snapshot_part < BX72_GRID_PARTS ||
        (uint32_t)(now_us - grid_snapshot_last_us) < BX72_GRID_SNAPSHOT_INTERVAL_US) return;
    grid_snapshot_last_us = now_us;
    boot = io_read(BX72_IO_BOOT_ID);
    domain = io_read(BX72_IO_ODOM_DOMAIN);
    begin = io_read(BX72_IO_SNAPSHOT);
    if (!boot || !domain || !bx72_nav_grid_snapshot(&snapshot, grid_snapshot_cells, deadline_us)) return;
    end = io_read(BX72_IO_SNAPSHOT);
    if ((uint32_t)(end - begin) > 100000u || snapshot.boot_id != boot ||
        snapshot.odom_domain != domain ||
        boot != io_read(BX72_IO_BOOT_ID) || domain != io_read(BX72_IO_ODOM_DOMAIN)) return;
    ++grid_snapshot_id;
    grid_snapshot_header[0] = boot;
    grid_snapshot_header[1] = domain;
    grid_snapshot_header[2] = grid_snapshot_id;
    grid_snapshot_header[3] = begin;
    grid_snapshot_header[4] = snapshot.generation;
    grid_snapshot_header[5] = snapshot.scan_generation;
    grid_snapshot_header[6] = (uint32_t)snapshot.origin_x_cell;
    grid_snapshot_header[7] = (uint32_t)snapshot.origin_y_cell;
    grid_snapshot_header[8] = snapshot.complete_scan_stamp_us;
    grid_snapshot_header[9] = snapshot.last_ray_stamp_us;
    grid_snapshot_header[10] = (uint32_t)snapshot.map_anchor_x_mm;
    grid_snapshot_header[11] = (uint32_t)snapshot.map_anchor_y_mm;
    grid_snapshot_header[12] = (uint32_t)snapshot.odom_anchor_x_mm;
    grid_snapshot_header[13] = (uint32_t)snapshot.odom_anchor_y_mm;
    grid_snapshot_header[14] = (uint32_t)snapshot.map_yaw_offset_mrad;
    grid_snapshot_header[15] = snapshot.grid_frame_epoch;
    grid_snapshot_header[16] = snapshot.transform_revision;
    grid_snapshot_header[17] = snapshot.flags;
    grid_snapshot_header[18] = end;
    grid_snapshot_part = 0u;
}

/* 每轮最多提交一片，ready保证前一片已被遥测链路接收。 */
static void grid_snapshot_emit(void)
{
    uint32_t i, base;
    if (grid_snapshot_part >= BX72_GRID_PARTS) return;
    if (grid_snapshot_header[0] != io_read(BX72_IO_BOOT_ID) ||
        grid_snapshot_header[1] != io_read(BX72_IO_ODOM_DOMAIN)) {
        grid_snapshot_part = BX72_GRID_PARTS;
        return;
    }
    if (!(io_read(BX72_IO_GRID_READY) & 1u)) return;
    for (i = 0u; i < 19u; ++i) io_write(BX72_IO_GRID_WORD0 + i * 4u, grid_snapshot_header[i]);
    io_write(BX72_IO_GRID_WORD0 + 19u * 4u,
             (1u << 24) | (50u << 16) | (BX72_GRID_PARTS << 8) | grid_snapshot_part);
    base = (uint32_t)grid_snapshot_part * BX72_GRID_PART_BYTES;
    for (i = 0u; i < BX72_GRID_PART_BYTES; i += 4u) {
        uint32_t word = ((uint32_t)grid_snapshot_cells[base + i] << 24) |
                        ((uint32_t)grid_snapshot_cells[base + i + 1u] << 16) |
                        ((uint32_t)grid_snapshot_cells[base + i + 2u] << 8) |
                        grid_snapshot_cells[base + i + 3u];
        io_write(BX72_IO_GRID_WORD0 + (20u + i / 4u) * 4u, word);
    }
    if (grid_snapshot_header[0] == io_read(BX72_IO_BOOT_ID) &&
        grid_snapshot_header[1] == io_read(BX72_IO_ODOM_DOMAIN)) {
        io_write(BX72_IO_GRID_COMMIT, BX72_COMMIT_MAGIC);
        ++grid_snapshot_part;
    } else grid_snapshot_part = BX72_GRID_PARTS;
}

static void grid_snapshot_emit_if_time(uint32_t now_us, uint32_t deadline_us)
{
    if ((int32_t)(deadline_us - now_us) >= 4000) grid_snapshot_emit();
}

static int32_t signed_half(uint32_t value)
{
    return (int32_t)(int16_t)(uint16_t)value;
}

static int32_t signed_high_half(uint32_t value)
{
    return (int32_t)(int16_t)(uint16_t)(value >> 16);
}

static int32_t abs_i32(int32_t value)
{
    if (value == (-2147483647 - 1)) return 2147483647;
    return value < 0 ? -value : value;
}

static void heartbeat(void)
{
    ++heartbeat_value;
    io_write(0x00cu, heartbeat_value);
}

static void read_inputs(bx72_lite_sample_t *input)
{
    uint32_t packed;

    /* 写0x108锁存采集快照；其读端返回实时now_us，时间戳独立对应每个采集器。 */
    io_write(BX72_IO_SNAPSHOT, 1u);
    input->now_us = io_read(BX72_IO_NOW);
    input->status = io_read(BX72_IO_STATUS);
    input->imu_seq = io_read(BX72_IO_IMU_SEQ);
    packed = io_read(BX72_IO_GX_GY);
    input->gx = signed_high_half(packed);
    input->gy = signed_half(packed);
    packed = io_read(BX72_IO_GZ_AX);
    input->gz = signed_high_half(packed);
    input->ax = signed_half(packed);
    packed = io_read(BX72_IO_AY_AZ);
    input->ay = signed_high_half(packed);
    input->az = signed_half(packed);
    input->wheel_seq = io_read(BX72_IO_WHEEL_SEQ);
    input->vx = (int32_t)io_read(BX72_IO_VX);
    input->vy = (int32_t)io_read(BX72_IO_VY);
    input->wz = (int32_t)io_read(BX72_IO_WZ);
    input->wheel_stamp_us = io_read(BX72_IO_WHEEL_STAMP);
    input->imu_stamp_us = io_read(BX72_IO_IMU_STAMP);
    input->command_seq = io_read(BX72_IO_COMMAND_SEQ);
    packed = io_read(BX72_IO_COMMAND_XY);
    input->command_vx = signed_high_half(packed);
    input->command_vy = signed_half(packed);
    input->command_wz = signed_half(io_read(BX72_IO_COMMAND_WZ));
    input->mode_seq = io_read(BX72_IO_MODE_SEQ);
}

static void fusion_update_if_due(const bx72_lite_sample_t *input, int force)
{
    int new_imu = input->imu_seq != fusion_state.last_imu_seq;
    int new_wheel = input->wheel_seq != fusion_state.last_wheel_seq;
    int predict_due = (uint32_t)(input->now_us - fusion_state.last_fusion_us) >=
                      BX72_FUSION_PREDICT_STEP_US;
    if (force || new_imu || new_wheel || predict_due)
        bx72_lite_fusion_update(&fusion_state, &pose, input);
}

static void commit_pose(void)
{
    io_write(BX72_IO_POSE_X, (uint32_t)pose.x_mm);
    io_write(BX72_IO_POSE_Y, (uint32_t)pose.y_mm);
    io_write(BX72_IO_POSE_YAW, (uint32_t)pose.yaw_mrad);
    io_write(BX72_IO_POSE_ROLL, (uint32_t)pose.roll_mrad);
    io_write(BX72_IO_POSE_PITCH, (uint32_t)pose.pitch_mrad);
    io_write(BX72_IO_POSE_FLAGS, pose.flags);
    io_write(BX72_IO_POSE_STAMP, pose.stamp_us);
    io_write(BX72_IO_POSE_COMMIT, BX72_COMMIT_MAGIC);
}

static void publish_diagnostics(void)
{
    uint32_t covariance[7];
    uint32_t accepted, rejected, csm_status;
    int32_t bias;
    unsigned int i;
    bx72_lite_fusion_diag(&fusion_state, covariance, &bias);
    for (i = 0; i < 3; ++i) io_write(BX72_IO_COV_DIAG0 + i * 4u, covariance[i]);
    io_write(BX72_IO_COV_DIAG0 + 12u, io_read(BX72_IO_LIDAR_RAW_COUNT));
    io_write(BX72_IO_COV_DIAG0 + 16u, io_read(BX72_IO_LIDAR_REDUCED_COUNT));
    io_write(BX72_IO_GYRO_BIAS, (uint32_t)bias);
    io_write(BX72_IO_EKF_VX, (uint32_t)pose.vx_mm_s);
    io_write(BX72_IO_EKF_VY, (uint32_t)pose.vy_mm_s);
    io_write(BX72_IO_EKF_WZ, (uint32_t)pose.wz_mrad_s);
    io_write(BX72_IO_MAX_CONTROL_GAP, max_control_gap_us);
    bx72_csm_diagnostics(&csm_state, &accepted, &rejected, &csm_status);
    io_write(BX72_IO_CSM_ACCEPTED, accepted);
    io_write(BX72_IO_CSM_REJECTED, rejected);
    io_write(BX72_IO_CSM_STATUS, csm_status);
}

static void commit_navigation(const bx72_lite_sample_t *input,
                              uint32_t status, int task_overflow)
{
    int link_down_manual = (status & BX72_STATUS_LINK_DOWN) != 0u &&
                           nav_output.autonomous == 0u;
    int force_stop = !pose.flags || link_down_manual ||
                     (status & BX72_STATUS_GUARD_STOP) != 0u ||
                     task_overflow ||
                     (nav_output.autonomous != 0u &&
                      (status & BX72_STATUS_LIDAR_OK) == 0u);
    int32_t vx = force_stop ? 0 : nav_output.vx_mm_s;
    int32_t vy = force_stop ? 0 : nav_output.vy_mm_s;
    int32_t wz = force_stop ? 0 : nav_output.wz_mrad_s;

    if (abs_i32(vx) > 400 || abs_i32(vy) > 400 || abs_i32(wz) > 800) {
        vx = 0;
        vy = 0;
        wz = 0;
        force_stop = 1;
    }
    io_write(BX72_IO_OUT_VX, (uint32_t)vx);
    io_write(BX72_IO_OUT_VY, (uint32_t)vy);
    io_write(BX72_IO_OUT_WZ, (uint32_t)wz);
    io_write(BX72_IO_OUT_AUTONOMOUS, nav_output.autonomous);
    io_write(BX72_IO_OUT_COMMAND_SEQ, input->command_seq);
    io_write(BX72_IO_OUT_MODE_SEQ, input->mode_seq);
    io_write(BX72_IO_OUT_COMMIT, BX72_COMMIT_MAGIC);
    commit_pose();
    io_write(BX72_IO_NAV_STATUS,
             (nav_output.status & 0xffu) | bx72_nav_diagnostic_status() |
             (force_stop ? (1u << 31) : 0u));
    io_write(BX72_IO_NAV_TASK, nav_output.task_id);
    io_write(BX72_IO_NAV_PROGRESS, nav_output.progress);
    io_write(BX72_IO_NAV_GOAL_X, (uint32_t)nav_output.goal_x_q16);
    io_write(BX72_IO_NAV_GOAL_Y, (uint32_t)nav_output.goal_y_q16);
    io_write(BX72_IO_NAV_EXPIRY, nav_output.expiry_us);
    io_write(BX72_IO_MAP_VERSION, nav_output.map_version);
    io_write(BX72_IO_SEARCH_EXPANSIONS, nav_output.planner_expansions);
    io_write(BX72_IO_MAP_AGE, nav_output.map_age_us);
    io_write(BX72_IO_LAST_TASK_ID, nav_output.last_task_id);
    io_write(BX72_IO_NAV_MODE, nav_output.task_mode);
    io_write(BX72_IO_PLAN_STAMP, nav_output.plan_stamp_us);
    io_write(BX72_IO_PLAN_START_X, (uint32_t)nav_output.plan_start_x_mm);
    io_write(BX72_IO_PLAN_START_Y, (uint32_t)nav_output.plan_start_y_mm);
    io_write(BX72_IO_PLAN_END_X, (uint32_t)nav_output.plan_end_x_mm);
    io_write(BX72_IO_PLAN_END_Y, (uint32_t)nav_output.plan_end_y_mm);
    io_write(BX72_IO_PLAN_VALID, nav_output.plan_valid);
    io_write(BX72_IO_NAV_COMMIT, BX72_COMMIT_MAGIC);
}

/* 建图与CSM共用5度最近回波及同一次历史位姿查询；原始点仍供独立保护。 */
static void check_lidar_discontinuity(void)
{
    uint32_t drops = io_read(BX72_IO_LIDAR_DROPS);
    uint32_t errors = io_read(BX72_IO_LIDAR_ERRORS);
    if (drops != last_drop_count || errors != last_error_count) {
        map_gap_cutoff_us = io_read(BX72_IO_SNAPSHOT);
        map_gap_cutoff_valid = 1u;
        bx72_nav_lidar_gap(map_gap_cutoff_us, drops != last_drop_count ?
                          BX72_NAV_GAP_FIFO : BX72_NAV_GAP_UPSTREAM);
        bx72_csm_discontinuity(&csm_state);
        last_drop_count = drops;
        last_error_count = errors;
    }
}

static void process_lidar(uint32_t now_us)
{
    uint32_t started = io_read(BX72_IO_SNAPSHOT);
    uint32_t i;
    (void)now_us;
    for (i = 0u; i < BX72_LIDAR_BUDGET; ++i) {
        uint32_t meta, head, stamp, live_now, reason = BX72_NAV_GAP_NONE;
        bx72_pose_t beam_pose;
        /* 丢点发生在队尾，逐点复查计数，并丢弃检测时刻之前的积压样本，
         * 防止先消费旧回卷后错误宣称完整圈已经恢复。 */
        check_lidar_discontinuity();
        meta = io_read(BX72_IO_LIDAR_META);
        if (!(meta & 1u)) break;
        if (i != 0u && (uint32_t)(io_read(BX72_IO_SNAPSHOT) - started) >=
                       BX72_LIDAR_SLICE_US) break;
        head = io_read(BX72_IO_LIDAR_HEAD);
        stamp = io_read(BX72_IO_LIDAR_STAMP);
        live_now = io_read(BX72_IO_SNAPSHOT);
        if (map_gap_cutoff_valid && (int32_t)(stamp - map_gap_cutoff_us) <= 0) {
            /* 旧完整圈按自身时标过期，积压点不参与重新同步。 */
        } else {
            if ((head >> 16) >= 36000u) reason = BX72_NAV_GAP_INVALID;
            else if ((uint32_t)(live_now - stamp) > BX72_LIDAR_SAMPLE_MAX_AGE_US) reason = BX72_NAV_GAP_STALE;
            else if (!bx72_lite_fusion_pose_at(&fusion_state, stamp, &beam_pose))
                reason = BX72_NAV_GAP_POSE;
            if (reason != BX72_NAV_GAP_NONE) {
                bx72_nav_lidar_gap(live_now, reason);
                bx72_csm_discontinuity(&csm_state);
            } else {
                map_gap_cutoff_valid = 0u;
                bx72_nav_lidar_validated(head >> 16, head & 0xffffu,
                    (meta >> 1) & 1u, &beam_pose, stamp, live_now);
                bx72_csm_point(&csm_state, head >> 16, head & 0xffffu,
                    (meta >> 1) & 1u, &beam_pose);
            }
        }
        io_write(BX72_IO_LIDAR_POP, 1u);
    }
    check_lidar_discontinuity();
}
static void process_task(uint32_t *task_overflow)
{
    uint32_t meta = io_read(BX72_IO_TASK_META);
    uint32_t epoch, now_us;
    uint32_t words[8];
    bx72_lite_sample_t task_input;
    unsigned int i;
    if ((meta & 3u) == 0u) return;
    /* 代次包住控制快照及分片读取。若中途发生来源切换/溢出，
     * 丢弃本次读值，下一轮读取新队首及其实际对应的控制状态。 */
    epoch = io_read(BX72_IO_TASK_EPOCH);
    read_inputs(&task_input);
    meta = io_read(BX72_IO_TASK_META);
    if ((meta & 3u) == 0u) return;
    if ((meta & 2u) != 0u) {
        if (epoch != io_read(BX72_IO_TASK_EPOCH)) return;
        now_us = io_read(BX72_IO_SNAPSHOT);
        bx72_nav_sync_inputs(now_us, task_input.command_seq, task_input.mode_seq);
        bx72_nav_task_input_lost(now_us);
        bx72_nav_control(&pose, now_us, task_input.status,
                         task_input.command_seq, task_input.command_vx, task_input.command_vy,
                         task_input.command_wz, task_input.mode_seq, &nav_output);
        /* 先提交零速，再确认丢片，避免硬件解除闭锁后重放旧速度。 */
        commit_navigation(&task_input, task_input.status, 1);
        io_write(BX72_IO_TASK_OVERFLOW_ACK, 1u);
        *task_overflow = 0u;
        return;
    }
    if ((meta & 1u) == 0u) return;
    for (i = 0; i < 8; ++i) words[i] = io_read(BX72_IO_TASK_WORD0 + i * 4u);
    if (epoch != io_read(BX72_IO_TASK_EPOCH)) return;
    now_us = io_read(BX72_IO_SNAPSHOT);
    bx72_nav_sync_inputs(now_us, task_input.command_seq, task_input.mode_seq);
    bx72_nav_fragment(words, now_us);
    io_write(BX72_IO_TASK_POP, epoch);
}

__attribute__((noinline)) static float fp_probe(float a, float b, float c)
{
    volatile float product = a * b;
    volatile float sum = product + c;
    return sum;
}

static float hardware_sqrtf(float value)
{
    float result;
    __asm__ volatile("fsqrt.s %0, %1" : "=f"(result) : "f"(value));
    return result;
}

static int fpu_selftest(void)
{
    volatile float a = 1.25f;
    volatile float b = 3.5f;
    volatile float c = 1.25f;
    volatile float radicand = 9.0f;
    volatile float result = fp_probe(a, b, c);
    volatile float root = hardware_sqrtf(radicand);
    return result > 5.624f && result < 5.626f && root > 2.999f && root < 3.001f;
}

static void firmware_selftest_fail(uint32_t result, uint32_t fault_code)
{
    io_write(BX72_IO_SELFTEST, 0x80000000u | result);
    io_write(0x010u, fault_code);
    for (;;) { }
}

void firmware_main(void)
{
    bx72_lite_sample_t input;
    uint32_t task_overflow = 0u;

    if (!fpu_selftest()) firmware_selftest_fail(1u, 0x71u);
    if (!bx72_lite_fusion_selftest(&fusion_state))
        firmware_selftest_fail(2u, 0x72u);
    if (!bx72_csm_selftest(&csm_state, &fusion_state))
        firmware_selftest_fail(4u, 0x73u);
    {
        uint32_t nav_result = bx72_nav_selftest();
        if (nav_result != 0u)
            firmware_selftest_fail(nav_result, 0x74u);
    }

    bx72_nav_init();
    bx72_lite_fusion_init(&fusion_state);
    bx72_csm_init(&csm_state);
    grid_snapshot_init();
    /* pose与nav_output由_start清零BSS；上述自检不接收这两个静态对象。 */
    nav_output.task_mode = 1u;

    /* 启动及FPU自检完成后才声明CPU就绪。 */
    io_write(BX72_IO_OUT_VX, 0u);
    io_write(BX72_IO_OUT_VY, 0u);
    io_write(BX72_IO_OUT_WZ, 0u);
    io_write(BX72_IO_OUT_AUTONOMOUS, 0u);
    io_write(BX72_IO_OUT_COMMAND_SEQ, 0u);
    io_write(BX72_IO_OUT_MODE_SEQ, 0u);
    io_write(BX72_IO_NAV_STATUS, 0u);
    io_write(BX72_IO_NAV_TASK, 0u);
    io_write(BX72_IO_NAV_PROGRESS, 0u);
    io_write(BX72_IO_NAV_GOAL_X, 0u);
    io_write(BX72_IO_NAV_GOAL_Y, 0u);
    io_write(BX72_IO_NAV_EXPIRY, 0u);
    io_write(BX72_IO_MAP_VERSION, 0u);
    io_write(BX72_IO_SEARCH_EXPANSIONS, 0u);
    io_write(BX72_IO_MAP_AGE, 0u);
    io_write(BX72_IO_LAST_TASK_ID, 0u);
    io_write(BX72_IO_NAV_MODE, 1u);
    publish_diagnostics();
    io_write(BX72_IO_OUT_COMMIT, BX72_COMMIT_MAGIC);
    commit_pose();
    io_write(BX72_IO_NAV_COMMIT, BX72_COMMIT_MAGIC);
    io_write(BX72_IO_SELFTEST, 0x80000000u);
    io_write(0x008u, BX72_CPU_READY_VALUE);

    for (;;) {
        uint32_t live_now;
        uint32_t task_meta;

        read_inputs(&input);
        fusion_update_if_due(&input, 0);
        heartbeat();

        live_now = io_read(BX72_IO_SNAPSHOT);
        task_meta = io_read(BX72_IO_TASK_META);
        task_overflow |= (task_meta >> 1) & 1u;
        if ((int32_t)(live_now - next_control_us) >= 0) {
            uint32_t gap = last_control_us == 0u ? BX72_LOOP_TICK_US :
                           (uint32_t)(live_now - last_control_us);
            if (gap > max_control_gap_us) max_control_gap_us = gap;
            input.now_us = live_now;
            fusion_update_if_due(&input, 1);
            bx72_nav_control(&pose, live_now, input.status,
                             input.command_seq, input.command_vx, input.command_vy,
                             input.command_wz, input.mode_seq, &nav_output);
            if (fusion_state.fusion_ready && fusion_state.last_fusion_us == live_now)
                pose.stamp_us = live_now;
            publish_diagnostics();
            commit_navigation(&input, input.status, (int)task_overflow);
            last_control_us = live_now;
            /* 保持绝对20ms节拍，避免每片执行尾差长期累加为45Hz。
               真正超期时跳过过期tick，不连续补发多次控制输出。 */
            next_control_us += BX72_LOOP_TICK_US;
            if ((int32_t)(live_now - next_control_us) >= 0)
                next_control_us = live_now + BX72_LOOP_TICK_US;
            grid_snapshot_capture(io_read(BX72_IO_SNAPSHOT), next_control_us);
        }

        /* 快照可能耗时；控制到期时先重采输入并控制，不能用旧now处理新片。 */
        live_now = io_read(BX72_IO_SNAPSHOT);
        if ((int32_t)(live_now - next_control_us) >= 0) continue;
        grid_snapshot_emit_if_time(live_now, next_control_us);
        live_now = io_read(BX72_IO_SNAPSHOT);
        if ((int32_t)(live_now - next_control_us) >= 0) continue;
        process_task(&task_overflow);
        process_lidar(live_now);
        live_now = io_read(BX72_IO_SNAPSHOT);
        /* 候选检查可用最多 4 ms；距离控制拍太近时让控制先运行。 */
        if ((int32_t)(next_control_us - live_now) > 5000)
            bx72_nav_step(&pose, live_now);
        live_now = io_read(BX72_IO_SNAPSHOT);
        if ((int32_t)(next_control_us - live_now) > 5000 && csm_state.icp_active) {
            bx72_pose_t old_now, new_now;
            uint32_t accepted_before = csm_state.accepted_count;
            int have_old = bx72_lite_fusion_pose_at(&fusion_state, live_now, &old_now);
            bx72_csm_step(&csm_state, &pose, live_now, &fusion_state,
                          BX72_CSM_CANDIDATES_PER_SLICE);
            if (accepted_before != csm_state.accepted_count) {
                int have_new = bx72_lite_fusion_pose_at(&fusion_state, live_now, &new_now);
                if (have_old && have_new) bx72_nav_rebase(&old_now, &new_now);
                else bx72_nav_map_reset(have_new ? &new_now : 0, live_now);
                if (have_new) pose = new_now;
            }
        }
    }
}
