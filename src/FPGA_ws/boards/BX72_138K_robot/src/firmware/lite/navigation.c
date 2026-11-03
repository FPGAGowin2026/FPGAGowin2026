#include "navigation.h"
#include "local_grid.h"
#include "local_path.h"
#include "manual_control.h"

#include <limits.h>

#ifndef BX72_NAV_MMIO_BASE
#define BX72_NAV_MMIO_BASE BX72_IO_BASE
#endif

#define NAV_Q16_ONE                    65536ll
#define NAV_PI_MRAD                    3142
#define NAV_TWO_PI_MRAD                6283
#define NAV_LIDAR_INPUT_MAX_AGE_US     500000u
#define NAV_POSE_MAX_AGE_US            100000u
#define NAV_LIDAR_GAP_MAX_CDEG         2200u
#define NAV_FRAGMENT_MAX_AGE_US        200000u
#define NAV_TASK_MAX_AGE_US            500000u
#define NAV_TASK_MAX_LIFETIME_US       120000000u
#define NAV_EVENT_US                   1000000u
#define NAV_TASK_MAX_SPEED_MM_S        400u
#define NAV_REBASE_SMALL_YAW_MRAD      10u
#define NAV_PLAN_HEADING_CHANGE_MRAD   250u
#define NAV_AUTO_MIN_YAW_MRAD_S        600
#define NAV_AUTO_MAX_YAW_MRAD_S        800
/* 实测和预测允许额外50 mrad/s，属于工程余量，并非实车噪声标定。 */
#define NAV_MOTION_MAX_YAW_MRAD_S      850u
#define NAV_YAW_ACCEL_MRAD_S2          1500u
#define NAV_NO_PROGRESS_US             20000000ull
#define NAV_BLOCKED_RETRY_US           1000000ull
#define NAV_TURN_DRIFT_TOLERANCE_MM    2u
#define NAV_DRIFT_NOISE_MM_S           20u
#define NAV_PATH_WAYPOINT_TOLERANCE_MM 25u
#define NAV_PATH_PREFETCH_MM           150u
#define NAV_PATH_PREFETCH_MOVE_MM      35u
#define NAV_PATH_RETRY_US              250000u
#define NAV_PATH_SLICE_US              1000u
#define NAV_DIRECT_GUIDE_MM            300u
#define NAV_TASK_ALLOW_DISCONNECT      1u
#define NAV_TASK_POSITION_ONLY         2u
#define NAV_TASK_FLAGS_MASK            (NAV_TASK_ALLOW_DISCONNECT | NAV_TASK_POSITION_ONLY)
/* 停稳角速的工程阈值；不等同于线速噪声，也尚未实车标定。 */
#define NAV_TURN_SETTLED_MRAD_S        20u
#define NAV_SESSION_BOOT_OFFSET        0x1a0u
#define NAV_SESSION_DOMAIN_OFFSET      0x1a4u
#define NAV_SESSION_TOKEN_LO_OFFSET    0x1a8u
#define NAV_SESSION_TOKEN_HI_OFFSET    0x1acu

typedef struct {
    int64_t goal_x_mm_q16;
    int64_t goal_y_mm_q16;
    int32_t hold_yaw_mrad;
    int32_t final_yaw_mrad;
    uint32_t task_id;
    uint32_t mode;
    uint32_t flags;
    uint32_t max_speed_mm_s;
    uint32_t max_accel_mm_s2;
    uint32_t tolerance_mm;
    uint32_t yaw_tolerance_mrad;
    uint64_t expiry_us;
} nav_task_t;

static nav_task_t task_queue[BX72_NAV_TASK_QUEUE_SIZE];
static uint8_t task_head;
static uint8_t task_count;
static uint32_t active_task_id;
static uint64_t task_start_distance_q16;
static uint8_t task_start_initialized;
static uint32_t last_task_id;
static uint8_t have_last_task_id;

static uint8_t fragment_payload[72];
static uint8_t fragment_collecting;
static uint8_t fragment_next_index;
static uint32_t fragment_next_sequence;
static uint64_t fragment_token;
static uint32_t fragment_started_us;

static uint32_t last_now_low;
static uint64_t now_extended_us;
static uint8_t clock_initialized;
static uint8_t clock_valid;
static uint8_t mode_initialized;
static uint32_t last_mode_sequence;
static uint32_t last_command_sequence;
static uint8_t command_sequence_initialized;
static uint32_t last_manual_motion_sequence;
static uint8_t manual_motion_sequence_initialized;
static uint8_t auto_enabled;
static uint8_t manual_inhibited;
static bx72_manual_state_t manual_state;

static uint32_t event_status;
static uint32_t event_task_id;
static uint64_t event_until_us;
static int32_t previous_vx_mm_s;
static int32_t previous_vy_mm_s;
static int32_t previous_wz_mrad_s;
static bx72_pose_t latest_pose;
static uint8_t latest_pose_valid;

static uint32_t session_boot_id;
static uint32_t session_domain;
static uint64_t session_token;
static uint8_t session_initialized;

static uint32_t planner_expansions;
static uint8_t progress_blocked;
static uint8_t unknown_in_swept_corridor;
static uint32_t best_goal_distance_mm;
static uint64_t last_progress_us;
static uint32_t blocked_scan_generation;

/* 地图坐标与当前odom之间的刚体变换；每次CSM校正只重定位这个锚点。 */
static int32_t map_odom_anchor_x, map_odom_anchor_y;
static int32_t map_anchor_x, map_anchor_y, map_yaw_offset;
static uint32_t grid_frame_epoch, transform_revision;
static uint8_t map_initialized;
static uint8_t map_angle_valid, map_synchronized, map_gap_latched;
static uint8_t last_map_gap_reason;
static uint32_t map_previous_angle, last_lidar_us;
static uint32_t map_scan_started_us, map_complete_started_us;
static uint8_t have_lidar_sample;

/* 候选每次后台只评估一个，最终动作仍在控制tick按实时地图复核。 */
#define NAV_CANDIDATE_COUNT 7u
static uint8_t planner_active, planner_complete, planner_cursor, planner_best_valid;
static uint8_t planner_inhibited, planner_at_goal, planner_timeout_pending;
static int32_t planner_best_vx, planner_best_wz;
static int32_t planner_heading_error;
static int64_t planner_best_score;
static bx72_pose_t planner_pose;
static uint32_t planner_task_id;
static uint8_t selected_valid;
static int32_t selected_vx, selected_wz;
static bx72_grid_unknown_exemption_t start_exemption;
static uint8_t start_exemption_valid;
static uint8_t start_confirmation_lost;
/* 地图重置前发送的任务不能为新地图确认起点。 */
static uint8_t task_reset_barrier_valid;
static uint64_t task_reset_barrier_us;
static uint8_t observe_active, observe_settled;
static int32_t observe_target_yaw_mrad;
static uint32_t observe_settle_us, observe_scan_generation;
/* 临时路径引导点保存在局部图坐标中；最终目标仍遵循固定 odom 合同。 */
static uint8_t detour_active;
static int32_t detour_x_mm, detour_y_mm;
static uint8_t direct_guide_active, direct_guide_blocked;
/* 记录引导点对应的最终目标，比较累计校正而不是只比较单次校正。 */
static int32_t guide_goal_x_mm, guide_goal_y_mm;
static uint8_t guide_goal_valid;
static uint8_t path_target_final, path_retry_pending, path_refresh_pending;
static uint32_t path_blocked_scan, path_retry_us;
static int32_t path_start_x_mm, path_start_y_mm;
static uint8_t turn_braking, turn_settle_count, arrival_settle_count;
static int32_t turn_brake_sign;
static uint32_t current_time_us(void);

static uint32_t abs_i32(int32_t value)
{
    return value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint64_t abs_i64(int64_t value)
{
    return value < 0 ? (uint64_t)(-value) : (uint64_t)value;
}

static int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint32_t clamp_u64_u32(uint64_t value)
{
    return value > 0xffffffffull ? 0xffffffffu : (uint32_t)value;
}

static uint32_t approx_distance_xy_mm(int64_t dx, int64_t dy)
{
    uint64_t ax = abs_i64(dx);
    uint64_t ay = abs_i64(dy);
    uint64_t hi = ax > ay ? ax : ay;
    uint64_t lo = ax > ay ? ay : ax;
    return clamp_u64_u32(hi + (lo * 414u) / 1000u);
}

static uint64_t approx_distance_q16(int64_t dx, int64_t dy)
{
    uint64_t ax = abs_i64(dx);
    uint64_t ay = abs_i64(dy);
    uint64_t hi = ax > ay ? ax : ay;
    uint64_t lo = ax > ay ? ay : ax;
    return hi + (lo * 414u) / 1000u;
}

static int32_t wrap_yaw_mrad(int32_t angle)
{
    angle %= NAV_TWO_PI_MRAD;
    if (angle > NAV_PI_MRAD) angle -= NAV_TWO_PI_MRAD;
    if (angle < -NAV_PI_MRAD) angle += NAV_TWO_PI_MRAD;
    return angle;
}

static uint64_t update_clock(uint32_t now_us)
{
    if (!clock_initialized) {
        clock_initialized = 1u;
        clock_valid = 1u;
        last_now_low = now_us;
        now_extended_us = now_us;
    } else {
        uint32_t delta = now_us - last_now_low;
        last_now_low = now_us;
        if (delta >= 0x80000000u) clock_valid = 0u;
        else if (clock_valid) now_extended_us += delta;
    }
    return now_extended_us;
}

static int timestamp_recent(uint32_t now_us, uint32_t stamp_us, uint32_t max_age_us)
{
    uint32_t age = now_us - stamp_us;
    if (age < 0x80000000u) return age <= max_age_us;
    return (uint32_t)(stamp_us - now_us) <= max_age_us;
}

static uint32_t timestamp_age(uint32_t now_us, uint32_t stamp_us)
{
    uint32_t age = now_us - stamp_us;
    return age < 0x80000000u ? age : 0u;
}

static int32_t atan2_approx_cdeg64(int64_t y, int64_t x)
{
    uint64_t ax = abs_i64(x);
    uint64_t ay = abs_i64(y);
    uint32_t ratio;
    uint32_t base;
    if (ax == 0u && ay == 0u) return 0;
    while (ax > 65535u || ay > 65535u) {
        ax >>= 1;
        ay >>= 1;
    }
    if (ax == 0u) base = 9000u;
    else if (ay == 0u) base = 0u;
    else {
        ratio = ax >= ay ? (uint32_t)((ay << 15) / ax) :
                           (uint32_t)((ax << 15) / ay);
        base = (ratio * (4500u + ((1564u * (32768u - ratio)) >> 15)) +
                16384u) >> 15;
        if (ay > ax) base = 9000u - base;
    }
    if (x >= 0 && y >= 0) return (int32_t)base;
    if (x < 0 && y >= 0) return 18000 - (int32_t)base;
    if (x < 0 && y < 0) return 18000 + (int32_t)base;
    return 36000 - (int32_t)base;
}

static void reset_follow_state(void)
{
    progress_blocked = 0u;
    unknown_in_swept_corridor = 0u;
    best_goal_distance_mm = 0u;
    last_progress_us = 0u;
    blocked_scan_generation = 0u;
    planner_active = planner_complete = planner_cursor = planner_best_valid = 0u;
    planner_inhibited = 1u;
    selected_valid = 0u;
    observe_active = observe_settled = 0u;
    detour_active = guide_goal_valid = direct_guide_active = direct_guide_blocked = 0u;
    bx72_local_path_reset();
    path_target_final = path_retry_pending = path_refresh_pending = 0u;
    path_blocked_scan = path_retry_us = 0u;
    turn_braking = turn_settle_count = arrival_settle_count = 0u;
    turn_brake_sign = 0;
}

static void publish_event(uint32_t status, uint32_t task_id, uint64_t now64)
{
    event_status = status;
    event_task_id = task_id;
    event_until_us = now64 + NAV_EVENT_US;
}

static uint32_t mmio_read_offset(uint32_t offset)
{
#ifdef BX72_NAV_HOST_TEST
    extern uint32_t bx72_nav_test_mmio[];
    return bx72_nav_test_mmio[offset / 4u];
#else
    volatile uint32_t *mmio = (volatile uint32_t *)(uintptr_t)BX72_NAV_MMIO_BASE;
    return mmio[offset / 4u];
#endif
}

static void cancel_plan(void)
{
    task_start_initialized = 0u;
    task_start_distance_q16 = 0u;
    reset_follow_state();
}

static void clear_tasks(uint64_t now64, uint32_t status, uint32_t task_id)
{
    task_count = 0u;
    task_head = 0u;
    active_task_id = 0u;
    manual_inhibited = 1u;
    cancel_plan();
    previous_vx_mm_s = 0;
    previous_vy_mm_s = 0;
    previous_wz_mrad_s = 0;
    publish_event(status, task_id, now64);
}

static void read_session(uint32_t *boot_id, uint32_t *domain, uint64_t *token)
{
    uint32_t token_lo = mmio_read_offset(NAV_SESSION_TOKEN_LO_OFFSET);
    uint32_t token_hi = mmio_read_offset(NAV_SESSION_TOKEN_HI_OFFSET);
    *boot_id = mmio_read_offset(NAV_SESSION_BOOT_OFFSET);
    *domain = mmio_read_offset(NAV_SESSION_DOMAIN_OFFSET);
    *token = ((uint64_t)token_hi << 32) | token_lo;
}

static void update_session(uint64_t now64)
{
    uint32_t boot_id;
    uint32_t domain;
    uint64_t token;
    read_session(&boot_id, &domain, &token);
    if (!session_initialized) {
        session_initialized = 1u;
        session_boot_id = boot_id;
        session_domain = domain;
        session_token = token;
        return;
    }
    if (boot_id != session_boot_id || domain != session_domain || token != session_token) {
        uint32_t old_task = active_task_id;
        if (domain != session_domain) {
            bx72_nav_map_reset(latest_pose_valid ? &latest_pose : 0, (uint32_t)now64);
            /* 若已有启动许可，map_reset锁阻；首次连接尚无许可时可启动。 */
        }
        clear_tasks(now64, BX72_NAV_STATUS_CANCELLED, old_task);
        fragment_collecting = 0u;
        fragment_next_index = 0u;
        fragment_next_sequence = 0u;
        fragment_token = 0u;
        fragment_started_us = 0u;
        last_task_id = 0u;
        have_last_task_id = 0u;
        session_boot_id = boot_id;
        session_domain = domain;
        session_token = token;
    }
}

static int task_sequence_newer(uint32_t incoming)
{
    uint32_t delta;
    if (!have_last_task_id) return incoming != 0u;
    delta = incoming - last_task_id;
    return delta != 0u && delta < 0x80000000u;
}

static int payload_reserved_zero(const uint8_t *payload)
{
    uint32_t i;
    for (i = 48u; i < 56u; ++i) if (payload[i] != 0u) return 0;
    for (i = 64u; i < 72u; ++i) if (payload[i] != 0u) return 0;
    return 1;
}

static int enqueue_task(const nav_task_t *task)
{
    uint8_t tail;
    if (task_count >= BX72_NAV_TASK_QUEUE_SIZE) return 0;
    tail = (uint8_t)((task_head + task_count) % BX72_NAV_TASK_QUEUE_SIZE);
    task_queue[tail] = *task;
    ++task_count;
    return 1;
}

static uint32_t goal_distance_mm(const bx72_pose_t *pose, const nav_task_t *task,
                                 uint64_t *distance_q16)
{
    int64_t dx_q16 = task->goal_x_mm_q16 - (int64_t)pose->x_mm * NAV_Q16_ONE;
    int64_t dy_q16 = task->goal_y_mm_q16 - (int64_t)pose->y_mm * NAV_Q16_ONE;
    *distance_q16 = approx_distance_q16(dx_q16, dy_q16);
    return clamp_u64_u32(*distance_q16 / NAV_Q16_ONE);
}

static int goal_position_reached(const bx72_pose_t *pose, const nav_task_t *task)
{
    uint64_t dx = abs_i64(task->goal_x_mm_q16 - (int64_t)pose->x_mm * NAV_Q16_ONE);
    uint64_t dy = abs_i64(task->goal_y_mm_q16 - (int64_t)pose->y_mm * NAV_Q16_ONE);
    uint64_t radius = (uint64_t)task->tolerance_mm * NAV_Q16_ONE;
    if (dx > radius || dy > radius) return 0;
    return dx * dx + dy * dy <= radius * radius;
}

static void activate_current_task(const bx72_pose_t *pose, uint64_t now64)
{
    nav_task_t *task;
    uint64_t distance_q16;
    if (task_count == 0u) {
        active_task_id = 0u;
        reset_follow_state();
        return;
    }
    task = &task_queue[task_head];
    active_task_id = task->task_id;
    task_start_distance_q16 = approx_distance_q16(
        task->goal_x_mm_q16 - (int64_t)pose->x_mm * NAV_Q16_ONE,
        task->goal_y_mm_q16 - (int64_t)pose->y_mm * NAV_Q16_ONE);
    if (task_start_distance_q16 == 0u) task_start_distance_q16 = 1u;
    task_start_initialized = 1u;
    reset_follow_state();
    best_goal_distance_mm = goal_distance_mm(pose, task, &distance_q16);
    last_progress_us = now64;
}

static int32_t read_s32_be(const uint8_t *bytes)
{
    uint32_t value = ((uint32_t)bytes[0] << 24) |
                     ((uint32_t)bytes[1] << 16) |
                     ((uint32_t)bytes[2] << 8) | bytes[3];
    return (int32_t)value;
}

static uint32_t read_u32_be(const uint8_t *bytes)
{
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) | bytes[3];
}

static uint64_t read_u64_be(const uint8_t *bytes)
{
    return ((uint64_t)read_u32_be(bytes) << 32) | read_u32_be(bytes + 4);
}

static uint16_t read_u16_be(const uint8_t *bytes)
{
    return (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

static int q16m_to_mm_q16(int32_t value_q16_m, int64_t *value_mm_q16)
{
    /* 米Q16直接乘1000，保留Q16的亚毫米部分。 */
    *value_mm_q16 = (int64_t)value_q16_m * 1000ll;
    return 1;
}

static void handle_complete_task(uint64_t now64, uint32_t now_us)
{
    const uint8_t *p = fragment_payload;
    nav_task_t task;
    uint32_t action = p[1];
    uint32_t mode = p[2];
    uint32_t flags = p[3];
    uint32_t task_id = read_u32_be(p + 4);
    uint32_t domain = read_u32_be(p + 8);
    uint32_t boot_id = read_u32_be(p + 12);
    uint64_t expiry_raw = read_u64_be(p + 40);
    uint64_t sent_raw = read_u64_be(p + 56);
    uint32_t expiry_low = (uint32_t)expiry_raw;
    uint32_t sent_low = (uint32_t)sent_raw;
    uint32_t age_us = now_us - sent_low;
    uint32_t remaining_us = expiry_low - now_us;
    uint32_t lifetime_us = expiry_low - sent_low;
    int32_t goal_x_q16_m = read_s32_be(p + 16);
    int32_t goal_y_q16_m = read_s32_be(p + 20);
    int32_t hold_yaw_q16 = read_s32_be(p + 24);
    int32_t final_yaw_q16 = read_s32_be(p + 28);
    uint32_t max_speed = read_u16_be(p + 32);
    uint32_t max_accel = read_u16_be(p + 34);
    uint32_t tolerance = read_u16_be(p + 36);
    uint32_t yaw_tolerance = read_u16_be(p + 38);

    if (p[0] != 1u || action < 1u || action > 3u || mode > 3u ||
        (flags & ~NAV_TASK_FLAGS_MASK) != 0u ||
        task_id == 0u || domain == 0u || boot_id == 0u || boot_id != session_boot_id ||
        domain != session_domain || fragment_token != session_token ||
        !payload_reserved_zero(p)) {
        publish_event(BX72_NAV_STATUS_REJECTED, task_id, now64);
        return;
    }
    if (!clock_valid || (expiry_raw >> 32) != 0u || (sent_raw >> 32) != 0u ||
        age_us > NAV_TASK_MAX_AGE_US || age_us >= 0x80000000u ||
        remaining_us == 0u || remaining_us >= 0x80000000u ||
        lifetime_us == 0u || lifetime_us > NAV_TASK_MAX_LIFETIME_US ||
        lifetime_us >= 0x80000000u) {
        publish_event(BX72_NAV_STATUS_REJECTED, task_id, now64);
        return;
    }
    if (!task_sequence_newer(task_id)) {
        publish_event(BX72_NAV_STATUS_REJECTED, task_id, now64);
        return;
    }
    if (action == 1u) {
        /* 同一微秒内无法区分重置前后的发送，必须严格晚于重置。 */
        if (task_reset_barrier_valid &&
            (now64 < task_reset_barrier_us ||
             (uint64_t)age_us >= now64 - task_reset_barrier_us)) {
            publish_event(BX72_NAV_STATUS_REJECTED, task_id, now64);
            return;
        }
        if (mode < 2u || max_speed == 0u || max_speed > NAV_TASK_MAX_SPEED_MM_S || max_accel == 0u ||
            max_accel > 1000u || tolerance == 0u || tolerance > 500u ||
            yaw_tolerance == 0u || yaw_tolerance > 1000u ||
            hold_yaw_q16 < -205887 || hold_yaw_q16 > 205887 ||
            final_yaw_q16 < -205887 || final_yaw_q16 > 205887) {
            publish_event(BX72_NAV_STATUS_REJECTED, task_id, now64);
            return;
        }
        q16m_to_mm_q16(goal_x_q16_m, &task.goal_x_mm_q16);
        q16m_to_mm_q16(goal_y_q16_m, &task.goal_y_mm_q16);
        task.hold_yaw_mrad = (hold_yaw_q16 * 1000) / 65536;
        task.final_yaw_mrad = (final_yaw_q16 * 1000) / 65536;
        task.task_id = task_id;
        task.mode = mode;
        task.flags = flags;
        task.max_speed_mm_s = max_speed;
        task.max_accel_mm_s2 = max_accel;
        /* 基础停止阈值最多100mm；更松的请求仍按安全阈值执行。 */
        task.tolerance_mm = tolerance > 100u ? 100u : tolerance;
        task.yaw_tolerance_mrad = yaw_tolerance;
        task.expiry_us = now64 + remaining_us;
        if (!enqueue_task(&task)) {
            publish_event(BX72_NAV_STATUS_REJECTED, task_id, now64);
            return;
        }
        last_task_id = task_id;
        have_last_task_id = 1u;
        if (task_count == 1u && latest_pose_valid)
            activate_current_task(&latest_pose, now64);
        publish_event(BX72_NAV_STATUS_RECEIVING, task_id, now64);
        return;
    }
    last_task_id = task_id;
    have_last_task_id = 1u;
    clear_tasks(now64, BX72_NAV_STATUS_CANCELLED, task_id);
}

static int goal_arrived(const bx72_pose_t *pose, const nav_task_t *task,
                        uint64_t *distance_q16, int32_t *yaw_error)
{
    int64_t dx = task->goal_x_mm_q16 - (int64_t)pose->x_mm * NAV_Q16_ONE;
    int64_t dy = task->goal_y_mm_q16 - (int64_t)pose->y_mm * NAV_Q16_ONE;
    *distance_q16 = approx_distance_q16(dx, dy);
    *yaw_error = (task->flags & NAV_TASK_POSITION_ONLY) ? 0 :
        wrap_yaw_mrad(task->final_yaw_mrad - pose->yaw_mrad);
    return goal_position_reached(pose, task) &&
        ((task->flags & NAV_TASK_POSITION_ONLY) || abs_i32(*yaw_error) <= task->yaw_tolerance_mrad);
}

static void pop_current_task(uint64_t now64, const bx72_pose_t *pose)
{
    uint32_t completed = task_queue[task_head].task_id;
    task_head = (uint8_t)((task_head + 1u) % BX72_NAV_TASK_QUEUE_SIZE);
    --task_count;
    if (task_count == 0u) {
        active_task_id = 0u;
        task_start_initialized = 0u;
        /* 结束导航后不能恢复任务之前遗留的人工速度。 */
        manual_inhibited = 1u;
        /* 到达不是人工重新确认；后续独立任务仍使用原启动许可圆。 */
        reset_follow_state();
        publish_event(BX72_NAV_STATUS_ARRIVED, completed, now64);
    } else {
        activate_current_task(pose, now64);
    }
}

static void limit_vector_acceleration(bx72_nav_output_t *out, uint32_t max_accel)
{
    int32_t max_delta = (int32_t)((max_accel + 49u) / 50u);
    int32_t dx = out->vx_mm_s - previous_vx_mm_s;
    int32_t dy = out->vy_mm_s - previous_vy_mm_s;
    uint32_t delta = approx_distance_xy_mm(dx, dy);
    int32_t yaw_delta = (int32_t)((NAV_YAW_ACCEL_MRAD_S2 + 49u) / 50u);
    if (max_delta < 1) max_delta = 1;
    if (delta > (uint32_t)max_delta) {
        out->vx_mm_s = previous_vx_mm_s + (int32_t)(((int64_t)dx * max_delta) / delta);
        out->vy_mm_s = previous_vy_mm_s + (int32_t)(((int64_t)dy * max_delta) / delta);
    }
    out->wz_mrad_s = clamp_i32(out->wz_mrad_s,
        previous_wz_mrad_s - yaw_delta, previous_wz_mrad_s + yaw_delta);
}

static void reset_command_slew(void)
{
    previous_vx_mm_s = 0;
    previous_vy_mm_s = 0;
    previous_wz_mrad_s = 0;
}

void bx72_nav_task_input_lost(uint32_t now_us)
{
    uint64_t now64 = update_clock(now_us);
    fragment_collecting = 0u;
    clear_tasks(now64, BX72_NAV_STATUS_REJECTED, active_task_id);
}

void bx72_nav_fragment(const uint32_t words[8], uint32_t now_us)
{
    uint8_t frame[32];
    uint32_t i;
    uint32_t sequence;
    uint32_t index;
    uint64_t token;
    uint64_t now64 = update_clock(now_us);
    /* 输入来自硬件已校验的任务分片 FIFO。 */
    update_session(now64);
    for (i = 0u; i < 8u; ++i) {
        frame[i * 4u] = (uint8_t)(words[i] >> 24);
        frame[i * 4u + 1u] = (uint8_t)(words[i] >> 16);
        frame[i * 4u + 2u] = (uint8_t)(words[i] >> 8);
        frame[i * 4u + 3u] = (uint8_t)words[i];
    }
    sequence = read_u32_be(frame + 8);
    token = read_u64_be(frame + 12);
    index = frame[20];
    if (session_boot_id == 0u || session_domain == 0u ||
        token != session_token || token == 0u) {
        fragment_collecting = 0u;
        publish_event(BX72_NAV_STATUS_REJECTED, 0u, now64);
        return;
    }
    if (fragment_collecting &&
        (uint32_t)(now_us - fragment_started_us) > NAV_FRAGMENT_MAX_AGE_US) {
        fragment_collecting = 0u;
        publish_event(BX72_NAV_STATUS_REJECTED, 0u, now64);
    }
    if (index == 0u) {
        if (fragment_collecting) {
            fragment_collecting = 0u;
            publish_event(BX72_NAV_STATUS_REJECTED, 0u, now64);
            return;
        }
        fragment_collecting = 1u;
        fragment_next_index = 1u;
        fragment_next_sequence = sequence + 1u;
        fragment_token = token;
        fragment_started_us = now_us;
    } else if (!fragment_collecting || index != fragment_next_index ||
               sequence != fragment_next_sequence || token != fragment_token) {
        fragment_collecting = 0u;
        publish_event(BX72_NAV_STATUS_REJECTED, 0u, now64);
        return;
    } else {
        ++fragment_next_index;
        ++fragment_next_sequence;
    }
    for (i = 0u; i < 9u; ++i)
        fragment_payload[index * 9u + i] = frame[21u + i];
    if (index == 7u) {
        fragment_collecting = 0u;
        handle_complete_task(now64, now_us);
    }
}


static int32_t clamp_i64_to_i32(int64_t value)
{
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

static void odom_to_grid(const bx72_pose_t *odom, bx72_pose_t *grid)
{
    int32_t sine = bx72_sin_q15(map_yaw_offset);
    int32_t cosine = bx72_cos_q15(map_yaw_offset);
    int64_t dx = (int64_t)odom->x_mm - map_odom_anchor_x;
    int64_t dy = (int64_t)odom->y_mm - map_odom_anchor_y;
    *grid = *odom;
    grid->x_mm = clamp_i64_to_i32((int64_t)map_anchor_x +
        ((dx * cosine - dy * sine) >> 15));
    grid->y_mm = clamp_i64_to_i32((int64_t)map_anchor_y +
        ((dx * sine + dy * cosine) >> 15));
    grid->yaw_mrad = wrap_yaw_mrad(odom->yaw_mrad + map_yaw_offset);
}

static void task_goal_to_grid(const nav_task_t *task, bx72_pose_t *grid)
{
    bx72_pose_t goal = {0};
    goal.x_mm = clamp_i64_to_i32(task->goal_x_mm_q16 / NAV_Q16_ONE);
    goal.y_mm = clamp_i64_to_i32(task->goal_y_mm_q16 / NAV_Q16_ONE);
    odom_to_grid(&goal, grid);
}

static void reset_local_path(void)
{
    bx72_local_path_reset();
    detour_active = path_target_final = guide_goal_valid = 0u;
    direct_guide_active = 0u;
    path_refresh_pending = 0u;
    planner_active = planner_complete = selected_valid = 0u;
    observe_active = observe_settled = 0u;
}

static void block_local_path(uint32_t scan_generation, uint32_t now_us)
{
    int direct = direct_guide_active;
    reset_local_path();
    if (direct) {
        /* 直通受阻后立即搜索绕行，避免重复选择相同的直通引导。 */
        direct_guide_blocked = 1u;
        path_retry_pending = 0u;
        return;
    }
    path_retry_pending = 1u;
    path_blocked_scan = scan_generation;
    path_retry_us = now_us;
}

void bx72_nav_map_reset(const bx72_pose_t *pose, uint32_t now_us)
{
    uint64_t reset_us = now_extended_us;
    (void)pose;
    if (clock_initialized) {
        uint32_t delta = now_us - last_now_low;
        if (delta < 0x80000000u) reset_us += delta;
        task_reset_barrier_us = reset_us;
        task_reset_barrier_valid = 1u;
    }
    if (start_exemption_valid || task_count != 0u) start_confirmation_lost = 1u;
    if (task_count != 0u)
        clear_tasks(reset_us, BX72_NAV_STATUS_CANCELLED, active_task_id);
    fragment_collecting = 0u;
    latest_pose_valid = 0u;
    ++grid_frame_epoch;
    map_odom_anchor_x = map_odom_anchor_y = 0;
    map_anchor_x = map_anchor_y = map_yaw_offset = 0;
    map_initialized = 0u;
    map_angle_valid = map_synchronized = 0u;
    map_scan_started_us = map_complete_started_us = 0u;
    map_gap_latched = 1u;
    last_map_gap_reason = BX72_NAV_GAP_RESET;
    bx72_grid_reset(0, 0, now_us);
    start_exemption_valid = 0u;
    reset_local_path();
    path_retry_pending = 0u;
    planner_active = planner_complete = planner_best_valid = 0u;
    selected_valid = 0u;
}

static void ensure_grid(const bx72_pose_t *odom, uint32_t now_us)
{
    if (!map_initialized) {
        map_odom_anchor_x = odom->x_mm;
        map_odom_anchor_y = odom->y_mm;
        map_anchor_x = odom->x_mm;
        map_anchor_y = odom->y_mm;
        map_yaw_offset = 0;
        bx72_grid_reset(odom->x_mm, odom->y_mm, now_us);
        map_initialized = 1u;
    }
}

/* 小幅重定位可继续跟踪旧引导点；累计目标投影或方向变化过大则重算。 */
static int rebase_keeps_guide(const bx72_pose_t *grid_pose, const bx72_pose_t *goal)
{
    int32_t old_bearing, new_bearing;
    if (!detour_active || !guide_goal_valid ||
        approx_distance_xy_mm((int64_t)goal->x_mm - guide_goal_x_mm,
                              (int64_t)goal->y_mm - guide_goal_y_mm) > 100u) return 0;
    old_bearing = atan2_approx_cdeg64((int64_t)guide_goal_y_mm - grid_pose->y_mm,
                                     (int64_t)guide_goal_x_mm - grid_pose->x_mm);
    new_bearing = atan2_approx_cdeg64((int64_t)goal->y_mm - grid_pose->y_mm,
                                     (int64_t)goal->x_mm - grid_pose->x_mm);
    return abs_i32(wrap_yaw_mrad((int32_t)(((int64_t)(new_bearing - old_bearing) *
                                           NAV_PI_MRAD) / 18000))) <=
           NAV_PLAN_HEADING_CHANGE_MRAD;
}

void bx72_nav_rebase(const bx72_pose_t *old_odom, const bx72_pose_t *new_odom)
{
    bx72_pose_t old_grid, goal_grid;
    int keep_guide = 0;
    if (!map_initialized) return;
    if (old_odom == 0 || new_odom == 0 ||
        !(old_odom->flags & BX72_POSE_VALID) ||
        !(new_odom->flags & BX72_POSE_VALID) ||
        old_odom->stamp_us != new_odom->stamp_us) {
        bx72_nav_map_reset(new_odom, current_time_us());
        return;
    }
    odom_to_grid(old_odom, &old_grid);
    map_odom_anchor_x = new_odom->x_mm;
    map_odom_anchor_y = new_odom->y_mm;
    map_anchor_x = old_grid.x_mm;
    map_anchor_y = old_grid.y_mm;
    map_yaw_offset = wrap_yaw_mrad(old_grid.yaw_mrad - new_odom->yaw_mrad);
    if (observe_active)
        observe_target_yaw_mrad = wrap_yaw_mrad(observe_target_yaw_mrad +
            wrap_yaw_mrad(new_odom->yaw_mrad - old_odom->yaw_mrad));
    ++transform_revision;
    if (task_count != 0u) {
        enum bx72_local_path_state state = bx72_local_path_state();
        int final;
        task_goal_to_grid(&task_queue[task_head], &goal_grid);
        keep_guide = rebase_keeps_guide(&old_grid, &goal_grid);
        if (!detour_active && selected_valid && planner_at_goal && selected_vx == 0 &&
            goal_position_reached(new_odom, &task_queue[task_head])) keep_guide = 1;
        if (detour_active && !keep_guide) {
            reset_local_path();
            path_retry_pending = 0u;
        } else if (state != BX72_PATH_IDLE &&
                   !bx72_local_path_retarget(goal_grid.x_mm, goal_grid.y_mm)) {
            if (keep_guide && !path_target_final) {
                /* 旧搜索若只能给出非最终引导点，可继续完成，避免被反复重启。 */
                if (!(state == BX72_PATH_WORKING && path_refresh_pending &&
                      !bx72_local_path_goal_inside())) {
                    bx72_local_path_reset();
                    path_refresh_pending = 1u;
                }
            } else {
                reset_local_path();
                keep_guide = 0;
            }
            path_retry_pending = 0u;
        } else if (state != BX72_PATH_IDLE && detour_active &&
                   bx72_local_path_target(&detour_x_mm, &detour_y_mm, &final)) {
            path_target_final = final != 0;
        }
    }
    /* 校正后后台重算；旧车体系动作每次输出前仍以新位姿和地图复核。 */
    planner_active = planner_complete = planner_best_valid = 0u;
    if (!keep_guide) selected_valid = 0u;
    latest_pose_valid = 0u;
}

int bx72_nav_grid_snapshot(bx72_nav_grid_snapshot_t *snapshot,
                           uint8_t cells[BX72_GRID_EXPORT_BYTES], uint32_t deadline_us)
{
    bx72_grid_info_t info;
    uint32_t offset;
    /* 同一次调用内分块检查截止；未完成帧不发布，也不跨主循环交错建图。 */
    for (offset = 0u; offset < BX72_GRID_EXPORT_BYTES; offset += 16u) {
        if ((int32_t)(deadline_us - current_time_us()) <= 1000 ||
            bx72_grid_export(offset, cells + offset, 16u) != 16u) return 0;
    }
    if ((int32_t)(deadline_us - current_time_us()) <= 1000) return 0;
    bx72_grid_info(&info);
    snapshot->boot_id = session_boot_id;
    snapshot->odom_domain = session_domain;
    snapshot->generation = info.generation;
    snapshot->scan_generation = info.scan_generation;
    snapshot->origin_x_cell = info.origin_x_cell;
    snapshot->origin_y_cell = info.origin_y_cell;
    snapshot->complete_scan_stamp_us = info.complete_scan_stamp_us;
    snapshot->last_ray_stamp_us = info.last_ray_stamp_us;
    snapshot->map_anchor_x_mm = map_anchor_x;
    snapshot->map_anchor_y_mm = map_anchor_y;
    snapshot->odom_anchor_x_mm = map_odom_anchor_x;
    snapshot->odom_anchor_y_mm = map_odom_anchor_y;
    snapshot->map_yaw_offset_mrad = map_yaw_offset;
    snapshot->grid_frame_epoch = grid_frame_epoch;
    snapshot->transform_revision = transform_revision;
    snapshot->flags = (map_initialized ? 1u : 0u) |
        (info.fresh ? 2u : 0u) | (info.scan_generation ? 4u : 0u) |
        (map_gap_latched ? 8u : 0u) | (start_confirmation_lost ? 16u : 0u);
    return 1;
}

void bx72_nav_lidar_gap(uint32_t now_us, uint32_t reason)
{
    (void)now_us;
    map_angle_valid = map_synchronized = 0u;
    map_scan_started_us = 0u;
    map_gap_latched = 1u;
    last_map_gap_reason = (uint8_t)reason;
    bx72_grid_gap();
}

void bx72_nav_lidar_validated(uint32_t angle_cdeg, uint32_t distance_mm, uint32_t hit,
                              const bx72_pose_t *pose, uint32_t stamp_us, uint32_t now_us)
{
    uint32_t delta;
    int wrapped = 0;
    bx72_pose_t grid_pose;
    ensure_grid(pose, now_us);
    if (map_angle_valid) {
        wrapped = angle_cdeg < map_previous_angle &&
                  map_previous_angle >= 35000u && angle_cdeg <= 1000u;
        delta = angle_cdeg >= map_previous_angle ? angle_cdeg - map_previous_angle :
                36000u - map_previous_angle + angle_cdeg;
        if ((!wrapped && angle_cdeg < map_previous_angle) || delta > NAV_LIDAR_GAP_MAX_CDEG) {
            bx72_nav_lidar_gap(now_us, BX72_NAV_GAP_ANGLE);
            wrapped = 0;
        }
    }
    if (wrapped) {
        if (map_synchronized) {
            bx72_grid_info_t before, after;
            bx72_grid_info(&before);
            bx72_grid_scan_complete(now_us);
            bx72_grid_info(&after);
            if (after.scan_generation != before.scan_generation) {
                map_gap_latched = 0u;
                map_complete_started_us = map_scan_started_us;
            }
        } else {
            map_synchronized = 1u;
        }
        bx72_grid_scan_begin();
        map_scan_started_us = stamp_us;
    }
    map_previous_angle = angle_cdeg;
    map_angle_valid = 1u;
    odom_to_grid(pose, &grid_pose);
    (void)bx72_grid_lidar_validated(angle_cdeg, distance_mm, hit,
                                    &grid_pose, stamp_us, now_us);
    last_lidar_us = stamp_us;
    have_lidar_sample = 1u;
}

uint32_t bx72_nav_diagnostic_status(void)
{
    bx72_grid_info_t info;
    uint32_t flags = BX72_NAV_DIAG_GRID_V1;
    bx72_grid_info(&info);
    if (info.fresh) flags |= BX72_NAV_DIAG_MAP_FRESH;
    if (info.scan_generation != 0u) flags |= BX72_NAV_DIAG_MAP_READY;
    if (map_gap_latched) flags |= BX72_NAV_DIAG_GAP_LATCHED;
    if (planner_active && !planner_complete) flags |= BX72_NAV_DIAG_PLANNING;
    if (progress_blocked) flags |= BX72_NAV_DIAG_PROGRESS_BLOCKED;
    if (unknown_in_swept_corridor) flags |= BX72_NAV_DIAG_UNKNOWN_PATH;
    if (map_gap_latched || !info.fresh) flags |= BX72_NAV_DIAG_RECOVERY_REQUIRED;
    flags |= (uint32_t)last_map_gap_reason << BX72_NAV_DIAG_GAP_REASON_SHIFT;
    if (start_confirmation_lost) flags |= BX72_NAV_DIAG_START_LOST;
    return flags;
}

#define NAV_WHEEL_LIMIT_MM_S 500
#define NAV_WHEEL_STEP_MM_S 10
#define NAV_WHEEL_GEOMETRY_MM 178
#define NAV_HOLD_STEPS 10u
#define NAV_BRAKE_STEPS 50u
#define NAV_SWEEP_GROUP_STEPS 10u
#define NAV_FRONT_GROUP_STEPS 5u
/* 200ms弦：轮速<=500mm/s、每20ms每轮变化<=10mm/s，角速<=850mrad/s。
 * 四轮变换及整数舍入使相邻机身速度矢量变化小于13mm/s；叠加转向及
 * 三角误差，相邻世界速度变化小于24mm/s，10段对弦偏差至多6mm。
 * 端点整毫米、Q8积分和三角误差余量后仍小于10mm。该扩展只用于已知
 * 占用碰撞检查，不扩大未知许可圆；推导不代表实车惯性已标定。 */
#define NAV_SWEEP_MARGIN_MM 10u
#define NAV_EVAL_BUDGET_US 4000u

static uint32_t current_time_us(void)
{
#if defined(BX72_NAV_TEST_TIMED)
    extern uint32_t bx72_nav_test_live_time(void);
    return bx72_nav_test_live_time();
#elif defined(BX72_NAV_HOST_TEST)
    return last_now_low;
#else
    return mmio_read_offset(BX72_IO_SNAPSHOT);
#endif
}

static int evaluation_timed_out(uint32_t started_us)
{
    if ((uint32_t)(current_time_us() - started_us) < NAV_EVAL_BUDGET_US) return 0;
    /* 超时后先丢弃所有候选，控制拍确认零速前禁止后台重新填充。 */
    planner_active = planner_complete = planner_best_valid = selected_valid = 0u;
    planner_timeout_pending = 1u;
    return 1;
}

static void wheel_vector(int32_t vx, int32_t vy, int32_t wz, int32_t wheel[4])
{
    int32_t rotation = (wz * NAV_WHEEL_GEOMETRY_MM) / 1000;
    wheel[0] = vx - vy - rotation;
    wheel[1] = vx + vy + rotation;
    wheel[2] = vx + vy - rotation;
    wheel[3] = vx - vy + rotation;
}

static void wheel_approach(int32_t wheel[4], const int32_t target[4])
{
    int32_t delta[4];
    uint32_t i, peak = 0u;
    for (i = 0u; i < 4u; ++i) {
        delta[i] = target[i] - wheel[i];
        if (abs_i32(delta[i]) > peak) peak = abs_i32(delta[i]);
    }
    for (i = 0u; i < 4u; ++i)
        wheel[i] += peak > NAV_WHEEL_STEP_MM_S ?
            delta[i] * NAV_WHEEL_STEP_MM_S / (int32_t)peak : delta[i];
}

static void proposed_command(int32_t vx, int32_t wz, uint32_t accel,
                             int32_t *command_vx, int32_t *command_wz)
{
    bx72_nav_output_t out = {0};
    out.vx_mm_s = vx;
    out.wz_mrad_s = wz;
    limit_vector_acceleration(&out, accel);
    *command_vx = out.vx_mm_s;
    *command_wz = out.wz_mrad_s;
}

/* 前向未知按本次预测的整段通道一次遍历，避免逐点64位除法和重复查格。
 * 矩形外扩一格半对齐误差并只检查原始车前半平面；固定启动圆仅豁免
 * 完全在圆内的格。侧后盲区不凭空标记为空闲，已知占用由完整扫掠检查。 */
typedef struct { int32_t min_f, max_f, min_s, max_s; } nav_front_bounds_t;
#define NAV_FRONT_CELL_PAD_MM 46

static int32_t floor_cell32(int32_t mm)
{
    int32_t q = mm / (int32_t)BX72_GRID_CELL_MM;
    if (mm % (int32_t)BX72_GRID_CELL_MM < 0) --q;
    return q;
}

static void include_front_bounds(nav_front_bounds_t *b, const bx72_pose_t *start,
                                 const bx72_pose_t *current)
{
    int32_t base_s = bx72_sin_q15(start->yaw_mrad);
    int32_t base_c = bx72_cos_q15(start->yaw_mrad);
    int32_t rel = wrap_yaw_mrad(current->yaw_mrad - start->yaw_mrad);
    int32_t sine = bx72_sin_q15(rel), cosine = bx72_cos_q15(rel);
    int32_t dx = current->x_mm - start->x_mm, dy = current->y_mm - start->y_mm;
    int32_t center_f = (int32_t)(((int64_t)dx * base_c + (int64_t)dy * base_s) >> 15);
    int32_t center_s = (int32_t)((-(int64_t)dx * base_s + (int64_t)dy * base_c) >> 15);
    int f, side;
    const int half_width = (int)BX72_GRID_BODY_HALF_WIDTH_MM;
    for (f = 100; f <= 300; f += 200) {
        for (side = -half_width; side <= half_width; side += 2 * half_width) {
            int32_t pf = center_f + (f * cosine - side * sine) / 32768;
            int32_t ps = center_s + (f * sine + side * cosine) / 32768;
            if (pf < b->min_f) b->min_f = pf;
            if (pf > b->max_f) b->max_f = pf;
            if (ps < b->min_s) b->min_s = ps;
            if (ps > b->max_s) b->max_s = ps;
        }
    }
}

/* 181mm车体半径加2mm量化余量，须完全位于固定的231mm确认圆内。 */
static __attribute__((noinline)) int drift_inside_start(int32_t x, int32_t y)
{
    int64_t dx = (int64_t)x - start_exemption.x_mm;
    int64_t dy = (int64_t)y - start_exemption.y_mm;
    if (!start_exemption_valid || dx < -48 || dx > 48 || dy < -48 || dy > 48) return 0;
    return (int32_t)dx * (int32_t)dx + (int32_t)dy * (int32_t)dy <= 48 * 48;
}

static int front_corridor_known(const bx72_pose_t *start, const nav_front_bounds_t *b,
                                uint32_t started_us)
{
    int32_t sine = bx72_sin_q15(start->yaw_mrad), cosine = bx72_cos_q15(start->yaw_mrad);
    int64_t sensor_forward = (int64_t)bx72_lidar_offset_component(cosine) * cosine +
                             (int64_t)bx72_lidar_offset_component(sine) * sine;
    int32_t min_x = INT32_MAX, max_x = INT32_MIN, min_y = INT32_MAX, max_y = INT32_MIN;
    int32_t first_x, last_x, first_y, last_y, x, y;
    int f, side;
    for (f = 0; f < 2; ++f) for (side = 0; side < 2; ++side) {
        int32_t lf = f ? b->max_f + 50 : b->min_f - 50;
        int32_t ls = side ? b->max_s + 50 : b->min_s - 50;
        int32_t wx = clamp_i64_to_i32((int64_t)start->x_mm +
                                    (((int64_t)lf * cosine - (int64_t)ls * sine) >> 15));
        int32_t wy = clamp_i64_to_i32((int64_t)start->y_mm +
                                    (((int64_t)lf * sine + (int64_t)ls * cosine) >> 15));
        if (wx < min_x) min_x = wx;
        if (wx > max_x) max_x = wx;
        if (wy < min_y) min_y = wy;
        if (wy > max_y) max_y = wy;
    }
    first_x = floor_cell32(min_x); last_x = floor_cell32(max_x);
    first_y = floor_cell32(min_y); last_y = floor_cell32(max_y);
    /* 一圈局部图有界；任何异常预测均不能产生运动。 */
    if ((int64_t)last_x - first_x > 64 || (int64_t)last_y - first_y > 64) return 0;
    for (y = first_y; y <= last_y; ++y) {
        for (x = first_x; x <= last_x; ++x) {
            uint8_t cell = bx72_grid_cell_at(x, y);
            int64_t dx, dy, forward_q15, forward, lateral;
            if (cell == BX72_GRID_FREE || cell == BX72_GRID_OCCUPIED) continue;
            dx = (int64_t)x * 50 + 25 - start->x_mm;
            dy = (int64_t)y * 50 + 25 - start->y_mm;
            forward_q15 = dx * cosine + dy * sine;
            forward = forward_q15 >> 15;
            lateral = (-dx * sine + dy * cosine) >> 15;
            /* 沿用前向格中心检查：雷达后方未知不保证安全，已知占用仍由整车扫掠检查。 */
            if (forward_q15 <= sensor_forward ||
                forward < b->min_f - NAV_FRONT_CELL_PAD_MM ||
                forward > b->max_f + NAV_FRONT_CELL_PAD_MM ||
                lateral < b->min_s - NAV_FRONT_CELL_PAD_MM ||
                lateral > b->max_s + NAV_FRONT_CELL_PAD_MM) continue;
            if (cell == BX72_GRID_OUTSIDE) return 0;
            if (bx72_grid_unknown_blocking(x, y, start_exemption_valid ? &start_exemption : 0))
                return 0;
        }
        if (evaluation_timed_out(started_us)) return -1;
    }
    return 1;
}

/* 0=碰撞或未知，1=安全，-1=本片超时。输出是检查过的局部方向预览。 */
static int evaluate_motion(const bx72_pose_t *odom, int32_t command_vx,
                           int32_t command_wz, int check_front,
                           int32_t *end_grid_x, int32_t *end_grid_y,
                           uint8_t *unknown, uint32_t started_us)
{
    static const int32_t zero_wheels[4] = {0, 0, 0, 0};
    bx72_pose_t p, start;
    nav_front_bounds_t front_bounds = {INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN};
    nav_front_bounds_t drift_bounds = {0, 0, 0, 0};
    struct {
        int32_t x0, y0, x1, y1, yaw;
        uint32_t yaw_delta;
    } sweeps[(NAV_HOLD_STEPS + NAV_BRAKE_STEPS + NAV_SWEEP_GROUP_STEPS - 1u) /
             NAV_SWEEP_GROUP_STEPS];
    int32_t wheel[4], prior[4], desired[4];
    int64_t x_q8, y_q8, yaw_q8, segment_min_yaw_q8, segment_max_yaw_q8;
    int32_t segment_x, segment_y;
    uint32_t i, j, sweep_count = 0u;
    int drift = odom->vx_mm_s < -(int32_t)NAV_DRIFT_NOISE_MM_S ||
                abs_i32(odom->vy_mm_s) > NAV_DRIFT_NOISE_MM_S ||
                (command_vx == 0 && abs_i32(odom->vx_mm_s) > NAV_DRIFT_NOISE_MM_S);
    *unknown = 0u;
    if (command_vx < 0 || abs_i32(command_wz) > NAV_AUTO_MAX_YAW_MRAD_S ||
        odom->vx_mm_s < -(int32_t)NAV_DRIFT_NOISE_MM_S ||
        abs_i32(odom->vx_mm_s) > NAV_WHEEL_LIMIT_MM_S ||
        abs_i32(odom->vy_mm_s) > NAV_WHEEL_LIMIT_MM_S ||
        abs_i32(odom->wz_mrad_s) > NAV_MOTION_MAX_YAW_MRAD_S)
        return 0;
    odom_to_grid(odom, &p);
    start = p;
    if (check_front) include_front_bounds(&front_bounds, &start, &p);
    wheel_vector(odom->vx_mm_s, odom->vy_mm_s, odom->wz_mrad_s, wheel);
    wheel_vector(previous_vx_mm_s, previous_vy_mm_s, previous_wz_mrad_s, prior);
    wheel_vector(command_vx, 0, command_wz, desired);
    for (j = 0u; j < 4u; ++j) {
        /* 软件上一命令与实测可能存在电机跟踪延迟；同向取较大值。 */
        if ((wheel[j] >= 0 && prior[j] >= 0) || (wheel[j] <= 0 && prior[j] <= 0)) {
            if (abs_i32(prior[j]) > abs_i32(wheel[j])) wheel[j] = prior[j];
        } else {
            if (abs_i32(prior[j]) > 10u && abs_i32(wheel[j]) > 10u) return 0;
            if (abs_i32(wheel[j]) <= 10u && abs_i32(prior[j]) > 10u) wheel[j] = prior[j];
        }
        if (abs_i32(wheel[j]) > NAV_WHEEL_LIMIT_MM_S ||
            abs_i32(desired[j]) > NAV_WHEEL_LIMIT_MM_S) return 0;
    }
    x_q8 = (int64_t)p.x_mm * 256;
    y_q8 = (int64_t)p.y_mm * 256;
    yaw_q8 = (int64_t)p.yaw_mrad * 256;
    segment_min_yaw_q8 = segment_max_yaw_q8 = yaw_q8;
    segment_x = p.x_mm;
    segment_y = p.y_mm;
    if (bx72_grid_sweep_body_occupied(segment_x, segment_y, segment_x, segment_y,
                                      p.yaw_mrad, 0u, 0u) &
        (BX72_GRID_CHECK_OCCUPIED | BX72_GRID_CHECK_OUTSIDE)) return 0;
    *end_grid_x = p.x_mm;
    *end_grid_y = p.y_mm;
    for (i = 0u; i < NAV_HOLD_STEPS + NAV_BRAKE_STEPS; ++i) {
        int32_t vx, vy, wz, sine, cosine;
        int32_t new_x, new_y;
        if (i < NAV_HOLD_STEPS) wheel_approach(wheel, desired);
        vx = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) / 4;
        vy = (-wheel[0] + wheel[1] + wheel[2] - wheel[3]) / 4;
        wz = (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) * 1000 /
             (4 * NAV_WHEEL_GEOMETRY_MM);
        if (vx < -(int32_t)NAV_DRIFT_NOISE_MM_S ||
            abs_i32(vy) > NAV_WHEEL_LIMIT_MM_S || abs_i32(wz) > NAV_MOTION_MAX_YAW_MRAD_S) return 0;
        if (vx < -(int32_t)NAV_DRIFT_NOISE_MM_S || abs_i32(vy) > NAV_DRIFT_NOISE_MM_S ||
            (command_vx == 0 && abs_i32(vx) > NAV_DRIFT_NOISE_MM_S)) drift = 1;
        p.yaw_mrad = wrap_yaw_mrad((int32_t)(yaw_q8 / 256) + wz / 100);
        sine = bx72_sin_q15(p.yaw_mrad);
        cosine = bx72_cos_q15(p.yaw_mrad);
        x_q8 += (((int64_t)vx * cosine - (int64_t)vy * sine) * 256) / (50 * 32768);
        y_q8 += (((int64_t)vx * sine + (int64_t)vy * cosine) * 256) / (50 * 32768);
        yaw_q8 += (int64_t)wz * 256 / 50;
        if (yaw_q8 < segment_min_yaw_q8) segment_min_yaw_q8 = yaw_q8;
        if (yaw_q8 > segment_max_yaw_q8) segment_max_yaw_q8 = yaw_q8;
        if (i >= NAV_HOLD_STEPS) wheel_approach(wheel, zero_wheels);
        new_x = clamp_i64_to_i32(x_q8 / 256);
        new_y = clamp_i64_to_i32(y_q8 / 256);
        {
            int32_t dx = (int32_t)((int64_t)new_x - start.x_mm);
            int32_t dy = (int32_t)((int64_t)new_y - start.y_mm);
            if (dx < drift_bounds.min_f) drift_bounds.min_f = dx;
            if (dx > drift_bounds.max_f) drift_bounds.max_f = dx;
            if (dy < drift_bounds.min_s) drift_bounds.min_s = dy;
            if (dy > drift_bounds.max_s) drift_bounds.max_s = dy;
        }
        if (i + 1u == NAV_HOLD_STEPS) {
            *end_grid_x = new_x;
            *end_grid_y = new_y;
        }
        if (((i + 1u) % NAV_SWEEP_GROUP_STEPS) == 0u ||
            i + 1u == NAV_HOLD_STEPS + NAV_BRAKE_STEPS ||
            (i >= NAV_HOLD_STEPS && !wheel[0] && !wheel[1] && !wheel[2] && !wheel[3])) {
            int64_t reference_mrad = ((segment_min_yaw_q8 + segment_max_yaw_q8) / 2) / 256;
            uint64_t low_delta = abs_i64(segment_min_yaw_q8 - reference_mrad * 256);
            uint64_t high_delta = abs_i64(segment_max_yaw_q8 - reference_mrad * 256);
            uint64_t max_delta = low_delta > high_delta ? low_delta : high_delta;
            sweeps[sweep_count].x0 = segment_x;
            sweeps[sweep_count].y0 = segment_y;
            sweeps[sweep_count].x1 = new_x;
            sweeps[sweep_count].y1 = new_y;
            sweeps[sweep_count].yaw = wrap_yaw_mrad((int32_t)reference_mrad);
            sweeps[sweep_count].yaw_delta = (uint32_t)((max_delta + 255u) / 256u);
            if (bx72_grid_sweep_body_occupied(segment_x, segment_y, new_x, new_y,
                    sweeps[sweep_count].yaw, sweeps[sweep_count].yaw_delta,
                    NAV_SWEEP_MARGIN_MM) &
                (BX72_GRID_CHECK_OCCUPIED | BX72_GRID_CHECK_OUTSIDE)) return 0;
            ++sweep_count;
            segment_x = new_x;
            segment_y = new_y;
            segment_min_yaw_q8 = segment_max_yaw_q8 = yaw_q8;
            if (evaluation_timed_out(started_us)) return -1;
        }
        if (check_front && (((i + 1u) % NAV_FRONT_GROUP_STEPS) == 0u ||
            (i >= NAV_HOLD_STEPS && !wheel[0] && !wheel[1] && !wheel[2] && !wheel[3]))) {
            p.x_mm = new_x;
            p.y_mm = new_y;
            p.yaw_mrad = wrap_yaw_mrad((int32_t)(yaw_q8 / 256));
            include_front_bounds(&front_bounds, &start, &p);
            if (evaluation_timed_out(started_us)) return -1;
        }
        if (i >= NAV_HOLD_STEPS && !wheel[0] && !wheel[1] && !wheel[2] && !wheel[3]) break;
    }
    if (check_front) {
        int known = front_corridor_known(&start, &front_bounds, started_us);
        if (known < 0) return -1;
        if (!known) { *unknown = 1u; return 0; }
    }
    /* 181mm物理包络及整段中心轨迹须完全在唯一的231mm确认圆内。 */
    /* 纯转向的毫米级积分量化不触发未知区域停车；实际占用扫掠仍逐段复核。 */
    if (drift && (command_vx != 0 ||
                  abs_i32(drift_bounds.min_f) > NAV_TURN_DRIFT_TOLERANCE_MM ||
                  abs_i32(drift_bounds.max_f) > NAV_TURN_DRIFT_TOLERANCE_MM ||
                  abs_i32(drift_bounds.min_s) > NAV_TURN_DRIFT_TOLERANCE_MM ||
                  abs_i32(drift_bounds.max_s) > NAV_TURN_DRIFT_TOLERANCE_MM) &&
                  !(drift_inside_start(start.x_mm + drift_bounds.min_f, start.y_mm + drift_bounds.min_s) &&
                   drift_inside_start(start.x_mm + drift_bounds.max_f, start.y_mm + drift_bounds.min_s) &&
                   drift_inside_start(start.x_mm + drift_bounds.min_f, start.y_mm + drift_bounds.max_s) &&
                   drift_inside_start(start.x_mm + drift_bounds.max_f, start.y_mm + drift_bounds.max_s))) {
        /* 真实侧漂须观察实际车体扫掠，不再用231mm方框包含墙后未知。 */
        for (j = 0u; j < sweep_count; ++j) {
            uint32_t flags = bx72_grid_sweep_body_unknown(sweeps[j].x0, sweeps[j].y0,
                sweeps[j].x1, sweeps[j].y1, sweeps[j].yaw, sweeps[j].yaw_delta,
                NAV_SWEEP_MARGIN_MM, start_exemption_valid ? &start_exemption : 0);
            if (evaluation_timed_out(started_us)) return -1;
            if (flags & (BX72_GRID_CHECK_UNKNOWN | BX72_GRID_CHECK_OUTSIDE)) {
                *unknown = 1u;
                return 0;
            }
        }
    }
    return evaluation_timed_out(started_us) ? -1 : 1;
}

static int32_t task_heading_error(const bx72_pose_t *pose, const nav_task_t *task)
{
    bx72_pose_t grid_pose;
    int64_t dx, dy;
    int32_t yaw = pose->yaw_mrad;
    if (detour_active) {
        odom_to_grid(pose, &grid_pose);
        dx = (int64_t)detour_x_mm - grid_pose.x_mm;
        dy = (int64_t)detour_y_mm - grid_pose.y_mm;
        yaw = grid_pose.yaw_mrad;
    } else {
        dx = task->goal_x_mm_q16 - (int64_t)pose->x_mm * NAV_Q16_ONE;
        dy = task->goal_y_mm_q16 - (int64_t)pose->y_mm * NAV_Q16_ONE;
    }
    int32_t bearing_cdeg = atan2_approx_cdeg64(dy, dx);
    int32_t bearing_mrad = (int32_t)(((int64_t)bearing_cdeg * NAV_PI_MRAD) / 18000);
    return wrap_yaw_mrad(bearing_mrad - yaw);
}

static uint32_t detour_distance_mm(const bx72_pose_t *pose)
{
    bx72_pose_t grid_pose;
    odom_to_grid(pose, &grid_pose);
    return approx_distance_xy_mm((int64_t)detour_x_mm - grid_pose.x_mm,
                                 (int64_t)detour_y_mm - grid_pose.y_mm);
}

/* 后台续段不能在拐点尚未到达时倒退或突然选到另一侧。
 * 夹角超过约 26° 或新点未越过旧点时，先跟踪旧点。 */
static int path_guide_aligned(const bx72_pose_t *grid_pose, int32_t next_x, int32_t next_y)
{
    int64_t ax = (int64_t)detour_x_mm - grid_pose->x_mm;
    int64_t ay = (int64_t)detour_y_mm - grid_pose->y_mm;
    int64_t bx = (int64_t)next_x - grid_pose->x_mm;
    int64_t by = (int64_t)next_y - grid_pose->y_mm;
    int64_t dot, cross;
    if (abs_i64(ax) > BX72_GRID_SIZE * BX72_GRID_CELL_MM ||
        abs_i64(ay) > BX72_GRID_SIZE * BX72_GRID_CELL_MM ||
        abs_i64(bx) > BX72_GRID_SIZE * BX72_GRID_CELL_MM ||
        abs_i64(by) > BX72_GRID_SIZE * BX72_GRID_CELL_MM) return 0;
    dot = ax * bx + ay * by;
    cross = ax * by - ay * bx;
    return dot > ax * ax + ay * ay && abs_i64(cross) <= (uint64_t)dot / 2u;
}

/* 候选和执行时使用同一个接近目标速度下限；仅未到位时调用。 */
static uint32_t near_goal_speed(uint32_t distance_mm, uint32_t speed_limit)
{
    uint32_t limit = distance_mm / 2u;
    if (limit < 1u) limit = 1u;
    return limit < speed_limit ? limit : speed_limit;
}

/* 中间引导点不是停车目标；临近时留出时间计算下一段。
 * 真正的航点仍沿用原终点减速和停稳判定。 */
static uint32_t guide_speed(uint32_t distance_mm, uint32_t speed_limit)
{
    if (detour_active && !path_target_final) {
        uint32_t limit = distance_mm > NAV_PATH_PREFETCH_MM ? speed_limit : distance_mm;
        if (limit < 1u) limit = 1u;
        return limit < speed_limit ? limit : speed_limit;
    }
    return near_goal_speed(distance_mm, speed_limit);
}

static void candidate_command(uint32_t index, const bx72_pose_t *pose,
                              const nav_task_t *task, int32_t *vx, int32_t *wz)
{
    static const int16_t turns[5] = {0, 150, -150, 300, -300};
    uint64_t distance_q16;
    uint32_t distance = detour_active ? detour_distance_mm(pose) :
        goal_distance_mm(pose, task, &distance_q16);
    int32_t error = task_heading_error(pose, task);
    *vx = 0;
    *wz = 0;
    if (observe_active || goal_position_reached(pose, task)) {
        error = wrap_yaw_mrad((observe_active ? observe_target_yaw_mrad :
                               task->final_yaw_mrad) - pose->yaw_mrad);
        /* π两侧的等价方向延续当前转向，避免每次重算时反向。 */
        if (abs_i32(error) >= NAV_PI_MRAD - NAV_REBASE_SMALL_YAW_MRAD &&
            selected_valid && selected_vx == 0 && selected_wz != 0)
            error = selected_wz > 0 ? NAV_PI_MRAD : -NAV_PI_MRAD;
        /* 仅原地转向设置非零目标下限；实际输出仍经过角加速度限制。 */
        if (index == 5u && error > 0)
            *wz = clamp_i32(error + error / 2,
                            NAV_AUTO_MIN_YAW_MRAD_S, NAV_AUTO_MAX_YAW_MRAD_S);
        else if (index == 6u && error < 0)
            *wz = clamp_i32(error + error / 2,
                            -NAV_AUTO_MAX_YAW_MRAD_S, -NAV_AUTO_MIN_YAW_MRAD_S);
        return;
    }
    if (index < 5u) {
        if (abs_i32(error) > 800u) return;
        *vx = (int32_t)guide_speed(distance, task->max_speed_mm_s);
        /* 首个候选按目标误差小幅修正，避免轻微偏航在窄通道积累。 */
        *wz = index == 0u ? clamp_i32(error, -150, 150) : turns[index];
    } else {
        *wz = index == 5u ? NAV_AUTO_MAX_YAW_MRAD_S : -NAV_AUTO_MAX_YAW_MRAD_S;
    }
}

static int64_t candidate_score(const bx72_pose_t *pose, const nav_task_t *task,
                               int32_t vx, int32_t wz)
{
    int32_t error = observe_active ? wrap_yaw_mrad(observe_target_yaw_mrad - pose->yaw_mrad) :
        goal_position_reached(pose, task) ?
        wrap_yaw_mrad(task->final_yaw_mrad - pose->yaw_mrad) :
        task_heading_error(pose, task);
    int32_t future_error = wrap_yaw_mrad(error - wz);
    /* 搜索负责决定绕行方向；评分只选择跟踪引导点的速度。
     * 每个候选和实际输出都先通过实时车体与制动扫掠。 */
    return (vx != 0 ? 1000000ll : 0ll) -
           (int64_t)abs_i32(future_error) * 100ll + (int64_t)vx * 10ll - abs_i32(wz);
}

static void begin_planning(const bx72_pose_t *pose, const nav_task_t *task)
{
    planner_pose = *pose;
    planner_task_id = task->task_id;
    planner_cursor = 0u;
    planner_expansions = 0u;
    planner_best_valid = 0u;
    planner_active = 1u;
    planner_complete = 0u;
}

static int planning_pose_changed(const bx72_pose_t *pose)
{
    return approx_distance_xy_mm((int64_t)pose->x_mm - planner_pose.x_mm,
                                 (int64_t)pose->y_mm - planner_pose.y_mm) > 100u ||
           abs_i32(wrap_yaw_mrad(pose->yaw_mrad - planner_pose.yaw_mrad)) > 250u;
}

static void planner_background_step(const bx72_pose_t *pose, uint32_t now_us)
{
    nav_task_t *task = &task_queue[task_head];
    int32_t vx, wz, command_vx, command_wz, end_x, end_y;
    int valid;
    uint8_t unknown;
    uint32_t started_us;
    (void)now_us;
    if (!planner_active || planner_task_id != task->task_id || planning_pose_changed(pose)) {
        begin_planning(pose, task);
    }
    candidate_command(planner_cursor, &planner_pose, task, &vx, &wz);
    if (vx != 0 || wz != 0) {
        started_us = current_time_us();
        proposed_command(vx, wz, task->max_accel_mm_s2, &command_vx, &command_wz);
        valid = evaluate_motion(&planner_pose, command_vx, command_wz,
                                command_vx > 0 || planner_pose.vx_mm_s > 20,
                                &end_x, &end_y, &unknown, started_us);
        if (valid < 0 || evaluation_timed_out(started_us)) return;
        if (unknown) unknown_in_swept_corridor = 1u;
        if (valid > 0) {
            int64_t score = candidate_score(&planner_pose, task, vx, wz);
            if (planner_timeout_pending || evaluation_timed_out(started_us)) return;
            if (!planner_best_valid || score > planner_best_score) {
                planner_best_valid = 1u;
                planner_best_score = score;
                planner_best_vx = vx;
                planner_best_wz = wz;
            }
        }
    }
    ++planner_cursor;
    planner_expansions = planner_cursor;
    if (planner_cursor == NAV_CANDIDATE_COUNT) {
        selected_valid = planner_best_valid;
        if (selected_valid) {
            selected_vx = planner_best_vx;
            selected_wz = planner_best_wz;
            planner_heading_error = task_heading_error(&planner_pose, task);
            planner_at_goal = goal_position_reached(&planner_pose, task);
        }
        planner_complete = 1u;
        planner_active = 0u;
    }
}

static void fill_output_metadata(bx72_nav_output_t *out, uint64_t now64)
{
    bx72_grid_info_t info;
    bx72_grid_info(&info);
    out->last_task_id = have_last_task_id ? last_task_id : 0u;
    out->planner_expansions = planner_expansions;
    out->map_version = info.generation;
    out->map_age_us = info.scan_generation != 0u ?
        timestamp_age((uint32_t)now64, info.complete_scan_stamp_us) : 0xffffffffu;
    out->autonomous = task_count != 0u;
    out->task_mode = task_count != 0u ? task_queue[task_head].mode : 1u;
    if (task_count != 0u) {
        const nav_task_t *task = &task_queue[task_head];
        out->expiry_us = (uint32_t)task->expiry_us;
        out->goal_x_q16 = (int32_t)(task->goal_x_mm_q16 / 1000ll);
        out->goal_y_q16 = (int32_t)(task->goal_y_mm_q16 / 1000ll);
    } else {
        out->expiry_us = 0u;
        out->goal_x_q16 = out->goal_y_q16 = 0;
    }
}

void bx72_nav_init(void)
{
    uint32_t boot_id, domain;
    uint64_t token;
    read_session(&boot_id, &domain, &token);
    task_head = task_count = 0u;
    active_task_id = 0u;
    task_start_distance_q16 = 0u;
    task_start_initialized = 0u;
    last_task_id = 0u;
    have_last_task_id = 0u;
    fragment_collecting = fragment_next_index = 0u;
    fragment_next_sequence = 0u;
    fragment_token = 0u;
    fragment_started_us = 0u;
    clock_initialized = 0u;
    clock_valid = 1u;
    last_now_low = 0u;
    now_extended_us = 0u;
    mode_initialized = command_sequence_initialized = 0u;
    last_mode_sequence = last_command_sequence = 0u;
    manual_motion_sequence_initialized = 0u;
    last_manual_motion_sequence = 0u;
    auto_enabled = manual_inhibited = 0u;
    bx72_manual_init(&manual_state);
    event_status = BX72_NAV_STATUS_IDLE;
    event_task_id = 0u;
    event_until_us = 0u;
    previous_vx_mm_s = previous_vy_mm_s = previous_wz_mrad_s = 0;
    latest_pose_valid = 0u;
    planner_expansions = 0u;
    planner_timeout_pending = 0u;
    grid_frame_epoch = transform_revision = 0u;
    start_exemption_valid = start_confirmation_lost = 0u;
    task_reset_barrier_valid = 0u;
    task_reset_barrier_us = 0u;
    have_lidar_sample = 0u;
    last_lidar_us = 0u;
    reset_follow_state();
    bx72_nav_map_reset(0, 0u);
    last_map_gap_reason = BX72_NAV_GAP_NONE;
    session_boot_id = boot_id;
    session_domain = domain;
    session_token = token;
    session_initialized = 1u;
}

void bx72_nav_step(const bx72_pose_t *pose, uint32_t now_us)
{
    bx72_grid_info_t info;
    bx72_pose_t grid_pose;
    uint64_t now64 = update_clock(now_us);
    update_session(now64);
    if (pose != 0 && (pose->flags & BX72_POSE_VALID) &&
        timestamp_recent(now_us, pose->stamp_us, NAV_POSE_MAX_AGE_US)) {
        latest_pose = *pose;
        latest_pose_valid = 1u;
        ensure_grid(pose, now_us);
        odom_to_grid(pose, &grid_pose);
        bx72_grid_recenter(grid_pose.x_mm, grid_pose.y_mm, now_us);
    } else latest_pose_valid = 0u;
    bx72_grid_advance(now_us);
    bx72_grid_info(&info);
    if (clock_valid && latest_pose_valid && task_count != 0u && info.fresh && !planner_inhibited &&
        start_exemption_valid && !start_confirmation_lost && !observe_settled &&
        !progress_blocked && !planner_timeout_pending) {
        nav_task_t *task = &task_queue[task_head];
        if (!goal_position_reached(&latest_pose, task)) {
            enum bx72_local_path_state state;
            bx72_pose_t goal_grid;
            int32_t next_x, next_y;
            int final;
            /* 沿目标方向提出短引导；候选及每拍输出仍复用矩形、未知和制动扫掠。
             * 失败时切换到 A*，此处不额外运行一遍长距离扫掠。 */
            if (direct_guide_active ||
                (!detour_active && !direct_guide_blocked && !path_retry_pending &&
                 bx72_local_path_state() == BX72_PATH_IDLE &&
                 abs_i32(task_heading_error(&latest_pose, task)) <= NAV_PLAN_HEADING_CHANGE_MRAD)) {
                int64_t dx, dy;
                uint32_t distance;
                task_goal_to_grid(task, &goal_grid);
                dx = (int64_t)goal_grid.x_mm - grid_pose.x_mm;
                dy = (int64_t)goal_grid.y_mm - grid_pose.y_mm;
                distance = approx_distance_xy_mm(dx, dy);
                if (direct_guide_active || distance > NAV_DIRECT_GUIDE_MM) {
                    uint32_t lookahead = distance > NAV_DIRECT_GUIDE_MM ? NAV_DIRECT_GUIDE_MM : distance;
                    if (distance != 0u) {
                        detour_x_mm = clamp_i64_to_i32((int64_t)grid_pose.x_mm + dx * lookahead / distance);
                        detour_y_mm = clamp_i64_to_i32((int64_t)grid_pose.y_mm + dy * lookahead / distance);
                        detour_active = direct_guide_active = guide_goal_valid = 1u;
                        guide_goal_x_mm = goal_grid.x_mm;
                        guide_goal_y_mm = goal_grid.y_mm;
                        path_target_final = distance <= NAV_DIRECT_GUIDE_MM;
                    }
                }
            }
            if (detour_active && !path_target_final && !path_refresh_pending && !observe_active &&
                detour_distance_mm(&latest_pose) <= NAV_PATH_PREFETCH_MM &&
                approx_distance_xy_mm((int64_t)grid_pose.x_mm - path_start_x_mm,
                                      (int64_t)grid_pose.y_mm - path_start_y_mm) >= NAV_PATH_PREFETCH_MOVE_MM) {
                /* 保留已选引导点和输出，在后台从当前位置搜索下一段。 */
                task_goal_to_grid(task, &goal_grid);
                path_start_x_mm = grid_pose.x_mm;
                path_start_y_mm = grid_pose.y_mm;
                bx72_local_path_begin(grid_pose.x_mm, grid_pose.y_mm, goal_grid.x_mm, goal_grid.y_mm);
                path_refresh_pending = 1u;
            }
            if (!detour_active || path_refresh_pending) {
            if (path_retry_pending) {
                if (info.scan_generation == path_blocked_scan ||
                    (uint32_t)(now_us - path_retry_us) < NAV_PATH_RETRY_US) return;
                path_retry_pending = 0u;
            }
            state = bx72_local_path_state();
            if (state == BX72_PATH_WORKING &&
                approx_distance_xy_mm((int64_t)grid_pose.x_mm - path_start_x_mm,
                                      (int64_t)grid_pose.y_mm - path_start_y_mm) > 100u) {
                /* 后台搜索过期时只重算搜索，不抹去仍受实时扫掠约束的旧引导点。 */
                if (path_refresh_pending) bx72_local_path_reset();
                else reset_local_path();
                state = BX72_PATH_IDLE;
            }
            if (state == BX72_PATH_IDLE) {
                task_goal_to_grid(task, &goal_grid);
                path_start_x_mm = grid_pose.x_mm;
                path_start_y_mm = grid_pose.y_mm;
                bx72_local_path_begin(grid_pose.x_mm, grid_pose.y_mm,
                                      goal_grid.x_mm, goal_grid.y_mm);
            }
            bx72_local_path_step(current_time_us, NAV_PATH_SLICE_US);
            state = bx72_local_path_state();
            if (state == BX72_PATH_BLOCKED) {
                block_local_path(info.scan_generation, now_us);
                return;
            }
            if (state == BX72_PATH_FOUND && bx72_local_path_target(&next_x, &next_y, &final)) {
                if (path_refresh_pending && detour_active &&
                    (next_x != detour_x_mm || next_y != detour_y_mm) &&
                    detour_distance_mm(&latest_pose) > NAV_PATH_WAYPOINT_TOLERANCE_MM &&
                    !path_guide_aligned(&grid_pose, next_x, next_y)) {
                    /* 新方向不连续；保留旧点，移动后或到点时再搜索。 */
                    bx72_local_path_reset();
                    path_refresh_pending = 0u;
                } else {
                if (!guide_goal_valid || !detour_active || next_x != detour_x_mm ||
                    next_y != detour_y_mm || (final != 0) != path_target_final) {
                    task_goal_to_grid(task, &goal_grid);
                    guide_goal_x_mm = goal_grid.x_mm;
                    guide_goal_y_mm = goal_grid.y_mm;
                    guide_goal_valid = 1u;
                }
                if (detour_active && (next_x != detour_x_mm || next_y != detour_y_mm ||
                                      (final != 0) != path_target_final)) {
                    /* 旧输出仍须逐拍扫掠；方向变化过大由控制拍撤销并重新评分。 */
                    planner_active = planner_complete = 0u;
                    if (observe_active) {
                        observe_active = observe_settled = 0u;
                        selected_valid = 0u;
                    }
                }
                detour_x_mm = next_x;
                detour_y_mm = next_y;
                detour_active = 1u;
                direct_guide_active = direct_guide_blocked = 0u;
                path_target_final = final != 0;
                path_refresh_pending = 0u;
                }
            } else if (!detour_active) return;
            }
        }
        if (!planner_complete) planner_background_step(&latest_pose, now_us);
    }
}

static int lidar_input_fresh(uint32_t now_us)
{
    if (have_lidar_sample &&
        !timestamp_recent(now_us, last_lidar_us, NAV_LIDAR_INPUT_MAX_AGE_US))
        have_lidar_sample = 0u;
    return have_lidar_sample;
}

static uint32_t progress_permille(uint64_t current_distance_q16)
{
    uint64_t travelled;
    if (task_start_distance_q16 == 0u || current_distance_q16 >= task_start_distance_q16)
        return 0u;
    travelled = task_start_distance_q16 - current_distance_q16;
    return travelled >= task_start_distance_q16 ? 999u :
        (uint32_t)((travelled * 1000u) / task_start_distance_q16);
}

/* 以原有角加速度限幅估算减速角程，并保留一个20ms控制拍的余量。
 * 该模型不能替代实车惯性或制动距离标定。 */
static uint32_t turn_stopping_distance(uint32_t speed)
{
    uint32_t step = (NAV_YAW_ACCEL_MRAD_S2 + 49u) / 50u;
    uint32_t sum = speed;
    while (speed != 0u) {
        speed = speed > step ? speed - step : 0u;
        sum += speed;
    }
    return (sum + 49u) / 50u;
}

/* 分支逻辑在控制流程之后定义；状态更新保持单线程、同一个控制拍。 */
static uint32_t update_input_sequences(uint64_t now64, uint32_t mode_seq,
                                       uint32_t command_seq);
static void manual_control(uint32_t status, int pose_ok, uint32_t command_seq,
                           uint32_t mode_seq, int32_t vx, int32_t vy, int32_t wz,
                           bx72_nav_output_t *out);
static int handle_turn_braking(const bx72_pose_t *pose, const nav_task_t *task,
                               int32_t yaw_error, int position_reached, int reached,
                               uint32_t yaw_settle_limit, bx72_nav_output_t *out);
static int update_observation(const bx72_pose_t *pose, int position_reached,
                              int32_t heading_error, uint32_t now_us,
                              uint32_t scan_generation);

void bx72_nav_control(const bx72_pose_t *pose, uint32_t now_us, uint32_t status,
                      uint32_t command_seq, int32_t vx, int32_t vy, int32_t wz,
                      uint32_t mode_seq, bx72_nav_output_t *out)
{
    uint64_t now64 = update_clock(now_us), goal_distance_q16 = 0u;
    uint32_t nav_status = BX72_NAV_STATUS_IDLE, nav_task = active_task_id;
    uint32_t progress = 0u, distance_mm, yaw_settle_limit;
    int32_t yaw_error = 0, heading_error = 0;
    int pose_ok, lidar_fresh, position_reached, reached;
    nav_task_t *task;
    bx72_grid_info_t info;
    bx72_pose_t grid_pose;
    if (!out) return;
    planner_inhibited = 1u;
    out->plan_valid = 0u;
    out->plan_stamp_us = now_us;
    out->plan_start_x_mm = out->plan_start_y_mm = 0;
    out->plan_end_x_mm = out->plan_end_y_mm = 0;
    out->vx_mm_s = out->vy_mm_s = out->wz_mrad_s = 0;
    out->status = BX72_NAV_STATUS_IDLE;
    out->task_id = out->progress = 0u;
    auto_enabled = (status & BX72_STATUS_AUTO) != 0u;
    update_session(now64);
    pose_ok = pose && (pose->flags & BX72_POSE_VALID) &&
              timestamp_recent(now_us, pose->stamp_us, NAV_POSE_MAX_AGE_US);
    lidar_fresh = lidar_input_fresh(now_us);
    if (pose_ok) {
        latest_pose = *pose;
        latest_pose_valid = 1u;
        ensure_grid(pose, now_us);
    }
    bx72_grid_advance(now_us);
    bx72_grid_info(&info);
    nav_status = update_input_sequences(now64, mode_seq, command_seq);
    if (!clock_valid) {
        clear_tasks(now64, BX72_NAV_STATUS_STALE, active_task_id);
        out->status = BX72_NAV_STATUS_STALE;
        out->task_id = event_task_id;
        fill_output_metadata(out, now64);
        return;
    }
    if (planner_timeout_pending) {
        planner_timeout_pending = 0u;
        reset_command_slew();
        out->status = task_count ? BX72_NAV_STATUS_PLANNING : nav_status;
        out->task_id = active_task_id;
        fill_output_metadata(out, now64);
        return;
    }
    if (task_count == 0u) {
        if (nav_status == BX72_NAV_STATUS_IDLE && now64 < event_until_us) {
            nav_status = event_status;
            nav_task = event_task_id;
        } else if (nav_status == BX72_NAV_STATUS_IDLE) nav_task = 0u;
        manual_control(status, pose_ok, command_seq, mode_seq, vx, vy, wz, out);
        out->status = nav_status;
        out->task_id = nav_task;
        fill_output_metadata(out, now64);
        return;
    }
    task = &task_queue[task_head];
    nav_task = active_task_id = task->task_id;
    if (now64 >= task->expiry_us ||
        (task->mode == 2u && !(task->flags & NAV_TASK_ALLOW_DISCONNECT) &&
         (status & BX72_STATUS_LINK_DOWN))) {
        clear_tasks(now64, BX72_NAV_STATUS_STALE, nav_task);
        out->status = BX72_NAV_STATUS_STALE;
        out->task_id = nav_task;
        fill_output_metadata(out, now64);
        return;
    }
    if (!pose_ok || !lidar_fresh ||
        (status & (BX72_STATUS_IMU_OK | BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK)) !=
            (BX72_STATUS_IMU_OK | BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK)) {
        out->status = BX72_NAV_STATUS_STALE;
        out->task_id = nav_task;
        planner_active = planner_complete = selected_valid = 0u;
        reset_command_slew();
        fill_output_metadata(out, now64);
        return;
    }
    if (!info.fresh || (status & BX72_STATUS_GUARD_STOP)) {
        out->status = BX72_NAV_STATUS_PLANNING;
        out->task_id = nav_task;
        planner_active = planner_complete = selected_valid = 0u;
        reset_command_slew();
        fill_output_metadata(out, now64);
        return;
    }
    if (!task_start_initialized) activate_current_task(pose, now64);
    task = &task_queue[task_head];
    /* 重置清空旧任务；此处的任务经过发送时间和完整协议校验。 */
    start_confirmation_lost = 0u;
    if (!start_exemption_valid) {
        odom_to_grid(pose, &grid_pose);
        start_exemption.x_mm = grid_pose.x_mm;
        start_exemption.y_mm = grid_pose.y_mm;
        start_exemption.radius_mm = BX72_GRID_START_CONFIRM_RADIUS_MM;
        start_exemption_valid = 1u;
    }
    reached = goal_arrived(pose, task, &goal_distance_q16, &yaw_error);
    position_reached = goal_position_reached(pose, task);
    /* 小容差需要更低的停稳角速，避免用20mrad/s噪声门限虚报精确到达。 */
    yaw_settle_limit = (task->flags & NAV_TASK_POSITION_ONLY) ? NAV_TURN_SETTLED_MRAD_S :
        task->yaw_tolerance_mrad / 4u;
    if (yaw_settle_limit > NAV_TURN_SETTLED_MRAD_S) yaw_settle_limit = NAV_TURN_SETTLED_MRAD_S;
    if (!reached) arrival_settle_count = 0u;
    if (reached && !turn_braking && previous_vx_mm_s == 0 &&
        previous_vy_mm_s == 0 && previous_wz_mrad_s == 0 &&
        abs_i32(pose->vx_mm_s) <= NAV_DRIFT_NOISE_MM_S &&
        abs_i32(pose->vy_mm_s) <= NAV_DRIFT_NOISE_MM_S &&
        abs_i32(pose->wz_mrad_s) <= yaw_settle_limit) {
        if (++arrival_settle_count < 3u) {
            planner_active = planner_complete = selected_valid = 0u;
            planner_inhibited = 1u;
            out->status = BX72_NAV_STATUS_PLANNING;
            out->task_id = nav_task;
            out->progress = progress_permille(goal_distance_q16);
            fill_output_metadata(out, now64);
            return;
        }
        pop_current_task(now64, pose);
        if (task_count == 0u) {
            out->status = BX72_NAV_STATUS_ARRIVED;
            out->task_id = event_task_id;
            out->progress = 1000u;
            reset_command_slew();
            fill_output_metadata(out, now64);
            return;
        }
        task = &task_queue[task_head];
        nav_task = task->task_id;
        yaw_settle_limit = (task->flags & NAV_TASK_POSITION_ONLY) ? NAV_TURN_SETTLED_MRAD_S :
            task->yaw_tolerance_mrad / 4u;
        if (yaw_settle_limit > NAV_TURN_SETTLED_MRAD_S) yaw_settle_limit = NAV_TURN_SETTLED_MRAD_S;
        (void)goal_arrived(pose, task, &goal_distance_q16, &yaw_error);
        position_reached = goal_position_reached(pose, task);
        reached = 0;
    }
    distance_mm = clamp_u64_u32(goal_distance_q16 / NAV_Q16_ONE);
    if (handle_turn_braking(pose, task, yaw_error, position_reached, reached,
                            yaw_settle_limit, out)) {
        out->status = BX72_NAV_STATUS_PLANNING;
        out->task_id = nav_task;
        out->progress = progress_permille(goal_distance_q16);
        fill_output_metadata(out, now64);
        return;
    }
    planner_inhibited = 0u;
    progress = progress_permille(goal_distance_q16);
    if (position_reached || best_goal_distance_mm == 0u ||
        distance_mm + 100u < best_goal_distance_mm) {
        best_goal_distance_mm = distance_mm;
        last_progress_us = now64;
    } else if (now64 - last_progress_us >= NAV_NO_PROGRESS_US && !progress_blocked) {
        progress_blocked = 1u;
        blocked_scan_generation = info.scan_generation;
        last_progress_us = now64;
    }
    if (progress_blocked) {
        if (info.scan_generation != blocked_scan_generation &&
            now64 - last_progress_us >= NAV_BLOCKED_RETRY_US) {
            progress_blocked = 0u;
            planner_active = planner_complete = selected_valid = 0u;
            last_progress_us = now64;
        } else {
            out->status = BX72_NAV_STATUS_BLOCKED;
            out->task_id = nav_task;
            out->progress = progress;
            reset_command_slew();
            fill_output_metadata(out, now64);
            return;
        }
    }
    /* 局部自动命令仅前进；倒退反馈和先前人工侧移须先停稳。 */
    if (pose->vx_mm_s < -(int32_t)NAV_DRIFT_NOISE_MM_S ||
        previous_vx_mm_s < 0 || previous_vy_mm_s != 0) {
        out->status = BX72_NAV_STATUS_PLANNING;
        out->task_id = nav_task;
        planner_inhibited = 1u;
        reset_command_slew();
        planner_active = planner_complete = selected_valid = 0u;
        fill_output_metadata(out, now64);
        return;
    }
    if (position_reached && detour_active) {
        reset_local_path();
        path_retry_pending = 0u;
    } else if (detour_active && !path_target_final &&
               detour_distance_mm(pose) <= NAV_PATH_WAYPOINT_TOLERANCE_MM) {
        if (path_refresh_pending) {
            /* 后台尚未算完时停在旧点附近；不得追过旧引导点。 */
            out->status = BX72_NAV_STATUS_PLANNING;
            out->task_id = nav_task;
            out->progress = progress;
            reset_command_slew();
            fill_output_metadata(out, now64);
            return;
        }
        /* 后台未启动或仍返回旧点时，再从当前位置搜索。 */
        reset_local_path();
        path_retry_pending = 0u;
    }
    if (!position_reached && !detour_active) {
        out->status = path_retry_pending ? BX72_NAV_STATUS_BLOCKED : BX72_NAV_STATUS_PLANNING;
        out->task_id = nav_task;
        out->progress = progress;
        reset_command_slew();
        fill_output_metadata(out, now64);
        return;
    }
    if (!position_reached) heading_error = task_heading_error(pose, task);
    if (update_observation(pose, position_reached, heading_error, now_us,
                           info.scan_generation)) {
        out->status = BX72_NAV_STATUS_PLANNING;
        out->task_id = nav_task;
        out->progress = progress;
        fill_output_metadata(out, now64);
        return;
    }
    if (planner_complete && planning_pose_changed(pose)) {
        planner_complete = planner_active = 0u;
    }
    if (selected_valid) {
        if ((!observe_active && planner_at_goal != position_reached) ||
            (selected_vx > 0 && abs_i32(wrap_yaw_mrad(heading_error - planner_heading_error)) >
             NAV_PLAN_HEADING_CHANGE_MRAD)) {
            planner_complete = planner_active = selected_valid = 0u;
        } else if (selected_vx == 0 && selected_wz != 0 &&
                   (observe_active || position_reached)) {
            int32_t turn_error = observe_active ?
                wrap_yaw_mrad(observe_target_yaw_mrad - pose->yaw_mrad) : yaw_error;
            /* 正负π是同一方向；微小坐标修正跨回绕边界不等于转过目标。 */
            if (abs_i32(turn_error) < NAV_PI_MRAD - NAV_REBASE_SMALL_YAW_MRAD &&
                ((selected_wz > 0 && turn_error <= 0) || (selected_wz < 0 && turn_error >= 0)))
                planner_complete = planner_active = selected_valid = 0u;
        }
    }
    if (!planner_complete && !selected_valid) {
        out->status = BX72_NAV_STATUS_PLANNING;
        out->task_id = nav_task;
        out->progress = progress;
        reset_command_slew();
        fill_output_metadata(out, now64);
        return;
    }
    if (!selected_valid) {
        out->status = BX72_NAV_STATUS_BLOCKED;
        out->task_id = nav_task;
        out->progress = progress;
        block_local_path(info.scan_generation, now_us);
        reset_command_slew();
        fill_output_metadata(out, now64);
        return;
    }
    if (!observe_active && !position_reached && selected_vx == 0) {
        /* 候选无法沿搜索路径前进时停下，等待新扫描再搜索。 */
        block_local_path(info.scan_generation, now_us);
        reset_command_slew();
        out->status = BX72_NAV_STATUS_BLOCKED;
        out->task_id = nav_task;
        out->progress = progress;
        fill_output_metadata(out, now64);
        return;
    }
    {
        int32_t command_vx, command_wz, end_x, end_y, target_vx = selected_vx;
        int valid;
        uint32_t started_us;
        uint8_t unknown;
        if (target_vx > 0)
            target_vx = (int32_t)guide_speed(detour_active ? detour_distance_mm(pose) :
                                             distance_mm, (uint32_t)target_vx);
        proposed_command(target_vx, selected_wz, task->max_accel_mm_s2,
                         &command_vx, &command_wz);
        unknown_in_swept_corridor = 0u;
        started_us = current_time_us();
        valid = evaluate_motion(pose, command_vx, command_wz,
                                command_vx > 0 || pose->vx_mm_s > 20,
                                &end_x, &end_y, &unknown, started_us);
        if (valid >= 0 && evaluation_timed_out(started_us)) valid = -1;
        if (valid <= 0) {
            unknown_in_swept_corridor = unknown;
            planner_active = planner_complete = selected_valid = 0u;
            if (valid == 0 && !position_reached)
                block_local_path(info.scan_generation, now_us);
            out->status = valid == 0 ? BX72_NAV_STATUS_BLOCKED : BX72_NAV_STATUS_PLANNING;
            out->task_id = nav_task;
            out->progress = progress;
            reset_command_slew();
            fill_output_metadata(out, now64);
            return;
        }
        out->vx_mm_s = command_vx;
        out->vy_mm_s = 0;
        out->wz_mrad_s = command_wz;
        previous_vx_mm_s = command_vx;
        previous_vy_mm_s = 0;
        previous_wz_mrad_s = command_wz;
        out->status = observe_active ? BX72_NAV_STATUS_PLANNING : BX72_NAV_STATUS_FOLLOWING;
        out->task_id = nav_task;
        out->progress = progress;
        if (command_vx > 0) {
            int32_t sine, cosine;
            int64_t dx, dy;
            odom_to_grid(pose, &grid_pose);
            dx = (int64_t)end_x - grid_pose.x_mm;
            dy = (int64_t)end_y - grid_pose.y_mm;
            sine = bx72_sin_q15(map_yaw_offset);
            cosine = bx72_cos_q15(map_yaw_offset);
            out->plan_valid = 1u;
            out->plan_start_x_mm = pose->x_mm;
            out->plan_start_y_mm = pose->y_mm;
            out->plan_end_x_mm = clamp_i64_to_i32((int64_t)pose->x_mm +
                                    ((dx * cosine + dy * sine) >> 15));
            out->plan_end_y_mm = clamp_i64_to_i32((int64_t)pose->y_mm +
                                    ((-dx * sine + dy * cosine) >> 15));
        }
    }
    fill_output_metadata(out, now64);
}

static uint32_t update_input_sequences(uint64_t now64, uint32_t mode_seq,
                                       uint32_t command_seq)
{
    uint32_t nav_status = BX72_NAV_STATUS_IDLE;
    uint32_t manual_motion_sequence = mmio_read_offset(BX72_IO_MANUAL_MOTION_SEQ);
    if (!manual_motion_sequence_initialized) {
        manual_motion_sequence_initialized = 1u;
        last_manual_motion_sequence = manual_motion_sequence;
    } else if (manual_motion_sequence != last_manual_motion_sequence) {
        last_manual_motion_sequence = manual_motion_sequence;
        /* 非零人工接管后车可能移动；下个任务再确认新起点。
         * 周期零速屏障及纯模式切换不改变固定的启动豁免圆。 */
        start_exemption_valid = 0u;
    }
    if (!mode_initialized) {
        mode_initialized = 1u;
        last_mode_sequence = mode_seq;
    } else if (mode_seq != last_mode_sequence) {
        last_mode_sequence = mode_seq;
        fragment_collecting = 0u;
        if (task_count != 0u) {
            clear_tasks(now64, BX72_NAV_STATUS_CANCELLED, active_task_id);
            nav_status = BX72_NAV_STATUS_CANCELLED;
        }
        /* 网桥在已到达后自动发送模式命令；单凭mode_seq变更不能
         * 认定发生过人工重新确认，更不能创建新的启动许可圆。 */
    }
    if (!command_sequence_initialized) {
        command_sequence_initialized = 1u;
        last_command_sequence = command_seq;
    } else if (command_seq != last_command_sequence) {
        last_command_sequence = command_seq;
        fragment_collecting = 0u;
        /* type3 的零速也是明确接管/停车；不能在下一拍恢复旧任务。 */
        if (task_count != 0u) {
            clear_tasks(now64, BX72_NAV_STATUS_CANCELLED, active_task_id);
            nav_status = BX72_NAV_STATUS_CANCELLED;
        }
        manual_inhibited = 0u;
    }
    return nav_status;
}

void bx72_nav_sync_inputs(uint32_t now_us, uint32_t command_seq, uint32_t mode_seq)
{
    uint64_t now64 = update_clock(now_us);
    update_session(now64);
    (void)update_input_sequences(now64, mode_seq, command_seq);
}

static void manual_control(uint32_t status, int pose_ok, uint32_t command_seq,
                           uint32_t mode_seq, int32_t vx, int32_t vy, int32_t wz,
                           bx72_nav_output_t *out)
{
    bx72_velocity_t velocity;
    /* 手动和 Jetson 速度控制复用现有实现；只依赖原始雷达状态，
     * 不把局部建图/CSM历史位姿是否匹配再变成一道人为速度门槛。 */
    (void)bx72_manual_control(&manual_state, status,
                               pose_ok && !auto_enabled && !manual_inhibited,
                               command_seq, mode_seq, vx, vy, wz, &velocity);
    out->vx_mm_s = velocity.vx_mm_s;
    out->vy_mm_s = velocity.vy_mm_s;
    out->wz_mrad_s = velocity.wz_mrad_s;
    previous_vx_mm_s = velocity.vx_mm_s;
    previous_vy_mm_s = velocity.vy_mm_s;
    previous_wz_mrad_s = velocity.wz_mrad_s;
}

static int handle_turn_braking(const bx72_pose_t *pose, const nav_task_t *task,
                               int32_t yaw_error, int position_reached, int reached,
                               uint32_t yaw_settle_limit, bx72_nav_output_t *out)
{
    /* 已进入容差但尚在运动，先撤销输出；停稳后再判断是否真正到达。 */
    if (reached && !turn_braking) {
        arrival_settle_count = 0u;
        turn_braking = 1u;
        turn_settle_count = 0u;
        turn_brake_sign = previous_wz_mrad_s > 0 || pose->wz_mrad_s > 0 ? 1 : -1;
        reset_command_slew();
    }
    if (!turn_braking && selected_valid && selected_vx == 0 && selected_wz != 0 &&
        (position_reached || observe_active)) {
        int32_t error = observe_active && !position_reached ?
            wrap_yaw_mrad(observe_target_yaw_mrad - pose->yaw_mrad) : yaw_error;
        uint32_t speed = abs_i32(previous_wz_mrad_s);
        int crossed = abs_i32(error) < NAV_PI_MRAD - NAV_REBASE_SMALL_YAW_MRAD &&
            ((selected_wz > 0 && error <= 0) || (selected_wz < 0 && error >= 0));
        if (abs_i32(pose->wz_mrad_s) > speed) speed = abs_i32(pose->wz_mrad_s);
        /* 估算循环有界；原始超限反馈仍由真实运动复核拒绝。 */
        if (speed > NAV_MOTION_MAX_YAW_MRAD_S) speed = NAV_MOTION_MAX_YAW_MRAD_S;
        if (crossed || (speed != 0u && abs_i32(error) <= turn_stopping_distance(speed))) {
            turn_braking = 1u;
            turn_settle_count = 0u;
            turn_brake_sign = selected_wz > 0 ? 1 : -1;
            if (crossed) reset_command_slew();
        }
    }
    if (turn_braking) {
        int32_t command_vx, command_wz, end_x, end_y;
        int32_t error = observe_active && !position_reached ?
            wrap_yaw_mrad(observe_target_yaw_mrad - pose->yaw_mrad) : yaw_error;
        uint8_t unknown = 0u;
        int valid = 1;
        if (abs_i32(error) < NAV_PI_MRAD - NAV_REBASE_SMALL_YAW_MRAD &&
            ((turn_brake_sign > 0 && error <= 0) || (turn_brake_sign < 0 && error >= 0)))
            reset_command_slew();
        previous_vx_mm_s = previous_vy_mm_s = 0;
        proposed_command(0, 0, task->max_accel_mm_s2, &command_vx, &command_wz);
        if (command_wz != 0) {
            uint32_t started_us = current_time_us();
            valid = evaluate_motion(pose, 0, command_wz, pose->vx_mm_s > 20,
                                    &end_x, &end_y, &unknown, started_us);
            if (valid >= 0 && evaluation_timed_out(started_us)) valid = -1;
        }
        if (valid <= 0) {
            reset_command_slew();
            command_wz = 0;
            unknown_in_swept_corridor = unknown;
        }
        previous_wz_mrad_s = command_wz;
        out->wz_mrad_s = command_wz;
        planner_active = planner_complete = selected_valid = 0u;
        planner_inhibited = 1u;
        if (command_wz == 0 && abs_i32(pose->vx_mm_s) <= NAV_DRIFT_NOISE_MM_S &&
            abs_i32(pose->vy_mm_s) <= NAV_DRIFT_NOISE_MM_S &&
            abs_i32(pose->wz_mrad_s) <= (position_reached ? yaw_settle_limit : NAV_TURN_SETTLED_MRAD_S)) {
            if (++turn_settle_count >= 3u) {
                turn_braking = turn_settle_count = 0u;
                arrival_settle_count = reached ? 2u : 0u;
            }
        } else turn_settle_count = 0u;
        return 1;
    }
    return 0;
}

static int update_observation(const bx72_pose_t *pose, int position_reached,
                              int32_t heading_error, uint32_t now_us,
                              uint32_t scan_generation)
{
    if (position_reached && observe_active) {
        observe_active = observe_settled = 0u;
        planner_active = planner_complete = selected_valid = 0u;
    }
    if (!position_reached && !observe_active && abs_i32(heading_error) > 600u) {
        observe_active = 1u;
        observe_settled = 0u;
        observe_target_yaw_mrad = wrap_yaw_mrad(pose->yaw_mrad + heading_error);
        planner_active = planner_complete = selected_valid = 0u;
    }
    if (observe_active && !position_reached) {
        int32_t observe_error = wrap_yaw_mrad(observe_target_yaw_mrad - pose->yaw_mrad);
        if (abs_i32(observe_error) <= 100u) {
            reset_command_slew();
            planner_active = planner_complete = selected_valid = 0u;
            if (abs_i32(pose->vx_mm_s) <= 20u && abs_i32(pose->vy_mm_s) <= 20u &&
                abs_i32(pose->wz_mrad_s) <= 50u) {
                if (!observe_settled) {
                    observe_settled = 1u;
                    observe_settle_us = now_us;
                    observe_scan_generation = scan_generation;
                } else if (scan_generation != observe_scan_generation &&
                           (int32_t)(map_complete_started_us - observe_settle_us) > 0) {
                    observe_active = observe_settled = 0u;
                }
            } else observe_settled = 0u;
            return 1;
        }
        observe_settled = 0u;
    }
    return 0;
}

uint32_t bx72_nav_selftest(void)
{
    int32_t wheels[4], target[4] = {100, 100, 100, 100};
    int64_t converted = 0;
    uint32_t failures = 0u;
    bx72_nav_init();
    q16m_to_mm_q16(65536, &converted);
    if (converted != 1000ll * NAV_Q16_ONE) failures |= 1u;
    wheel_vector(0, 0, 0, wheels);
    wheel_approach(wheels, target);
    if (wheels[0] != 10 || wheels[1] != 10 || wheels[2] != 10 || wheels[3] != 10)
        failures |= 2u;
    bx72_nav_init();
    return failures;
}
