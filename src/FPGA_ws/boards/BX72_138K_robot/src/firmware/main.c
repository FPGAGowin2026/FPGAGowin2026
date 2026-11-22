#include "board_io.h"
#include "odometry.h"
#include "robot_config.h"
#include "robot_navigation.h"

#define CONTROL_PERIOD_US 20000u
#define INPUT_MAX_AGE_US 250000u
#define TASK_FRAGMENT_TIMEOUT_US 200000u
#define TASK_MAX_AGE_US 500000u
#define TASK_MAX_LIFETIME_US 120000000u
#define GYRO_RAD_PER_LSB 0.0003054326191f

typedef struct {
    uint32_t now, status, imu_seq, wheel_seq, imu_stamp, wheel_stamp;
    uint32_t command_seq, mode_seq;
    int32_t gyro_z, wheels[4], command_vx, command_vy, command_wz;
} input_snapshot_t;

typedef struct {
    uint8_t payload[72];
    uint8_t collecting, next_index;
    uint32_t next_sequence, started_us;
    uint64_t token;
} task_fragments_t;

static odometry_t odom;
static robot_navigation_t navigation;
static task_fragments_t fragments;
static uint32_t last_imu_seq, last_wheel_seq, last_command_seq, last_task_id;
static uint32_t last_lidar_drops, last_lidar_errors, heartbeat_counter;
static uint32_t lidar_discard_before;
static uint32_t last_control_us, max_control_gap_us, next_control_us;
static uint8_t odom_started, imu_seen, wheel_seen, command_seen, task_mode, lidar_gap_active;
static local_velocity_t previous_command;
static uint8_t previous_auto_guard_stop;
static uint8_t grid_cells[1024], grid_part;
static uint32_t grid_header[20], grid_id, last_grid_us;

static const odometry_config_t odom_config = {
    .process_position = 0.001f, .process_yaw = 0.001f,
    .process_vx = 0.03f, .process_vy = 0.06f, .process_yaw_rate = 0.02f,
    .process_gyro_bias = 0.0001f,
    .wheel_vx_variance = 0.0025f, .wheel_vy_variance = 0.01f,
    .wheel_yaw_rate_variance = 0.01f, .gyro_variance = 0.0004f,
    .initial_position_variance = 0.001f, .initial_yaw_variance = 0.01f,
    .initial_velocity_variance = 0.1f, .initial_yaw_rate_variance = 0.1f,
    .initial_bias_variance = 0.01f, .initial_gyro_bias_rad_s = 0.0f,
    .geometry_mm = ROBOT_GEOMETRY_MM, .max_prediction_gap_us = INPUT_MAX_AGE_US,
    .wheel_max_age_us = INPUT_MAX_AGE_US, .imu_max_age_us = INPUT_MAX_AGE_US
};

static int32_t signed_low(uint32_t value)
{
    return (int32_t)(int16_t)(uint16_t)value;
}

static int32_t signed_high(uint32_t value)
{
    return signed_low(value >> 16);
}

static int32_t scaled_float(float value)
{
    value *= 1000.0f;
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static uint32_t float_bits(float value)
{
    union { float f; uint32_t u; } bits;
    bits.f = value;
    return bits.u;
}

static void read_inputs(input_snapshot_t *s)
{
    uint32_t packed, i;
    io_write(IO_SNAPSHOT, 1);
    s->now = io_read(IO_NOW);
    s->status = io_read(IO_STATUS);
    s->imu_seq = io_read(IO_IMU_SEQ);
    s->gyro_z = signed_high(io_read(IO_GZ_AX));
    s->wheel_seq = io_read(IO_WHEEL_SEQ);
    for (i = 0; i < 4; ++i) s->wheels[i] = (int32_t)io_read(IO_WHEEL_0 + i * 4u);
    s->wheel_stamp = io_read(IO_WHEEL_STAMP);
    s->imu_stamp = io_read(IO_IMU_STAMP);
    s->command_seq = io_read(IO_COMMAND_SEQ);
    packed = io_read(IO_COMMAND_XY);
    s->command_vx = signed_high(packed);
    s->command_vy = signed_low(packed);
    s->command_wz = signed_low(io_read(IO_COMMAND_WZ));
    s->mode_seq = io_read(IO_MODE_SEQ);
}

static void invalidate_odometry(uint32_t now_us)
{
    uint32_t domain = io_read(IO_ODOM_DOMAIN) + 1u;
    if (!domain) domain = 1;
    io_write(IO_SET_ODOM_DOMAIN, domain);
    robot_navigation_reset_map(&navigation, now_us);
    odom_started = imu_seen = wheel_seen = 0;
    fragments.collecting = 0;
}

static void update_wheels(const input_snapshot_t *s)
{
    if (!wheel_seen || s->wheel_seq != last_wheel_seq) {
        last_wheel_seq = s->wheel_seq;
        wheel_seen = 1;
        if ((int32_t)(s->wheel_stamp - odom.last_update_us) >= 0)
            (void)odometry_update_wheels(&odom, s->wheels, s->wheel_stamp);
    }
}

static void update_imu(const input_snapshot_t *s)
{
    if (!imu_seen || s->imu_seq != last_imu_seq) {
        last_imu_seq = s->imu_seq;
        imu_seen = 1;
        if ((int32_t)(s->imu_stamp - odom.last_update_us) >= 0)
            (void)odometry_update_gyro(&odom,
                (float)(ROBOT_IMU_YAW_SIGN * s->gyro_z) * GYRO_RAD_PER_LSB, s->imu_stamp);
    }
}

static void update_odometry(const input_snapshot_t *s)
{
    uint32_t first;
    if ((s->status & (STATUS_IMU_OK | STATUS_WHEEL_OK)) != (STATUS_IMU_OK | STATUS_WHEEL_OK)) return;
    if ((uint32_t)(s->now - s->imu_stamp) > INPUT_MAX_AGE_US ||
        (uint32_t)(s->now - s->wheel_stamp) > INPUT_MAX_AGE_US) return;
    if (!odom_started) {
        first = (int32_t)(s->imu_stamp - s->wheel_stamp) <= 0 ? s->imu_stamp : s->wheel_stamp;
        if (!odometry_init(&odom, &odom_config, first)) return;
        odom_started = 1;
        imu_seen = wheel_seen = 0;
    }
    if ((int32_t)(s->imu_stamp - s->wheel_stamp) <= 0) {
        update_imu(s);
        update_wheels(s);
    } else {
        update_wheels(s);
        update_imu(s);
    }
    if (!odom.healthy) invalidate_odometry(s->now);
}

static void process_lidar(uint32_t now_us, uint32_t deadline_us)
{
    uint32_t i, drops = io_read(IO_LIDAR_DROPS), errors = io_read(IO_LIDAR_ERRORS);
    if (drops != last_lidar_drops || errors != last_lidar_errors) {
        robot_navigation_lidar_gap(&navigation,
            odom_started ? scaled_float(odom.state[0]) : 0,
            odom_started ? scaled_float(odom.state[1]) : 0, now_us);
        lidar_discard_before = now_us;
        lidar_gap_active = 1;
        last_lidar_drops = drops;
        last_lidar_errors = errors;
    }
    for (i = 0; i < 16; ++i) {
        now_us = io_read(IO_SNAPSHOT);
        if ((int32_t)(deadline_us - now_us) <= 3000) break;
        uint32_t meta = io_read(IO_LIDAR_META);
        uint32_t point, stamp, age;
        if (!(meta & 1u)) break;
        point = io_read(IO_LIDAR_HEAD);
        stamp = io_read(IO_LIDAR_STAMP);
        age = now_us - stamp;
        if (lidar_gap_active && (int32_t)(stamp - lidar_discard_before) <= 0) {
            io_write(IO_LIDAR_POP, 1);
            continue;
        }
        lidar_gap_active = 0;
        if (odom_started && odom.healthy && (int32_t)(stamp - odom.last_update_us) > 0 &&
            age < 100000u) break;
        if (odom_started && odometry_valid(&odom, now_us)) {
            if (!navigation.goal_active)
                local_map_recenter(&navigation.map, scaled_float(odom.state[0]),
                                   scaled_float(odom.state[1]), now_us);
            robot_navigation_observe(&navigation, &odom, (uint16_t)(point >> 16),
                (uint16_t)point, (meta & 2u) != 0, stamp, now_us);
        }
        io_write(IO_LIDAR_POP, 1);
    }
}

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static uint64_t be64(const uint8_t *p)
{
    return ((uint64_t)be32(p) << 32) | be32(p + 4);
}

static int32_t q16_to_milli(uint32_t bits)
{
    return (int32_t)((int64_t)(int32_t)bits * 1000 / 65536);
}

static void accept_task(uint32_t now_us)
{
    const uint8_t *p = fragments.payload;
    uint32_t task_id = be32(p + 4), mode = p[2], flags = p[3];
    uint32_t sent = be32(p + 60), expiry = be32(p + 44), i;
    uint64_t token = ((uint64_t)io_read(IO_SESSION_HI) << 32) | io_read(IO_SESSION_LO);
    if (p[0] != 1 || p[1] < 1 || p[1] > 3 || mode < 2 || mode > 3 ||
        (flags & ~15u) || !task_id || !token || token != fragments.token ||
        be32(p + 8) != io_read(IO_ODOM_DOMAIN) || be32(p + 12) != io_read(IO_BOOT_ID) ||
        be32(p + 40) || be32(p + 56) || (uint32_t)(now_us - sent) > TASK_MAX_AGE_US ||
        (int32_t)(expiry - now_us) <= 0 || (uint32_t)(expiry - sent) > TASK_MAX_LIFETIME_US ||
        (last_task_id && (int32_t)(task_id - last_task_id) <= 0) ||
        (mode == 3 && !(flags & 1u))) {
        navigation.status = ROBOT_NAV_REJECTED;
        return;
    }
    if (!(flags & 8u)) {
        for (i = 48; i < 56; ++i) if (p[i]) { navigation.status = ROBOT_NAV_REJECTED; return; }
        for (i = 64; i < 68; ++i) if (p[i]) { navigation.status = ROBOT_NAV_REJECTED; return; }
    } else {
        int64_t dx = (int64_t)q16_to_milli(be32(p + 48)) - scaled_float(odom.state[0]);
        int64_t dy = (int64_t)q16_to_milli(be32(p + 52)) - scaled_float(odom.state[1]);
        int32_t vx = scaled_float(odom.state[3]), vy = scaled_float(odom.state[4]);
        int32_t wz = scaled_float(odom.state[5]);
        uint32_t until = be32(p + 64);
        if (p[1] != 1 || !odom_started || !odometry_valid(&odom, now_us) ||
            dx * dx + dy * dy > 2500 || vx < -20 || vx > 20 || vy < -20 || vy > 20 ||
            wz < -20 || wz > 20 || (int32_t)(until - now_us) <= 0 ||
            (uint32_t)(until - now_us) > 10000000u) {
            navigation.status = ROBOT_NAV_REJECTED;
            return;
        }
    }
    for (i = 68; i < 72; ++i) if (p[i]) { navigation.status = ROBOT_NAV_REJECTED; return; }
    if (p[1] != 1) {
        robot_navigation_cancel(&navigation);
        last_task_id = task_id;
        task_mode = 1;
        return;
    }
    if (!robot_navigation_set_goal(&navigation, task_id,
            q16_to_milli(be32(p + 16)), q16_to_milli(be32(p + 20)),
            q16_to_milli(be32(p + 24)), q16_to_milli(be32(p + 28)),
            be16(p + 38), (flags & 2u) != 0, (flags & 4u) != 0,
            be16(p + 32), be16(p + 34), be16(p + 36), expiry, now_us)) {
        navigation.status = ROBOT_NAV_REJECTED;
        return;
    }
    if (flags & 8u)
        local_map_confirm_start(&navigation.map, q16_to_milli(be32(p + 48)),
                                q16_to_milli(be32(p + 52)), be32(p + 64));
    task_mode = (uint8_t)mode;
    last_task_id = task_id;
}

static void process_task(uint32_t now_us)
{
    uint32_t meta = io_read(IO_TASK_META), epoch, words[8], i, sequence, index;
    uint8_t frame[32];
    uint64_t token;
    if (!(meta & 3u)) return;
    epoch = io_read(IO_TASK_EPOCH);
    if (meta & 2u) {
        robot_navigation_cancel(&navigation);
        fragments.collecting = 0;
        io_write(IO_OUT_VX, 0); io_write(IO_OUT_VY, 0); io_write(IO_OUT_WZ, 0);
        io_write(IO_OUT_COMMIT, IO_COMMIT_MAGIC);
        io_write(IO_TASK_OVERFLOW_ACK, 1);
        return;
    }
    for (i = 0; i < 8; ++i) words[i] = io_read(IO_TASK_WORD0 + i * 4u);
    if (epoch != io_read(IO_TASK_EPOCH)) return;
    for (i = 0; i < 8; ++i) {
        frame[i * 4] = (uint8_t)(words[i] >> 24);
        frame[i * 4 + 1] = (uint8_t)(words[i] >> 16);
        frame[i * 4 + 2] = (uint8_t)(words[i] >> 8);
        frame[i * 4 + 3] = (uint8_t)words[i];
    }
    io_write(IO_TASK_POP, epoch);
    sequence = be32(frame + 8);
    token = be64(frame + 12);
    index = frame[20];
    if (index >= 8 || !token) { fragments.collecting = 0; return; }
    if (index == 0) {
        fragments.collecting = 1;
        fragments.next_index = 0;
        fragments.next_sequence = sequence;
        fragments.started_us = now_us;
        fragments.token = token;
    }
    if (!fragments.collecting || index != fragments.next_index ||
        sequence != fragments.next_sequence || token != fragments.token ||
        (uint32_t)(now_us - fragments.started_us) > TASK_FRAGMENT_TIMEOUT_US) {
        fragments.collecting = 0;
        navigation.status = ROBOT_NAV_REJECTED;
        return;
    }
    for (i = 0; i < 9; ++i) fragments.payload[index * 9u + i] = frame[21u + i];
    ++fragments.next_index;
    ++fragments.next_sequence;
    if (index == 7) { fragments.collecting = 0; accept_task(now_us); }
}

static void capture_grid(uint32_t now_us, uint32_t deadline_us)
{
    uint32_t byte_index, j, boot, domain;
    local_map_t *map = &navigation.map;
    if (grid_part < 8 || !odom_started || !odom.healthy ||
        (uint32_t)(now_us - last_grid_us) < 250000u ||
        (int32_t)(deadline_us - now_us) < 8000) return;
    boot = io_read(IO_BOOT_ID);
    domain = io_read(IO_ODOM_DOMAIN);
    if (!boot || !domain) return;
    local_map_advance(map, now_us);
    for (byte_index = 0; byte_index < 1024; ++byte_index) {
        uint8_t packed = 0;
        if ((byte_index & 63u) == 0 && (int32_t)(deadline_us - io_read(IO_SNAPSHOT)) <= 1000) return;
        for (j = 0; j < 4; ++j) {
            uint32_t index = byte_index * 4u + j;
            int32_t x = map->origin_x_cell + (int32_t)(index % LOCAL_MAP_SIDE);
            int32_t y = map->origin_y_cell + (int32_t)(index / LOCAL_MAP_SIDE);
            uint32_t slot = ((uint32_t)y & 63u) * LOCAL_MAP_SIDE + ((uint32_t)x & 63u);
            packed |= (uint8_t)((map->cells[slot] >> 6) << (j * 2u));
        }
        grid_cells[byte_index] = packed;
    }
    if (boot != io_read(IO_BOOT_ID) || domain != io_read(IO_ODOM_DOMAIN)) return;
    ++grid_id;
    grid_header[0] = boot; grid_header[1] = domain; grid_header[2] = grid_id;
    grid_header[3] = now_us; grid_header[4] = map->generation; grid_header[5] = 0;
    grid_header[6] = (uint32_t)map->origin_x_cell; grid_header[7] = (uint32_t)map->origin_y_cell;
    grid_header[8] = 0; grid_header[9] = navigation.have_angle ? navigation.last_ray_us : 0;
    grid_header[10] = grid_header[11] = grid_header[12] = grid_header[13] = grid_header[14] = 0;
    grid_header[15] = domain; grid_header[16] = 0; grid_header[17] = 1;
    grid_header[18] = io_read(IO_SNAPSHOT);
    grid_part = 0;
    last_grid_us = now_us;
}

static void emit_grid(void)
{
    uint32_t i, base;
    if (grid_part >= 8 || !(io_read(IO_GRID_READY) & 1u)) return;
    if (grid_header[0] != io_read(IO_BOOT_ID) || grid_header[1] != io_read(IO_ODOM_DOMAIN)) {
        grid_part = 8;
        return;
    }
    for (i = 0; i < 19; ++i) io_write(IO_GRID_WORD0 + 4u * i, grid_header[i]);
    io_write(IO_GRID_WORD0 + 76u, (1u << 24) | (50u << 16) | (8u << 8) | grid_part);
    base = (uint32_t)grid_part * 128u;
    for (i = 0; i < 128; i += 4) {
        uint32_t word = ((uint32_t)grid_cells[base + i] << 24) |
            ((uint32_t)grid_cells[base + i + 1u] << 16) |
            ((uint32_t)grid_cells[base + i + 2u] << 8) | grid_cells[base + i + 3u];
        io_write(IO_GRID_WORD0 + (20u + i / 4u) * 4u, word);
    }
    io_write(IO_GRID_COMMIT, IO_COMMIT_MAGIC);
    ++grid_part;
}

static int32_t limit_delta(int32_t value, int32_t previous, int32_t delta)
{
    int32_t result = value;
    if (value > previous + delta) result = previous + delta;
    else if (value < previous - delta) result = previous - delta;
    /* A ramp must not retain an unchecked direction or exceed the checked request. */
    if (value >= 0) {
        if (result < 0) return 0;
        if (result > value) return value;
    } else {
        if (result > 0) return 0;
        if (result < value) return value;
    }
    return result;
}

static uint32_t stopping_distance(uint32_t speed, uint32_t acceleration, uint32_t delay_us)
{
    uint64_t latency = ((uint64_t)speed * delay_us + 999999u) / 1000000u;
    uint64_t braking = ((uint64_t)speed * speed + 2u * acceleration - 1u) /
                       (2u * acceleration);
    uint64_t total = latency + braking;
    return total > 8000u ? 8000u : (uint32_t)total;
}

static void publish_front_threshold(const local_velocity_t *command, int odom_valid_now)
{
    int32_t measured_vx, measured_wz, requested_wz, forward_signed;
    uint32_t forward, angular, tangent, tangent_accel, delay_us, stop;
    if (!odom_valid_now || !navigation.calibration_ready) {
        io_write(IO_FRONT_STOP_MM, 0);
        io_write(IO_FRONT_SLOW_MM, 0);
        return;
    }
    measured_vx = scaled_float(odom.state[3]);
    measured_wz = scaled_float(odom.state[5]);
    requested_wz = command->wz_mrad_s;
    forward_signed = measured_vx > command->vx_mm_s ? measured_vx : command->vx_mm_s;
    forward = forward_signed > 0 ? (uint32_t)forward_signed : 0u;
    angular = (uint32_t)(measured_wz < 0 ? -measured_wz : measured_wz);
    if ((uint32_t)(requested_wz < 0 ? -requested_wz : requested_wz) > angular)
        angular = (uint32_t)(requested_wz < 0 ? -requested_wz : requested_wz);
    tangent = (angular * 181u + 999u) / 1000u;
    tangent_accel = ((uint32_t)navigation.guard.brake_yaw_mrad_s2 * 181u) / 1000u;
    if (!tangent_accel) {
        io_write(IO_FRONT_STOP_MM, 0); io_write(IO_FRONT_SLOW_MM, 0);
        return;
    }
    delay_us = navigation.guard.reaction_us + navigation.guard.observation_uncertainty_us;
    stop = ROBOT_BODY_MARGIN_MM +
        stopping_distance(forward, (uint32_t)navigation.guard.brake_x_mm_s2, delay_us) +
        stopping_distance(tangent, tangent_accel, delay_us);
    if (angular) stop += 181u - ROBOT_LIDAR_X_MM;
    if (!stop || stop > 7800u) stop = 0;
    io_write(IO_FRONT_STOP_MM, stop);
    io_write(IO_FRONT_SLOW_MM, stop ? stop + 200u : 0);
}

static void publish(const input_snapshot_t *s, uint32_t now_us)
{
    local_velocity_t command = {0, 0, 0};
    int valid = odom_started && odometry_valid(&odom, now_us);
    int autonomous = navigation.goal_active &&
        (task_mode == 3 || !(s->status & STATUS_LINK_DOWN));
    int sensors = (s->status & (STATUS_IMU_OK | STATUS_LIDAR_OK | STATUS_WHEEL_OK)) ==
                  (STATUS_IMU_OK | STATUS_LIDAR_OK | STATUS_WHEEL_OK);
    int auto_guard_stop = autonomous && valid && sensors && (s->status & STATUS_GUARD_STOP);
    if (autonomous && valid && sensors) {
        // The hardware guard gates the wheels; retain the checked navigation
        // request so a stop cannot erase its own direction and threshold.
        robot_navigation_step(&navigation, &odom, (s->status & STATUS_LIDAR_OK) != 0,
            1, now_us, &command);
        if (auto_guard_stop && navigation.status == ROBOT_NAV_FOLLOWING)
            navigation.status = ROBOT_NAV_BLOCKED;
    } else if (!autonomous && valid && sensors && navigation.calibration_ready &&
             !(s->status & STATUS_LINK_DOWN)) {
        /* The hardware guard stops the motor; keep the request visible so its
           direction and stopping threshold do not disappear while blocked. */
        command.vx_mm_s = s->command_vx;
        command.vy_mm_s = s->command_vy;
        command.wz_mrad_s = s->command_wz;
    }
    if (autonomous && (command.vx_mm_s || command.vy_mm_s || command.wz_mrad_s)) {
        int32_t delta = ((int32_t)navigation.max_accel_mm_s2 * (int32_t)CONTROL_PERIOD_US) / 1000000;
        if (previous_auto_guard_stop && !auto_guard_stop) {
            previous_command.vx_mm_s = scaled_float(odom.state[3]);
            previous_command.vy_mm_s = scaled_float(odom.state[4]);
            previous_command.wz_mrad_s = scaled_float(odom.state[5]);
        }
        if (s->status & STATUS_GUARD_SIDE) {
            previous_command.vx_mm_s = scaled_float(odom.state[3]);
            previous_command.wz_mrad_s = scaled_float(odom.state[5]);
        }
        if (delta < 1) delta = 1;
        command.vx_mm_s = limit_delta(command.vx_mm_s, previous_command.vx_mm_s, delta);
        command.vy_mm_s = limit_delta(command.vy_mm_s, previous_command.vy_mm_s, delta);
        command.wz_mrad_s = limit_delta(command.wz_mrad_s, previous_command.wz_mrad_s, 30);
    }
    previous_command = command;
    publish_front_threshold(&command, valid);
    if (auto_guard_stop) {
        // Preserve direction for the guard with only a 1-unit probe until the
        // next control cycle observes release and restarts the speed ramp.
        command.vx_mm_s = (command.vx_mm_s > 0) - (command.vx_mm_s < 0);
        command.vy_mm_s = (command.vy_mm_s > 0) - (command.vy_mm_s < 0);
        command.wz_mrad_s = (command.wz_mrad_s > 0) - (command.wz_mrad_s < 0);
    }
    previous_auto_guard_stop = (uint8_t)auto_guard_stop;
    io_write(IO_OUT_VX, (uint32_t)command.vx_mm_s);
    io_write(IO_OUT_VY, (uint32_t)command.vy_mm_s);
    io_write(IO_OUT_WZ, (uint32_t)command.wz_mrad_s);
    io_write(IO_OUT_AUTONOMOUS, (uint32_t)autonomous);
    io_write(IO_OUT_COMMAND_SEQ, s->command_seq);
    io_write(IO_OUT_MODE_SEQ, s->mode_seq);
    io_write(IO_OUT_COMMIT, IO_COMMIT_MAGIC);
    io_write(IO_POSE_X, valid ? (uint32_t)scaled_float(odom.state[0]) : 0);
    io_write(IO_POSE_Y, valid ? (uint32_t)scaled_float(odom.state[1]) : 0);
    io_write(IO_POSE_YAW, valid ? (uint32_t)scaled_float(odom.state[2]) : 0);
    io_write(IO_POSE_ROLL, 0); io_write(IO_POSE_PITCH, 0);
    io_write(IO_POSE_FLAGS, (uint32_t)valid);
    io_write(IO_POSE_STAMP, valid ? odom.last_update_us : now_us);
    io_write(IO_POSE_COMMIT, IO_COMMIT_MAGIC);
    // Fusion telemetry's covariance layout uses mm² for x/y and rad² for yaw.
    io_write(IO_COV_DIAG0, float_bits(odom.covariance[0][0] * 1000000.0f));
    io_write(IO_COV_DIAG0 + 4u, float_bits(odom.covariance[1][1] * 1000000.0f));
    io_write(IO_COV_DIAG0 + 8u, float_bits(odom.covariance[2][2]));
    io_write(IO_GYRO_BIAS, (uint32_t)scaled_float(odom.state[6]));
    io_write(IO_EKF_VX, (uint32_t)scaled_float(odom.state[3]));
    io_write(IO_EKF_VY, (uint32_t)scaled_float(odom.state[4]));
    io_write(IO_EKF_WZ, (uint32_t)scaled_float(odom.state[5]));
    io_write(IO_MAX_CONTROL_GAP, max_control_gap_us);
    io_write(IO_NAV_STATUS, (uint32_t)navigation.status |
        ((uint32_t)navigation.last_guard_result << 8) |
        (!navigation.calibration_ready ? (1u << 30) : 0u));
    io_write(IO_NAV_TASK, navigation.task_id);
    io_write(IO_NAV_PROGRESS, 0);
    io_write(IO_NAV_GOAL_X, (uint32_t)((int64_t)navigation.goal_x_mm * 65536 / 1000));
    io_write(IO_NAV_GOAL_Y, (uint32_t)((int64_t)navigation.goal_y_mm * 65536 / 1000));
    io_write(IO_NAV_EXPIRY, navigation.deadline_us);
    io_write(IO_MAP_VERSION, navigation.map.generation);
    io_write(IO_SEARCH_EXPANSIONS, navigation.plan.expansions);
    io_write(IO_MAP_AGE, now_us - navigation.last_ray_us);
    io_write(IO_LAST_TASK_ID, last_task_id);
    io_write(IO_NAV_MODE, task_mode);
    io_write(IO_NAV_COMMIT, IO_COMMIT_MAGIC);
}

void firmware_main(void)
{
    input_snapshot_t input;
    volatile float fp_probe = 1.25f;
    uint32_t now;
    fp_probe = fp_probe * 3.5f + 1.25f;
    if (!(fp_probe > 5.624f && fp_probe < 5.626f) || !robot_navigation_init(&navigation, 0)) {
        io_write(IO_SELFTEST, 0x80000001u);
        io_write(IO_FAULT, 0x71u);
        for (;;) { }
    }
    io_write(IO_SELFTEST, 0x80000000u);
    io_write(IO_READY, 1);
    task_mode = 1;
    grid_part = 8;
    for (;;) {
        read_inputs(&input);
        update_odometry(&input);
        io_write(IO_HEARTBEAT, ++heartbeat_counter);
        if (navigation.goal_active && task_mode == 2 && (input.status & STATUS_LINK_DOWN))
            robot_navigation_cancel(&navigation);
        if (command_seen && input.command_seq != last_command_seq && navigation.goal_active)
            robot_navigation_cancel(&navigation);
        command_seen = 1;
        last_command_seq = input.command_seq;
        now = io_read(IO_SNAPSHOT);
        if ((int32_t)(now - next_control_us) >= 0) {
            uint32_t gap = last_control_us ? now - last_control_us : CONTROL_PERIOD_US;
            if (gap > max_control_gap_us) max_control_gap_us = gap;
            publish(&input, now);
            last_control_us = now;
            next_control_us += CONTROL_PERIOD_US;
            if ((int32_t)(now - next_control_us) >= 0) next_control_us = now + CONTROL_PERIOD_US;
        }
        process_task(now);
        process_lidar(io_read(IO_SNAPSHOT), next_control_us);
        now = io_read(IO_SNAPSHOT);
        if ((int32_t)(next_control_us - now) > 4000)
            robot_navigation_work(&navigation, &odom, now);
        now = io_read(IO_SNAPSHOT);
        if ((int32_t)(next_control_us - now) > 8000) {
            capture_grid(now, next_control_us);
            if ((int32_t)(next_control_us - io_read(IO_SNAPSHOT)) > 4000) emit_grid();
        }
    }
}
