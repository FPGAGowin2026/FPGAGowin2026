#include "fusion.h"
#include "lite_math.h"

#define FUSION_SENSOR_TIMEOUT_US 500000u
#define FUSION_HISTORY_STEP_US 5000u
#define FUSION_MAX_PREDICT_US 100000u
#define FUSION_POSE_FUTURE_US 20000u
#define FUSION_GYRO_RAD_PER_LSB 0.00030543262f /* IMU660RC, +/-500 dps. */
#define FUSION_WHEEL_YAW_GAIN 0.08f

typedef struct {
    float x_mm, y_mm, yaw_rad;
    float vx_mm_s, vy_mm_s, wz_rad_s;
} fusion_pose_f_t;

static int32_t trig_cache_angle_mrad;
static int32_t trig_cache_sine;
static int32_t trig_cache_cosine;
static uint8_t trig_cache_valid;

static float clamp_f(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int finite_f(float value)
{
    return value == value && value < 1.0e20f && value > -1.0e20f;
}

/* Fast atan2 approximation is used only to establish/re-level roll and pitch. */
static float atan2_approx(float y, float x)
{
    float ax = abs_f(x);
    float ay = abs_f(y);
    float maximum = ax > ay ? ax : ay;
    float ratio = maximum > 1.0e-20f ? (ax < ay ? ax : ay) / maximum : 0.0f;
    float angle = ratio * (0.78539816339f + 0.273f * (1.0f - ratio));
    if (ay > ax) angle = 1.57079632679f - angle;
    if (x < 0.0f) angle = 3.14159265359f - angle;
    return y < 0.0f ? -angle : angle;
}

static void sincos_rad(float angle, float *sine, float *cosine)
{
    int32_t angle_mrad = float_to_i32(wrap_angle(angle) * 1000.0f);
    if (!trig_cache_valid || angle_mrad != trig_cache_angle_mrad) {
        trig_cache_angle_mrad = angle_mrad;
        trig_cache_sine = bx72_sin_q15(angle_mrad);
        trig_cache_cosine = bx72_cos_q15(angle_mrad);
        trig_cache_valid = 1u;
    }
    *sine = (float)trig_cache_sine * (1.0f / 32768.0f);
    *cosine = (float)trig_cache_cosine * (1.0f / 32768.0f);
}

void bx72_lite_imu_axes_transform(const bx72_lite_sample_t *raw,
                                  uint8_t flip_x_180,
                                  bx72_imu_axes_t *body)
{
    body->gx = raw->gx;
    body->ax = raw->ax;
    if (flip_x_180 != 0u) {
        body->gy = -raw->gy;
        body->gz = -raw->gz;
        body->ay = -raw->ay;
        body->az = -raw->az;
    } else {
        body->gy = raw->gy;
        body->gz = raw->gz;
        body->ay = raw->ay;
        body->az = raw->az;
    }
}

static int stamp_is_fresh(uint32_t stamp, uint32_t now, uint32_t maximum_age)
{
    int32_t age;
    if (stamp == 0u) return 0;
    age = (int32_t)(now - stamp);
    return age >= 0 && (uint32_t)age <= maximum_age;
}

static int stamp_is_newer(uint32_t stamp, uint32_t previous)
{
    return stamp != 0u && (previous == 0u || (int32_t)(stamp - previous) > 0);
}

static void history_push(bx72_lite_fusion_state_t *state, uint32_t stamp_us)
{
    bx72_fusion_history_t *entry;
    uint16_t index;
    if (state->history_count != 0u &&
        (uint32_t)(stamp_us - state->last_history_us) < FUSION_HISTORY_STEP_US)
        return;

    index = state->history_head;
    entry = &state->history[index];
    entry->stamp_us = stamp_us;
    entry->x_mm = state->x[0];
    entry->y_mm = state->x[1];
    entry->yaw_rad = wrap_angle(state->x[2]);
    entry->vx_mm_s = state->x[3];
    entry->vy_mm_s = state->x[4];
    entry->wz_rad_s = state->x[5];
    state->history_head = (uint16_t)((index + 1u) % BX72_FUSION_HISTORY);
    if (state->history_count < BX72_FUSION_HISTORY) ++state->history_count;
    state->last_history_us = stamp_us;
}

static uint16_t history_oldest(const bx72_lite_fusion_state_t *state)
{
    return (uint16_t)((state->history_head + BX72_FUSION_HISTORY -
                       state->history_count) % BX72_FUSION_HISTORY);
}

static void fill_pose(const bx72_lite_fusion_state_t *state,
                      const fusion_pose_f_t *sample,
                      uint32_t stamp_us,
                      bx72_pose_t *pose_out)
{
    pose_out->x_mm = float_to_i32(sample->x_mm);
    pose_out->y_mm = float_to_i32(sample->y_mm);
    pose_out->yaw_mrad = float_to_i32(wrap_angle(sample->yaw_rad) * 1000.0f);
    pose_out->roll_mrad = float_to_i32(state->roll_rad * 1000.0f);
    pose_out->pitch_mrad = float_to_i32(state->pitch_rad * 1000.0f);
    pose_out->vx_mm_s = float_to_i32(sample->vx_mm_s);
    pose_out->vy_mm_s = float_to_i32(sample->vy_mm_s);
    pose_out->wz_mrad_s = float_to_i32(sample->wz_rad_s * 1000.0f);
    pose_out->stamp_us = stamp_us;
    pose_out->flags = state->fusion_ready ? BX72_POSE_VALID : 0u;
}

/* 从最旧样本查找短历史，保留回绕、插值和最多20ms外推语义。 */
static int history_eval(const bx72_lite_fusion_state_t *state,
                        uint32_t stamp_us,
                        fusion_pose_f_t *out)
{
    uint16_t oldest, offset, index;
    const bx72_fusion_history_t *left;
    int32_t from_oldest;
    if (!state->fusion_ready || state->history_count == 0u) return 0;
    oldest = history_oldest(state);
    from_oldest = (int32_t)(stamp_us - state->history[oldest].stamp_us);
    if (from_oldest < 0) return 0;

    index = oldest;
    offset = 0u;
    left = &state->history[index];
    while (offset + 1u < state->history_count) {
        uint16_t right_index = (uint16_t)((index + 1u) % BX72_FUSION_HISTORY);
        const bx72_fusion_history_t *right = &state->history[right_index];
        int32_t to_right = (int32_t)(right->stamp_us - stamp_us);
        if (to_right >= 0) {
            uint32_t span = (uint32_t)(right->stamp_us - left->stamp_us);
            float ratio = span == 0u ? 0.0f :
                (float)(uint32_t)(stamp_us - left->stamp_us) / (float)span;
            float yaw_delta = wrap_angle(right->yaw_rad - left->yaw_rad);
            out->x_mm = left->x_mm + (right->x_mm - left->x_mm) * ratio;
            out->y_mm = left->y_mm + (right->y_mm - left->y_mm) * ratio;
            out->yaw_rad = wrap_angle(left->yaw_rad + yaw_delta * ratio);
            out->vx_mm_s = left->vx_mm_s + (right->vx_mm_s - left->vx_mm_s) * ratio;
            out->vy_mm_s = left->vy_mm_s + (right->vy_mm_s - left->vy_mm_s) * ratio;
            out->wz_rad_s = left->wz_rad_s + (right->wz_rad_s - left->wz_rad_s) * ratio;
            return 1;
        }
        index = right_index;
        left = right;
        ++offset;
    }

    {
        int32_t ahead = (int32_t)(stamp_us - left->stamp_us);
        if (ahead < 0) return 0;
        if ((uint32_t)ahead > FUSION_POSE_FUTURE_US) return 0;
        if ((uint32_t)ahead != 0u) {
            float dt = (float)(uint32_t)ahead * 1.0e-6f;
            float sine, cosine;
            sincos_rad(left->yaw_rad, &sine, &cosine);
            out->x_mm = left->x_mm +
                (left->vx_mm_s * cosine - left->vy_mm_s * sine) * dt;
            out->y_mm = left->y_mm +
                (left->vx_mm_s * sine + left->vy_mm_s * cosine) * dt;
            out->yaw_rad = wrap_angle(left->yaw_rad + left->wz_rad_s * dt);
        } else {
            out->x_mm = left->x_mm;
            out->y_mm = left->y_mm;
            out->yaw_rad = wrap_angle(left->yaw_rad);
        }
        out->vx_mm_s = left->vx_mm_s;
        out->vy_mm_s = left->vy_mm_s;
        out->wz_rad_s = left->wz_rad_s;
    }
    return 1;
}

static void publish_pose(const bx72_lite_fusion_state_t *state,
                         bx72_pose_t *pose, uint32_t stamp_us)
{
    fusion_pose_f_t sample;
    sample.x_mm = state->x[0];
    sample.y_mm = state->x[1];
    sample.yaw_rad = state->x[2];
    sample.vx_mm_s = state->x[3];
    sample.vy_mm_s = state->x[4];
    sample.wz_rad_s = state->x[5];
    fill_pose(state, &sample, stamp_us, pose);
}

void bx72_lite_fusion_init(bx72_lite_fusion_state_t *state)
{
    unsigned char *bytes = (unsigned char *)state;
    unsigned int i;
    for (i = 0; i < sizeof(*state); ++i) bytes[i] = 0u;
    state->pose_cov[0][0] = 625.0f;
    state->pose_cov[1][1] = 625.0f;
    state->pose_cov[2][2] = 0.000625f;
}

static void invalidate_pose(bx72_lite_fusion_state_t *state,
                            bx72_pose_t *pose, uint32_t now_us)
{
    /* 输入失效期间保留协方差并继续增加不确定度，恢复不能重置为初始精度。 */
    if (state->last_fusion_us != 0u) {
        uint32_t elapsed = now_us - state->last_fusion_us;
        if (elapsed < 0x80000000u) {
            float dt = (float)elapsed * 1.0e-6f;
            state->pose_cov[0][0] += 400.0f * dt;
            state->pose_cov[1][1] += 400.0f * dt;
            state->pose_cov[2][2] += 0.0004f * dt;
        }
    }
    state->fusion_ready = 0u;
    state->history_count = 0u;
    state->history_head = 0u;
    state->last_history_us = 0u;
    state->roll_pitch_ready = 0u;
    state->last_fusion_us = now_us;
    pose->flags = 0u;
    pose->vx_mm_s = 0;
    pose->vy_mm_s = 0;
    pose->wz_mrad_s = 0;
}

static void update_bias(bx72_lite_fusion_state_t *state,
                        const bx72_lite_sample_t *sample,
                        float magnitude,
                        float raw_gz_rad_s)
{
    int wheels_still = abs_f((float)sample->vx) < 15.0f &&
                       abs_f((float)sample->vy) < 15.0f &&
                       abs_f((float)sample->wz) < 15.0f;
    int gravity_valid = magnitude > 6800.0f && magnitude < 9600.0f;

    if (!wheels_still || !gravity_valid) {
        state->stationary_timing = 0u;
        if (!state->gyro_bias_ready) {
            state->gyro_bias_samples = 0u;
            state->x[6] = 0.0f;
        }
        return;
    }

    if (!state->stationary_timing) {
        state->stationary_timing = 1u;
        state->stationary_since_us = sample->imu_stamp_us;
        state->gyro_bias_samples = 0u;
        if (!state->gyro_bias_ready) state->x[6] = 0.0f;
    }
    if (!state->gyro_bias_ready) {
        uint16_t count = state->gyro_bias_samples;
        if (count < 256u) {
            ++count;
            state->gyro_bias_samples = count;
            state->x[6] += (raw_gz_rad_s - state->x[6]) / (float)count;
        }
        if (state->gyro_bias_samples >= 64u &&
            (uint32_t)(sample->imu_stamp_us - state->stationary_since_us) >= 1000000u)
            state->gyro_bias_ready = 1u;
    } else {
        /* Slow stationary tracking after initial convergence; never adapt in motion. */
        state->x[6] += (raw_gz_rad_s - state->x[6]) * (1.0f / 1024.0f);
    }
}

static void update_roll_pitch(bx72_lite_fusion_state_t *state,
                              const bx72_imu_axes_t *imu_body,
                              float magnitude,
                              float gx, float gy, float gz,
                              uint32_t dt_us)
{
    float dt = (float)dt_us * 1.0e-6f;
    float roll_acc, pitch_acc;
    float ax = (float)imu_body->ax;
    float ay = (float)imu_body->ay;
    float az = (float)imu_body->az;
    int acceleration_usable;

    acceleration_usable = abs_f(ax) < 20000.0f && abs_f(ay) < 20000.0f &&
                          abs_f(az) < 20000.0f;
    acceleration_usable = acceleration_usable && magnitude > 6800.0f &&
                          magnitude < 9600.0f;

    if (!state->roll_pitch_ready && acceleration_usable) {
        state->roll_rad = atan2_approx(ay, az);
        state->pitch_rad = clamp_f(
            atan2_approx(-ax, sqrt_approx(ay * ay + az * az)), -1.55f, 1.55f);
        state->roll_pitch_ready = 1u;
        return;
    }

    if (dt_us != 0u && dt_us <= FUSION_MAX_PREDICT_US) {
        float roll = state->roll_rad;
        float pitch = state->pitch_rad;
        float sin_roll, cos_roll, sin_pitch, cos_pitch;
        float pitch_rate, roll_rate;
        sincos_rad(roll, &sin_roll, &cos_roll);
        sincos_rad(pitch, &sin_pitch, &cos_pitch);
        pitch_rate = cos_roll * gy - sin_roll * gz;
        roll_rate = gx + (sin_roll * gy + cos_roll * gz) *
            (abs_f(cos_pitch) > 0.1f ? sin_pitch / cos_pitch : 0.0f);
        state->roll_rad = wrap_angle(roll + roll_rate * dt);
        state->pitch_rad = clamp_f(pitch + pitch_rate * dt, -1.55f, 1.55f);
    }

    if (acceleration_usable) {
        float alpha = dt / (0.5f + dt);
        roll_acc = atan2_approx(ay, az);
        pitch_acc = atan2_approx(-ax, sqrt_approx(ay * ay + az * az));
        if (dt_us != 0u) {
            state->roll_rad = wrap_angle(state->roll_rad +
                alpha * wrap_angle(roll_acc - state->roll_rad));
            state->pitch_rad = clamp_f(state->pitch_rad +
                alpha * (pitch_acc - state->pitch_rad), -1.55f, 1.55f);
        }
    }
}

void bx72_lite_fusion_update(bx72_lite_fusion_state_t *state,
                             bx72_pose_t *pose,
                             const bx72_lite_sample_t *sample)
{
    int new_imu = sample->imu_seq != state->last_imu_seq;
    int new_wheel = sample->wheel_seq != state->last_wheel_seq;
    int imu_ok, wheel_ok;

    if (new_imu) state->last_imu_seq = sample->imu_seq;
    if (new_wheel) state->last_wheel_seq = sample->wheel_seq;

    imu_ok = (sample->status & BX72_STATUS_IMU_OK) != 0u &&
             sample->imu_seq != 0u &&
             stamp_is_fresh(sample->imu_stamp_us, sample->now_us,
                            FUSION_SENSOR_TIMEOUT_US);
    wheel_ok = (sample->status & BX72_STATUS_WHEEL_OK) != 0u &&
               sample->wheel_seq != 0u &&
               stamp_is_fresh(sample->wheel_stamp_us, sample->now_us,
                              FUSION_SENSOR_TIMEOUT_US);
    if (!imu_ok || !wheel_ok) {
        invalidate_pose(state, pose, sample->now_us);
        return;
    }

    if (!state->fusion_ready) {
        state->x[0] = (float)pose->x_mm;
        state->x[1] = (float)pose->y_mm;
        state->x[2] = (float)pose->yaw_mrad * 0.001f;
        state->x[3] = (float)sample->vx;
        state->x[4] = (float)sample->vy;
        state->wheel_wz_rad_s = (float)sample->wz * 0.001f;
        state->gyro_rate_rad_s = 0.0f;
        state->x[5] = state->wheel_wz_rad_s;
        state->last_fusion_us = sample->now_us;
        state->last_imu_stamp_us = 0u;
        state->last_wheel_stamp_us = sample->wheel_stamp_us;
        state->fusion_ready = 1u;
    } else {
        uint32_t dt_us = (uint32_t)(sample->now_us - state->last_fusion_us);
        if (dt_us > FUSION_MAX_PREDICT_US) {
            invalidate_pose(state, pose, sample->now_us);
            return;
        }
        state->last_fusion_us = sample->now_us;
        if (dt_us != 0u) {
            float dt = (float)dt_us * 1.0e-6f;
            float sine, cosine, dx, dy, f0, f1;
            float p00, p01, p02, p11, p12, p22;
            sincos_rad(state->x[2], &sine, &cosine);
            dx = (state->x[3] * cosine - state->x[4] * sine) * dt;
            dy = (state->x[3] * sine + state->x[4] * cosine) * dt;
            state->x[0] += dx;
            state->x[1] += dy;
            state->x[2] = wrap_angle(state->x[2] + state->x[5] * dt);
            /* F 为 [[1,0,-dy],[0,1,dx],[0,0,1]]；Q 为原型噪声。 */
            f0 = -dy; f1 = dx;
            p00 = state->pose_cov[0][0]; p01 = state->pose_cov[0][1];
            p02 = state->pose_cov[0][2]; p11 = state->pose_cov[1][1];
            p12 = state->pose_cov[1][2]; p22 = state->pose_cov[2][2];
            state->pose_cov[0][0] = p00 + 2.0f*f0*p02 + f0*f0*p22 + 400.0f*dt;
            state->pose_cov[0][1] = p01 + f0*p12 + f1*p02 + f0*f1*p22;
            state->pose_cov[1][1] = p11 + 2.0f*f1*p12 + f1*f1*p22 + 400.0f*dt;
            state->pose_cov[0][2] = p02 + f0*p22;
            state->pose_cov[1][2] = p12 + f1*p22;
            state->pose_cov[2][2] = p22 + 0.0004f*dt;
            state->pose_cov[1][0] = state->pose_cov[0][1];
            state->pose_cov[2][0] = state->pose_cov[0][2];
            state->pose_cov[2][1] = state->pose_cov[1][2];
        }
    }

    if (new_wheel && stamp_is_newer(sample->wheel_stamp_us,
                                    state->last_wheel_stamp_us)) {
        state->x[3] = (float)sample->vx;
        state->x[4] = (float)sample->vy;
        state->wheel_wz_rad_s = (float)sample->wz * 0.001f;
        state->last_wheel_stamp_us = sample->wheel_stamp_us;
    }

    if (new_imu && stamp_is_newer(sample->imu_stamp_us,
                                  state->last_imu_stamp_us)) {
        bx72_imu_axes_t imu_body;
        uint32_t imu_dt_us = state->last_imu_stamp_us == 0u ? 0u :
            (uint32_t)(sample->imu_stamp_us - state->last_imu_stamp_us);
        float gx, gy, gz, ax, ay, az, magnitude;
        bx72_lite_imu_axes_transform(sample, BX72_IMU_MOUNT_FLIP_X_180, &imu_body);
        gx = (float)imu_body.gx * FUSION_GYRO_RAD_PER_LSB;
        gy = (float)imu_body.gy * FUSION_GYRO_RAD_PER_LSB;
        gz = (float)imu_body.gz * FUSION_GYRO_RAD_PER_LSB;
        ax = (float)imu_body.ax;
        ay = (float)imu_body.ay;
        az = (float)imu_body.az;
        /* 同一IMU快照的加速度模长供零偏与姿态更新共用。 */
        magnitude = sqrt_approx(ax * ax + ay * ay + az * az);

        update_bias(state, sample, magnitude, gz);
        update_roll_pitch(state, &imu_body, magnitude, gx, gy, gz - state->x[6], imu_dt_us);
        state->gyro_rate_rad_s = gz - state->x[6];
        state->last_imu_stamp_us = sample->imu_stamp_us;
    }

    /* Gyro carries short-term yaw; fresh wheel wz slowly anchors long-term rate. */
    state->x[5] = state->gyro_rate_rad_s + FUSION_WHEEL_YAW_GAIN *
        (state->wheel_wz_rad_s - state->gyro_rate_rad_s);
    state->x[2] = wrap_angle(state->x[2]);
    publish_pose(state, pose, sample->now_us);
    history_push(state, sample->now_us);
}

int bx72_lite_fusion_pose_at(const bx72_lite_fusion_state_t *state,
                             uint32_t stamp_us,
                             bx72_pose_t *pose_out)
{
    fusion_pose_f_t sample;
    if (!history_eval(state, stamp_us, &sample)) return 0;
    fill_pose(state, &sample, stamp_us, pose_out);
    return 1;
}

static void transform_history_pose(bx72_fusion_history_t *entry,
                                   float source_x, float source_y,
                                   float delta_x, float delta_y,
                                   float delta_yaw)
{
    /* Rebase the whole short history in the corrected odometry frame. Applying
     * the transform to only one endpoint of the source interpolation bracket
     * would give ICP a partially corrected reference and a fully corrected now. */
    {
        float sine, cosine;
        float dx = entry->x_mm - source_x;
        float dy = entry->y_mm - source_y;
        sincos_rad(delta_yaw, &sine, &cosine);
        entry->x_mm = source_x + delta_x + cosine * dx - sine * dy;
        entry->y_mm = source_y + delta_y + sine * dx + cosine * dy;
        entry->yaw_rad = wrap_angle(entry->yaw_rad + delta_yaw);
    }
}

int bx72_lite_fusion_csm_update(bx72_lite_fusion_state_t *state,
                                uint32_t source_stamp_us,
                                uint32_t now_us,
                                float x_mm, float y_mm, float yaw_rad,
                                float position_variance_mm2,
                                float yaw_variance_rad2)
{
    fusion_pose_f_t source, current;
    uint16_t oldest, index, offset;
    float dx, dy, dyaw, correction_x, correction_y, correction_yaw;
    float innovation[3], correction[3] = {0.0f, 0.0f, 0.0f};
    float noise[3];
    unsigned int axis, i, j;
    float source_x, source_y;

    if (!state->fusion_ready ||
        (state->have_csm_stamp && !stamp_is_newer(source_stamp_us, state->last_csm_stamp_us)) ||
        !stamp_is_fresh(source_stamp_us, now_us, 250000u) ||
        !finite_f(x_mm) || !finite_f(y_mm) || !finite_f(yaw_rad) ||
        !finite_f(position_variance_mm2) || !finite_f(yaw_variance_rad2) ||
        !history_eval(state, source_stamp_us, &source) ||
        !history_eval(state, now_us, &current)) return 0;

    dx = x_mm - source.x_mm;
    dy = y_mm - source.y_mm;
    dyaw = wrap_angle(yaw_rad - source.yaw_rad);
    if (abs_f(dx) > 300.0f || abs_f(dy) > 300.0f || abs_f(dyaw) > 0.45f)
        return 0;

    /* CSM 与里程计历史相关：放大R并设置协方差下界，避免假收敛。
     * 延迟观测使用当前P近似，不能解释为独立绝对定位。 */
    noise[0] = noise[1] = max_f(2500.0f, max_f(position_variance_mm2, 0.0f) * 4.0f);
    noise[2] = max_f(0.0025f, max_f(yaw_variance_rad2, 0.0f) * 4.0f);
    innovation[0] = dx; innovation[1] = dy; innovation[2] = dyaw;
    for (axis = 0u; axis < 3u; ++axis) {
        float column[3], denominator, residual;
        noise[axis] = max_f(noise[axis], 4.0f * state->pose_cov[axis][axis]);
        denominator = state->pose_cov[axis][axis] + noise[axis];
        residual = innovation[axis] - correction[axis];
        for (i = 0u; i < 3u; ++i) column[i] = state->pose_cov[i][axis];
        for (i = 0u; i < 3u; ++i) correction[i] += column[i] * residual / denominator;
        for (i = 0u; i < 3u; ++i)
            for (j = i; j < 3u; ++j) {
                float value = state->pose_cov[i][j] - column[i] * column[j] / denominator;
                state->pose_cov[i][j] = value;
                state->pose_cov[j][i] = value;
            }
    }
    state->pose_cov[0][0] = max_f(state->pose_cov[0][0], 625.0f);
    state->pose_cov[1][1] = max_f(state->pose_cov[1][1], 625.0f);
    state->pose_cov[2][2] = max_f(state->pose_cov[2][2], 0.000625f);
    correction_x = clamp_f(correction[0], -50.0f, 50.0f);
    correction_y = clamp_f(correction[1], -50.0f, 50.0f);
    correction_yaw = clamp_f(correction[2], -0.06f, 0.06f);
    state->last_csm_stamp_us = source_stamp_us;
    state->have_csm_stamp = 1u;
    source_x = source.x_mm;
    source_y = source.y_mm;

    oldest = history_oldest(state);
    index = oldest;
    for (offset = 0u; offset < state->history_count; ++offset) {
        bx72_fusion_history_t *entry = &state->history[index];
        transform_history_pose(entry, source_x, source_y,
                               correction_x, correction_y, correction_yaw);
        index = (uint16_t)((index + 1u) % BX72_FUSION_HISTORY);
    }

    if ((int32_t)(now_us - source_stamp_us) >= 0) {
        float sine, cosine;
        float current_dx = current.x_mm - source_x;
        float current_dy = current.y_mm - source_y;
        sincos_rad(correction_yaw, &sine, &cosine);
        state->x[0] = source_x + correction_x + cosine * current_dx - sine * current_dy;
        state->x[1] = source_y + correction_y + sine * current_dx + cosine * current_dy;
        state->x[2] = wrap_angle(current.yaw_rad + correction_yaw);
    }

    state->last_fusion_us = now_us;
    history_push(state, now_us);
    return 1;
}

void bx72_lite_fusion_diag(const bx72_lite_fusion_state_t *state,
                           uint32_t diag_words[7],
                           int32_t *gyro_bias_mrad_s)
{
    union { float f; uint32_t u; } bits;
    unsigned int i;
    bits.f = sqrt_approx(state->pose_cov[0][0]);
    diag_words[0] = bits.u;
    bits.f = sqrt_approx(state->pose_cov[1][1]);
    diag_words[1] = bits.u;
    bits.f = sqrt_approx(state->pose_cov[2][2]) * 1000.0f;
    diag_words[2] = bits.u;
    for (i = 3u; i < 7u; ++i) diag_words[i] = 0u;
    *gyro_bias_mrad_s = float_to_i32(state->x[6] * 1000.0f);
}

static void selftest_sample(bx72_lite_sample_t *sample,
                            uint32_t now_us, uint32_t seq,
                            int32_t vx, int32_t vy, int32_t wz)
{
    sample->now_us = now_us;
    sample->status = BX72_STATUS_IMU_OK | BX72_STATUS_WHEEL_OK;
    sample->imu_seq = seq;
    sample->imu_stamp_us = now_us;
    sample->gx = 0;
    sample->gy = 0;
    sample->gz = 0;
    sample->ax = 0;
    sample->ay = 0;
    sample->az = BX72_IMU_MOUNT_FLIP_X_180 ? -8192 : 8192;
    sample->wheel_seq = seq;
    sample->wheel_stamp_us = now_us;
    sample->vx = vx;
    sample->vy = vy;
    sample->wz = wz;
    sample->command_seq = 0u;
    sample->command_vx = 0;
    sample->command_vy = 0;
    sample->command_wz = 0;
    sample->mode_seq = 0u;
}

int bx72_lite_fusion_selftest(bx72_lite_fusion_state_t *state)
{
    bx72_lite_sample_t sample;
    bx72_lite_sample_t raw;
    bx72_imu_axes_t body;
    bx72_pose_t pose;
    int32_t x_before, y_before;

    raw.gx = 101;
    raw.gy = -202;
    raw.gz = 303;
    raw.ax = -404;
    raw.ay = 505;
    raw.az = -606;
    bx72_lite_imu_axes_transform(&raw, 1u, &body);
    if (body.gx != 101 || body.gy != 202 || body.gz != -303 ||
        body.ax != -404 || body.ay != -505 || body.az != 606) return 0;

    bx72_lite_fusion_init(state);
    pose.x_mm = pose.y_mm = pose.yaw_mrad = 0;
    pose.roll_mrad = pose.pitch_mrad = 0;
    pose.vx_mm_s = pose.vy_mm_s = pose.wz_mrad_s = 0;
    pose.stamp_us = 0u;
    pose.flags = 0u;
    selftest_sample(&sample, 10000u, 1u, 0, 120, 0);
    bx72_lite_fusion_update(state, &pose, &sample);
    selftest_sample(&sample, 20000u, 2u, 0, 120, 0);
    bx72_lite_fusion_update(state, &pose, &sample);
    if (pose.flags == 0u || pose.x_mm < -1 || pose.x_mm > 1 ||
        pose.y_mm < 0 || pose.y_mm > 3) return 0;

    /* First valid inverted gravity sample establishes non-zero roll and pitch. */
    bx72_lite_fusion_init(state);
    pose.x_mm = pose.y_mm = pose.yaw_mrad = 0;
    pose.roll_mrad = pose.pitch_mrad = 0;
    pose.flags = 0u;
    selftest_sample(&sample, 30000u, 1u, 0, 0, 0);
    sample.ax = -4096;
    sample.ay = BX72_IMU_MOUNT_FLIP_X_180 ? -3547 : 3547;
    sample.az = BX72_IMU_MOUNT_FLIP_X_180 ? -6144 : 6144;
    bx72_lite_fusion_update(state, &pose, &sample);
    if (pose.roll_mrad < 400 || pose.roll_mrad > 650 ||
        pose.pitch_mrad < 400 || pose.pitch_mrad > 650) return 0;

    /* A fresh gyro/wheel sample must move yaw in the commanded lateral-turn sign. */
    bx72_lite_fusion_init(state);
    pose.x_mm = pose.y_mm = pose.yaw_mrad = 0;
    pose.roll_mrad = pose.pitch_mrad = 0;
    pose.flags = 0u;
    selftest_sample(&sample, 40000u, 1u, 0, 0, 0);
    bx72_lite_fusion_update(state, &pose, &sample);
    selftest_sample(&sample, 50000u, 2u, 0, 0, 1000);
    sample.gz = BX72_IMU_MOUNT_FLIP_X_180 ? -3277 : 3277;
    bx72_lite_fusion_update(state, &pose, &sample);
    selftest_sample(&sample, 60000u, 3u, 0, 0, 1000);
    sample.gz = BX72_IMU_MOUNT_FLIP_X_180 ? -3277 : 3277;
    bx72_lite_fusion_update(state, &pose, &sample);
    if (pose.yaw_mrad <= 0 || pose.yaw_mrad > 30) return 0;

    x_before = pose.x_mm;
    y_before = pose.y_mm;
    if (!bx72_lite_fusion_pose_at(state, 70000u, &pose)) return 0;
    if (pose.x_mm < x_before - 1 || pose.y_mm < y_before - 1) return 0;

    sample.now_us = 250000u;
    sample.imu_stamp_us = 50000u;
    sample.wheel_stamp_us = 50000u;
    bx72_lite_fusion_update(state, &pose, &sample);
    if (pose.flags != 0u) return 0;

    return 1;
}
