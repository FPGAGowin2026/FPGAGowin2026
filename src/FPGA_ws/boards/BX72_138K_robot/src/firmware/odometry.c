#include "odometry.h"

#define PI_F 3.14159265358979323846f
#define HALF_PI_F 1.57079632679489661923f
#define TWO_PI_F 6.28318530717958647692f
#define BIAS_WHEEL_STILL_MM_S 20
#define BIAS_GYRO_STILL_RAD_S 0.05f
#define GYRO_PRIMARY_MAX_AGE_US 20000u
#define BIAS_QUIET_DWELL_US 200000u
#define BIAS_WHEELS_QUIET 1u
#define BIAS_GYRO_QUIET 2u
#define BIAS_DWELL_STARTED 4u
#define BIAS_DWELL_READY 8u
#define BIAS_INPUT_MASK (BIAS_WHEELS_QUIET | BIAS_GYRO_QUIET)

enum { X, Y, YAW, VX, VY, YAW_RATE, GYRO_BIAS };

static float wrap_angle(float angle)
{
    while (angle > PI_F) angle -= TWO_PI_F;
    while (angle < -PI_F) angle += TWO_PI_F;
    return angle;
}

/* Argument reduction and polynomials avoid a dependency on bare-metal libm. */
static void sincos_approx(float angle, float *sine, float *cosine)
{
    float x = wrap_angle(angle), sign = 1.0f, x2;
    if (x > HALF_PI_F) { x = PI_F - x; sign = -1.0f; }
    else if (x < -HALF_PI_F) { x = -PI_F - x; sign = -1.0f; }
    x2 = x * x;
    *sine = x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f +
            x2 * (-1.0f / 5040.0f + x2 / 362880.0f))));
    *cosine = sign * (1.0f + x2 * (-1.0f / 2.0f + x2 * (1.0f / 24.0f +
              x2 * (-1.0f / 720.0f + x2 * (1.0f / 40320.0f -
              x2 / 3628800.0f)))));
}

void odometry_sincos(float angle_rad, float *sine, float *cosine)
{
    if (!sine || !cosine) return;
    sincos_approx(angle_rad, sine, cosine);
}

static int positive_finite(float value)
{
    return value > 0.0f && value < 1000000.0f;
}

static int quiet_bias_inputs(const odometry_t *odom)
{
    return (odom->bias_stationary & BIAS_INPUT_MASK) == BIAS_INPUT_MASK &&
        (uint32_t)(odom->last_update_us - odom->last_imu_us) <= GYRO_PRIMARY_MAX_AGE_US &&
        __builtin_fabsf(odom->state[YAW_RATE]) <= BIAS_GYRO_STILL_RAD_S;
}

static void update_bias_dwell(odometry_t *odom)
{
    if (!quiet_bias_inputs(odom)) {
        odom->bias_stationary = (uint8_t)(odom->bias_stationary & BIAS_INPUT_MASK);
    } else if (!(odom->bias_stationary & BIAS_DWELL_STARTED)) {
        odom->bias_quiet_since_us = odom->last_update_us;
        odom->bias_stationary = (uint8_t)(odom->bias_stationary | BIAS_DWELL_STARTED);
    } else if ((uint32_t)(odom->last_update_us - odom->bias_quiet_since_us) >= BIAS_QUIET_DWELL_US) {
        /* Latch until motion or a sample gap: long rest may wrap the clock. */
        odom->bias_stationary = (uint8_t)(odom->bias_stationary | BIAS_DWELL_READY);
    }
}

static int bias_learning_allowed(const odometry_t *odom)
{
    return (odom->bias_stationary & BIAS_DWELL_READY) && quiet_bias_inputs(odom);
}

static void history_record(odometry_t *odom, uint32_t stamp_us)
{
    uint16_t index = odom->history_next;
    if (odom->history_count)
        index = (uint16_t)((odom->history_next + ODOM_HISTORY_SIZE - 1u) % ODOM_HISTORY_SIZE);
    if (!odom->history_count || odom->history[index].stamp_us != stamp_us) {
        index = odom->history_next;
        odom->history_next = (uint16_t)((index + 1u) % ODOM_HISTORY_SIZE);
        if (odom->history_count < ODOM_HISTORY_SIZE) ++odom->history_count;
    }
    odom->history[index].stamp_us = stamp_us;
    odom->history[index].x_m = odom->state[X];
    odom->history[index].y_m = odom->state[Y];
    odom->history[index].yaw_rad = odom->state[YAW];
}

int odometry_init(odometry_t *odom, const odometry_config_t *config, uint32_t now_us)
{
    uint32_t i, j;
    if (!odom || !config || config->geometry_mm <= 0 || config->geometry_mm > 2000 ||
        !config->max_prediction_gap_us || !config->wheel_max_age_us || !config->imu_max_age_us ||
        config->max_prediction_gap_us >= 0x80000000u || config->wheel_max_age_us >= 0x80000000u ||
        config->imu_max_age_us >= 0x80000000u ||
        !positive_finite(config->process_position) || !positive_finite(config->process_yaw) ||
        !positive_finite(config->process_vx) || !positive_finite(config->process_vy) ||
        !positive_finite(config->process_yaw_rate) || !positive_finite(config->process_gyro_bias) ||
        !positive_finite(config->wheel_vx_variance) || !positive_finite(config->wheel_vy_variance) ||
        !positive_finite(config->wheel_yaw_rate_variance) || !positive_finite(config->gyro_variance) ||
        !positive_finite(config->initial_position_variance) || !positive_finite(config->initial_yaw_variance) ||
        !positive_finite(config->initial_velocity_variance) || !positive_finite(config->initial_yaw_rate_variance) ||
        !positive_finite(config->initial_bias_variance) ||
        !(config->initial_gyro_bias_rad_s > -50.0f && config->initial_gyro_bias_rad_s < 50.0f))
        return 0;
    odom->config = *config;
    for (i = 0; i < ODOM_STATE_SIZE; ++i) {
        odom->state[i] = 0.0f;
        for (j = 0; j < ODOM_STATE_SIZE; ++j) odom->covariance[i][j] = 0.0f;
    }
    odom->state[GYRO_BIAS] = config->initial_gyro_bias_rad_s;
    odom->covariance[X][X] = odom->covariance[Y][Y] = config->initial_position_variance;
    odom->covariance[YAW][YAW] = config->initial_yaw_variance;
    odom->covariance[VX][VX] = odom->covariance[VY][VY] = config->initial_velocity_variance;
    odom->covariance[YAW_RATE][YAW_RATE] = config->initial_yaw_rate_variance;
    odom->covariance[GYRO_BIAS][GYRO_BIAS] = config->initial_bias_variance;
    odom->last_update_us = odom->last_wheel_us = odom->last_imu_us = now_us;
    odom->history_next = odom->history_count = 0;
    odom->healthy = 1;
    odom->have_wheels = odom->have_imu = 0;
    odom->bias_stationary = 0;
    odom->bias_quiet_since_us = now_us;
    history_record(odom, now_us);
    return 1;
}

static int predict_to(odometry_t *odom, uint32_t stamp_us)
{
    uint32_t elapsed, i, j;
    float rows[3][ODOM_STATE_SIZE];
    float dt, sine, cosine, vx, vy, turn_x, turn_y, forward, lateral;
    const odometry_config_t *cfg = &odom->config;
    if (!odom->healthy) return 0;
    elapsed = stamp_us - odom->last_update_us;
    if (elapsed >= 0x80000000u) return 0;
    if ((odom->have_wheels && (uint32_t)(stamp_us - odom->last_wheel_us) > cfg->wheel_max_age_us) ||
        (odom->have_imu && (uint32_t)(stamp_us - odom->last_imu_us) > cfg->imu_max_age_us)) {
        odom->healthy = 0;
        odom->history_count = 0;
        return 0;
    }
    if (elapsed > cfg->max_prediction_gap_us) {
        odom->healthy = 0;
        odom->history_count = 0;
        return 0;
    }
    if (!elapsed) return 1;
    dt = (float)elapsed * 0.000001f;
    vx = odom->state[VX];
    vy = odom->state[VY];
    sincos_approx(odom->state[YAW], &sine, &cosine);
    turn_x = (-vx * sine - vy * cosine) * dt;
    turn_y = (vx * cosine - vy * sine) * dt;
    forward = cosine * dt;
    lateral = sine * dt;
    /* Only the three pose rows of F differ from identity. Save their F*P
       values before overwriting P; the velocity/bias block stays unchanged. */
    for (j = 0; j < ODOM_STATE_SIZE; ++j) {
        rows[X][j] = odom->covariance[X][j] + turn_x * odom->covariance[YAW][j] +
            forward * odom->covariance[VX][j] - lateral * odom->covariance[VY][j];
        rows[Y][j] = odom->covariance[Y][j] + turn_y * odom->covariance[YAW][j] +
            lateral * odom->covariance[VX][j] + forward * odom->covariance[VY][j];
        rows[YAW][j] = odom->covariance[YAW][j] + dt * odom->covariance[YAW_RATE][j];
    }
    for (i = X; i <= YAW; ++i) {
        for (j = i; j < ODOM_STATE_SIZE; ++j) {
            float value = rows[i][j];
            if (j == X)
                value += turn_x * rows[i][YAW] + forward * rows[i][VX] - lateral * rows[i][VY];
            else if (j == Y)
                value += turn_y * rows[i][YAW] + lateral * rows[i][VX] + forward * rows[i][VY];
            else if (j == YAW)
                value += dt * rows[i][YAW_RATE];
            odom->covariance[i][j] = value;
            odom->covariance[j][i] = value;
        }
    }
    odom->covariance[X][X] += cfg->process_position * dt;
    odom->covariance[Y][Y] += cfg->process_position * dt;
    odom->covariance[YAW][YAW] += cfg->process_yaw * dt;
    odom->covariance[VX][VX] += cfg->process_vx * dt;
    odom->covariance[VY][VY] += cfg->process_vy * dt;
    odom->covariance[YAW_RATE][YAW_RATE] += cfg->process_yaw_rate * dt;
    odom->covariance[GYRO_BIAS][GYRO_BIAS] += cfg->process_gyro_bias * dt;
    odom->state[X] += (vx * cosine - vy * sine) * dt;
    odom->state[Y] += (vx * sine + vy * cosine) * dt;
    odom->state[YAW] = wrap_angle(odom->state[YAW] + odom->state[YAW_RATE] * dt);
    odom->last_update_us = stamp_us;
    return 1;
}

static void scalar_update(odometry_t *odom, int state_a, int state_b,
                          float measurement, float variance)
{
    float column[ODOM_STATE_SIZE], gain[ODOM_STATE_SIZE];
    float expected = odom->state[state_a];
    float projected_variance, innovation_variance, innovation;
    uint32_t i, j;
    int learn_bias = bias_learning_allowed(odom);
    for (i = 0; i < ODOM_STATE_SIZE; ++i) {
        column[i] = odom->covariance[i][state_a];
        if (state_b >= 0) column[i] += odom->covariance[i][state_b];
    }
    projected_variance = column[state_a];
    if (state_b >= 0) {
        expected += odom->state[state_b];
        projected_variance += column[state_b];
    }
    innovation_variance = variance + projected_variance;
    innovation = measurement - expected;
    for (i = 0; i < ODOM_STATE_SIZE; ++i) {
        gain[i] = i == GYRO_BIAS && !learn_bias ?
                  0.0f : column[i] / innovation_variance;
        odom->state[i] += gain[i] * innovation;
    }
    /* Joseph form P' = (I-KH)P(I-KH)^T + KRK^T also applies when the
       bias gain is frozen. Keep P*H^T unchanged until the triangle is done. */
    for (i = 0; i < ODOM_STATE_SIZE; ++i) {
        const float residual_column = column[i] - gain[i] * projected_variance;
        for (j = i; j < ODOM_STATE_SIZE; ++j) {
            float value = odom->covariance[i][j] - gain[i] * column[j];
            value -= residual_column * gain[j];
            value += gain[i] * variance * gain[j];
            odom->covariance[i][j] = value;
            odom->covariance[j][i] = value;
        }
    }
    odom->state[YAW] = wrap_angle(odom->state[YAW]);
}

int odometry_update_wheels(odometry_t *odom, const int32_t wheel_mm_s[4], uint32_t stamp_us)
{
    float fl, fr, rl, rr, vx, vy, yaw_rate, geometry_m;
    uint32_t i;
    int had_both, stationary = 1;
    if (!odom || !wheel_mm_s) return 0;
    for (i = 0; i < 4; ++i) {
        if (wheel_mm_s[i] < -10000 || wheel_mm_s[i] > 10000) return 0;
        if (wheel_mm_s[i] < -BIAS_WHEEL_STILL_MM_S ||
            wheel_mm_s[i] > BIAS_WHEEL_STILL_MM_S) stationary = 0;
    }
    if (!predict_to(odom, stamp_us)) return 0;
    odom->bias_stationary = (uint8_t)((odom->bias_stationary & ~BIAS_WHEELS_QUIET) |
        (stationary ? BIAS_WHEELS_QUIET : 0u));
    update_bias_dwell(odom);
    had_both = odom->have_wheels && odom->have_imu;
    fl = (float)wheel_mm_s[0] * 0.001f;
    fr = (float)wheel_mm_s[1] * 0.001f;
    rl = (float)wheel_mm_s[2] * 0.001f;
    rr = (float)wheel_mm_s[3] * 0.001f;
    geometry_m = (float)odom->config.geometry_mm * 0.001f;
    vx = (fl + fr + rl + rr) * 0.25f;
    vy = (-fl + fr + rl - rr) * 0.25f;
    yaw_rate = (-fl + fr - rl + rr) / (4.0f * geometry_m);
    scalar_update(odom, VX, -1, vx, odom->config.wheel_vx_variance);
    scalar_update(odom, VY, -1, vy, odom->config.wheel_vy_variance);
    /* Fresh gyro carries motion yaw; quiet wheels and gyro permit zero-rate bias calibration. */
    if (!odom->have_imu || (uint32_t)(stamp_us - odom->last_imu_us) > GYRO_PRIMARY_MAX_AGE_US)
        scalar_update(odom, YAW_RATE, -1, yaw_rate, odom->config.wheel_yaw_rate_variance);
    else if (bias_learning_allowed(odom))
        scalar_update(odom, YAW_RATE, -1, 0.0f, odom->config.wheel_yaw_rate_variance);
    odom->last_wheel_us = stamp_us;
    odom->have_wheels = 1;
    if (odom->have_imu) {
        if (!had_both) odom->history_next = odom->history_count = 0;
        history_record(odom, stamp_us);
    }
    return 1;
}

int odometry_update_gyro(odometry_t *odom, float gyro_z_rad_s, uint32_t stamp_us)
{
    int had_both, reset_rate;
    uint32_t i;
    if (!odom || !(gyro_z_rad_s >= -50.0f && gyro_z_rad_s <= 50.0f)) return 0;
    if (!predict_to(odom, stamp_us)) return 0;
    reset_rate = !odom->have_imu ||
        (uint32_t)(stamp_us - odom->last_imu_us) > GYRO_PRIMARY_MAX_AGE_US;
    if (reset_rate)
        odom->bias_stationary = (uint8_t)(odom->bias_stationary & BIAS_INPUT_MASK);
    odom->bias_stationary = (uint8_t)((odom->bias_stationary & ~BIAS_GYRO_QUIET) |
        (__builtin_fabsf(gyro_z_rad_s) <= BIAS_GYRO_STILL_RAD_S ?
         BIAS_GYRO_QUIET : 0u));
    if (reset_rate) {
        /* A wheel-only rate prior must not suppress newly acquired gyro motion
           while bias is frozen. Set r=g-b and transform its complete covariance. */
        odom->state[YAW_RATE] = gyro_z_rad_s - odom->state[GYRO_BIAS];
        for (i = 0; i < ODOM_STATE_SIZE; ++i) {
            odom->covariance[YAW_RATE][i] = -odom->covariance[GYRO_BIAS][i];
            odom->covariance[i][YAW_RATE] = odom->covariance[YAW_RATE][i];
        }
        odom->covariance[YAW_RATE][YAW_RATE] = odom->config.gyro_variance +
            odom->covariance[GYRO_BIAS][GYRO_BIAS];
    }
    odom->last_imu_us = stamp_us;
    update_bias_dwell(odom);
    had_both = odom->have_wheels && odom->have_imu;
    if (!reset_rate)
        scalar_update(odom, YAW_RATE, GYRO_BIAS, gyro_z_rad_s, odom->config.gyro_variance);
    odom->have_imu = 1;
    if (odom->have_wheels) {
        if (!had_both) odom->history_next = odom->history_count = 0;
        history_record(odom, stamp_us);
    }
    return 1;
}

int odometry_valid(const odometry_t *odom, uint32_t now_us)
{
    uint32_t wheel_age, imu_age;
    if (!odom || !odom->healthy || !odom->have_wheels || !odom->have_imu) return 0;
    wheel_age = now_us - odom->last_wheel_us;
    imu_age = now_us - odom->last_imu_us;
    return wheel_age < 0x80000000u && imu_age < 0x80000000u &&
           wheel_age <= odom->config.wheel_max_age_us &&
           imu_age <= odom->config.imu_max_age_us;
}

int odometry_pose_at(const odometry_t *odom, uint32_t stamp_us,
                     float *x_m, float *y_m, float *yaw_rad)
{
    uint16_t offset;
    if (!odom || !x_m || !y_m || !yaw_rad || !odom->healthy ||
        !odom->have_wheels || !odom->have_imu || !odom->history_count)
        return 0;
    for (offset = 0; offset < odom->history_count; ++offset) {
        uint16_t index = (uint16_t)((odom->history_next + ODOM_HISTORY_SIZE - 1u - offset) %
                                    ODOM_HISTORY_SIZE);
        const odometry_history_entry_t *right = &odom->history[index];
        if (stamp_us == right->stamp_us) {
            *x_m = right->x_m; *y_m = right->y_m; *yaw_rad = right->yaw_rad;
            return 1;
        }
        if (offset + 1u < odom->history_count) {
            const odometry_history_entry_t *left =
                &odom->history[(index + ODOM_HISTORY_SIZE - 1u) % ODOM_HISTORY_SIZE];
            uint32_t span = right->stamp_us - left->stamp_us;
            uint32_t elapsed = stamp_us - left->stamp_us;
            if (span && span < 0x80000000u && elapsed < span) {
                float fraction = (float)elapsed / (float)span;
                *x_m = left->x_m + (right->x_m - left->x_m) * fraction;
                *y_m = left->y_m + (right->y_m - left->y_m) * fraction;
                *yaw_rad = wrap_angle(left->yaw_rad + wrap_angle(right->yaw_rad - left->yaw_rad) * fraction);
                return 1;
            }
        }
    }
    return 0;
}
