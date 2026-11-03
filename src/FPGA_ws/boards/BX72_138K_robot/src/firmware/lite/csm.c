#include "csm.h"
#include "lite_math.h"

#define CSM_MIN_POINTS 18u
#define CSM_MIN_COVERAGE 48u
#define CSM_MIN_INLIERS 16u
#define CSM_MIN_CLOUD_SPREAD_MM2 250000.0f
#define CSM_MAX_RANGE_MM 6000u
#define CSM_MAX_SCAN_AGE_US 150000u
#define CSM_MATCH_DISTANCE_SQ_MM2 32400.0f /* 180 mm correspondence gate. */
#define CSM_MAX_LINE_RESIDUAL_MM 100.0f
#define CSM_HUBER_MM 50.0f
#define CSM_MAX_ITERATION_RMS_MM 100.0f
#define CSM_FINAL_RMS_MM 65.0f
#define CSM_THETA_SCALE_MM 500.0f
#define CSM_MAX_STEP_MM 60.0f
#define CSM_MAX_STEP_YAW_RAD 0.06f
#define CSM_MAX_TOTAL_CORRECTION_MM 200.0f
#define CSM_MAX_TOTAL_CORRECTION_YAW_RAD 0.35f
#define CSM_MAX_POINTS_PER_STEP 12u
#define CSM_KEYFRAME_DISTANCE_MM 100.0f
#define CSM_KEYFRAME_YAW_RAD 0.1f
#define CSM_MAX_NEIGHBOR_SPAN_MM 600.0f

typedef struct {
    float x_mm, y_mm, yaw_rad;
} csm_pose_f_t;

static uint8_t pose_cache_valid;
static int32_t pose_cache_yaw_mrad;
static float pose_cache_sine;
static float pose_cache_cosine;

static int finite_f(float value)
{
    return value == value && value < 1.0e20f && value > -1.0e20f;
}

static void angle_sincos(uint32_t angle_cdeg, float *sine, float *cosine)
{
    int32_t sin_q15, cos_q15;
    bx72_sincos_cdeg_q15(angle_cdeg, &sin_q15, &cos_q15);
    *sine = (float)sin_q15 * (1.0f / 32768.0f);
    *cosine = (float)cos_q15 * (1.0f / 32768.0f);
}

static void pose_sincos(int32_t yaw_mrad, float *sine, float *cosine)
{
    if (!pose_cache_valid || yaw_mrad != pose_cache_yaw_mrad) {
        pose_cache_yaw_mrad = yaw_mrad;
        pose_cache_sine = (float)bx72_sin_q15(yaw_mrad) * (1.0f / 32768.0f);
        pose_cache_cosine = (float)bx72_cos_q15(yaw_mrad) * (1.0f / 32768.0f);
        pose_cache_valid = 1u;
    }
    *sine = pose_cache_sine;
    *cosine = pose_cache_cosine;
}

static void pose_to_float(const bx72_pose_t *pose, csm_pose_f_t *out)
{
    out->x_mm = (float)pose->x_mm;
    out->y_mm = (float)pose->y_mm;
    out->yaw_rad = (float)pose->yaw_mrad * 0.001f;
}

static uint8_t count_valid(const uint8_t valid[BX72_CSM_POINTS])
{
    uint8_t count = 0u;
    unsigned int i;
    for (i = 0u; i < BX72_CSM_POINTS; ++i)
        if (valid[i]) ++count;
    return count;
}

/* Reject point sets whose covariance is effectively one-dimensional. */
static int points_are_degenerate(const float x_points[BX72_CSM_POINTS],
                                 const float y_points[BX72_CSM_POINTS],
                                 const uint8_t valid[BX72_CSM_POINTS])
{
    float mean_x = 0.0f, mean_y = 0.0f;
    float xx = 0.0f, xy = 0.0f, yy = 0.0f;
    float trace, determinant;
    unsigned int i;
    uint8_t count = count_valid(valid);
    if (count < CSM_MIN_POINTS) return 1;
    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        if (!valid[i]) continue;
        mean_x += x_points[i];
        mean_y += y_points[i];
    }
    mean_x /= (float)count;
    mean_y /= (float)count;
    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        float dx, dy;
        if (!valid[i]) continue;
        dx = x_points[i] - mean_x;
        dy = y_points[i] - mean_y;
        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
    }
    trace = xx + yy;
    determinant = xx * yy - xy * xy;
    return trace < CSM_MIN_CLOUD_SPREAD_MM2 ||
           determinant < trace * trace * 0.003f;
}

static void clear_collect(bx72_csm_state_t *state)
{
    unsigned int i;
    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        state->collect_valid[i] = 0u;
        state->collect_seen[i] = 0u;
    }
    state->collect_count = 0u;
    state->collect_seen_count = 0u;
    state->collect_pose_valid = 0u;
    state->status &= ~BX72_CSM_STATUS_CAPTURE;
}

static void reject_scan(bx72_csm_state_t *state, int degenerate)
{
    ++state->rejected_count;
    state->last_result = 2u;
    state->icp_active = 0u;
    state->icp_point_index = 0u;
    state->status &= ~(BX72_CSM_STATUS_ACCEPTED | BX72_CSM_STATUS_ICP);
    state->status |= BX72_CSM_STATUS_REJECTED;
    if (degenerate) state->status |= BX72_CSM_STATUS_DEGENERATE;
}

static void skip_scan_while_busy(bx72_csm_state_t *state)
{
    ++state->rejected_count;
    state->last_result = 2u;
    state->status |= BX72_CSM_STATUS_REJECTED;
}

static void copy_candidate_to_reference(bx72_csm_state_t *state,
                                        float x_mm, float y_mm,
                                        float yaw_rad)
{
    unsigned int i;
    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        state->reference_valid[i] = state->candidate_valid[i];
        state->reference_x[i] = state->candidate_valid[i] ? state->candidate_x[i] : 0.0f;
        state->reference_y[i] = state->candidate_valid[i] ? state->candidate_y[i] : 0.0f;
    }
    state->reference_x_mm = x_mm;
    state->reference_y_mm = y_mm;
    state->reference_yaw_rad = wrap_angle(yaw_rad);
    state->status |= BX72_CSM_STATUS_REFERENCE;
}

static void reset_normal_equations(bx72_csm_state_t *state)
{
    state->icp_point_index = 0u;
    state->icp_inliers = 0u;
    state->icp_h00 = state->icp_h01 = state->icp_h02 = 0.0f;
    state->icp_h11 = state->icp_h12 = state->icp_h22 = 0.0f;
    state->icp_g0 = state->icp_g1 = state->icp_g2 = 0.0f;
    state->icp_normal_xx = state->icp_normal_xy = state->icp_normal_yy = 0.0f;
    state->icp_error_sq = 0.0f;
}

/* 原始刚性扫描：不使用轮里程计对点去畸变；快速运动仍会产生扫描畸变。 */
static void prepare_candidate_from_collect(bx72_csm_state_t *state)
{
    unsigned int i;
    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        if (!state->collect_valid[i]) {
            state->candidate_valid[i] = 0u;
            state->candidate_x[i] = 0.0f;
            state->candidate_y[i] = 0.0f;
            continue;
        }
        {
            state->candidate_x[i] = state->collect_scan_x[i];
            state->candidate_y[i] = state->collect_scan_y[i];
            state->candidate_valid[i] = 1u;
        }
    }
    state->candidate_pose = state->collect_last_pose;
}

static void finish_collected_scan(bx72_csm_state_t *state)
{
    csm_pose_f_t reference_pose, candidate_pose;
    float dx_world, dy_world, sine, cosine;
    float predicted_x, predicted_y, predicted_yaw;

    if (state->icp_active) {
        /* Keep the in-flight scan immutable; this later revolution is skipped. */
        skip_scan_while_busy(state);
        return;
    }
    if (!state->collect_pose_valid ||
        state->collect_seen_count < CSM_MIN_COVERAGE ||
        state->collect_count < CSM_MIN_POINTS) {
        reject_scan(state, 0);
        return;
    }
    if (points_are_degenerate(state->collect_scan_x, state->collect_scan_y,
                              state->collect_valid)) {
        reject_scan(state, 1);
        return;
    }

    prepare_candidate_from_collect(state);
    pose_to_float(&state->candidate_pose, &candidate_pose);

    if ((state->status & BX72_CSM_STATUS_REFERENCE) == 0u) {
        copy_candidate_to_reference(state, candidate_pose.x_mm,
                                    candidate_pose.y_mm, candidate_pose.yaw_rad);
        state->last_accepted_stamp_us = state->candidate_pose.stamp_us;
        state->status &= ~(BX72_CSM_STATUS_REJECTED | BX72_CSM_STATUS_DEGENERATE);
        state->last_result = 0u;
        return;
    }

    reference_pose.x_mm = state->reference_x_mm;
    reference_pose.y_mm = state->reference_y_mm;
    reference_pose.yaw_rad = state->reference_yaw_rad;
    dx_world = candidate_pose.x_mm - reference_pose.x_mm;
    dy_world = candidate_pose.y_mm - reference_pose.y_mm;
    pose_sincos(float_to_i32(reference_pose.yaw_rad * 1000.0f), &sine, &cosine);
    predicted_x = cosine * dx_world + sine * dy_world;
    predicted_y = -sine * dx_world + cosine * dy_world;
    predicted_yaw = wrap_angle(candidate_pose.yaw_rad - reference_pose.yaw_rad);

    if ((int32_t)(state->candidate_pose.stamp_us - state->last_accepted_stamp_us) <= 0) {
        reject_scan(state, 0);
        return;
    }

    state->icp_x_mm = predicted_x;
    state->icp_y_mm = predicted_y;
    state->icp_yaw_rad = predicted_yaw;
    state->icp_iteration = 0u;
    state->icp_previous_rms_mm = 0.0f;
    state->best_rms_mm = 0.0f;
    state->icp_active = 1u;
    state->status &= ~(BX72_CSM_STATUS_ACCEPTED | BX72_CSM_STATUS_REJECTED |
                       BX72_CSM_STATUS_DEGENERATE);
    state->status |= BX72_CSM_STATUS_ICP;
    state->last_result = 0u;
    reset_normal_equations(state);
}

static int line_normal(const bx72_csm_state_t *state,
                       unsigned int center,
                       float *normal_x, float *normal_y)
{
    int before = -1, after = -1;
    unsigned int step;
    float dx, dy, length_sq, inverse_length;
    for (step = 1u; step <= 2u; ++step) {
        unsigned int index = (center + BX72_CSM_POINTS - step) % BX72_CSM_POINTS;
        if (state->reference_valid[index]) { before = (int)index; break; }
    }
    for (step = 1u; step <= 2u; ++step) {
        unsigned int index = (center + step) % BX72_CSM_POINTS;
        if (state->reference_valid[index]) { after = (int)index; break; }
    }

    if (before >= 0 && after >= 0) {
        float span_x = state->reference_x[after] - state->reference_x[before];
        float span_y = state->reference_y[after] - state->reference_y[before];
        if (span_x * span_x + span_y * span_y >
            CSM_MAX_NEIGHBOR_SPAN_MM * CSM_MAX_NEIGHBOR_SPAN_MM) {
            before = -1;
            after = -1;
        }
    }
    if (before >= 0 && after >= 0) {
        dx = state->reference_x[after] - state->reference_x[before];
        dy = state->reference_y[after] - state->reference_y[before];
    } else if (after >= 0) {
        dx = state->reference_x[after] - state->reference_x[center];
        dy = state->reference_y[after] - state->reference_y[center];
    } else if (before >= 0) {
        dx = state->reference_x[center] - state->reference_x[before];
        dy = state->reference_y[center] - state->reference_y[before];
    } else {
        return 0;
    }
    length_sq = dx * dx + dy * dy;
    if (!(length_sq > 1.0f) ||
        length_sq > CSM_MAX_NEIGHBOR_SPAN_MM * CSM_MAX_NEIGHBOR_SPAN_MM)
        return 0;
    inverse_length = 1.0f / sqrt_approx(length_sq);
    *normal_x = -dy * inverse_length;
    *normal_y = dx * inverse_length;
    return 1;
}

static void process_icp_point(bx72_csm_state_t *state, unsigned int point_index)
{
    float sine, cosine, qx, qy;
    float nearest_sq = CSM_MATCH_DISTANCE_SQ_MM2;
    float normal_x, normal_y, residual;
    float j0, j1, j2, weight, absolute_residual;
    int nearest = -1;
    unsigned int i;
    if (!state->candidate_valid[point_index]) return;

    pose_sincos(float_to_i32(state->icp_yaw_rad * 1000.0f), &sine, &cosine);
    qx = state->icp_x_mm + cosine * state->candidate_x[point_index] -
         sine * state->candidate_y[point_index];
    qy = state->icp_y_mm + sine * state->candidate_x[point_index] +
         cosine * state->candidate_y[point_index];

    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        float dx, dy, distance_sq;
        if (!state->reference_valid[i]) continue;
        dx = qx - state->reference_x[i];
        dy = qy - state->reference_y[i];
        distance_sq = dx * dx + dy * dy;
        if (distance_sq < nearest_sq) {
            nearest_sq = distance_sq;
            nearest = (int)i;
        }
    }
    if (nearest < 0 || !line_normal(state, (unsigned int)nearest,
                                    &normal_x, &normal_y)) return;

    residual = normal_x * (qx - state->reference_x[nearest]) +
               normal_y * (qy - state->reference_y[nearest]);
    absolute_residual = abs_f(residual);
    if (absolute_residual > CSM_MAX_LINE_RESIDUAL_MM) return;
    weight = absolute_residual > CSM_HUBER_MM ?
             CSM_HUBER_MM / absolute_residual : 1.0f;

    j0 = normal_x;
    j1 = normal_y;
    j2 = (normal_x * (-sine * state->candidate_x[point_index] -
                      cosine * state->candidate_y[point_index]) +
          normal_y * (cosine * state->candidate_x[point_index] -
                      sine * state->candidate_y[point_index])) *
         (1.0f / CSM_THETA_SCALE_MM);
    state->icp_h00 += weight * j0 * j0;
    state->icp_h01 += weight * j0 * j1;
    state->icp_h02 += weight * j0 * j2;
    state->icp_h11 += weight * j1 * j1;
    state->icp_h12 += weight * j1 * j2;
    state->icp_h22 += weight * j2 * j2;
    state->icp_g0 += weight * j0 * residual;
    state->icp_g1 += weight * j1 * residual;
    state->icp_g2 += weight * j2 * residual;
    state->icp_normal_xx += weight * normal_x * normal_x;
    state->icp_normal_xy += weight * normal_x * normal_y;
    state->icp_normal_yy += weight * normal_y * normal_y;
    state->icp_error_sq += residual * residual;
    if (state->icp_inliers < 255u) ++state->icp_inliers;
}

static int solve_normal_equations(const bx72_csm_state_t *state,
                                  float *dx, float *dy, float *dtheta)
{
    float matrix[3][4];
    float trace = state->icp_h00 + state->icp_h11 + state->icp_h22;
    float normal_det = state->icp_normal_xx * state->icp_normal_yy -
                       state->icp_normal_xy * state->icp_normal_xy;
    float h_minor = state->icp_h00 * state->icp_h11 -
                    state->icp_h01 * state->icp_h01;
    float determinant;
    unsigned int column, row;

    if (!(trace > 1.0e-5f) ||
        normal_det < 0.015f *
            (state->icp_normal_xx + state->icp_normal_yy) *
            (state->icp_normal_xx + state->icp_normal_yy) ||
        h_minor < 1.0e-5f * state->icp_h00 * state->icp_h11)
        return 0;
    determinant = state->icp_h00 *
        (state->icp_h11 * state->icp_h22 - state->icp_h12 * state->icp_h12) -
        state->icp_h01 *
        (state->icp_h01 * state->icp_h22 - state->icp_h12 * state->icp_h02) +
        state->icp_h02 *
        (state->icp_h01 * state->icp_h12 - state->icp_h11 * state->icp_h02);
    if (!(determinant > 1.0e-6f *
          max_f(1.0f, state->icp_h00 * state->icp_h11 * state->icp_h22)))
        return 0;

    matrix[0][0] = state->icp_h00;
    matrix[0][1] = state->icp_h01;
    matrix[0][2] = state->icp_h02;
    matrix[0][3] = -state->icp_g0;
    matrix[1][0] = state->icp_h01;
    matrix[1][1] = state->icp_h11;
    matrix[1][2] = state->icp_h12;
    matrix[1][3] = -state->icp_g1;
    matrix[2][0] = state->icp_h02;
    matrix[2][1] = state->icp_h12;
    matrix[2][2] = state->icp_h22;
    matrix[2][3] = -state->icp_g2;

    for (column = 0u; column < 3u; ++column) {
        unsigned int pivot = column;
        float pivot_abs = abs_f(matrix[pivot][column]);
        for (row = column + 1u; row < 3u; ++row) {
            float candidate_abs = abs_f(matrix[row][column]);
            if (candidate_abs > pivot_abs) {
                pivot = row;
                pivot_abs = candidate_abs;
            }
        }
        if (!(pivot_abs > trace * 1.0e-7f)) return 0;
        if (pivot != column) {
            unsigned int k;
            for (k = column; k < 4u; ++k) {
                float swap = matrix[column][k];
                matrix[column][k] = matrix[pivot][k];
                matrix[pivot][k] = swap;
            }
        }
        {
            float inverse_pivot = 1.0f / matrix[column][column];
            unsigned int k;
            for (k = column; k < 4u; ++k) matrix[column][k] *= inverse_pivot;
        }
        for (row = 0u; row < 3u; ++row) {
            float scale;
            unsigned int k;
            if (row == column) continue;
            scale = matrix[row][column];
            for (k = column; k < 4u; ++k)
                matrix[row][k] -= scale * matrix[column][k];
        }
    }

    *dx = matrix[0][3];
    *dy = matrix[1][3];
    *dtheta = matrix[2][3] / CSM_THETA_SCALE_MM;
    return finite_f(*dx) && finite_f(*dy) && finite_f(*dtheta);
}

static void rebase_collect_pose(bx72_csm_state_t *state,
                                 const bx72_pose_t *old_pose,
                                 const bx72_pose_t *new_pose)
{
    float delta_yaw = wrap_angle((float)(new_pose->yaw_mrad - old_pose->yaw_mrad) * 0.001f);
    float sine, cosine;
    pose_sincos(float_to_i32(delta_yaw * 1000.0f), &sine, &cosine);
    /* 当前采集点固定于车体坐标，不随融合修正平移。 */
    if (state->collect_pose_valid) {
        float dx = (float)state->collect_last_pose.x_mm - (float)old_pose->x_mm;
        float dy = (float)state->collect_last_pose.y_mm - (float)old_pose->y_mm;
        state->collect_last_pose.x_mm = new_pose->x_mm +
            float_to_i32(cosine * dx - sine * dy);
        state->collect_last_pose.y_mm = new_pose->y_mm +
            float_to_i32(sine * dx + cosine * dy);
        state->collect_last_pose.yaw_mrad = new_pose->yaw_mrad +
            state->collect_last_pose.yaw_mrad - old_pose->yaw_mrad;
    }
}

static int finish_icp(bx72_csm_state_t *state,
                      const bx72_pose_t *pose_now,
                      uint32_t now_us,
                      bx72_lite_fusion_state_t *fusion)
{
    bx72_pose_t old_now, new_now;
    float sine, cosine;
    float measured_x, measured_y, measured_yaw;
    float odom_dx, odom_dy, odom_dyaw;
    float variance_mm2;

    pose_sincos(float_to_i32(state->reference_yaw_rad * 1000.0f), &sine, &cosine);
    measured_x = state->reference_x_mm + cosine * state->icp_x_mm -
                 sine * state->icp_y_mm;
    measured_y = state->reference_y_mm + sine * state->icp_x_mm +
                 cosine * state->icp_y_mm;
    measured_yaw = wrap_angle(state->reference_yaw_rad + state->icp_yaw_rad);
    odom_dx = measured_x - (float)state->candidate_pose.x_mm;
    odom_dy = measured_y - (float)state->candidate_pose.y_mm;
    odom_dyaw = wrap_angle(measured_yaw -
                           (float)state->candidate_pose.yaw_mrad * 0.001f);
    if (abs_f(odom_dx) > CSM_MAX_TOTAL_CORRECTION_MM ||
        abs_f(odom_dy) > CSM_MAX_TOTAL_CORRECTION_MM ||
        abs_f(odom_dyaw) > CSM_MAX_TOTAL_CORRECTION_YAW_RAD) {
        reject_scan(state, 0);
        return 0;
    }
    if (state->best_rms_mm > CSM_FINAL_RMS_MM ||
        state->best_rms_mm != state->best_rms_mm) {
        reject_scan(state, 0);
        return 0;
    }

    /* The delayed correction also rebases points already captured in this scan. */
    if (!bx72_lite_fusion_pose_at(fusion, now_us, &old_now)) {
        if (pose_now == 0 || (pose_now->flags & BX72_POSE_VALID) == 0u) {
            reject_scan(state, 0);
            return 0;
        }
        old_now = *pose_now;
    }
    variance_mm2 = state->best_rms_mm * state->best_rms_mm;
    if (!bx72_lite_fusion_csm_update(fusion, state->candidate_pose.stamp_us,
                                     now_us, measured_x, measured_y, measured_yaw,
                                     variance_mm2, 0.0004f)) {
        reject_scan(state, 0);
        return 0;
    }
    if (!bx72_lite_fusion_pose_at(fusion, now_us, &new_now)) {
        /* The correction is already bounded and applied; its state has a valid now. */
        new_now = old_now;
        new_now.x_mm = float_to_i32(fusion->x[0]);
        new_now.y_mm = float_to_i32(fusion->x[1]);
        new_now.yaw_mrad = float_to_i32(fusion->x[2] * 1000.0f);
    }
    rebase_collect_pose(state, &old_now, &new_now);

    /* 合格新扫描始终修正融合；仅在测得运动达到阈值时换关键帧。 */
    state->last_accepted_stamp_us = state->candidate_pose.stamp_us;
    if (state->icp_x_mm * state->icp_x_mm + state->icp_y_mm * state->icp_y_mm >
            CSM_KEYFRAME_DISTANCE_MM * CSM_KEYFRAME_DISTANCE_MM ||
        abs_f(wrap_angle(state->icp_yaw_rad)) > CSM_KEYFRAME_YAW_RAD) {
        copy_candidate_to_reference(state, measured_x, measured_y, measured_yaw);
    }
    state->best_x_mm = measured_x;
    state->best_y_mm = measured_y;
    state->best_yaw_rad = measured_yaw;
    ++state->accepted_count;
    state->last_result = 1u;
    state->icp_active = 0u;
    state->status &= ~(BX72_CSM_STATUS_REJECTED | BX72_CSM_STATUS_ICP |
                       BX72_CSM_STATUS_DEGENERATE);
    state->status |= BX72_CSM_STATUS_ACCEPTED | BX72_CSM_STATUS_REFERENCE;
    return 1;
}

static void finish_icp_iteration(bx72_csm_state_t *state,
                                const bx72_pose_t *pose_now,
                                uint32_t now_us,
                                bx72_lite_fusion_state_t *fusion)
{
    float rms, dx, dy, dtheta, step_mm;
    float normal_trace = state->icp_normal_xx + state->icp_normal_yy;
    if (state->icp_inliers < CSM_MIN_INLIERS || !(normal_trace > 0.0f)) {
        reject_scan(state, 1);
        return;
    }
    rms = sqrt_approx(state->icp_error_sq / (float)state->icp_inliers);
    if (!finite_f(rms) || rms > CSM_MAX_ITERATION_RMS_MM ||
        (state->icp_iteration != 0u &&
         rms > state->icp_previous_rms_mm * 1.4f + 15.0f)) {
        reject_scan(state, 0);
        return;
    }
    if (!solve_normal_equations(state, &dx, &dy, &dtheta)) {
        reject_scan(state, 1);
        return;
    }
    step_mm = sqrt_approx(dx * dx + dy * dy);
    if (step_mm > CSM_MAX_STEP_MM || abs_f(dtheta) > CSM_MAX_STEP_YAW_RAD) {
        reject_scan(state, 0);
        return;
    }
    state->icp_x_mm += dx;
    state->icp_y_mm += dy;
    state->icp_yaw_rad = wrap_angle(state->icp_yaw_rad + dtheta);
    state->icp_previous_rms_mm = rms;
    state->best_rms_mm = rms;
    ++state->icp_iteration;

    if ((rms <= CSM_FINAL_RMS_MM && step_mm <= 2.0f && abs_f(dtheta) <= 0.002f) ||
        state->icp_iteration >= 3u) {
        if (state->icp_iteration >= 3u &&
            (step_mm > 25.0f || abs_f(dtheta) > 0.025f)) {
            reject_scan(state, 0);
            return;
        }
        finish_icp(state, pose_now, now_us, fusion);
        return;
    }
    reset_normal_equations(state);
}

void bx72_csm_init(bx72_csm_state_t *state)
{
    unsigned char *bytes = (unsigned char *)state;
    unsigned int i;
    for (i = 0u; i < sizeof(*state); ++i) bytes[i] = 0u;
    state->status = BX72_CSM_STATUS_NEED_SYNC;
    state->best_rms_mm = 1.0e9f;
}

void bx72_csm_point(bx72_csm_state_t *state,
                    uint32_t angle_cdeg, uint32_t distance_mm, uint32_t hit,
                    const bx72_pose_t *pose_at_beam)
{
    uint16_t angle;
    int wrapped;
    unsigned int sector;
    angle = (uint16_t)angle_cdeg;
    wrapped = state->angle_ready && state->previous_angle_cdeg > 30000u &&
              angle < 6000u;
    if (wrapped) {
        if (state->synchronized) finish_collected_scan(state);
        else {
            state->synchronized = 1u;
            state->status &= ~BX72_CSM_STATUS_NEED_SYNC;
        }
        clear_collect(state);
    }
    state->previous_angle_cdeg = angle;
    state->angle_ready = 1u;
    state->status |= BX72_CSM_STATUS_CAPTURE;
    sector = angle / 500u;
    if (!state->collect_seen[sector]) {
        state->collect_seen[sector] = 1u;
        if (state->collect_seen_count < 255u) ++state->collect_seen_count;
    }
    if (pose_at_beam->stamp_us != 0u) {
        state->collect_last_pose = *pose_at_beam;
        state->collect_pose_valid = 1u;
    }
    /* Approximate vehicle occlusion: rear and two support pillars. Coverage
     * remains seen, but these returns must not anchor the scan matcher. */
    if ((angle_cdeg >= 13000u && angle_cdeg <= 23500u) ||
        hit == 0u ||
        distance_mm < 100u || distance_mm > CSM_MAX_RANGE_MM) return;

    {
        float bearing_sine, bearing_cosine, x, y;
        /* N10 raw angles increase clockwise; pose yaw is counterclockwise. */
        angle_sincos((36000u - angle) % 36000u, &bearing_sine, &bearing_cosine);
        /* 匹配点与参考位姿都使用车体坐标，原始极坐标仅在此平移一次。 */
        x = (float)BX72_LIDAR_FORWARD_OFFSET_MM + (float)distance_mm * bearing_cosine;
        y = (float)distance_mm * bearing_sine;
        /* 车体投影内的近回波不能作为静态环境匹配点。 */
        if (x >= -(float)(BX72_BODY_LENGTH_MM / 2u) && x <= (float)(BX72_BODY_LENGTH_MM / 2u) &&
            y >= -(float)(BX72_BODY_WIDTH_MM / 2u) && y <= (float)(BX72_BODY_WIDTH_MM / 2u))
            return;
        state->collect_scan_x[sector] = x;
        state->collect_scan_y[sector] = y;
        if (!state->collect_valid[sector] && state->collect_count < 255u)
            ++state->collect_count;
        state->collect_valid[sector] = 1u;
    }
}

void bx72_csm_step(bx72_csm_state_t *state,
                   const bx72_pose_t *pose_now,
                   uint32_t now_us,
                   bx72_lite_fusion_state_t *fusion,
                   uint32_t candidate_budget)
{
    uint32_t processed = 0u;
    if (!state->icp_active || candidate_budget == 0u) return;
    if ((uint32_t)(now_us - state->candidate_pose.stamp_us) > CSM_MAX_SCAN_AGE_US) {
        reject_scan(state, 0);
        return;
    }
    if (candidate_budget > CSM_MAX_POINTS_PER_STEP)
        candidate_budget = CSM_MAX_POINTS_PER_STEP;

    while (processed < candidate_budget && state->icp_active) {
        unsigned int point = state->icp_point_index;
        process_icp_point(state, point);
        ++processed;
        ++state->icp_point_index;
        if (state->icp_point_index >= BX72_CSM_POINTS)
            finish_icp_iteration(state, pose_now, now_us, fusion);
    }
}

void bx72_csm_discontinuity(bx72_csm_state_t *state)
{
    if (state->icp_active) reject_scan(state, 0);
    clear_collect(state);
    state->angle_ready = 0u;
    state->synchronized = 0u;
    // A discontinuous stream establishes a new reference in the existing odom
    // frame. Do not compare a recovered scan against a scene from before a gap.
    state->last_accepted_stamp_us = 0u;
    state->status |= BX72_CSM_STATUS_NEED_SYNC;
    state->status &= ~(BX72_CSM_STATUS_CAPTURE | BX72_CSM_STATUS_REFERENCE);
}

void bx72_csm_diagnostics(const bx72_csm_state_t *state,
                          uint32_t *accepted, uint32_t *rejected,
                          uint32_t *status)
{
    if (accepted != 0) *accepted = state->accepted_count;
    if (rejected != 0) *rejected = state->rejected_count;
    if (status != 0) *status = state->status;
}

static void selftest_seed_fusion(bx72_lite_fusion_state_t *fusion,
                                 float x_mm, float y_mm, float yaw_rad,
                                 uint32_t stamp_us)
{
    bx72_lite_fusion_init(fusion);
    fusion->fusion_ready = 1u;
    fusion->last_fusion_us = stamp_us;
    fusion->last_history_us = stamp_us;
    fusion->history_count = 1u;
    fusion->history_head = 1u;
    fusion->x[0] = x_mm;
    fusion->x[1] = y_mm;
    fusion->x[2] = yaw_rad;
    fusion->x[3] = 0.0f;
    fusion->x[4] = 0.0f;
    fusion->x[5] = 0.0f;
    fusion->history[0].stamp_us = stamp_us;
    fusion->history[0].x_mm = x_mm;
    fusion->history[0].y_mm = y_mm;
    fusion->history[0].yaw_rad = yaw_rad;
    fusion->history[0].vx_mm_s = 0.0f;
    fusion->history[0].vy_mm_s = 0.0f;
    fusion->history[0].wz_rad_s = 0.0f;
}

static void selftest_fill_scene(bx72_csm_state_t *state,
                                float truth_x, float truth_y,
                                float truth_yaw, int degenerate)
{
    unsigned int i;
    float sine, cosine;
    pose_sincos(float_to_i32(truth_yaw * 1000.0f), &sine, &cosine);
    for (i = 0u; i < BX72_CSM_POINTS; ++i) {
        float a = ((float)i + 0.5f) * (BX72_LITE_TWO_PI / (float)BX72_CSM_POINTS);
        float radius = 1400.0f + 250.0f * (float)bx72_sin_q15(float_to_i32(a * 1000.0f)) /
                       32768.0f +
                       100.0f * (float)bx72_cos_q15(float_to_i32(5.0f * a * 1000.0f)) /
                       32768.0f;
        float reference_x = radius * (float)bx72_cos_q15(float_to_i32(a * 1000.0f)) /
                            32768.0f;
        float reference_y = radius * (float)bx72_sin_q15(float_to_i32(a * 1000.0f)) /
                            32768.0f;
        if (degenerate) {
            reference_x = 200.0f + (float)i * 18.0f;
            reference_y = 100.0f;
        }
        state->reference_valid[i] = 1u;
        state->reference_x[i] = reference_x;
        state->reference_y[i] = reference_y;
        {
            float tx = reference_x - truth_x;
            float ty = reference_y - truth_y;
            state->candidate_x[i] = cosine * tx + sine * ty;
            state->candidate_y[i] = -sine * tx + cosine * ty;
            state->candidate_valid[i] = 1u;
        }
    }
}

int bx72_csm_selftest(bx72_csm_state_t *state,
                      bx72_lite_fusion_state_t *fusion)
{
    bx72_pose_t pose_now;
    uint32_t i;
    float truth_x = 40.0f, truth_y = -25.0f, truth_yaw = 0.04f;
    const float predicted_x = 35.0f, predicted_y = -21.0f, predicted_yaw = 0.035f;
    float sine, cosine;

    bx72_csm_init(state);
    angle_sincos(0u, &sine, &cosine);
    if (abs_f(sine) > 0.002f || cosine < 0.999f) return 0;
    bx72_csm_init(state);
    state->status |= BX72_CSM_STATUS_REFERENCE;
    state->reference_x_mm = 0.0f;
    state->reference_y_mm = 0.0f;
    state->reference_yaw_rad = 0.0f;
    state->last_accepted_stamp_us = 90000u;
    selftest_fill_scene(state, truth_x, truth_y, truth_yaw, 0);
    state->candidate_pose.x_mm = (int32_t)predicted_x;
    state->candidate_pose.y_mm = (int32_t)predicted_y;
    state->candidate_pose.yaw_mrad = (int32_t)(predicted_yaw * 1000.0f);
    state->candidate_pose.stamp_us = 100000u;
    state->candidate_pose.flags = BX72_POSE_VALID;
    selftest_seed_fusion(fusion, predicted_x, predicted_y, predicted_yaw, 100000u);
    pose_now = state->candidate_pose;
    state->icp_x_mm = predicted_x;
    state->icp_y_mm = predicted_y;
    state->icp_yaw_rad = predicted_yaw;
    state->icp_active = 1u;
    state->status |= BX72_CSM_STATUS_ICP;
    state->best_rms_mm = 0.0f;
    state->icp_iteration = 0u;
    reset_normal_equations(state);
    for (i = 0u; i < 36u && state->icp_active; ++i)
        bx72_csm_step(state, &pose_now, 100000u, fusion, CSM_MAX_POINTS_PER_STEP);
    if (state->icp_active || state->accepted_count != 1u ||
        (state->status & BX72_CSM_STATUS_ACCEPTED) == 0u ||
        state->best_rms_mm > CSM_FINAL_RMS_MM ||
        fusion->x[0] <= predicted_x || fusion->x[1] >= predicted_y ||
        fusion->x[2] <= predicted_yaw) return 0;

    bx72_csm_init(state);
    state->status |= BX72_CSM_STATUS_REFERENCE;
    state->reference_x_mm = 0.0f;
    state->reference_y_mm = 0.0f;
    state->reference_yaw_rad = 0.0f;
    selftest_fill_scene(state, truth_x, truth_y, truth_yaw, 1);
    state->candidate_pose.x_mm = 50;
    state->candidate_pose.y_mm = 0;
    state->candidate_pose.yaw_mrad = 0;
    state->candidate_pose.stamp_us = 100000u;
    state->candidate_pose.flags = BX72_POSE_VALID;
    state->icp_x_mm = 50.0f;
    state->icp_y_mm = 0.0f;
    state->icp_yaw_rad = 0.0f;
    state->icp_active = 1u;
    reset_normal_equations(state);
    for (i = 0u; i < 36u && state->icp_active; ++i)
        bx72_csm_step(state, &pose_now, 100000u, fusion, CSM_MAX_POINTS_PER_STEP);
    if (state->accepted_count != 0u || state->rejected_count == 0u ||
        (state->status & BX72_CSM_STATUS_DEGENERATE) == 0u) return 0;
    return 1;
}
