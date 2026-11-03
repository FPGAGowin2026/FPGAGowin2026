/*
 * Build command for the parent verifier (documented only; do not run here):
 * gcc -std=c99 -O2 -DBX72_HOST_TEST_NO_RV32_ASM -Isrc/firmware/lite \
 *     sim/lite/polar_pose_test.c src/firmware/lite/fusion.c \
 *     src/firmware/lite/csm.c src/firmware/lite/trig.c -lm \
 *     -o build/polar_pose_test
 */
#include "csm.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_POINTS 72u

static void sample_at(bx72_lite_sample_t *sample,
                      uint32_t now_us, uint32_t sequence,
                      int32_t vx, int32_t vy, int32_t wz)
{
    memset(sample, 0, sizeof(*sample));
    sample->now_us = now_us;
    sample->status = BX72_STATUS_IMU_OK | BX72_STATUS_WHEEL_OK;
    sample->imu_seq = sequence;
    sample->imu_stamp_us = now_us;
    sample->az = BX72_IMU_MOUNT_FLIP_X_180 ? -8192 : 8192;
    sample->wheel_seq = sequence;
    sample->wheel_stamp_us = now_us;
    sample->vx = vx;
    sample->vy = vy;
    sample->wz = wz;
}

static int test_sensor_timeout(void)
{
    bx72_lite_fusion_state_t fusion;
    bx72_lite_sample_t sample;
    bx72_pose_t pose;
    uint32_t now;
    memset(&pose, 0, sizeof(pose));
    bx72_lite_fusion_init(&fusion);
    sample_at(&sample, 10000u, 1u, 0, 0, 0);
    bx72_lite_fusion_update(&fusion, &pose, &sample);
    for (now = 110000u; now <= 510000u; now += 100000u) {
        sample.now_us = now;
        bx72_lite_fusion_update(&fusion, &pose, &sample);
        if (!(pose.flags & BX72_POSE_VALID)) return 0;
    }
    sample.now_us = 510001u;
    bx72_lite_fusion_update(&fusion, &pose, &sample);
    return !(pose.flags & BX72_POSE_VALID);
}

static int test_imu_mount_and_attitude(void)
{
    bx72_lite_sample_t raw;
    bx72_imu_axes_t body;
    bx72_lite_fusion_state_t fusion;
    bx72_pose_t pose;
    bx72_lite_sample_t sample;
    memset(&raw, 0, sizeof(raw));
    raw.gx = 123;
    raw.gy = -234;
    raw.gz = 345;
    raw.ax = -456;
    raw.ay = 567;
    raw.az = -678;
    bx72_lite_imu_axes_transform(&raw, 1u, &body);
    if (body.gx != 123 || body.gy != 234 || body.gz != -345 ||
        body.ax != -456 || body.ay != -567 || body.az != 678) return 0;

    bx72_lite_fusion_init(&fusion);
    memset(&pose, 0, sizeof(pose));
    sample_at(&sample, 10000u, 1u, 0, 0, 0);
    sample.ax = -4096;
    sample.ay = BX72_IMU_MOUNT_FLIP_X_180 ? -3547 : 3547;
    sample.az = BX72_IMU_MOUNT_FLIP_X_180 ? -6144 : 6144;
    bx72_lite_fusion_update(&fusion, &pose, &sample);
    return pose.flags != 0u && pose.roll_mrad > 400 && pose.roll_mrad < 650 &&
           pose.pitch_mrad > 400 && pose.pitch_mrad < 650;
}

static int test_wheel_lateral_odometry_and_pose_age(void)
{
    bx72_lite_fusion_state_t fusion;
    bx72_lite_sample_t sample;
    bx72_pose_t pose, interpolated;
    memset(&pose, 0, sizeof(pose));
    bx72_lite_fusion_init(&fusion);
    sample_at(&sample, 10000u, 1u, 0, 120, 0);
    bx72_lite_fusion_update(&fusion, &pose, &sample);
    sample_at(&sample, 20000u, 2u, 0, 120, 0);
    bx72_lite_fusion_update(&fusion, &pose, &sample);
    if (pose.flags == 0u || pose.x_mm < -1 || pose.x_mm > 1 ||
        pose.y_mm < 0 || pose.y_mm > 3) return 0;
    if (!bx72_lite_fusion_pose_at(&fusion, 35000u, &interpolated)) return 0;
    if (interpolated.y_mm < 2 || interpolated.y_mm > 5) return 0;
    if (bx72_lite_fusion_pose_at(&fusion, 41000u, &interpolated)) return 0;
    return 1;
}

static int test_delayed_correction_between_history_samples(void)
{
    bx72_lite_fusion_state_t fusion;
    bx72_lite_sample_t sample;
    bx72_pose_t pose, source_before, source_after, now_before, now_after;
    unsigned int i;
    memset(&pose, 0, sizeof(pose));
    bx72_lite_fusion_init(&fusion);
    for (i = 1; i <= 3; ++i) {
        sample_at(&sample, i * 10000u, i, 100, 0, 0);
        bx72_lite_fusion_update(&fusion, &pose, &sample);
    }
    if (!bx72_lite_fusion_pose_at(&fusion, 25000u, &source_before) ||
        !bx72_lite_fusion_pose_at(&fusion, 30000u, &now_before)) return 0;
    if (!bx72_lite_fusion_csm_update(&fusion, 25000u, 30000u,
        (float)source_before.x_mm + 100.0f, (float)source_before.y_mm,
        (float)source_before.yaw_mrad * 0.001f, 0.0f, 0.0f)) return 0;
    if (!bx72_lite_fusion_pose_at(&fusion, 25000u, &source_after) ||
        !bx72_lite_fusion_pose_at(&fusion, 30000u, &now_after)) return 0;
    return abs(source_after.x_mm - source_before.x_mm -
               (now_after.x_mm - now_before.x_mm)) <= 1 &&
           now_after.x_mm - now_before.x_mm >= 18;
}

static void prime_fusion_pose(bx72_lite_fusion_state_t *fusion,
                              bx72_pose_t *pose,
                              float x_mm, float y_mm, float yaw_rad)
{
    bx72_lite_sample_t sample;
    memset(pose, 0, sizeof(*pose));
    pose->x_mm = (int32_t)x_mm;
    pose->y_mm = (int32_t)y_mm;
    pose->yaw_mrad = (int32_t)(yaw_rad * 1000.0f);
    bx72_lite_fusion_init(fusion);
    sample_at(&sample, 100000u, 1u, 0, 0, 0);
    bx72_lite_fusion_update(fusion, pose, &sample);
}

static void make_scan_fixture(bx72_csm_state_t *csm,
                              float truth_x, float truth_y,
                              float truth_yaw, int collinear)
{
    unsigned int i;
    float sine = sinf(truth_yaw);
    float cosine = cosf(truth_yaw);
    for (i = 0u; i < TEST_POINTS; ++i) {
        float angle = ((float)i + 0.5f) * (2.0f * TEST_PI / (float)TEST_POINTS);
        float radius = 1400.0f + 250.0f * sinf(3.0f * angle) +
                       100.0f * cosf(5.0f * angle);
        float rx = radius * cosf(angle);
        float ry = radius * sinf(angle);
        if (collinear) {
            rx = 200.0f + (float)i * 18.0f;
            ry = 100.0f;
        }
        csm->reference_valid[i] = 1u;
        csm->reference_x[i] = rx;
        csm->reference_y[i] = ry;
        csm->candidate_valid[i] = 1u;
        csm->candidate_x[i] = cosine * (rx - truth_x) +
                              sine * (ry - truth_y);
        csm->candidate_y[i] = -sine * (rx - truth_x) +
                               cosine * (ry - truth_y);
    }
}

static int test_point_line_icp_and_degeneracy(void)
{
    bx72_csm_state_t csm;
    bx72_lite_fusion_state_t fusion;
    bx72_pose_t pose_now;
    unsigned int slice;
    const float predicted_x = 35.0f;
    const float predicted_y = -21.0f;
    const float predicted_yaw = 0.035f;

    bx72_csm_init(&csm);
    prime_fusion_pose(&fusion, &pose_now, predicted_x, predicted_y, predicted_yaw);
    csm.status |= BX72_CSM_STATUS_REFERENCE;
    csm.reference_x_mm = 0.0f;
    csm.reference_y_mm = 0.0f;
    csm.reference_yaw_rad = 0.0f;
    csm.last_accepted_stamp_us = 90000u;
    make_scan_fixture(&csm, 40.0f, -25.0f, 0.04f, 0);
    csm.candidate_pose = pose_now;
    csm.candidate_pose.stamp_us = 100000u;
    csm.icp_x_mm = predicted_x;
    csm.icp_y_mm = predicted_y;
    csm.icp_yaw_rad = predicted_yaw;
    csm.icp_active = 1u;
    csm.status |= BX72_CSM_STATUS_ICP;
    for (slice = 0u; slice < 36u && csm.icp_active; ++slice)
        bx72_csm_step(&csm, &pose_now, 100000u, &fusion, 12u);
    if (csm.icp_active || csm.accepted_count != 1u ||
        (csm.status & BX72_CSM_STATUS_ACCEPTED) == 0u ||
        csm.best_rms_mm > 65.0f || fusion.x[0] <= predicted_x ||
        fusion.x[1] >= predicted_y || fusion.x[2] <= predicted_yaw)
        return 0;

    bx72_csm_init(&csm);
    prime_fusion_pose(&fusion, &pose_now, 50.0f, 0.0f, 0.0f);
    csm.status |= BX72_CSM_STATUS_REFERENCE;
    csm.reference_x_mm = 0.0f;
    csm.reference_y_mm = 0.0f;
    csm.reference_yaw_rad = 0.0f;
    csm.last_accepted_stamp_us = 90000u;
    make_scan_fixture(&csm, 40.0f, -25.0f, 0.04f, 1);
    csm.candidate_pose = pose_now;
    csm.candidate_pose.stamp_us = 100000u;
    csm.icp_x_mm = 50.0f;
    csm.icp_y_mm = 0.0f;
    csm.icp_yaw_rad = 0.0f;
    csm.icp_active = 1u;
    csm.status |= BX72_CSM_STATUS_ICP;
    for (slice = 0u; slice < 12u && csm.icp_active; ++slice)
        bx72_csm_step(&csm, &pose_now, 100000u, &fusion, 12u);
    return csm.accepted_count == 0u && csm.rejected_count != 0u &&
           (csm.status & BX72_CSM_STATUS_DEGENERATE) != 0u;
}

static int test_72_bin_capture(void)
{
    bx72_csm_state_t csm;
    bx72_pose_t pose;
    unsigned int i;
    memset(&pose, 0, sizeof(pose));
    pose.flags = BX72_POSE_VALID;
    pose.stamp_us = 100000u;
    bx72_csm_init(&csm);
    csm.synchronized = 1u;
    csm.angle_ready = 1u;
    csm.previous_angle_cdeg = 35500u;
    for (i = 0u; i < TEST_POINTS; ++i) {
        pose.stamp_us += 1000u;
        bx72_csm_point(&csm, 250u + i * 500u, 0u, 0u, &pose);
    }
    return csm.collect_seen_count == TEST_POINTS && csm.collect_count == 0u &&
           csm.collect_pose_valid;
}

int main(void)
{
    if (!test_imu_mount_and_attitude()) {
        puts("FAIL imu_mount_and_attitude");
        return 1;
    }
    if (!test_wheel_lateral_odometry_and_pose_age()) {
        puts("FAIL wheel_lateral_odometry_and_pose_age");
        return 1;
    }
    if (!test_point_line_icp_and_degeneracy()) {
        puts("FAIL point_line_icp_and_degeneracy");
        return 1;
    }
    if (!test_72_bin_capture()) {
        puts("FAIL 72_bin_capture");
        return 1;
    }
    if (!test_delayed_correction_between_history_samples()) {
        puts("FAIL delayed correction interpolation continuity");
        return 1;
    }
    if (!test_sensor_timeout()) {
        puts("FAIL sensor timeout");
        return 1;
    }
    puts("PASS fusion and CSM pose host checks");
    return 0;
}
