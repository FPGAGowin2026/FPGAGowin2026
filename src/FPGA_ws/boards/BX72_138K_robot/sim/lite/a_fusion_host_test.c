/*
 * 主机侧融合回归用例，交付后从boards/BX72_138K_robot目录执行：
 * cc -std=c11 -DBX72_HOST_TEST_NO_RV32_ASM sim/lite/a_fusion_host_test.c \
 *   src/firmware/lite/fusion.c src/firmware/lite/trig.c \
 *   -lm -o /tmp/a_fusion_host_test
 * 本线程不运行；生产固件仍使用真实fsqrt.s。
 */
#include "../../src/firmware/lite/fusion.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static void reset_case(bx72_lite_fusion_state_t *state, bx72_pose_t *pose)
{
    bx72_lite_fusion_init(state);
    pose->x_mm = 0;
    pose->y_mm = 0;
    pose->yaw_mrad = 0;
    pose->roll_mrad = 0;
    pose->pitch_mrad = 0;
    pose->vx_mm_s = 0;
    pose->vy_mm_s = 0;
    pose->wz_mrad_s = 0;
    pose->stamp_us = 0;
    pose->flags = 0;
}

static bx72_lite_sample_t valid_sample(void)
{
    bx72_lite_sample_t sample = {0};
    sample.status = BX72_STATUS_IMU_OK | BX72_STATUS_WHEEL_OK;
    sample.imu_seq = 1;
    sample.wheel_seq = 1;
    sample.now_us = 10000u;
    sample.imu_stamp_us = 10000u;
    sample.wheel_stamp_us = 10000u;
    sample.az = BX72_IMU_MOUNT_FLIP_X_180 ? -8196 : 8196;
    return sample;
}

static void refresh_sequences(bx72_lite_sample_t *sample, unsigned int step)
{
    if ((step % 80u) == 0) {
        ++sample->imu_seq;    /* 约8 ms一次IMU样本。 */
        sample->imu_stamp_us = sample->now_us;
    }
    if ((step % 10u) == 0) {
        ++sample->wheel_seq;  /* 约1 ms一次轮速样本。 */
        sample->wheel_stamp_us = sample->now_us;
    }
}

static void test_low_speed_position(void)
{
    bx72_lite_fusion_state_t state;
    bx72_pose_t pose = {0};
    bx72_lite_sample_t sample = valid_sample();
    unsigned int step;

    reset_case(&state, &pose);
    sample.vx = 100;
    bx72_lite_fusion_update(&state, &pose, &sample);
    for (step = 1; step <= 10000u; ++step) {
        sample.now_us += 100u;
        refresh_sequences(&sample, step);
        bx72_lite_fusion_update(&state, &pose, &sample);
    }
    assert(pose.flags & BX72_POSE_VALID);
    assert(pose.x_mm >= 99 && pose.x_mm <= 101);
    assert(pose.y_mm >= -1 && pose.y_mm <= 1);
}

static void test_low_angular_rate(void)
{
    bx72_lite_fusion_state_t state;
    bx72_pose_t pose = {0};
    bx72_lite_sample_t sample = valid_sample();
    unsigned int step;

    reset_case(&state, &pose);
    sample.vx = 20; /* 避免静止bias估计把测试角速度吸收掉。 */
    sample.gz = BX72_IMU_MOUNT_FLIP_X_180 ? -100 : 100;
    sample.wz = 30; /* 与约30.5mrad/s陀螺读数一致，避免把差异估计成bias。 */
    bx72_lite_fusion_update(&state, &pose, &sample);
    for (step = 1; step <= 10000u; ++step) {
        sample.now_us += 100u;
        refresh_sequences(&sample, step);
        bx72_lite_fusion_update(&state, &pose, &sample);
    }
    assert(pose.flags & BX72_POSE_VALID);
    assert(pose.yaw_mrad >= 20 && pose.yaw_mrad <= 35);
}

static void test_known_tilt(void)
{
    bx72_lite_fusion_state_t state;
    bx72_pose_t pose = {0};
    bx72_lite_sample_t sample = valid_sample();
    unsigned int step;

    reset_case(&state, &pose);
    /* roll=pitch=30度时的约重力向量，单位为原始LSB。 */
    sample.ax = -4098;
    sample.ay = BX72_IMU_MOUNT_FLIP_X_180 ? -3549 : 3549;
    sample.az = BX72_IMU_MOUNT_FLIP_X_180 ? -6147 : 6147;
    bx72_lite_fusion_update(&state, &pose, &sample);
    for (step = 1; step <= 1000u; ++step) {
        sample.now_us += 8333u;
        ++sample.imu_seq;
        ++sample.wheel_seq;
        sample.imu_stamp_us = sample.now_us;
        sample.wheel_stamp_us = sample.now_us;
        bx72_lite_fusion_update(&state, &pose, &sample);
    }
    assert(pose.flags & BX72_POSE_VALID);
    assert(pose.roll_mrad >= 500 && pose.roll_mrad <= 545);
    assert(pose.pitch_mrad >= 500 && pose.pitch_mrad <= 545);
}

static void test_mount_equivalence_and_inverted_rest(void)
{
    bx72_lite_sample_t upright = {0};
    bx72_lite_sample_t inverted = {0};
    bx72_imu_axes_t body_upright, body_inverted, body_configured;
    bx72_lite_fusion_state_t state;
    bx72_pose_t pose = {0};
    bx72_lite_sample_t sample = valid_sample();
    unsigned int step;

    upright.gx = 100;
    upright.gy = -200;
    upright.gz = 300;
    upright.ax = -400;
    upright.ay = 500;
    upright.az = 600;
    inverted = upright;
    inverted.gy = -upright.gy;
    inverted.gz = -upright.gz;
    inverted.ay = -upright.ay;
    inverted.az = -upright.az;
    bx72_lite_imu_axes_transform(&upright, 0u, &body_upright);
    bx72_lite_imu_axes_transform(&inverted, 1u, &body_inverted);
    bx72_lite_imu_axes_transform(
        BX72_IMU_MOUNT_FLIP_X_180 ? &inverted : &upright,
        BX72_IMU_MOUNT_FLIP_X_180, &body_configured);
    assert(body_upright.gx == body_inverted.gx &&
           body_upright.gy == body_inverted.gy && body_upright.gz == body_inverted.gz &&
           body_upright.ax == body_inverted.ax && body_upright.ay == body_inverted.ay &&
           body_upright.az == body_inverted.az);
    assert(body_configured.gx == body_upright.gx &&
           body_configured.gy == body_upright.gy && body_configured.gz == body_upright.gz &&
           body_configured.ax == body_upright.ax && body_configured.ay == body_upright.ay &&
           body_configured.az == body_upright.az);

    reset_case(&state, &pose);
    sample = valid_sample();
    sample.az = BX72_IMU_MOUNT_FLIP_X_180 ? -8196 : 8196;
    bx72_lite_fusion_update(&state, &pose, &sample);
    for (step = 0; step < 200u; ++step) {
        sample.now_us += 8333u;
        sample.imu_stamp_us = sample.now_us;
        sample.wheel_stamp_us = sample.now_us;
        ++sample.imu_seq;
        ++sample.wheel_seq;
        bx72_lite_fusion_update(&state, &pose, &sample);
    }
    assert(pose.flags & BX72_POSE_VALID);
    assert(pose.roll_mrad >= -2 && pose.roll_mrad <= 2);
    assert(pose.pitch_mrad >= -2 && pose.pitch_mrad <= 2);
}

static void test_invalid_and_recovery(void)
{
    bx72_lite_fusion_state_t state;
    bx72_pose_t pose = {0};
    bx72_lite_sample_t sample = valid_sample();
    int32_t old_x;

    reset_case(&state, &pose);
    sample.vx = 100;
    bx72_lite_fusion_update(&state, &pose, &sample);
    sample.now_us = 20000u;
    sample.imu_seq = 2;
    sample.imu_stamp_us = sample.now_us;
    sample.status = BX72_STATUS_IMU_OK;
    bx72_lite_fusion_update(&state, &pose, &sample);
    assert((pose.flags & BX72_POSE_VALID) == 0);
    old_x = pose.x_mm;

    sample.now_us = 21000u;
    sample.imu_seq = 3;
    sample.wheel_seq = 2;
    sample.imu_stamp_us = sample.now_us;
    sample.wheel_stamp_us = sample.now_us;
    sample.status = BX72_STATUS_IMU_OK | BX72_STATUS_WHEEL_OK;
    bx72_lite_fusion_update(&state, &pose, &sample);
    assert(pose.flags & BX72_POSE_VALID);
    assert(pose.x_mm == old_x); /* 失效窗口不积分，恢复时只重新建时间基线。 */
}

static void test_sensor_freshness_and_order(void)
{
    bx72_lite_fusion_state_t state;
    bx72_pose_t pose = {0};
    bx72_lite_sample_t sample;
    float gyro_before;
    unsigned int sensor;

    /* 两路传感器各自允许 500 ms，超过后失效，新数据到达再恢复。 */
    for (sensor = 0; sensor < 2; ++sensor) {
        reset_case(&state, &pose);
        sample = valid_sample();
        sample.now_us = 510000u;
        sample.imu_stamp_us = sample.wheel_stamp_us = 10000u;
        bx72_lite_fusion_update(&state, &pose, &sample);
        assert(pose.flags & BX72_POSE_VALID);

        ++sample.now_us;
        ++sample.imu_seq;
        ++sample.wheel_seq;
        if (sensor == 0) sample.wheel_stamp_us = sample.now_us;
        else sample.imu_stamp_us = sample.now_us;
        bx72_lite_fusion_update(&state, &pose, &sample);
        assert((pose.flags & BX72_POSE_VALID) == 0);

        ++sample.now_us;
        ++sample.imu_seq;
        ++sample.wheel_seq;
        sample.imu_stamp_us = sample.wheel_stamp_us = sample.now_us;
        bx72_lite_fusion_update(&state, &pose, &sample);
        assert(pose.flags & BX72_POSE_VALID);

        ++sample.now_us;
        ++sample.imu_seq;
        ++sample.wheel_seq;
        sample.imu_stamp_us = sample.wheel_stamp_us = sample.now_us;
        if (sensor == 0) sample.imu_stamp_us = sample.now_us + 1u;
        else sample.wheel_stamp_us = sample.now_us + 1u;
        bx72_lite_fusion_update(&state, &pose, &sample);
        assert((pose.flags & BX72_POSE_VALID) == 0);
    }

    /* 序号更新不能让相同或回退的时间戳覆盖已有传感器读数。 */
    reset_case(&state, &pose);
    sample = valid_sample();
    sample.now_us = 1010000u;
    sample.imu_stamp_us = sample.wheel_stamp_us = 1000000u;
    sample.vx = 120;
    sample.gz = 100;
    bx72_lite_fusion_update(&state, &pose, &sample);
    gyro_before = state.gyro_rate_rad_s;
    for (sensor = 0; sensor < 2; ++sensor) {
        sample.now_us += 1000u;
        ++sample.imu_seq;
        ++sample.wheel_seq;
        sample.imu_stamp_us = sample.wheel_stamp_us = 1000000u - sensor;
        sample.vx = 160;
        sample.gz = 400;
        bx72_lite_fusion_update(&state, &pose, &sample);
        assert(pose.flags & BX72_POSE_VALID);
        assert(state.last_imu_stamp_us == 1000000u);
        assert(state.last_wheel_stamp_us == 1000000u);
        assert(state.x[3] == 120.0f && state.gyro_rate_rad_s == gyro_before);
    }
    sample.now_us += 1000u;
    ++sample.imu_seq;
    ++sample.wheel_seq;
    sample.imu_stamp_us = sample.wheel_stamp_us = 1000001u;
    bx72_lite_fusion_update(&state, &pose, &sample);
    assert(state.last_imu_stamp_us == 1000001u);
    assert(state.last_wheel_stamp_us == 1000001u);
    assert(state.x[3] == 160.0f && state.gyro_rate_rad_s != gyro_before);

    /* 32 位微秒计数回绕后，时间戳仍按模运算判断递增。 */
    reset_case(&state, &pose);
    sample = valid_sample();
    sample.now_us = UINT32_MAX - 100u;
    sample.imu_stamp_us = sample.wheel_stamp_us = UINT32_MAX - 600u;
    sample.vx = 120;
    bx72_lite_fusion_update(&state, &pose, &sample);
    sample.now_us = 1000u;
    sample.imu_stamp_us = sample.wheel_stamp_us = 900u;
    sample.vx = 160;
    ++sample.imu_seq;
    ++sample.wheel_seq;
    bx72_lite_fusion_update(&state, &pose, &sample);
    assert(pose.flags & BX72_POSE_VALID);
    assert(state.last_imu_stamp_us == 900u && state.last_wheel_stamp_us == 900u);
    assert(state.x[3] == 160.0f);
}

int main(void)
{
    test_low_speed_position();
    test_low_angular_rate();
    test_known_tilt();
    test_invalid_and_recovery();
    test_sensor_freshness_and_order();
    test_mount_equivalence_and_inverted_rest();
    puts("lite fusion host checks passed");
    return 0;
}
