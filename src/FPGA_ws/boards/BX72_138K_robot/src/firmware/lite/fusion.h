#ifndef BX72_LITE_FUSION_H
#define BX72_LITE_FUSION_H

#include "bx72_lite.h"

#define BX72_FUSION_HISTORY 32u

/* The sensor package is mounted upside down about body X. */
#ifndef BX72_IMU_MOUNT_FLIP_X_180
#define BX72_IMU_MOUNT_FLIP_X_180 1u
#endif

typedef struct {
    uint32_t now_us;
    uint32_t status;
    uint32_t imu_seq;
    uint32_t imu_stamp_us;
    int32_t gx, gy, gz;
    int32_t ax, ay, az;
    uint32_t wheel_seq;
    uint32_t wheel_stamp_us;
    int32_t vx, vy, wz;
    uint32_t command_seq;
    int32_t command_vx, command_vy, command_wz;
    uint32_t mode_seq;
} bx72_lite_sample_t;

typedef struct {
    int32_t gx, gy, gz;
    int32_t ax, ay, az;
} bx72_imu_axes_t;

typedef struct {
    uint32_t stamp_us;
    float x_mm, y_mm, yaw_rad;
    float vx_mm_s, vy_mm_s, wz_rad_s;
} bx72_fusion_history_t;

/* x[0:2] 为位置毫米及航向弧度；其余为机体速度和陀螺零偏。 */
typedef struct {
    float x[7];
    uint32_t last_imu_seq;
    uint32_t last_wheel_seq;
    uint32_t last_imu_stamp_us;
    uint32_t last_wheel_stamp_us;
    uint32_t last_fusion_us;
    uint32_t last_history_us;
    uint32_t stationary_since_us;
    uint16_t history_head;
    uint16_t history_count;
    uint16_t gyro_bias_samples;
    uint8_t fusion_ready;
    uint8_t roll_pitch_ready;
    uint8_t gyro_bias_ready;
    uint8_t stationary_timing;
    float gyro_rate_rad_s;
    float wheel_wz_rad_s;
    float roll_rad;
    float pitch_rad;
    float pose_cov[3][3];
    uint32_t last_csm_stamp_us;
    uint8_t have_csm_stamp;
    bx72_fusion_history_t history[BX72_FUSION_HISTORY];
} bx72_lite_fusion_state_t;

void bx72_lite_fusion_init(bx72_lite_fusion_state_t *state);
int bx72_lite_fusion_selftest(bx72_lite_fusion_state_t *state);
void bx72_lite_imu_axes_transform(const bx72_lite_sample_t *raw,
                                  uint8_t flip_x_180,
                                  bx72_imu_axes_t *body);
void bx72_lite_fusion_update(bx72_lite_fusion_state_t *state,
                             bx72_pose_t *pose,
                             const bx72_lite_sample_t *sample);
int bx72_lite_fusion_pose_at(const bx72_lite_fusion_state_t *state,
                             uint32_t stamp_us,
                             bx72_pose_t *pose_out);
/* 延迟扫描匹配校正；协方差是保守的相关观测近似。 */
int bx72_lite_fusion_csm_update(bx72_lite_fusion_state_t *state,
                                uint32_t source_stamp_us,
                                uint32_t now_us,
                                float x_mm, float y_mm, float yaw_rad,
                                float position_variance_mm2,
                                float yaw_variance_rad2);
/* diag[0:2] are float estimates in mm, mm, and mrad; [3:6] are zero. */
void bx72_lite_fusion_diag(const bx72_lite_fusion_state_t *state,
                           uint32_t diag_words[7],
                           int32_t *gyro_bias_mrad_s);

#endif
