#ifndef ROBOT_ODOMETRY_H
#define ROBOT_ODOMETRY_H

#include <stdint.h>

#define ODOM_STATE_SIZE 7
#define ODOM_HISTORY_SIZE 128

/*
 * State: x, y, yaw, vx, vy, yaw_rate, gyro_bias.
 * Distances and speeds are metres and m/s; angular quantities are radians.
 * Motion yaw follows a fresh gyro; 200 ms of fresh quiet wheel/gyro input
 * permits zero-rate bias calibration. This dwell rejects short transients,
 * not sustained unobservable slow rotation.
 * First/reacquired gyro input establishes rate without requiring a quiet start;
 * its covariance includes the frozen bias uncertainty and cross-correlations.
 * Wheel velocity remains a wheel-ground-contact assumption: this filter has no
 * independent translation observation for detecting slip or wheel-free carrying.
 * Map localization handles those corrections without resetting this odom frame.
 * Measurement and initial variances use squared SI units. Process-noise
 * entries are variance per second and are multiplied by the elapsed time.
 */
typedef struct {
    float process_position;
    float process_yaw;
    float process_vx;
    float process_vy;
    float process_yaw_rate;
    float process_gyro_bias;
    float wheel_vx_variance;
    float wheel_vy_variance;
    float wheel_yaw_rate_variance;
    float gyro_variance;
    float initial_position_variance;
    float initial_yaw_variance;
    float initial_velocity_variance;
    float initial_yaw_rate_variance;
    float initial_bias_variance;
    float initial_gyro_bias_rad_s;
    int32_t geometry_mm;
    uint32_t max_prediction_gap_us;
    uint32_t wheel_max_age_us;
    uint32_t imu_max_age_us;
} odometry_config_t;

typedef struct {
    uint32_t stamp_us;
    float x_m, y_m, yaw_rad;
} odometry_history_entry_t;

typedef struct {
    float state[ODOM_STATE_SIZE];
    float covariance[ODOM_STATE_SIZE][ODOM_STATE_SIZE];
    odometry_history_entry_t history[ODOM_HISTORY_SIZE];
    odometry_config_t config;
    uint32_t last_update_us, last_wheel_us, last_imu_us;
    uint16_t history_next, history_count;
    uint8_t healthy, have_wheels, have_imu;
    uint8_t bias_stationary; /* bits 0/1: quiet wheels/gyro; 2/3: dwell started/ready */
    uint32_t bias_quiet_since_us;
} odometry_t;

/*
 * Time is one monotonic 32-bit microsecond clock. Deliver wheel and IMU samples
 * in timestamp order. Gyro input is calibrated body-frame z in rad/s.
 * After a prediction gap or stale sensor invalidates the filter, reinitialise
 * it and coordinate the odom-frame reset with the map and Jetson.
 * Wheel input is FL, FR, RL, RR in mm/s with positive forward wheel rotation.
 */
int odometry_init(odometry_t *odom, const odometry_config_t *config, uint32_t now_us);
int odometry_update_wheels(odometry_t *odom, const int32_t wheel_mm_s[4], uint32_t stamp_us);
int odometry_update_gyro(odometry_t *odom, float gyro_z_rad_s, uint32_t stamp_us);
int odometry_valid(const odometry_t *odom, uint32_t now_us);

/* Interpolates previously processed poses; it does not extrapolate. */
int odometry_pose_at(const odometry_t *odom, uint32_t stamp_us,
                     float *x_m, float *y_m, float *yaw_rad);

/* The same bounded approximation is used for pose and lidar transforms. */
void odometry_sincos(float angle_rad, float *sine, float *cosine);

#endif
