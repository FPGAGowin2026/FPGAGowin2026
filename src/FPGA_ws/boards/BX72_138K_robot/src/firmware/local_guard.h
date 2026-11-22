#ifndef ROBOT_LOCAL_GUARD_H
#define ROBOT_LOCAL_GUARD_H

#include "local_plan.h"

#include <stdint.h>

typedef struct {
    int32_t vx_mm_s;
    int32_t vy_mm_s;
    int32_t wz_mrad_s;
} local_velocity_t;

typedef struct {
    int32_t geometry_mm;
    int32_t max_wheel_mm_s;
    int32_t min_lateral_mm_s;
    int32_t brake_x_mm_s2;
    int32_t brake_y_mm_s2;
    int32_t brake_yaw_mrad_s2;
    uint32_t reaction_us;
    uint32_t observation_uncertainty_us;
} local_guard_config_t;

enum local_guard_result {
    LOCAL_GUARD_ALLOWED,
    LOCAL_GUARD_STOP,
    LOCAL_GUARD_INVALID,
    LOCAL_GUARD_NO_FRONT_SCAN,
    LOCAL_GUARD_FRONT_BLOCKED,
    LOCAL_GUARD_LATERAL_TOO_SLOW,
    LOCAL_GUARD_UNOBSERVED,
    LOCAL_GUARD_EXPIRED,
    LOCAL_GUARD_OCCUPIED
};

/*
 * Tests the current map for a conservative motion-and-braking envelope.
 * Braking parameters must be conservative for combined motions and come from
 * actual vehicle measurements; reaction_us includes the worst control latency.
 * This gate
 * does not replace the continuously operating front-lidar hardware guard.
 * Call before each autonomous command and again as motion/observations change.
 * A zero requested velocity always produces STOP, even when inputs are invalid.
 */
enum local_guard_result local_guard_evaluate(
    local_map_t *map, const local_plan_pose_t *pose, const local_guard_config_t *config,
    const local_velocity_t *measured, const local_velocity_t *requested,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us,
    local_velocity_t *allowed);

/* Forward exploration at the higher speed requires observed braking space.
 * Slower front probing and observation turns may enter unknown cells. */
enum local_guard_result local_guard_evaluate_exploration(
    local_map_t *map, const local_plan_pose_t *pose, const local_guard_config_t *config,
    const local_velocity_t *measured, const local_velocity_t *requested,
    int front_scan_fresh, int front_guard_clear, uint32_t now_us,
    local_velocity_t *allowed);

#endif
