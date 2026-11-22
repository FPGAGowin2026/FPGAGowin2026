#ifndef ROBOT_LOCAL_PLAN_H
#define ROBOT_LOCAL_PLAN_H

#include "local_map.h"

#include <stdint.h>

#define LOCAL_PLAN_SIDE 32
#define LOCAL_PLAN_NODES (LOCAL_PLAN_SIDE * LOCAL_PLAN_SIDE)
#define LOCAL_PLAN_MAX_POINTS 32

enum local_plan_status {
    LOCAL_PLAN_IDLE,
    LOCAL_PLAN_RUNNING,
    LOCAL_PLAN_COMPLETE,
    LOCAL_PLAN_PARTIAL,
    LOCAL_PLAN_BLOCKED
};

typedef struct {
    int32_t x_mm;
    int32_t y_mm;
} local_plan_point_t;

typedef struct {
    int32_t x_mm, y_mm;
    int32_t cos_q15, sin_q15;
    int32_t body_length_mm, body_width_mm;
    int32_t margin_mm;
    uint32_t required_fresh_us;
} local_plan_pose_t;

typedef struct {
    uint16_t score[LOCAL_PLAN_NODES];
    uint16_t heap[LOCAL_PLAN_NODES];
    uint16_t heap_position[LOCAL_PLAN_NODES];
    uint16_t reverse_path[LOCAL_PLAN_NODES];
    uint8_t parent[LOCAL_PLAN_NODES];
    uint8_t node_state[LOCAL_PLAN_NODES];
    uint8_t walkability[LOCAL_PLAN_NODES];
    uint8_t map_snapshot[LOCAL_MAP_CELLS / 4];
    local_plan_pose_t pose;
    int32_t snapshot_x, snapshot_y;
    int32_t window_x, window_y;
    int32_t goal_x_mm, goal_y_mm;
    uint32_t map_generation;
    uint16_t start_node, goal_node, best_node, heap_size;
    uint16_t best_heuristic;
    uint32_t expansions;
    uint8_t goal_inside;
    enum local_plan_status status;
} local_plan_t;

/*
 * Plan using a bounded 32 x 32 window and a snapshot of the 64 x 64 local
 * map. Unknown or insufficiently fresh cells are not traversable. Heading
 * remains fixed during the planned translation; rotation requires a separate
 * swept-footprint check. The caller supplies unit-vector sine/cosine in Q15.
 * LOCAL_PLAN_COMPLETE means a path was found, not that the robot has arrived.
 */
int local_plan_begin(local_plan_t *plan, local_map_t *map,
                     const local_plan_pose_t *pose, int32_t goal_x_mm,
                     int32_t goal_y_mm, uint32_t now_us);

/* Limits work per call. The caller must continue servicing sensors/control. */
enum local_plan_status local_plan_step(local_plan_t *plan, uint16_t max_expansions);

/*
 * Outputs points from the start of the path and reports whether the returned
 * prefix ends at the exact goal. A partial result is an interim reference, not
 * proof that the final goal is reachable. A path uses a snapshot:
 * check the current map, observation ages and stopping sweep again before motion.
 */
uint16_t local_plan_result(local_plan_t *plan, local_plan_point_t *points,
                           uint16_t capacity, int *ends_at_goal);

#endif
