#include "local_plan.h"
#include "footprint.h"

#define NODE_OPEN 1u
#define NODE_CLOSED 2u
#define WALKABLE 1u
#define BLOCKED 2u
#define NO_SCORE 65535u
#define HALF_CELL_MM (LOCAL_MAP_CELL_MM / 2)
#define Q15_ONE 32768
/* Covers translation between adjacent node centres, including diagonals. */
#define STEP_MARGIN_MM 36

static const int8_t step_x[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int8_t step_y[8] = {0, 1, 1, 1, 0, -1, -1, -1};

static int32_t floor_cell(int32_t mm)
{
    int32_t value = mm / LOCAL_MAP_CELL_MM;
    if (mm % LOCAL_MAP_CELL_MM < 0) --value;
    return value;
}

static int64_t abs64(int64_t value)
{
    return value < 0 ? -value : value;
}

static uint8_t snapshot_get(const local_plan_t *plan, int32_t x, int32_t y)
{
    uint32_t index, shift;
    if (x < plan->snapshot_x || y < plan->snapshot_y ||
        x >= plan->snapshot_x + LOCAL_MAP_SIDE || y >= plan->snapshot_y + LOCAL_MAP_SIDE)
        return LOCAL_MAP_UNKNOWN;
    index = (uint32_t)(y - plan->snapshot_y) * LOCAL_MAP_SIDE +
            (uint32_t)(x - plan->snapshot_x);
    shift = (index & 3u) * 2u;
    return (uint8_t)(((uint32_t)plan->map_snapshot[index >> 2] >> shift) & 3u);
}

static int cell_inside_start_body(const local_plan_pose_t *pose, int32_t x, int32_t y)
{
    int32_t half_length = pose->body_length_mm / 2 - pose->margin_mm - 1;
    int32_t half_width = pose->body_width_mm / 2 - pose->margin_mm - 1;
    return footprint_cell_inside(x, y, pose->x_mm, pose->y_mm,
                                 pose->cos_q15, pose->sin_q15, half_length, half_width);
}

static int is_walkable(local_plan_t *plan, uint16_t node)
{
    int32_t cx, cy, x, y, extent, extent_x, extent_y, min_x, max_x, min_y, max_y, extra;
    int32_t half_length, half_width, abs_cos, abs_sin;
    if (plan->walkability[node]) return plan->walkability[node] == WALKABLE;
    cx = (plan->window_x + node % LOCAL_PLAN_SIDE) * LOCAL_MAP_CELL_MM + HALF_CELL_MM;
    cy = (plan->window_y + node / LOCAL_PLAN_SIDE) * LOCAL_MAP_CELL_MM + HALF_CELL_MM;
    extra = plan->pose.margin_mm + STEP_MARGIN_MM + 1;
    half_length = plan->pose.body_length_mm / 2 + extra;
    half_width = plan->pose.body_width_mm / 2 + extra;
    abs_cos = plan->pose.cos_q15 < 0 ? -plan->pose.cos_q15 : plan->pose.cos_q15;
    abs_sin = plan->pose.sin_q15 < 0 ? -plan->pose.sin_q15 : plan->pose.sin_q15;
    /* Pose dimensions and Q15 orientation are validated by local_plan_begin. */
    extent_x = (half_length * abs_cos + half_width * abs_sin + Q15_ONE - 1) / Q15_ONE + HALF_CELL_MM;
    extent_y = (half_length * abs_sin + half_width * abs_cos + Q15_ONE - 1) / Q15_ONE + HALF_CELL_MM;
    extent = plan->pose.body_length_mm / 2 + plan->pose.body_width_mm / 2 + extra + HALF_CELL_MM;
    if (extent_x > extent) extent_x = extent;
    if (extent_y > extent) extent_y = extent;
    min_x = floor_cell(cx - extent_x);
    max_x = floor_cell(cx + extent_x);
    min_y = floor_cell(cy - extent_y);
    max_y = floor_cell(cy + extent_y);
    for (y = min_y; y <= max_y; ++y) {
        for (x = min_x; x <= max_x; ++x) {
            uint8_t state;
            if (!footprint_cell_intersects(x, y, cx, cy, plan->pose.cos_q15,
                plan->pose.sin_q15, half_length, half_width)) continue;
            state = snapshot_get(plan, x, y);
            if (state == LOCAL_MAP_OCCUPIED ||
                (state != LOCAL_MAP_FREE && !cell_inside_start_body(&plan->pose, x, y))) {
                plan->walkability[node] = BLOCKED;
                return 0;
            }
        }
    }
    plan->walkability[node] = WALKABLE;
    return 1;
}

static uint16_t heuristic(uint16_t node, uint16_t goal)
{
    int32_t x = node % LOCAL_PLAN_SIDE, y = node / LOCAL_PLAN_SIDE;
    int32_t gx = goal % LOCAL_PLAN_SIDE, gy = goal / LOCAL_PLAN_SIDE;
    int32_t dx = x > gx ? x - gx : gx - x;
    int32_t dy = y > gy ? y - gy : gy - y;
    int32_t diagonal = dx < dy ? dx : dy;
    return (uint16_t)(14 * diagonal + 10 * (dx + dy - 2 * diagonal));
}

static int heap_less(const local_plan_t *plan, uint16_t a, uint16_t b)
{
    uint32_t fa = (uint32_t)plan->score[a] + heuristic(a, plan->goal_node);
    uint32_t fb = (uint32_t)plan->score[b] + heuristic(b, plan->goal_node);
    return fa < fb || (fa == fb && heuristic(a, plan->goal_node) < heuristic(b, plan->goal_node));
}

static void heap_swap(local_plan_t *plan, uint16_t a, uint16_t b)
{
    uint16_t node = plan->heap[a];
    plan->heap[a] = plan->heap[b];
    plan->heap[b] = node;
    plan->heap_position[plan->heap[a]] = (uint16_t)(a + 1u);
    plan->heap_position[plan->heap[b]] = (uint16_t)(b + 1u);
}

static void heap_push_or_update(local_plan_t *plan, uint16_t node)
{
    uint16_t position;
    if (!plan->heap_position[node]) {
        position = plan->heap_size++;
        plan->heap[position] = node;
        plan->heap_position[node] = (uint16_t)(position + 1u);
    } else {
        position = (uint16_t)(plan->heap_position[node] - 1u);
    }
    while (position > 0) {
        uint16_t parent = (uint16_t)((position - 1u) / 2u);
        if (!heap_less(plan, plan->heap[position], plan->heap[parent])) break;
        heap_swap(plan, position, parent);
        position = parent;
    }
}

static uint16_t heap_pop(local_plan_t *plan)
{
    uint16_t node = plan->heap[0], position = 0;
    plan->heap_position[node] = 0;
    --plan->heap_size;
    if (!plan->heap_size) return node;
    plan->heap[0] = plan->heap[plan->heap_size];
    plan->heap_position[plan->heap[0]] = 1;
    while (1) {
        uint16_t left = (uint16_t)(position * 2u + 1u);
        uint16_t right = (uint16_t)(left + 1u);
        uint16_t best = position;
        if (left < plan->heap_size && heap_less(plan, plan->heap[left], plan->heap[best])) best = left;
        if (right < plan->heap_size && heap_less(plan, plan->heap[right], plan->heap[best])) best = right;
        if (best == position) break;
        heap_swap(plan, position, best);
        position = best;
    }
    return node;
}

int local_plan_begin(local_plan_t *plan, local_map_t *map,
                     const local_plan_pose_t *pose, int32_t goal_x_mm,
                     int32_t goal_y_mm, uint32_t now_us)
{
    uint32_t index;
    int32_t start_x, start_y, goal_x, goal_y;
    int64_t dx, dy, maximum;
    if (plan) plan->status = LOCAL_PLAN_IDLE;
    if (!plan || !map || !map->initialized || !pose ||
        pose->body_length_mm <= 0 || pose->body_width_mm <= 0 || pose->margin_mm < 0 ||
        pose->body_length_mm > 2000 || pose->body_width_mm > 2000 || pose->margin_mm > 1000 ||
        !footprint_orientation_valid(pose->cos_q15, pose->sin_q15))
        return 0;

    start_x = floor_cell(pose->x_mm);
    start_y = floor_cell(pose->y_mm);
    if (start_x < map->origin_x_cell || start_y < map->origin_y_cell ||
        start_x >= map->origin_x_cell + LOCAL_MAP_SIDE ||
        start_y >= map->origin_y_cell + LOCAL_MAP_SIDE)
        return 0;
    plan->pose = *pose;
    plan->goal_x_mm = goal_x_mm;
    plan->goal_y_mm = goal_y_mm;
    plan->snapshot_x = map->origin_x_cell;
    plan->snapshot_y = map->origin_y_cell;
    if (!local_map_snapshot(map, now_us, pose->required_fresh_us, plan->map_snapshot)) return 0;
    local_map_overlay_confirmed(map, now_us, pose->required_fresh_us, plan->map_snapshot);
    plan->map_generation = map->generation;
    goal_x = floor_cell(goal_x_mm);
    goal_y = floor_cell(goal_y_mm);
    plan->window_x = start_x - LOCAL_PLAN_SIDE / 2;
    plan->window_y = start_y - LOCAL_PLAN_SIDE / 2;
    dx = (int64_t)goal_x - start_x;
    dy = (int64_t)goal_y - start_y;
    maximum = abs64(dx) > abs64(dy) ? abs64(dx) : abs64(dy);
    plan->goal_inside = (uint8_t)(maximum <= LOCAL_PLAN_SIDE / 2 - 1);
    if (!plan->goal_inside) {
        dx = dx * (LOCAL_PLAN_SIDE / 2 - 1) / maximum;
        dy = dy * (LOCAL_PLAN_SIDE / 2 - 1) / maximum;
    }
    plan->start_node = (uint16_t)((LOCAL_PLAN_SIDE / 2) * LOCAL_PLAN_SIDE + LOCAL_PLAN_SIDE / 2);
    plan->goal_node = (uint16_t)((LOCAL_PLAN_SIDE / 2 + dy) * LOCAL_PLAN_SIDE +
                                 LOCAL_PLAN_SIDE / 2 + dx);
    plan->best_node = plan->start_node;
    plan->best_heuristic = heuristic(plan->start_node, plan->goal_node);
    plan->expansions = 0;
    plan->heap_size = 0;
    for (index = 0; index < LOCAL_PLAN_NODES; ++index) {
        plan->score[index] = NO_SCORE;
        plan->heap_position[index] = 0;
        plan->parent[index] = 0;
        plan->node_state[index] = 0;
        plan->walkability[index] = 0;
    }
    plan->score[plan->start_node] = 0;
    plan->node_state[plan->start_node] = NODE_OPEN;
    heap_push_or_update(plan, plan->start_node);
    plan->status = LOCAL_PLAN_RUNNING;
    return 1;
}

enum local_plan_status local_plan_step(local_plan_t *plan, uint16_t max_expansions)
{
    uint16_t count;
    if (!plan || plan->status != LOCAL_PLAN_RUNNING) return plan ? plan->status : LOCAL_PLAN_IDLE;
    for (count = 0; count < max_expansions && plan->heap_size; ++count) {
        uint16_t current = heap_pop(plan);
        int32_t x = current % LOCAL_PLAN_SIDE, y = current / LOCAL_PLAN_SIDE;
        uint16_t current_h = heuristic(current, plan->goal_node);
        int direction;
        plan->node_state[current] = NODE_CLOSED;
        ++plan->expansions;
        if (current_h < plan->best_heuristic) {
            plan->best_heuristic = current_h;
            plan->best_node = current;
        }
        if (current == plan->goal_node) {
            plan->best_node = current;
            plan->status = plan->goal_inside ? LOCAL_PLAN_COMPLETE : LOCAL_PLAN_PARTIAL;
            return plan->status;
        }
        for (direction = 0; direction < 8; ++direction) {
            int32_t nx = x + step_x[direction], ny = y + step_y[direction];
            uint16_t next, candidate;
            if (nx < 0 || ny < 0 || nx >= LOCAL_PLAN_SIDE || ny >= LOCAL_PLAN_SIDE) continue;
            next = (uint16_t)(ny * LOCAL_PLAN_SIDE + nx);
            if (plan->node_state[next] == NODE_CLOSED || !is_walkable(plan, next)) continue;
            if (step_x[direction] && step_y[direction]) {
                uint16_t side_x = (uint16_t)(y * LOCAL_PLAN_SIDE + nx);
                uint16_t side_y = (uint16_t)(ny * LOCAL_PLAN_SIDE + x);
                if (!is_walkable(plan, side_x) || !is_walkable(plan, side_y)) continue;
            }
            candidate = (uint16_t)(plan->score[current] +
                                   (step_x[direction] && step_y[direction] ? 14u : 10u));
            if (candidate >= plan->score[next]) continue;
            plan->score[next] = candidate;
            plan->parent[next] = (uint8_t)direction;
            plan->node_state[next] = NODE_OPEN;
            heap_push_or_update(plan, next);
        }
    }
    if (!plan->heap_size) {
        plan->status = plan->best_node == plan->start_node ? LOCAL_PLAN_BLOCKED : LOCAL_PLAN_PARTIAL;
    }
    return plan->status;
}

uint16_t local_plan_result(local_plan_t *plan, local_plan_point_t *points,
                           uint16_t capacity, int *ends_at_goal)
{
    uint16_t current, count = 0, written = 0;
    if (ends_at_goal) *ends_at_goal = 0;
    if (!plan || !points || !capacity ||
        (plan->status != LOCAL_PLAN_COMPLETE && plan->status != LOCAL_PLAN_PARTIAL))
        return 0;
    current = plan->best_node;
    while (current != plan->start_node && count < LOCAL_PLAN_NODES) {
        uint8_t direction = plan->parent[current];
        int32_t x = current % LOCAL_PLAN_SIDE, y = current / LOCAL_PLAN_SIDE;
        plan->reverse_path[count++] = current;
        x -= step_x[direction];
        y -= step_y[direction];
        if (x < 0 || y < 0 || x >= LOCAL_PLAN_SIDE || y >= LOCAL_PLAN_SIDE) return 0;
        current = (uint16_t)(y * LOCAL_PLAN_SIDE + x);
    }
    if (current != plan->start_node) return 0;
    while (count && written < capacity && written < LOCAL_PLAN_MAX_POINTS) {
        uint16_t node = plan->reverse_path[--count];
        points[written].x_mm = (plan->window_x + node % LOCAL_PLAN_SIDE) *
                                LOCAL_MAP_CELL_MM + HALF_CELL_MM;
        points[written].y_mm = (plan->window_y + node / LOCAL_PLAN_SIDE) *
                                LOCAL_MAP_CELL_MM + HALF_CELL_MM;
        ++written;
    }
    if (!written && plan->status == LOCAL_PLAN_COMPLETE) {
        points[0].x_mm = plan->goal_x_mm;
        points[0].y_mm = plan->goal_y_mm;
        if (ends_at_goal) *ends_at_goal = 1;
        return 1;
    }
    if (!count && written && plan->status == LOCAL_PLAN_COMPLETE) {
        points[written - 1].x_mm = plan->goal_x_mm;
        points[written - 1].y_mm = plan->goal_y_mm;
        if (ends_at_goal) *ends_at_goal = 1;
    }
    return written;
}
