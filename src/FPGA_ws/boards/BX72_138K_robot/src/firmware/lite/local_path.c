#include "local_path.h"
#include "local_grid.h"

#define PATH_SIDE 32
#define PATH_NODES (PATH_SIDE * PATH_SIDE)
#define PATH_PADDING 4
#define PATH_SNAPSHOT_SIDE (PATH_SIDE + PATH_PADDING * 2)
#define PATH_SNAPSHOT_CELLS (PATH_SNAPSHOT_SIDE * PATH_SNAPSHOT_SIDE)
#define PATH_NONE 0xffffu
#define PATH_PARENT_NONE 0xffu
#define PATH_PARENT_START 8u
#define PATH_PARENT_CLOSED 0x80u
/* 优先走已观测通道；未知只可用于提出待观察方向，不能直接授权运动。
 * 最长无环路径为 1023 步，即使均为对角未知格，总代价仍小于 65535。 */
#define PATH_UNKNOWN_COST 48u
#define PATH_MAX_STEPS 32u
#define PATH_MAX_AGE_US 1200000u

enum path_phase { PHASE_IDLE, PHASE_CAPTURE, PHASE_SEARCH, PHASE_BACKTRACK,
                  PHASE_FOUND, PHASE_BLOCKED };

/* 约 7.6 KiB；占用是局部搜索快照，不修改唯一的实时建图实例。 */
static uint16_t score[PATH_NODES], heap[PATH_NODES], heap_position[PATH_NODES];
static uint8_t parent[PATH_NODES], blocked[PATH_NODES / 8];
static uint8_t snapshot[(PATH_SNAPSHOT_CELLS + 3) / 4];
static const int8_t step_x[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int8_t step_y[8] = {0, 1, 1, 1, 0, -1, -1, -1};
/* 第一轮用车体外接圆和余量；无路后以内切圆产生候选引导，
 * 路径并非碰撞保证，执行层仍逐拍检查矩形车体和制动扫掠。 */
static const int8_t inflation_half_width[9] = {2, 3, 4, 4, 4, 4, 4, 3, 2};
static const int8_t inflation_fallback[9] = {-1, 1, 2, 3, 3, 3, 2, 1, -1};
static enum path_phase phase;
static uint16_t cursor, heap_size, start_node, end_node, backtrack_node;
static int32_t window_x, window_y, source_origin_x, source_origin_y;
static int32_t goal_cell_x, goal_cell_y, exact_goal_x, exact_goal_y;
static int32_t target_x, target_y;
static uint32_t search_started_us;
static uint8_t started_clock, goal_inside, target_is_final, fallback_mode;

static int32_t floor_cell(int32_t mm)
{
    int32_t result = mm / (int32_t)BX72_GRID_CELL_MM;
    if (mm < 0 && mm % (int32_t)BX72_GRID_CELL_MM) --result;
    return result;
}

static uint32_t absolute_difference(int32_t a, int32_t b)
{
    int64_t delta = (int64_t)a - b;
    return (uint32_t)(delta < 0 ? -delta : delta);
}

static void snapshot_set(uint16_t index, uint8_t value)
{
    uint32_t shift = (index & 3u) * 2u;
    snapshot[index >> 2] = (uint8_t)((snapshot[index >> 2] & ~(3u << shift)) |
                                      ((uint32_t)value << shift));
}

static uint8_t snapshot_get(uint16_t index)
{
    return (uint8_t)((snapshot[index >> 2] >> ((index & 3u) * 2u)) & 3u);
}

static int blocked_node(uint16_t node)
{
    return node != start_node && (blocked[node >> 3] & (1u << (node & 7u))) != 0;
}

static uint32_t heuristic(uint16_t node)
{
    uint32_t dx = absolute_difference(window_x + node % PATH_SIDE, goal_cell_x);
    uint32_t dy = absolute_difference(window_y + node / PATH_SIDE, goal_cell_y);
    uint32_t minimum = dx < dy ? dx : dy;
    return (dx > dy ? dx : dy) * 10u + minimum * 4u;
}

static int heap_less(uint16_t a, uint16_t b)
{
    uint32_t fa = (uint32_t)score[a] + heuristic(a);
    uint32_t fb = (uint32_t)score[b] + heuristic(b);
    return fa < fb || (fa == fb && score[a] > score[b]);
}

static void heap_swap(uint16_t a, uint16_t b)
{
    uint16_t n = heap[a];
    heap[a] = heap[b];
    heap[b] = n;
    heap_position[heap[a]] = a;
    heap_position[heap[b]] = b;
}

static void heap_up(uint16_t position)
{
    while (position) {
        uint16_t up = (uint16_t)((position - 1u) / 2u);
        if (!heap_less(heap[position], heap[up])) break;
        heap_swap(position, up);
        position = up;
    }
}

static void heap_insert_or_decrease(uint16_t node)
{
    uint16_t position = heap_position[node];
    if (position == PATH_NONE) {
        position = heap_size++;
        heap[position] = node;
        heap_position[node] = position;
    }
    heap_up(position);
}

static uint16_t heap_remove(void)
{
    uint16_t result = heap[0], position = 0u;
    heap_position[result] = PATH_NONE;
    --heap_size;
    if (!heap_size) return result;
    heap[0] = heap[heap_size];
    heap_position[heap[0]] = 0u;
    for (;;) {
        uint16_t left = (uint16_t)(position * 2u + 1u), best = position;
        uint16_t right = (uint16_t)(left + 1u);
        if (left < heap_size && heap_less(heap[left], heap[best])) best = left;
        if (right < heap_size && heap_less(heap[right], heap[best])) best = right;
        if (best == position) break;
        heap_swap(position, best);
        position = best;
    }
    return result;
}

static void inflate_snapshot_cell(uint16_t index)
{
    int x = (int)(index % PATH_SNAPSHOT_SIDE) - PATH_PADDING;
    int y = (int)(index / PATH_SNAPSHOT_SIDE) - PATH_PADDING;
    int dy;
    uint8_t value = snapshot_get(index);
    if (value != BX72_GRID_OCCUPIED && value != BX72_GRID_OUTSIDE) return;
    for (dy = -4; dy <= 4; ++dy) {
        int cy = y + dy, dx;
        int half_width = fallback_mode ? inflation_fallback[dy + 4] : inflation_half_width[dy + 4];
        if (cy < 0 || cy >= PATH_SIDE || half_width < 0) continue;
        for (dx = -half_width; dx <= half_width; ++dx) {
            int cx = x + dx;
            uint16_t node;
            if (cx < 0 || cx >= PATH_SIDE) continue;
            node = (uint16_t)(cy * PATH_SIDE + cx);
            blocked[node >> 3] |= (uint8_t)(1u << (node & 7u));
        }
    }
}

/* 检查网格中心线，转角不穿过障碍的对角缝；实时车体扫掠由执行层负责。 */
static int line_clear(uint16_t from, uint16_t to)
{
    int x = from % PATH_SIDE, y = from / PATH_SIDE;
    int tx = to % PATH_SIDE, ty = to / PATH_SIDE;
    int dx = tx >= x ? tx - x : x - tx, sx = x < tx ? 1 : -1;
    int dy = ty >= y ? y - ty : ty - y, sy = y < ty ? 1 : -1;
    int error = dx + dy;
    while (x != tx || y != ty) {
        int twice = error * 2, nx = x, ny = y;
        if (twice >= dy) { error += dy; nx += sx; }
        if (twice <= dx) { error += dx; ny += sy; }
        if (nx != x && ny != y &&
            (blocked_node((uint16_t)(y * PATH_SIDE + nx)) ||
             blocked_node((uint16_t)(ny * PATH_SIDE + x)))) return 0;
        x = nx;
        y = ny;
        if (blocked_node((uint16_t)(y * PATH_SIDE + x))) return 0;
        /* 不得用路径简化跨过未观测格；未知终点只作为下一观察方向。 */
        if ((x != tx || y != ty) &&
            snapshot_get((uint16_t)((y + PATH_PADDING) * PATH_SNAPSHOT_SIDE +
                                    x + PATH_PADDING)) == BX72_GRID_UNKNOWN) return 0;
    }
    return 1;
}

void bx72_local_path_reset(void)
{
    phase = PHASE_IDLE;
    started_clock = 0u;
}

void bx72_local_path_begin(int32_t start_x_mm, int32_t start_y_mm,
                           int32_t goal_x_mm, int32_t goal_y_mm)
{
    bx72_grid_info_t info;
    int32_t start_x = floor_cell(start_x_mm), start_y = floor_cell(start_y_mm);
    uint16_t i;
    bx72_grid_info(&info);
    source_origin_x = info.origin_x_cell;
    source_origin_y = info.origin_y_cell;
    window_x = start_x - PATH_SIDE / 2;
    window_y = start_y - PATH_SIDE / 2;
    if (window_x < source_origin_x) window_x = source_origin_x;
    if (window_y < source_origin_y) window_y = source_origin_y;
    if (window_x > source_origin_x + (int32_t)BX72_GRID_SIZE - PATH_SIDE)
        window_x = source_origin_x + (int32_t)BX72_GRID_SIZE - PATH_SIDE;
    if (window_y > source_origin_y + (int32_t)BX72_GRID_SIZE - PATH_SIDE)
        window_y = source_origin_y + (int32_t)BX72_GRID_SIZE - PATH_SIDE;
    if (start_x < window_x || start_x >= window_x + PATH_SIDE ||
        start_y < window_y || start_y >= window_y + PATH_SIDE) {
        phase = PHASE_BLOCKED;
        return;
    }
    start_node = (uint16_t)((start_y - window_y) * PATH_SIDE + start_x - window_x);
    goal_cell_x = floor_cell(goal_x_mm);
    goal_cell_y = floor_cell(goal_y_mm);
    exact_goal_x = goal_x_mm;
    exact_goal_y = goal_y_mm;
    goal_inside = goal_cell_x >= window_x && goal_cell_x < window_x + PATH_SIDE &&
                  goal_cell_y >= window_y && goal_cell_y < window_y + PATH_SIDE;
    target_is_final = 0u;
    fallback_mode = 0u;
    for (i = 0u; i < sizeof(blocked); ++i) blocked[i] = 0u;
    cursor = heap_size = 0u;
    started_clock = 0u;
    phase = PHASE_CAPTURE;
}

int bx72_local_path_retarget(int32_t goal_x_mm, int32_t goal_y_mm)
{
    if (phase == PHASE_IDLE || phase == PHASE_BLOCKED ||
        floor_cell(goal_x_mm) != goal_cell_x || floor_cell(goal_y_mm) != goal_cell_y)
        return 0;
    exact_goal_x = goal_x_mm;
    exact_goal_y = goal_y_mm;
    if (phase == PHASE_FOUND && target_is_final) {
        target_x = goal_x_mm;
        target_y = goal_y_mm;
    }
    return 1;
}

int bx72_local_path_goal_inside(void)
{
    return goal_inside != 0u;
}

static void search_one(void)
{
    uint16_t node;
    int x, y, d;
    if (!heap_size) {
        if (!fallback_mode) {
            unsigned int i;
            /* 重新捕获实时栅格，只进行一次较小包络搜索。 */
            fallback_mode = 1u;
            for (i = 0u; i < sizeof(blocked); ++i) blocked[i] = 0u;
            cursor = heap_size = 0u;
            started_clock = 0u;
            phase = PHASE_CAPTURE;
        } else phase = PHASE_BLOCKED;
        return;
    }
    node = heap_remove();
    x = node % PATH_SIDE;
    y = node / PATH_SIDE;
    parent[node] |= PATH_PARENT_CLOSED;
    if ((goal_inside && window_x + x == goal_cell_x && window_y + y == goal_cell_y) ||
        (!goal_inside && node != start_node &&
         (x == 0 || y == 0 || x == PATH_SIDE - 1 || y == PATH_SIDE - 1))) {
        end_node = backtrack_node = node;
        phase = PHASE_BACKTRACK;
        return;
    }
    for (d = 0; d < 8; ++d) {
        int nx = x + step_x[d], ny = y + step_y[d];
        uint16_t next, snap_index;
        uint32_t cost;
        if (nx < 0 || ny < 0 || nx >= PATH_SIDE || ny >= PATH_SIDE) continue;
        next = (uint16_t)(ny * PATH_SIDE + nx);
        if (blocked_node(next) || (parent[next] != PATH_PARENT_NONE &&
                                   (parent[next] & PATH_PARENT_CLOSED))) continue;
        if (step_x[d] && step_y[d] &&
            (blocked_node((uint16_t)(y * PATH_SIDE + nx)) ||
             blocked_node((uint16_t)(ny * PATH_SIDE + x)))) continue;
        snap_index = (uint16_t)((ny + PATH_PADDING) * PATH_SNAPSHOT_SIDE + nx + PATH_PADDING);
        cost = (uint32_t)score[node] + (step_x[d] && step_y[d] ? 14u : 10u) +
               (snapshot_get(snap_index) == BX72_GRID_UNKNOWN ? PATH_UNKNOWN_COST : 0u);
        if (cost >= PATH_NONE || cost >= score[next]) continue;
        score[next] = (uint16_t)cost;
        parent[next] = (uint8_t)((d + 4) & 7);
        heap_insert_or_decrease(next);
    }
}

static void backtrack_one(void)
{
    int dx = (int)(backtrack_node % PATH_SIDE) - (int)(start_node % PATH_SIDE);
    int dy = (int)(backtrack_node / PATH_SIDE) - (int)(start_node / PATH_SIDE);
    uint8_t direction;
    if (backtrack_node == start_node) {
        if (!goal_inside || end_node != start_node) { phase = PHASE_BLOCKED; return; }
        target_x = exact_goal_x; target_y = exact_goal_y; target_is_final = 1u;
        phase = PHASE_FOUND;
        return;
    }
    if (dx * dx + dy * dy <= 100 && line_clear(start_node, backtrack_node)) {
        target_is_final = goal_inside && backtrack_node == end_node;
        target_x = target_is_final ? exact_goal_x : (window_x + backtrack_node % PATH_SIDE) * 50 + 25;
        target_y = target_is_final ? exact_goal_y : (window_y + backtrack_node / PATH_SIDE) * 50 + 25;
        phase = PHASE_FOUND;
        return;
    }
    direction = parent[backtrack_node] & 7u;
    backtrack_node = (uint16_t)(((int)(backtrack_node / PATH_SIDE) + step_y[direction]) * PATH_SIDE +
                                (int)(backtrack_node % PATH_SIDE) + step_x[direction]);
}

void bx72_local_path_step(uint32_t (*clock_us)(void), uint32_t budget_us)
{
    uint32_t now, started, work;
    bx72_grid_info_t info;
    if (!clock_us || !budget_us || phase == PHASE_IDLE || phase >= PHASE_FOUND) return;
    now = clock_us();
    if (!started_clock) { search_started_us = now; started_clock = 1u; }
    if ((uint32_t)(now - search_started_us) >= PATH_MAX_AGE_US) { phase = PHASE_BLOCKED; return; }
    bx72_grid_info(&info);
    /* 环形地图小幅居中不会改变重叠格的世界坐标，静止时 1 格抖动不应饿死搜索。 */
    if (absolute_difference(info.origin_x_cell, source_origin_x) > 2u ||
        absolute_difference(info.origin_y_cell, source_origin_y) > 2u) {
        phase = PHASE_BLOCKED;
        return;
    }
    started = now;
    for (work = 0u; work < PATH_MAX_STEPS; ++work) {
        if ((uint32_t)(clock_us() - started) >= budget_us) return;
        if (phase == PHASE_CAPTURE) {
            int32_t x = window_x + (int32_t)(cursor % PATH_SNAPSHOT_SIDE) - PATH_PADDING;
            int32_t y = window_y + (int32_t)(cursor / PATH_SNAPSHOT_SIDE) - PATH_PADDING;
            snapshot_set(cursor, bx72_grid_cell_at(x, y));
            inflate_snapshot_cell(cursor);
            if (cursor < PATH_NODES) {
                score[cursor] = heap_position[cursor] = PATH_NONE;
                parent[cursor] = PATH_PARENT_NONE;
            }
            if (++cursor == PATH_SNAPSHOT_CELLS) {
                score[start_node] = 0u;
                parent[start_node] = PATH_PARENT_START;
                heap_insert_or_decrease(start_node);
                phase = PHASE_SEARCH;
            }
        } else if (phase == PHASE_SEARCH) search_one();
        else if (phase == PHASE_BACKTRACK) backtrack_one();
        else return;
    }
}

enum bx72_local_path_state bx72_local_path_state(void)
{
    if (phase == PHASE_IDLE) return BX72_PATH_IDLE;
    if (phase == PHASE_FOUND) return BX72_PATH_FOUND;
    if (phase == PHASE_BLOCKED) return BX72_PATH_BLOCKED;
    return BX72_PATH_WORKING;
}

int bx72_local_path_target(int32_t *x_mm, int32_t *y_mm, int *final)
{
    if (phase != PHASE_FOUND || !x_mm || !y_mm || !final) return 0;
    *x_mm = target_x;
    *y_mm = target_y;
    *final = target_is_final;
    return 1;
}
