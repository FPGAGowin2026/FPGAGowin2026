#include "local_map.h"

#define STATE_SHIFT 6u
#define TICK_MASK 63u
#define CONFIRMED_RADIUS_MM 300

static int32_t floor_cell(int32_t mm)
{
    int32_t cell = mm / LOCAL_MAP_CELL_MM;
    if (mm % LOCAL_MAP_CELL_MM < 0) --cell;
    return cell;
}

static uint32_t slot(int32_t x, int32_t y)
{
    return ((uint32_t)y & (LOCAL_MAP_SIDE - 1u)) * LOCAL_MAP_SIDE +
           ((uint32_t)x & (LOCAL_MAP_SIDE - 1u));
}

static int inside(const local_map_t *map, int32_t x, int32_t y)
{
    return x >= map->origin_x_cell && x < map->origin_x_cell + LOCAL_MAP_SIDE &&
           y >= map->origin_y_cell && y < map->origin_y_cell + LOCAL_MAP_SIDE;
}

static void clear_hit(local_map_t *map, uint32_t index)
{
    map->scan_hits[index >> 3] &= (uint8_t)~(1u << (index & 7u));
}

static void clear_all(local_map_t *map)
{
    uint32_t i;
    for (i = 0; i < LOCAL_MAP_CELLS; ++i) map->cells[i] = 0;
    for (i = 0; i < LOCAL_MAP_CELLS / 8; ++i) map->scan_hits[i] = 0;
    ++map->generation;
}

static void expire_cells(local_map_t *map)
{
    uint32_t i;
    for (i = 0; i < LOCAL_MAP_CELLS; ++i) {
        uint8_t value = map->cells[i];
        uint8_t age = (uint8_t)((map->current_tick - value) & TICK_MASK);
        if ((value >> STATE_SHIFT) != LOCAL_MAP_UNKNOWN && age >= map->ttl_ticks) {
            map->cells[i] = 0;
            clear_hit(map, i);
            ++map->generation;
        }
    }
}

static int write_cell(local_map_t *map, uint32_t index, enum local_map_state state,
                      uint8_t observation_tick)
{
    uint8_t value = (uint8_t)(((uint8_t)state << STATE_SHIFT) | observation_tick);
    uint8_t old_value = map->cells[index];
    uint8_t old_age = (uint8_t)((map->current_tick - old_value) & TICK_MASK);
    uint8_t new_age = (uint8_t)((map->current_tick - observation_tick) & TICK_MASK);
    if ((old_value >> STATE_SHIFT) != LOCAL_MAP_UNKNOWN && old_age < new_age) return 0;
    if (map->cells[index] != value) {
        map->cells[index] = value;
        ++map->generation;
    }
    return 1;
}

int local_map_init(local_map_t *map, int32_t center_x_mm, int32_t center_y_mm,
                   uint8_t ttl_ticks, uint32_t now_us)
{
    if (!map || ttl_ticks == 0 || ttl_ticks > 31) return 0;
    map->origin_x_cell = floor_cell(center_x_mm) - LOCAL_MAP_SIDE / 2;
    map->origin_y_cell = floor_cell(center_y_mm) - LOCAL_MAP_SIDE / 2;
    map->last_update_us = now_us;
    map->tick_phase_us = 0;
    map->generation = 0;
    map->current_tick = 0;
    map->ttl_ticks = ttl_ticks;
    map->initialized = 1;
    map->confirmed_valid = 0;
    clear_all(map);
    return 1;
}

void local_map_advance(local_map_t *map, uint32_t now_us)
{
    uint32_t delta, total, elapsed_ticks;
    if (!map || !map->initialized) return;
    delta = now_us - map->last_update_us;
    map->last_update_us = now_us;
    if (delta >= 0x80000000u) {
        clear_all(map);
        map->confirmed_valid = 0;
        map->tick_phase_us = 0;
        map->current_tick = 0;
        return;
    }
    total = map->tick_phase_us + delta;
    elapsed_ticks = total / LOCAL_MAP_TICK_US;
    map->tick_phase_us = total % LOCAL_MAP_TICK_US;
    if (!elapsed_ticks) return;
    map->current_tick = (uint8_t)((map->current_tick + elapsed_ticks) & TICK_MASK);
    if (elapsed_ticks >= map->ttl_ticks) {
        clear_all(map);
        return;
    }
    expire_cells(map);
}

void local_map_set_ttl(local_map_t *map, uint8_t ttl_ticks, uint32_t now_us)
{
    if (map->ttl_ticks == ttl_ticks) return;
    local_map_advance(map, now_us);
    map->ttl_ticks = ttl_ticks;
    expire_cells(map);
}

void local_map_recenter(local_map_t *map, int32_t center_x_mm, int32_t center_y_mm,
                        uint32_t now_us)
{
    int32_t next_x, next_y, x, y;
    if (!map || !map->initialized) return;
    local_map_advance(map, now_us);
    next_x = floor_cell(center_x_mm) - LOCAL_MAP_SIDE / 2;
    next_y = floor_cell(center_y_mm) - LOCAL_MAP_SIDE / 2;
    if (next_x == map->origin_x_cell && next_y == map->origin_y_cell) return;
    if (next_x >= map->origin_x_cell + LOCAL_MAP_SIDE ||
        next_x + LOCAL_MAP_SIDE <= map->origin_x_cell ||
        next_y >= map->origin_y_cell + LOCAL_MAP_SIDE ||
        next_y + LOCAL_MAP_SIDE <= map->origin_y_cell) {
        clear_all(map);
    } else {
        for (y = next_y; y < next_y + LOCAL_MAP_SIDE; ++y) {
            for (x = next_x; x < next_x + LOCAL_MAP_SIDE; ++x) {
                if (!inside(map, x, y)) {
                    uint32_t index = slot(x, y);
                    map->cells[index] = 0;
                    clear_hit(map, index);
                }
            }
        }
        ++map->generation;
    }
    map->origin_x_cell = next_x;
    map->origin_y_cell = next_y;
}

void local_map_begin_scan(local_map_t *map, uint32_t now_us)
{
    uint32_t i;
    if (!map || !map->initialized) return;
    local_map_advance(map, now_us);
    for (i = 0; i < LOCAL_MAP_CELLS / 8; ++i) map->scan_hits[i] = 0;
}

int local_map_observe_ray(local_map_t *map, int32_t sensor_x_mm, int32_t sensor_y_mm,
                          int32_t end_x_mm, int32_t end_y_mm, int endpoint_is_hit,
                          uint32_t sample_us, uint32_t now_us)
{
    int32_t x, y, end_x, end_y, dx, dy, step_x, step_y, error;
    uint32_t age_us, age_ticks;
    uint8_t observation_tick;
    if (!map || !map->initialized) return 0;
    local_map_advance(map, now_us);
    age_us = now_us - sample_us;
    if (age_us >= 0x80000000u) return 0;
    age_ticks = age_us <= map->tick_phase_us ? 0 :
        1u + (age_us - map->tick_phase_us - 1u) / LOCAL_MAP_TICK_US;
    if (age_ticks >= map->ttl_ticks) return 0;
    observation_tick = (uint8_t)((map->current_tick - age_ticks) & TICK_MASK);
    x = floor_cell(sensor_x_mm);
    y = floor_cell(sensor_y_mm);
    end_x = floor_cell(end_x_mm);
    end_y = floor_cell(end_y_mm);
    if (!inside(map, x, y)) return 0;
    dx = end_x >= x ? end_x - x : x - end_x;
    dy = end_y >= y ? end_y - y : y - end_y;
    step_x = x < end_x ? 1 : -1;
    step_y = y < end_y ? 1 : -1;
    error = dx - dy;

    while (1) {
        if (x == end_x && y == end_y) {
            if (endpoint_is_hit) {
                uint32_t index = slot(x, y);
                int64_t hit_dx = (int64_t)end_x_mm - map->confirmed_x_mm;
                int64_t hit_dy = (int64_t)end_y_mm - map->confirmed_y_mm;
                if (map->confirmed_valid && hit_dx >= -CONFIRMED_RADIUS_MM &&
                    hit_dx <= CONFIRMED_RADIUS_MM && hit_dy >= -CONFIRMED_RADIUS_MM &&
                    hit_dy <= CONFIRMED_RADIUS_MM &&
                    hit_dx * hit_dx + hit_dy * hit_dy <= CONFIRMED_RADIUS_MM * CONFIRMED_RADIUS_MM)
                    map->confirmed_valid = 0;
                if (write_cell(map, index, LOCAL_MAP_OCCUPIED, observation_tick))
                    map->scan_hits[index >> 3] |= (uint8_t)(1u << (index & 7u));
            }
            break;
        }
        {
            int32_t twice_error = 2 * error;
            if (twice_error > -dy) { error -= dy; x += step_x; }
            if (twice_error < dx) { error += dx; y += step_y; }
        }
        if (!inside(map, x, y)) break;
        if (x != end_x || y != end_y || !endpoint_is_hit) {
            uint32_t index = slot(x, y);
            if (!(map->scan_hits[index >> 3] & (uint8_t)(1u << (index & 7u))))
                write_cell(map, index, LOCAL_MAP_FREE, observation_tick);
        }
    }
    return 1;
}

void local_map_confirm_start(local_map_t *map, int32_t x_mm, int32_t y_mm,
                             uint32_t until_us)
{
    map->confirmed_x_mm = x_mm;
    map->confirmed_y_mm = y_mm;
    map->confirmed_until_us = until_us;
    map->confirmed_valid = 1;
}

int local_map_confirmed_free(const local_map_t *map, int32_t x_cell, int32_t y_cell,
                             uint32_t now_us, uint32_t required_us)
{
    int64_t x0, x1, y0, y1, dx, dy;
    int32_t remaining;
    if (!map || !map->initialized || !map->confirmed_valid ||
        !inside(map, x_cell, y_cell)) return 0;
    remaining = (int32_t)(map->confirmed_until_us - now_us);
    if (remaining <= 0 || (uint32_t)remaining < required_us) return 0;
    x0 = (int64_t)x_cell * LOCAL_MAP_CELL_MM - map->confirmed_x_mm;
    y0 = (int64_t)y_cell * LOCAL_MAP_CELL_MM - map->confirmed_y_mm;
    x1 = x0 + LOCAL_MAP_CELL_MM;
    y1 = y0 + LOCAL_MAP_CELL_MM;
    dx = (x0 < 0 ? -x0 : x0) > (x1 < 0 ? -x1 : x1) ?
         (x0 < 0 ? -x0 : x0) : (x1 < 0 ? -x1 : x1);
    dy = (y0 < 0 ? -y0 : y0) > (y1 < 0 ? -y1 : y1) ?
         (y0 < 0 ? -y0 : y0) : (y1 < 0 ? -y1 : y1);
    if (dx > CONFIRMED_RADIUS_MM || dy > CONFIRMED_RADIUS_MM) return 0;
    return dx * dx + dy * dy <= CONFIRMED_RADIUS_MM * CONFIRMED_RADIUS_MM;
}

int local_map_confirmed_footprint(const local_map_t *map, int32_t center_x_mm,
                                  int32_t center_y_mm, int32_t cos_q15, int32_t sin_q15,
                                  int32_t half_length_mm, int32_t half_width_mm,
                                  uint32_t now_us, uint32_t required_us)
{
    const int64_t scale = 32768;
    const int64_t radius = (int64_t)CONFIRMED_RADIUS_MM * scale;
    int64_t dx, dy, x, y;
    int32_t remaining;
    int sx, sy;
    if (!map || !map->initialized || !map->confirmed_valid ||
        half_length_mm < 0 || half_length_mm > CONFIRMED_RADIUS_MM ||
        half_width_mm < 0 || half_width_mm > CONFIRMED_RADIUS_MM) return 0;
    remaining = (int32_t)(map->confirmed_until_us - now_us);
    if (remaining <= 0 || (uint32_t)remaining < required_us) return 0;
    dx = (int64_t)center_x_mm - map->confirmed_x_mm;
    dy = (int64_t)center_y_mm - map->confirmed_y_mm;
    if (dx < -CONFIRMED_RADIUS_MM || dx > CONFIRMED_RADIUS_MM ||
        dy < -CONFIRMED_RADIUS_MM || dy > CONFIRMED_RADIUS_MM) return 0;
    for (sx = -1; sx <= 1; sx += 2) {
        for (sy = -1; sy <= 1; sy += 2) {
            x = dx * scale + (int64_t)sx * half_length_mm * cos_q15 -
                (int64_t)sy * half_width_mm * sin_q15;
            y = dy * scale + (int64_t)sx * half_length_mm * sin_q15 +
                (int64_t)sy * half_width_mm * cos_q15;
            if (x * x + y * y > radius * radius) return 0;
        }
    }
    return 1;
}

void local_map_overlay_confirmed(const local_map_t *map, uint32_t now_us,
                                 uint32_t required_us, uint8_t packed[LOCAL_MAP_CELLS / 4])
{
    int32_t x, y, cx, cy;
    if (!map || !map->confirmed_valid || !packed) return;
    cx = floor_cell(map->confirmed_x_mm);
    cy = floor_cell(map->confirmed_y_mm);
    for (y = cy - 6; y <= cy + 6; ++y) {
        for (x = cx - 6; x <= cx + 6; ++x) {
            uint32_t index, shift;
            if (!local_map_confirmed_free(map, x, y, now_us, required_us)) continue;
            index = (uint32_t)(y - map->origin_y_cell) * LOCAL_MAP_SIDE +
                    (uint32_t)(x - map->origin_x_cell);
            shift = (index & 3u) * 2u;
            if ((((uint32_t)packed[index >> 2] >> shift) & 3u) == LOCAL_MAP_UNKNOWN)
                packed[index >> 2] |= (uint8_t)(LOCAL_MAP_FREE << shift);
        }
    }
}

enum local_map_state local_map_get(local_map_t *map, int32_t world_x_cell,
                                   int32_t world_y_cell, uint32_t now_us,
                                   uint32_t *remaining_us)
{
    uint8_t value, age;
    enum local_map_state state;
    if (remaining_us) *remaining_us = 0;
    if (!map || !map->initialized) return LOCAL_MAP_UNKNOWN;
    local_map_advance(map, now_us);
    if (!inside(map, world_x_cell, world_y_cell)) return LOCAL_MAP_UNKNOWN;
    value = map->cells[slot(world_x_cell, world_y_cell)];
    state = (enum local_map_state)(value >> STATE_SHIFT);
    if (state == LOCAL_MAP_UNKNOWN) return state;
    age = (uint8_t)((map->current_tick - value) & TICK_MASK);
    if (remaining_us)
        *remaining_us = (uint32_t)(map->ttl_ticks - age) * LOCAL_MAP_TICK_US -
                        map->tick_phase_us;
    return state;
}

int local_map_snapshot(local_map_t *map, uint32_t now_us, uint32_t required_fresh_us,
                       uint8_t packed[LOCAL_MAP_CELLS / 4])
{
    uint32_t x, y, bit;
    if (!map || !map->initialized || !packed) return 0;
    local_map_advance(map, now_us);
    for (y = 0; y < LOCAL_MAP_SIDE; ++y) {
        uint32_t row = (((uint32_t)map->origin_y_cell + y) & (LOCAL_MAP_SIDE - 1u)) * LOCAL_MAP_SIDE;
        if (!required_fresh_us) {
            for (x = 0; x < LOCAL_MAP_SIDE; x += 4u) {
                uint32_t column = (uint32_t)map->origin_x_cell + x;
                uint8_t a = map->cells[row + (column & (LOCAL_MAP_SIDE - 1u))];
                uint8_t b = map->cells[row + ((column + 1u) & (LOCAL_MAP_SIDE - 1u))];
                uint8_t c = map->cells[row + ((column + 2u) & (LOCAL_MAP_SIDE - 1u))];
                uint8_t d = map->cells[row + ((column + 3u) & (LOCAL_MAP_SIDE - 1u))];
                packed[(y * LOCAL_MAP_SIDE + x) / 4u] = (uint8_t)((a >> 6) |
                    ((b >> 4) & 0x0cu) | ((c >> 2) & 0x30u) | (d & 0xc0u));
            }
            continue;
        }
        for (x = 0; x < LOCAL_MAP_SIDE; x += 4u) {
            uint8_t result = 0;
            for (bit = 0; bit < 4u; ++bit) {
                uint32_t column = ((uint32_t)map->origin_x_cell + x + bit) & (LOCAL_MAP_SIDE - 1u);
                uint8_t value = map->cells[row + column];
                uint8_t state = (uint8_t)(value >> STATE_SHIFT);
                if (state == LOCAL_MAP_FREE) {
                    uint8_t age = (uint8_t)((map->current_tick - value) & TICK_MASK);
                    uint32_t remaining = (uint32_t)(map->ttl_ticks - age) * LOCAL_MAP_TICK_US -
                                         map->tick_phase_us;
                    if (remaining < required_fresh_us) state = LOCAL_MAP_UNKNOWN;
                }
                result = (uint8_t)(result | (uint8_t)(state << (bit * 2u)));
            }
            packed[(y * LOCAL_MAP_SIDE + x) / 4u] = result;
        }
    }
    return 1;
}
