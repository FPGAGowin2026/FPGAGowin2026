#include "local_grid.h"

#include <limits.h>

/* 两位状态、六位观测时刻。每250ms推进一格，20格后变未知。
 * 每个时刻推进前检查256格，16个时刻遍历全图，保证模64时刻不复活。
 * 两次调用间跨20格或至少5秒则整图过期，不依赖时间回绕猜测。 */
#define GRID_TICK_US 250000u
#define GRID_TTL_TICKS 20u
#define GRID_TICK_MASK 63u
#define GRID_STATE_SHIFT 6u
#define GRID_CELL_COUNT (BX72_GRID_SIZE * BX72_GRID_SIZE)
#define GRID_HIT_BYTES (GRID_CELL_COUNT / 8u)
#define GRID_MAX_RANGE_MM 8000u
#define GRID_MIN_RANGE_MM 50u

static union {
    uint8_t bytes[GRID_CELL_COUNT];
    uint32_t words[GRID_CELL_COUNT / 4u];
} cell_storage;
static union {
    uint8_t bytes[GRID_HIT_BYTES];
    uint32_t words[GRID_HIT_BYTES / 4u];
} scan_storage;
#define cells cell_storage.bytes
#define scan_hits scan_storage.bytes
/* 仅跳过无占用的整行；过期占用可暂时留位，精查时再按年龄排除。 */
static uint32_t occupied_bits[GRID_CELL_COUNT / 32u];
static int32_t origin_x, origin_y;
static uint32_t generation, scan_generation;
static uint32_t last_advance_us, tick_base_us;
static uint32_t last_ray_stamp_us, scan_first_stamp_us, complete_scan_stamp_us;
static uint16_t sweep_cursor;
static uint8_t tick_now, initialized, fresh, scan_active, scan_ok, scan_has_ray;

static void bump_generation(void)
{
    ++generation;
    if (generation == 0u) ++generation;
}

/* C有符号除法向零取整，世界格坐标必须向负无穷取整。 */
static int32_t floor_cell_i32(int32_t mm)
{
    int32_t result = mm / (int32_t)BX72_GRID_CELL_MM;
    if (mm < 0 && result * (int32_t)BX72_GRID_CELL_MM != mm) --result;
    return result;
}

static int32_t floor_cell(int64_t mm)
{
    if (mm >= INT32_MIN && mm <= INT32_MAX) return floor_cell_i32((int32_t)mm);
    int64_t result = mm / (int64_t)BX72_GRID_CELL_MM;
    if (mm % (int64_t)BX72_GRID_CELL_MM < 0) --result;
    return (int32_t)result;
}

/* 按世界格坐标低位映射环形存储；移动原点时重叠格无需复制。 */
static uint32_t slot(int32_t x, int32_t y)
{
    return ((uint32_t)y & 63u) * BX72_GRID_SIZE + ((uint32_t)x & 63u);
}

static void clear_all(void)
{
    uint32_t i;
    for (i = 0u; i < GRID_CELL_COUNT / 4u; ++i) cell_storage.words[i] = 0u;
    for (i = 0u; i < GRID_HIT_BYTES / 4u; ++i) scan_storage.words[i] = 0u;
    for (i = 0u; i < GRID_CELL_COUNT / 32u; ++i) occupied_bits[i] = 0u;
    sweep_cursor = 0u;
}

static void sweep_expired(uint8_t at_tick, uint32_t count)
{
    uint32_t n;
    if (count > GRID_CELL_COUNT) count = GRID_CELL_COUNT;
    for (n = 0u; n < count; ++n) {
        uint32_t i = sweep_cursor;
        uint8_t v = cells[i];
        sweep_cursor = (uint16_t)((i + 1u) & (GRID_CELL_COUNT - 1u));
        if ((v >> GRID_STATE_SHIFT) != BX72_GRID_UNKNOWN &&
            (((uint32_t)at_tick - v) & GRID_TICK_MASK) >= GRID_TTL_TICKS) {
            cells[i] = 0u;
            if ((v >> GRID_STATE_SHIFT) == BX72_GRID_OCCUPIED)
                occupied_bits[i >> 5] &= ~(1u << (i & 31u));
        }
    }
}

static int age_within(uint32_t now_us, uint32_t stamp_us, uint32_t limit_us)
{
    return (uint32_t)(now_us - stamp_us) <= limit_us;
}

void bx72_grid_reset(int32_t center_x_mm, int32_t center_y_mm, uint32_t now_us)
{
    clear_all();
    origin_x = floor_cell_i32(center_x_mm) - (int32_t)(BX72_GRID_SIZE / 2u);
    origin_y = floor_cell_i32(center_y_mm) - (int32_t)(BX72_GRID_SIZE / 2u);
    last_advance_us = tick_base_us = now_us;
    tick_now = 0u;
    initialized = 1u;
    fresh = scan_active = scan_ok = scan_has_ray = 0u;
    scan_generation = 0u;
    last_ray_stamp_us = scan_first_stamp_us = complete_scan_stamp_us = 0u;
    bump_generation();
}

void bx72_grid_advance(uint32_t now_us)
{
    uint32_t delta, ticks;
    uint8_t old_tick;
    int changed = 0;
    if (!initialized) return;
    delta = now_us - last_advance_us;
    last_advance_us = now_us;
    if (delta >= BX72_GRID_TTL_US) {
        clear_all();
        tick_base_us = now_us;
        tick_now = 0u;
        fresh = scan_ok = scan_has_ray = 0u;
        scan_generation = 0u;
        last_ray_stamp_us = complete_scan_stamp_us = 0u;
        bump_generation();
        return;
    }

    ticks = (uint32_t)(now_us - tick_base_us) / GRID_TICK_US;
    if (ticks >= GRID_TTL_TICKS) {
        /* 上次访问后加上剩余不足一个时刻的量化余数，可提前过期。 */
        clear_all();
        tick_base_us = now_us;
        tick_now = 0u;
        fresh = scan_ok = scan_has_ray = 0u;
        scan_generation = 0u;
        last_ray_stamp_us = complete_scan_stamp_us = 0u;
        bump_generation();
        return;
    }
    if (ticks != 0u) {
        old_tick = tick_now;
        sweep_expired(old_tick, ticks * 256u);
        tick_now = (uint8_t)(((uint32_t)old_tick + ticks) & GRID_TICK_MASK);
        tick_base_us += ticks * GRID_TICK_US;
        changed = 1;
    }
    if (fresh && (!age_within(now_us, complete_scan_stamp_us, BX72_GRID_FRESH_US) ||
                  !age_within(now_us, last_ray_stamp_us, BX72_GRID_FRESH_US))) {
        fresh = 0u;
        changed = 1;
    }
    if (scan_active && scan_has_ray &&
        !age_within(now_us, scan_first_stamp_us, BX72_GRID_FRESH_US))
        scan_ok = 0u;
    if (changed) bump_generation();
}

void bx72_grid_recenter(int32_t center_x_mm, int32_t center_y_mm, uint32_t now_us)
{
    int32_t new_x, new_y, dx, dy, x0, x1, x, y;
    if (!initialized) {
        bx72_grid_reset(center_x_mm, center_y_mm, now_us);
        return;
    }
    bx72_grid_advance(now_us);
    new_x = floor_cell_i32(center_x_mm) - (int32_t)(BX72_GRID_SIZE / 2u);
    new_y = floor_cell_i32(center_y_mm) - (int32_t)(BX72_GRID_SIZE / 2u);
    dx = new_x - origin_x;
    dy = new_y - origin_y;
    if (dx == 0 && dy == 0) return;
    if (dx >= 64 || dx <= -64 || dy >= 64 || dy <= -64) {
        clear_all();
        fresh = scan_ok = scan_has_ray = 0u;
        scan_generation = 0u;
        last_ray_stamp_us = complete_scan_stamp_us = 0u;
    } else {
        uint32_t mask_lo = 0u, mask_hi = 0u;
        /* 重叠行只清新增列；新增行整行清零，重叠区域原样保留。 */
        x0 = dx > 0 ? origin_x + 64 : new_x;
        x1 = dx > 0 ? new_x + 64 : origin_x;
        for (x = x0; x < x1; ++x) {
            uint32_t bit = (uint32_t)x & 63u;
            if (bit < 32u) mask_lo |= 1u << bit;
            else mask_hi |= 1u << (bit - 32u);
        }
        for (y = new_y; y < new_y + 64; ++y) {
            uint32_t row = ((uint32_t)y & 63u) * 64u, j;
            if (y < origin_y || y >= origin_y + 64) {
                for (j = 0u; j < 16u; ++j) cell_storage.words[(row >> 2) + j] = 0u;
                scan_storage.words[row >> 5] = scan_storage.words[(row >> 5) + 1u] = 0u;
                occupied_bits[row >> 5] = occupied_bits[(row >> 5) + 1u] = 0u;
            } else if (dx != 0) {
                x = x0;
                while (x < x1) {
                    uint32_t column = (uint32_t)x & 63u;
                    if ((column & 3u) == 0u && x1 - x >= 4) {
                        cell_storage.words[(row + column) >> 2] = 0u;
                        x += 4;
                    } else {
                        cells[row + column] = 0u;
                        ++x;
                    }
                }
                occupied_bits[row >> 5] &= ~mask_lo;
                occupied_bits[(row >> 5) + 1u] &= ~mask_hi;
                for (j = 0u; j < 4u; ++j) {
                    scan_hits[(row >> 3) + j] &= (uint8_t)~(mask_lo >> (j * 8u));
                    scan_hits[(row >> 3) + 4u + j] &= (uint8_t)~(mask_hi >> (j * 8u));
                }
            }
        }
    }
    origin_x = new_x;
    origin_y = new_y;
    bump_generation();
}

void bx72_grid_scan_begin(void)
{
    uint32_t i;
    if (!initialized) return;
    for (i = 0u; i < GRID_HIT_BYTES / 4u; ++i) scan_storage.words[i] = 0u;
    scan_active = scan_ok = 1u;
    scan_has_ray = 0u;
}

void bx72_grid_scan_complete(uint32_t now_us)
{
    bx72_grid_advance(now_us);
    if (!initialized) return;
    if (scan_active && scan_ok && scan_has_ray &&
        age_within(now_us, scan_first_stamp_us, BX72_GRID_FRESH_US)) {
        complete_scan_stamp_us = scan_first_stamp_us;
        fresh = 1u;
        ++scan_generation;
        if (scan_generation == 0u) ++scan_generation;
    }
    scan_active = 0u;
    bump_generation();
}

void bx72_grid_gap(void)
{
    if (!initialized) return;
    /* 保留同圈命中标记和上一完整圈原时限，直到下一有效整圈。 */
    scan_ok = 0u;
    bump_generation();
}

static void mark_cell(int32_t x, int32_t y, int hit, uint8_t stamp_tick)
{
    uint32_t i = slot(x, y);
    uint8_t mask = (uint8_t)(1u << (i & 7u));
    if (hit) {
        scan_hits[i >> 3] |= mask;
        cells[i] = (uint8_t)((BX72_GRID_OCCUPIED << GRID_STATE_SHIFT) | stamp_tick);
        occupied_bits[i >> 5] |= 1u << (i & 31u);
    } else if ((scan_hits[i >> 3] & mask) == 0u) {
        if ((cells[i] >> GRID_STATE_SHIFT) == BX72_GRID_OCCUPIED)
            occupied_bits[i >> 5] &= ~(1u << (i & 31u));
        cells[i] = (uint8_t)((BX72_GRID_FREE << GRID_STATE_SHIFT) | stamp_tick);
    }
}

static int32_t projected_mm(uint32_t distance_mm, int32_t q15)
{
    int64_t product = (int64_t)distance_mm * q15;
    product += product >= 0 ? 16384 : -16384;
    return (int32_t)(product / 32768);
}

int bx72_grid_lidar_validated(uint32_t angle_cdeg, uint32_t distance_mm,
                              uint32_t hit, const bx72_pose_t *pose,
                              uint32_t sample_stamp_us, uint32_t now_us)
{
    int64_t angle, ray_x, ray_y;
    int32_t sine, cosine, x, y, end_x, end_y, dx, dy, sx, sy, error;
    uint32_t i, age_ticks;
    uint8_t stamp_tick;
    bx72_grid_advance(now_us);
    if (!initialized || !scan_active ||
        (angle_cdeg >= 9000u && angle_cdeg <= 27000u) ||
        !hit || distance_mm < GRID_MIN_RANGE_MM || distance_mm > GRID_MAX_RANGE_MM)
        return 0;
    /* 输入位姿属于车体中心；射线始终从真实雷达原点开始。 */
    ray_x = (int64_t)pose->x_mm + bx72_lidar_offset_component(bx72_cos_q15(pose->yaw_mrad));
    ray_y = (int64_t)pose->y_mm + bx72_lidar_offset_component(bx72_sin_q15(pose->yaw_mrad));
    x = floor_cell(ray_x) - origin_x;
    y = floor_cell(ray_y) - origin_y;
    if (x < 0 || x >= 64 || y < 0 || y >= 64) return 0;
    if (scan_has_ray && (uint32_t)(sample_stamp_us - last_ray_stamp_us) >= 0x80000000u) {
        bx72_grid_gap();
        return 0;
    }
    if (!scan_has_ray) scan_first_stamp_us = sample_stamp_us;
    scan_has_ray = 1u;
    last_ray_stamp_us = sample_stamp_us;
    age_ticks = ((uint32_t)(now_us - sample_stamp_us) + GRID_TICK_US - 1u) / GRID_TICK_US;
    stamp_tick = (uint8_t)(((uint32_t)tick_now - age_ticks) & GRID_TICK_MASK);

    angle = ((int64_t)pose->yaw_mrad * 572958ll) / 100000ll - angle_cdeg;
    angle %= 36000ll;
    if (angle < 0) angle += 36000ll;
    bx72_sincos_cdeg_q15((uint32_t)angle, &sine, &cosine);
    end_x = floor_cell(ray_x + projected_mm(distance_mm, cosine)) - origin_x;
    end_y = floor_cell(ray_y + projected_mm(distance_mm, sine)) - origin_y;
    dx = end_x >= x ? end_x - x : x - end_x;
    dy = end_y >= y ? y - end_y : end_y - y;
    sx = x < end_x ? 1 : -1;
    sy = y < end_y ? 1 : -1;
    error = dx + dy;
    /* 单调步进，64x64图内最多127格；出界立即截断，不产生边界命中。 */
    for (i = 0u; i < 128u && x >= 0 && x < 64 && y >= 0 && y < 64; ++i) {
        int32_t twice;
        if (x == end_x && y == end_y) {
            mark_cell(x + origin_x, y + origin_y, 1, stamp_tick);
            break;
        }
        mark_cell(x + origin_x, y + origin_y, 0, stamp_tick);
        twice = 2 * error;
        if (twice >= dy) { error += dy; x += sx; }
        if (twice <= dx) { error += dx; y += sy; }
    }
    bump_generation();
    return 1;
}

uint8_t bx72_grid_cell_at(int32_t x, int32_t y)
{
    uint8_t value;
    if (!initialized || x < origin_x || x >= origin_x + 64 ||
        y < origin_y || y >= origin_y + 64) return BX72_GRID_OUTSIDE;
    value = cells[slot(x, y)];
    if ((((uint32_t)tick_now - value) & GRID_TICK_MASK) >= GRID_TTL_TICKS)
        return BX72_GRID_UNKNOWN;
    return (uint8_t)(value >> GRID_STATE_SHIFT);
}

uint8_t bx72_grid_cell(int32_t x_mm, int32_t y_mm)
{
    return bx72_grid_cell_at(floor_cell_i32(x_mm), floor_cell_i32(y_mm));
}

static int inside_exemption(int32_t x, int32_t y,
                            const bx72_grid_unknown_exemption_t *exemption)
{
    int64_t x0, x1, y0, y1, ax, ay, r;
    if (!exemption || exemption->radius_mm > 3200u) return 0;
    x0 = (int64_t)x * BX72_GRID_CELL_MM - exemption->x_mm;
    x1 = x0 + BX72_GRID_CELL_MM;
    y0 = (int64_t)y * BX72_GRID_CELL_MM - exemption->y_mm;
    y1 = y0 + BX72_GRID_CELL_MM;
    if (x0 < 0) x0 = -x0;
    if (x1 < 0) x1 = -x1;
    if (y0 < 0) y0 = -y0;
    if (y1 < 0) y1 = -y1;
    ax = x0 > x1 ? x0 : x1;
    ay = y0 > y1 ? y0 : y1;
    r = exemption->radius_mm;
    return ax <= r && ay <= r && ax * ax + ay * ay <= r * r;
}

int bx72_grid_unknown_blocking(int32_t x, int32_t y,
                               const bx72_grid_unknown_exemption_t *exemption)
{
    /* 邻格可通行不能证明这一格被观测；只保留起点确认圈的显式豁免。 */
    return !inside_exemption(x, y, exemption);
}

/* 分数区间裁剪，所有乘法均在小范围差值上计算，不做有损除法。 */
static int clip_axis(int64_t start, int64_t delta, int64_t lo, int64_t hi,
                     int64_t *low_n, int64_t *low_d, int64_t *high_n, int64_t *high_d)
{
    int64_t a, b, d;
    if (delta == 0) return start >= lo && start <= hi;
    if (delta > 0) { a = lo - start; b = hi - start; d = delta; }
    else { a = start - hi; b = start - lo; d = -delta; }
    if (a * *low_d > *low_n * d) { *low_n = a; *low_d = d; }
    if (b * *high_d < *high_n * d) { *high_n = b; *high_d = d; }
    return *low_n * *high_d <= *high_n * *low_d;
}

static int endpoint_near_box(int64_t x, int64_t y, int64_t left, int64_t bottom,
                             int64_t right, int64_t top, int64_t radius_squared)
{
    int64_t dx = x < left ? left - x : (x > right ? x - right : 0);
    int64_t dy = y < bottom ? bottom - y : (y > top ? y - top : 0);
    return dx * dx + dy * dy <= radius_squared;
}

static int corner_near_segment(int64_t x, int64_t y, int64_t x0, int64_t y0,
                               int64_t vx, int64_t vy, int64_t length_squared,
                               int64_t radius_squared)
{
    int64_t wx = x - x0, wy = y - y0;
    int64_t dot = wx * vx + wy * vy;
    int64_t cross;
    if (dot <= 0 || length_squared == 0)
        return wx * wx + wy * wy <= radius_squared;
    if (dot >= length_squared) {
        wx -= vx; wy -= vy;
        return wx * wx + wy * wy <= radius_squared;
    }
    cross = wx * vy - wy * vx;
    return cross * cross <= radius_squared * length_squared;
}

static int sweep_intersects_box(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                                int64_t left, int64_t bottom, int64_t r)
{
    int64_t right = left + BX72_GRID_CELL_MM, top = bottom + BX72_GRID_CELL_MM;
    int64_t vx = (int64_t)x1 - x0, vy = (int64_t)y1 - y0;
    int64_t r2 = r * r, len2 = vx * vx + vy * vy;
    int64_t low_n = 0, low_d = 1, high_n = 1, high_d = 1;
    if (endpoint_near_box(x0, y0, left, bottom, right, top, r2) ||
        endpoint_near_box(x1, y1, left, bottom, right, top, r2)) return 1;
    if (clip_axis(x0, vx, left, right, &low_n, &low_d, &high_n, &high_d) &&
        clip_axis(y0, vy, bottom, top, &low_n, &low_d, &high_n, &high_d)) return 1;
    return corner_near_segment(left, bottom, x0, y0, vx, vy, len2, r2) ||
           corner_near_segment(left, top, x0, y0, vx, vy, len2, r2) ||
           corner_near_segment(right, bottom, x0, y0, vx, vy, len2, r2) ||
           corner_near_segment(right, top, x0, y0, vx, vy, len2, r2);
}

static int sweep_inside_exemption(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                                  uint32_t radius_mm,
                                  const bx72_grid_unknown_exemption_t *exemption)
{
    int64_t dx0, dy0, dx1, dy1, allowed;
    if (!exemption || exemption->radius_mm > 3200u || radius_mm > exemption->radius_mm)
        return 0;
    allowed = exemption->radius_mm - radius_mm;
    dx0 = (int64_t)x0 - exemption->x_mm; dy0 = (int64_t)y0 - exemption->y_mm;
    dx1 = (int64_t)x1 - exemption->x_mm; dy1 = (int64_t)y1 - exemption->y_mm;
    if (dx0 < -allowed || dx0 > allowed || dy0 < -allowed || dy0 > allowed ||
        dx1 < -allowed || dx1 > allowed || dy1 < -allowed || dy1 > allowed) return 0;
    return dx0 * dx0 + dy0 * dy0 <= allowed * allowed &&
           dx1 * dx1 + dy1 * dy1 <= allowed * allowed;
}

static uint32_t check_sweep(int32_t x0_mm, int32_t y0_mm,
                            int32_t x1_mm, int32_t y1_mm, uint32_t radius_mm,
                            const bx72_grid_unknown_exemption_t *exemption)
{
    int64_t min_x, min_y, max_x, max_y, left, right, bottom, top, r, vx, vy;
    int32_t first_x, last_x, first_y, last_y, x, y;
    uint32_t result = 0u;
    int all_inside;
    vx = (int64_t)x1_mm - x0_mm;
    vy = (int64_t)y1_mm - y0_mm;
    if (!initialized || radius_mm > 3200u || vx < -3200 || vx > 3200 ||
        vy < -3200 || vy > 3200)
        return BX72_GRID_CHECK_OUTSIDE | BX72_GRID_CHECK_UNKNOWN;
    r = radius_mm;
    min_x = (x0_mm < x1_mm ? x0_mm : x1_mm) - r;
    max_x = (x0_mm > x1_mm ? x0_mm : x1_mm) + r;
    min_y = (y0_mm < y1_mm ? y0_mm : y1_mm) - r;
    max_y = (y0_mm > y1_mm ? y0_mm : y1_mm) + r;
    left = (int64_t)origin_x * BX72_GRID_CELL_MM;
    right = (int64_t)(origin_x + 64) * BX72_GRID_CELL_MM;
    bottom = (int64_t)origin_y * BX72_GRID_CELL_MM;
    top = (int64_t)(origin_y + 64) * BX72_GRID_CELL_MM;
    if (min_x <= left || max_x >= right || min_y <= bottom || max_y >= top)
        result |= BX72_GRID_CHECK_OUTSIDE;
    if (max_x < left || min_x > right || max_y < bottom || min_y > top)
        return result;
    all_inside = sweep_inside_exemption(x0_mm, y0_mm, x1_mm, y1_mm,
                                       radius_mm, exemption);
    /* 含相切的邻格；将搜索框裁到图内，循环最多4096格。 */
    first_x = min_x <= left ? origin_x : floor_cell(min_x - 1);
    last_x = max_x >= right ? origin_x + 63 : floor_cell(max_x);
    first_y = min_y <= bottom ? origin_y : floor_cell(min_y - 1);
    last_y = max_y >= top ? origin_y + 63 : floor_cell(max_y);
    for (y = first_y; y <= last_y; ++y) {
        for (x = first_x; x <= last_x; ++x) {
            uint8_t value = bx72_grid_cell_at(x, y);
            if (value == BX72_GRID_FREE ||
                (value == BX72_GRID_UNKNOWN && (all_inside ||
                                                inside_exemption(x, y, exemption)))) continue;
            if (!sweep_intersects_box(x0_mm, y0_mm, x1_mm, y1_mm,
                                      (int64_t)x * BX72_GRID_CELL_MM,
                                      (int64_t)y * BX72_GRID_CELL_MM, r)) continue;
            if (value == BX72_GRID_OCCUPIED) {
                result |= BX72_GRID_CHECK_OCCUPIED;
            } else if (value == BX72_GRID_UNKNOWN) result |= BX72_GRID_CHECK_UNKNOWN;
        }
    }
    return result;
}

uint32_t bx72_grid_check_sweep(int32_t x0_mm, int32_t y0_mm,
                               int32_t x1_mm, int32_t y1_mm, uint32_t radius_mm,
                               const bx72_grid_unknown_exemption_t *exemption)
{
    return check_sweep(x0_mm, y0_mm, x1_mm, y1_mm, radius_mm, exemption);
}

uint32_t bx72_grid_sweep_occupied(int32_t x0_mm, int32_t y0_mm,
                                  int32_t x1_mm, int32_t y1_mm,
                                  uint32_t radius_mm)
{
    int64_t dx = (int64_t)x1_mm - x0_mm, dy = (int64_t)y1_mm - y0_mm;
    int64_t r = radius_mm, min_x, max_x, min_y, max_y;
    int64_t left, right, bottom, top;
    int32_t first_x, last_x, first_y, last_y, x, y;
    if (!initialized || radius_mm > 3200u || dx < -3200 || dx > 3200 ||
        dy < -3200 || dy > 3200) return BX72_GRID_CHECK_OUTSIDE;
    min_x = (x0_mm < x1_mm ? x0_mm : x1_mm) - r;
    max_x = (x0_mm > x1_mm ? x0_mm : x1_mm) + r;
    min_y = (y0_mm < y1_mm ? y0_mm : y1_mm) - r;
    max_y = (y0_mm > y1_mm ? y0_mm : y1_mm) + r;
    left = (int64_t)origin_x * BX72_GRID_CELL_MM;
    right = (int64_t)(origin_x + 64) * BX72_GRID_CELL_MM;
    bottom = (int64_t)origin_y * BX72_GRID_CELL_MM;
    top = (int64_t)(origin_y + 64) * BX72_GRID_CELL_MM;
    if (min_x <= left || max_x >= right || min_y <= bottom || max_y >= top)
        return BX72_GRID_CHECK_OUTSIDE;
    first_x = floor_cell(min_x - 1);
    last_x = floor_cell(max_x);
    first_y = floor_cell(min_y - 1);
    last_y = floor_cell(max_y);
    for (y = first_y; y <= last_y; ++y) {
        uint32_t row = ((uint32_t)y & 63u) * BX72_GRID_SIZE;
        if ((occupied_bits[row >> 5] | occupied_bits[(row >> 5) + 1u]) == 0u)
            continue;
        for (x = first_x; x <= last_x; ++x) {
            uint8_t value = cells[row + ((uint32_t)x & 63u)];
            /* 快速筛掉未知、空闲和过期格，再做精确的整数扫掠相交。 */
            if ((value >> GRID_STATE_SHIFT) == BX72_GRID_OCCUPIED &&
                (((uint32_t)tick_now - value) & GRID_TICK_MASK) < GRID_TTL_TICKS &&
                sweep_intersects_box(x0_mm, y0_mm, x1_mm, y1_mm,
                                     (int64_t)x * BX72_GRID_CELL_MM,
                                     (int64_t)y * BX72_GRID_CELL_MM, r))
                return BX72_GRID_CHECK_OCCUPIED;
        }
    }
    return 0u;
}

static int64_t grid_abs_i64(int64_t value)
{
    return value < 0 ? -value : value;
}

/* 方向矩形沿线段的扫掠是凸多边形；方格相交使用世界轴、车体轴
 * 和线段法线的分离轴检查。坐标乘2后可精确表示方格和线段中心。 */
static uint32_t sweep_body(int32_t x0_mm, int32_t y0_mm,
                           int32_t x1_mm, int32_t y1_mm,
                           int32_t yaw_mrad, uint32_t max_yaw_delta_mrad,
                           uint32_t margin_mm, int check_unknown,
                           const bx72_grid_unknown_exemption_t *exemption)
{
    int64_t dx = (int64_t)x1_mm - x0_mm, dy = (int64_t)y1_mm - y0_mm;
    int64_t sine, cosine, abs_sine, abs_cosine, half_length_q15, half_width_q15;
    int64_t extent_x, extent_y, min_x, max_x, min_y, max_y;
    int64_t limit_x, limit_y, limit_forward, limit_side, limit_normal = 0;
    int64_t along, across;
    int32_t first_x, last_x, first_y, last_y, x, y;

    if (!initialized || margin_mm > 3200u || max_yaw_delta_mrad > 6284u ||
        dx < -3200 || dx > 3200 || dy < -3200 || dy > 3200)
        return BX72_GRID_CHECK_OUTSIDE;
    /* |sin(d)| <= |d|、|cos(d)| <= 1：各轴分别覆盖整组最大转角，
     * 即使组内角速反向也成立；1mm覆盖Q15三角量化。 */
    half_length_q15 = (BX72_GRID_BODY_HALF_LENGTH_MM + margin_mm + 1u) * 32768ll +
        (BX72_GRID_BODY_HALF_WIDTH_MM * (int64_t)max_yaw_delta_mrad * 32768 + 999) / 1000;
    half_width_q15 = (BX72_GRID_BODY_HALF_WIDTH_MM + margin_mm + 1u) * 32768ll +
        (BX72_GRID_BODY_HALF_LENGTH_MM * (int64_t)max_yaw_delta_mrad * 32768 + 999) / 1000;
    sine = bx72_sin_q15(yaw_mrad);
    cosine = bx72_cos_q15(yaw_mrad);
    abs_sine = grid_abs_i64(sine);
    abs_cosine = grid_abs_i64(cosine);
    extent_x = (half_length_q15 * abs_cosine + half_width_q15 * abs_sine + 32767) / 32768;
    extent_y = (half_length_q15 * abs_sine + half_width_q15 * abs_cosine + 32767) / 32768;
    min_x = (x0_mm < x1_mm ? x0_mm : x1_mm) - (extent_x + 32767) / 32768;
    max_x = (x0_mm > x1_mm ? x0_mm : x1_mm) + (extent_x + 32767) / 32768;
    min_y = (y0_mm < y1_mm ? y0_mm : y1_mm) - (extent_y + 32767) / 32768;
    max_y = (y0_mm > y1_mm ? y0_mm : y1_mm) + (extent_y + 32767) / 32768;
    if (min_x <= (int64_t)origin_x * BX72_GRID_CELL_MM ||
        max_x >= (int64_t)(origin_x + 64) * BX72_GRID_CELL_MM ||
        min_y <= (int64_t)origin_y * BX72_GRID_CELL_MM ||
        max_y >= (int64_t)(origin_y + 64) * BX72_GRID_CELL_MM)
        return BX72_GRID_CHECK_OUTSIDE;

    along = dx * cosine + dy * sine;
    across = -dx * sine + dy * cosine;
    limit_x = 2 * extent_x + (grid_abs_i64(dx) + BX72_GRID_CELL_MM) * 32768;
    limit_y = 2 * extent_y + (grid_abs_i64(dy) + BX72_GRID_CELL_MM) * 32768;
    limit_forward = 2 * half_length_q15 +
        BX72_GRID_CELL_MM * (abs_cosine + abs_sine) + grid_abs_i64(along);
    limit_side = 2 * half_width_q15 +
        BX72_GRID_CELL_MM * (abs_cosine + abs_sine) + grid_abs_i64(across);
    if (dx && dy)
        limit_normal = 2 * ((half_length_q15 * grid_abs_i64(across) +
            half_width_q15 * grid_abs_i64(along) + 32767) / 32768) +
            BX72_GRID_CELL_MM * (grid_abs_i64(dx) + grid_abs_i64(dy)) * 32768;
    first_x = floor_cell(min_x - 1); last_x = floor_cell(max_x);
    first_y = floor_cell(min_y - 1); last_y = floor_cell(max_y);
    for (y = first_y; y <= last_y; ++y) {
        uint32_t row = ((uint32_t)y & 63u) * BX72_GRID_SIZE;
        if (!check_unknown && (occupied_bits[row >> 5] | occupied_bits[(row >> 5) + 1u]) == 0u)
            continue;
        for (x = first_x; x <= last_x; ++x) {
            uint8_t value = cells[row + ((uint32_t)x & 63u)];
            int64_t px, py;
            if (check_unknown) {
                if (bx72_grid_cell_at(x, y) != BX72_GRID_UNKNOWN) continue;
            } else if ((value >> GRID_STATE_SHIFT) != BX72_GRID_OCCUPIED ||
                       (((uint32_t)tick_now - value) & GRID_TICK_MASK) >= GRID_TTL_TICKS) continue;
            px = (int64_t)x * (2 * BX72_GRID_CELL_MM) + BX72_GRID_CELL_MM - x0_mm - x1_mm;
            py = (int64_t)y * (2 * BX72_GRID_CELL_MM) + BX72_GRID_CELL_MM - y0_mm - y1_mm;
            if (grid_abs_i64(px) * 32768 > limit_x || grid_abs_i64(py) * 32768 > limit_y ||
                grid_abs_i64(px * cosine + py * sine) > limit_forward ||
                grid_abs_i64(-px * sine + py * cosine) > limit_side ||
                (dx && dy && grid_abs_i64(-px * dy + py * dx) * 32768 > limit_normal))
                continue;
            if (!check_unknown) return BX72_GRID_CHECK_OCCUPIED;
            if (bx72_grid_unknown_blocking(x, y, exemption)) return BX72_GRID_CHECK_UNKNOWN;
        }
    }
    return 0u;
}

uint32_t bx72_grid_sweep_body_occupied(int32_t x0_mm, int32_t y0_mm,
                                       int32_t x1_mm, int32_t y1_mm,
                                       int32_t yaw_mrad, uint32_t max_yaw_delta_mrad,
                                       uint32_t margin_mm)
{
    return sweep_body(x0_mm, y0_mm, x1_mm, y1_mm, yaw_mrad,
                      max_yaw_delta_mrad, margin_mm, 0, 0);
}

uint32_t bx72_grid_sweep_body_unknown(int32_t x0_mm, int32_t y0_mm,
                                      int32_t x1_mm, int32_t y1_mm,
                                      int32_t yaw_mrad, uint32_t max_yaw_delta_mrad,
                                      uint32_t margin_mm,
                                      const bx72_grid_unknown_exemption_t *exemption)
{
    return sweep_body(x0_mm, y0_mm, x1_mm, y1_mm, yaw_mrad,
                      max_yaw_delta_mrad, margin_mm, 1, exemption);
}

uint32_t bx72_grid_check_circle(int32_t x_mm, int32_t y_mm, uint32_t radius_mm,
                                const bx72_grid_unknown_exemption_t *exemption)
{
    return bx72_grid_check_sweep(x_mm, y_mm, x_mm, y_mm, radius_mm, exemption);
}

uint32_t bx72_grid_export(uint32_t byte_offset, uint8_t *out, uint32_t capacity)
{
    uint32_t n, i, j;
    if (!initialized || !out || byte_offset >= BX72_GRID_EXPORT_BYTES) return 0u;
    n = BX72_GRID_EXPORT_BYTES - byte_offset;
    if (n > capacity) n = capacity;
    for (i = 0u; i < n; ++i) {
        uint32_t first = (byte_offset + i) * 4u;
        uint32_t row = ((uint32_t)origin_y + (first >> 6)) & 63u;
        uint32_t column = ((uint32_t)origin_x + (first & 63u)) & 63u;
        uint8_t packed = 0u;
        for (j = 0u; j < 4u; ++j) {
            uint8_t value = cells[row * 64u + ((column + j) & 63u)];
            if ((((uint32_t)tick_now - value) & GRID_TICK_MASK) >= GRID_TTL_TICKS)
                value = 0u;
            packed |= (uint8_t)((value >> GRID_STATE_SHIFT) << (2u * j));
        }
        out[i] = packed;
    }
    return n;
}

void bx72_grid_info(bx72_grid_info_t *out)
{
    if (!out) return;
    out->origin_x_cell = origin_x;
    out->origin_y_cell = origin_y;
    out->generation = generation;
    out->scan_generation = scan_generation;
    out->last_ray_stamp_us = last_ray_stamp_us;
    out->complete_scan_stamp_us = complete_scan_stamp_us;
    out->fresh = fresh;
}
