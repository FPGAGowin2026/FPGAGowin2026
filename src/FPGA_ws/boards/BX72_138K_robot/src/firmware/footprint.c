#include "footprint.h"
#include "local_map.h"

#define Q15_ONE 32768
#define HALF_CELL_MM (LOCAL_MAP_CELL_MM / 2)

static int64_t abs64(int64_t value)
{
    return value < 0 ? -value : value;
}

int footprint_orientation_valid(int32_t cos_q15, int32_t sin_q15)
{
    int64_t norm_squared;
    if (cos_q15 < -Q15_ONE || cos_q15 > Q15_ONE ||
        sin_q15 < -Q15_ONE || sin_q15 > Q15_ONE) return 0;
    norm_squared = (int64_t)cos_q15 * cos_q15 + (int64_t)sin_q15 * sin_q15;
    return norm_squared >= (int64_t)Q15_ONE * Q15_ONE * 999 / 1000 &&
           norm_squared <= (int64_t)Q15_ONE * Q15_ONE * 1001 / 1000;
}

int footprint_cell_intersects(int32_t cell_x, int32_t cell_y,
                              int32_t center_x_mm, int32_t center_y_mm,
                              int32_t cos_q15, int32_t sin_q15,
                              int32_t half_length_mm, int32_t half_width_mm)
{
    int64_t dx = (int64_t)cell_x * LOCAL_MAP_CELL_MM + HALF_CELL_MM - center_x_mm;
    int64_t dy = (int64_t)cell_y * LOCAL_MAP_CELL_MM + HALF_CELL_MM - center_y_mm;
    int64_t c = cos_q15, s = sin_q15;
    int64_t ac = abs64(c), as = abs64(s);
    int64_t projection = HALF_CELL_MM * (ac + as);
    if (abs64(dx * c + dy * s) > (int64_t)half_length_mm * Q15_ONE + projection ||
        abs64(-dx * s + dy * c) > (int64_t)half_width_mm * Q15_ONE + projection)
        return 0;
    if (abs64(dx) * Q15_ONE > (int64_t)half_length_mm * ac +
        (int64_t)half_width_mm * as + HALF_CELL_MM * Q15_ONE ||
        abs64(dy) * Q15_ONE > (int64_t)half_length_mm * as +
        (int64_t)half_width_mm * ac + HALF_CELL_MM * Q15_ONE)
        return 0;
    return 1;
}

int footprint_cell_inside(int32_t cell_x, int32_t cell_y,
                          int32_t center_x_mm, int32_t center_y_mm,
                          int32_t cos_q15, int32_t sin_q15,
                          int32_t half_length_mm, int32_t half_width_mm)
{
    int64_t dx = (int64_t)cell_x * LOCAL_MAP_CELL_MM + HALF_CELL_MM - center_x_mm;
    int64_t dy = (int64_t)cell_y * LOCAL_MAP_CELL_MM + HALF_CELL_MM - center_y_mm;
    int64_t c = cos_q15, s = sin_q15;
    int64_t projection = HALF_CELL_MM * (abs64(c) + abs64(s));
    if (half_length_mm <= 0 || half_width_mm <= 0) return 0;
    return abs64(dx * c + dy * s) + projection <= (int64_t)half_length_mm * Q15_ONE &&
           abs64(-dx * s + dy * c) + projection <= (int64_t)half_width_mm * Q15_ONE;
}
