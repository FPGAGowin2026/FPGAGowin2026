/* 在独立翻译单元中装载现场栅格，碰撞检查仍调用当前生产代码。 */
#include "local_grid.c"
#include <stdio.h>

int load_recorded_grid(int origin_x_in, int origin_y_in)
{
    uint32_t x, y, value, index;
    bx72_grid_reset(0, 0, 0);
    origin_x = origin_x_in;
    origin_y = origin_y_in;
    for (y = 0; y < 64u; ++y) for (x = 0; x < 64u; ++x) {
        if (scanf("%u", &value) != 1 || value > 2u) return 0;
        index = slot(origin_x + (int32_t)x, origin_y + (int32_t)y);
        cells[index] = (uint8_t)(value << GRID_STATE_SHIFT);
        if (value == BX72_GRID_OCCUPIED) occupied_bits[index >> 5] |= 1u << (index & 31u);
    }
    return 1;
}
