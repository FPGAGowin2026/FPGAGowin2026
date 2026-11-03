#include "local_path.h"
#include "local_grid.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Fixed world-coordinate fixture; origin may move while overlapping cells persist. */
static uint8_t cells[BX72_GRID_SIZE][BX72_GRID_SIZE];
static int32_t origin_x, origin_y;
static uint32_t fake_time;

void bx72_grid_info(bx72_grid_info_t *out)
{
    memset(out, 0, sizeof(*out));
    out->origin_x_cell = origin_x;
    out->origin_y_cell = origin_y;
    out->fresh = 1u;
}

uint8_t bx72_grid_cell_at(int32_t x, int32_t y)
{
    if (x < origin_x || x >= origin_x + 64 || y < origin_y || y >= origin_y + 64 ||
        x < -32 || x >= 32 || y < -32 || y >= 32) return BX72_GRID_OUTSIDE;
    return cells[y + 32][x + 32];
}

static uint32_t clock_us(void)
{
    fake_time += 10u;
    return fake_time;
}

static void fixture(uint8_t value)
{
    memset(cells, value, sizeof(cells));
    origin_x = origin_y = -32;
    fake_time = 0u;
    bx72_local_path_reset();
}

static void run_search(void)
{
    unsigned steps;
    for (steps = 0u; steps < 10000u && bx72_local_path_state() == BX72_PATH_WORKING; ++steps)
        bx72_local_path_step(clock_us, 500u);
    assert(bx72_local_path_state() != BX72_PATH_WORKING);
}

static void target(int32_t *x, int32_t *y, int *final)
{
    assert(bx72_local_path_state() == BX72_PATH_FOUND);
    assert(bx72_local_path_target(x, y, final));
}

int main(void)
{
    int32_t x, y;
    int final, row;

    fixture(BX72_GRID_FREE);
    bx72_local_path_begin(0, 0, 400, 0);
    run_search();
    target(&x, &y, &final);
    assert(final && x == 400 && y == 0);
    assert(bx72_local_path_retarget(420, 0));
    target(&x, &y, &final);
    assert(final && x == 420 && y == 0);
    assert(!bx72_local_path_retarget(450, 0));

    fixture(BX72_GRID_FREE);
    cells[32][36] = BX72_GRID_UNKNOWN;
    bx72_local_path_begin(0, 0, 400, 0);
    run_search();
    target(&x, &y, &final);
    assert(!final && y != 0);

    fixture(BX72_GRID_FREE);
    cells[32][40] = BX72_GRID_OCCUPIED;
    bx72_local_path_begin(0, 0, 400, 0);
    run_search();
    assert(bx72_local_path_state() == BX72_PATH_BLOCKED);

    fixture(BX72_GRID_FREE);
    for (row = -20; row <= 20; ++row) cells[row + 32][36] = BX72_GRID_OCCUPIED;
    bx72_local_path_begin(0, 0, 600, 0);
    run_search();
    assert(bx72_local_path_state() == BX72_PATH_BLOCKED);

    fixture(BX72_GRID_FREE);
    for (row = -4; row <= 4; ++row) cells[row + 32][36] = BX72_GRID_OCCUPIED;
    bx72_local_path_begin(0, 0, 600, 0);
    run_search();
    target(&x, &y, &final);
    assert(!final && (x < 0 || y != 0));

    /* 350mm 已知通道允许沿车体长轴通过，外接圆首轮无路时仍需给出候选。 */
    fixture(BX72_GRID_FREE);
    for (row = -28; row <= 28; ++row) {
        cells[28][row + 32] = BX72_GRID_OCCUPIED;
        cells[36][row + 32] = BX72_GRID_OCCUPIED;
    }
    bx72_local_path_begin(0, 25, 600, 25);
    run_search();
    target(&x, &y, &final);
    assert(x > 0 && y == 25);

    fixture(BX72_GRID_FREE);
    bx72_local_path_begin(0, 0, 3000, 0);
    run_search();
    target(&x, &y, &final);
    assert(!final && x > 0);

    fixture(BX72_GRID_UNKNOWN);
    bx72_local_path_begin(0, 0, 400, 0);
    run_search();
    target(&x, &y, &final);
    assert(!final && cells[32][36] == BX72_GRID_UNKNOWN);

    fixture(BX72_GRID_FREE);
    bx72_local_path_begin(-5, -5, -400, -5);
    bx72_local_path_step(clock_us, 500u);
    ++origin_x;
    run_search();
    target(&x, &y, &final);
    assert(final && x == -400 && y == -5);

    fixture(BX72_GRID_FREE);
    bx72_local_path_begin(0, 0, 400, 0);
    bx72_local_path_step(clock_us, 500u);
    origin_x += 3;
    bx72_local_path_step(clock_us, 500u);
    assert(bx72_local_path_state() == BX72_PATH_BLOCKED);

    puts("local path search, obstacle, frontier, unknown and rebase cases passed");
    return 0;
}
