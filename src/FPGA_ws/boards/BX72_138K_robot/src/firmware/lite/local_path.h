#ifndef BX72_LITE_LOCAL_PATH_H
#define BX72_LITE_LOCAL_PATH_H

#include <stdint.h>

/* 单实例、单线程的有界局部搜索。目标和输出均为局部栅格系中的毫米。
 * UNKNOWN 只增加规划代价，并不表示安全；执行层仍须用实时地图复核。 */
enum bx72_local_path_state {
    BX72_PATH_IDLE = 0,
    BX72_PATH_WORKING = 1,
    BX72_PATH_FOUND = 2,
    BX72_PATH_BLOCKED = 3
};

void bx72_local_path_reset(void);
void bx72_local_path_begin(int32_t start_x_mm, int32_t start_y_mm,
                           int32_t goal_x_mm, int32_t goal_y_mm);
/* 小坐标修正未跨目标格时保留搜索进度，并更新最终精确目标。 */
int bx72_local_path_retarget(int32_t goal_x_mm, int32_t goal_y_mm);
/* 工作中的搜索能否以当前固定目标作为最终点。 */
int bx72_local_path_goal_inside(void);
void bx72_local_path_step(uint32_t (*clock_us)(void), uint32_t budget_us);
enum bx72_local_path_state bx72_local_path_state(void);
/* 返回局部搜索的中间引导点；final 表示原目标所在格，仍须实时复核。 */
int bx72_local_path_target(int32_t *x_mm, int32_t *y_mm, int *final);

#endif
