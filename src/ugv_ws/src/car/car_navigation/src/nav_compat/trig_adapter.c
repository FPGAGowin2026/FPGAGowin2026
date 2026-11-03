#include "bx72_lite.h"
#include "../bx72_local_grid_trig.h"

/* 规划器和局部地图复用同一套定点三角函数。 */
int32_t bx72_sin_q15(int32_t angle_mrad) { return bx72_grid_sin_q15(angle_mrad); }
int32_t bx72_cos_q15(int32_t angle_mrad) { return bx72_grid_cos_q15(angle_mrad); }
void bx72_sincos_cdeg_q15(uint32_t angle_cdeg, int32_t *sine, int32_t *cosine)
{
    bx72_grid_sincos_cdeg_q15(angle_cdeg, sine, cosine);
}
