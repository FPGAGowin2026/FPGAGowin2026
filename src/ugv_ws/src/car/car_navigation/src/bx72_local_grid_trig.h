#ifndef CAR_NAVIGATION_BX72_LOCAL_GRID_TRIG_H
#define CAR_NAVIGATION_BX72_LOCAL_GRID_TRIG_H

#include <stdint.h>

int32_t bx72_grid_sin_q15(int32_t angle_mrad);
int32_t bx72_grid_cos_q15(int32_t angle_mrad);
void bx72_grid_sincos_cdeg_q15(uint32_t angle_cdeg,
                               int32_t *sine_q15, int32_t *cosine_q15);

#endif
