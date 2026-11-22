#ifndef ROBOT_FOOTPRINT_H
#define ROBOT_FOOTPRINT_H

#include <stdint.h>

int footprint_orientation_valid(int32_t cos_q15, int32_t sin_q15);
int footprint_cell_intersects(int32_t cell_x, int32_t cell_y,
                              int32_t center_x_mm, int32_t center_y_mm,
                              int32_t cos_q15, int32_t sin_q15,
                              int32_t half_length_mm, int32_t half_width_mm);
int footprint_cell_inside(int32_t cell_x, int32_t cell_y,
                          int32_t center_x_mm, int32_t center_y_mm,
                          int32_t cos_q15, int32_t sin_q15,
                          int32_t half_length_mm, int32_t half_width_mm);

#endif
