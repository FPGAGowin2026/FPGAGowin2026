/* 编译 FPGA 唯一的路径搜索实现，接入 Jetson 的同一个实时栅格实例。
 * 两端射线输入的位姿类型不同；路径搜索只使用共用的 grid 信息与格值 API。 */
#include "local_grid.h"
#define BX72_LITE_LOCAL_GRID_H
#include <local_path.c>
