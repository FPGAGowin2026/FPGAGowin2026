#ifndef CAR_NAVIGATION_BX72_LOCAL_GRID_H
#define CAR_NAVIGATION_BX72_LOCAL_GRID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 从 FPGA 局部栅格提炼，供 Jetson 后续接入；本库不启动节点或控制电机。
 * C ABI 的 libbx72_local_grid.so 安装在 car_navigation 包安装前缀的 lib/，
 * C/C++ 可通过本头文件调用，Python 可使用 ctypes 加载。 */
typedef struct {
    int32_t x_mm, y_mm, yaw_mrad;
} bx72_grid_pose_t;

#define BX72_GRID_SIZE 64u
#define BX72_GRID_CELL_MM 50u
/* 以下几何参数与当前 FPGA 候选一致；接入时须与实际安装及电机配置核对。 */
#define BX72_GRID_LIDAR_FORWARD_OFFSET_MM 130u
#define BX72_GRID_BODY_LENGTH_MM 260u
#define BX72_GRID_BODY_WIDTH_MM 250u
#define BX72_GRID_BODY_HALF_LENGTH_MM (BX72_GRID_BODY_LENGTH_MM / 2u)
#define BX72_GRID_BODY_HALF_WIDTH_MM (BX72_GRID_BODY_WIDTH_MM / 2u)
#define BX72_GRID_BODY_RADIUS_MM 181u
#define BX72_GRID_START_CONFIRM_RADIUS_MM 231u
#define BX72_GRID_TTL_US 5000000u
#define BX72_GRID_FRESH_US 1500000u
#define BX72_GRID_EXPORT_BYTES 1024u

/* 轴对齐、左下到右上；越界始终是未知，不能当作障碍或空闲。 */
#define BX72_GRID_UNKNOWN 0u
#define BX72_GRID_FREE 1u
#define BX72_GRID_OCCUPIED 2u
#define BX72_GRID_OUTSIDE 3u

#define BX72_GRID_CHECK_OCCUPIED (1u << 0)
#define BX72_GRID_CHECK_UNKNOWN (1u << 1)
#define BX72_GRID_CHECK_OUTSIDE (1u << 2)

typedef struct {
    int32_t x_mm, y_mm;
    uint32_t radius_mm;
} bx72_grid_unknown_exemption_t;

typedef struct {
    /* 左下角的世界格坐标，实际边界 = 坐标 * 50 mm。 */
    int32_t origin_x_cell, origin_y_cell;
    uint32_t generation, scan_generation;
    uint32_t last_ray_stamp_us, complete_scan_stamp_us;
    uint8_t fresh;
} bx72_grid_info_t;

/* 全局单实例、不可并发调用；输入的位姿始终属于连续、世界轴对齐的局部图坐标系；它不一定等于
 * 经CSM修正后的odom。初始化/坐标域更换时reset；每次控制前调用
 * recenter/advance。时间戳为同一32位微秒时基，调用间隔须小于一次
 * 32位计数回绕；长暂停会令已有观测过期。没有动态分配。 */
void bx72_grid_reset(int32_t center_x_mm, int32_t center_y_mm, uint32_t now_us);
void bx72_grid_recenter(int32_t center_x_mm, int32_t center_y_mm, uint32_t now_us);
void bx72_grid_advance(uint32_t now_us);

/* 调用方检测旋转和缺包：每圈开始时begin，完整无缺口的整圈结束时
 * complete，丢包/解析或时间错误时gap。gap使构建中的圈无效，最近
 * 完整圈按原采样时限自然过期，不因gap或不完整圈续期。 */
void bx72_grid_scan_begin(void);
void bx72_grid_scan_complete(uint32_t now_us);
void bx72_grid_gap(void);

/* 调用方须验证角度和采样时间；逐束实时输入应使用该束采样时刻的有效位姿，
 * 采样不晚于now且年龄不超过120ms。不得把到包时间称作采样时间。
 * 当前 ROS /scan 在整圈完成后发布，Jetson 节点限制整圈起始时标不超过 500ms，
 * 以起始位姿近似全部点；运动期间有畸变，不等价于原逐束 120ms 输入。
 * 本库不自行做时标验证或运动补偿，使用这种近似前须验证其适用范围。
 * 位姿在连续图坐标系中，角度顺时针、航向逆时针。无回波、超出地图
 * 量程、后方90..270度和图外原点不画射线。同圈命中优先；返回1
 * 表示接收了一条前向有效射线。 */
int bx72_grid_lidar_validated(uint32_t angle_cdeg, uint32_t distance_mm,
                              uint32_t hit, const bx72_grid_pose_t *pose_at_sample,
                              uint32_t sample_stamp_us, uint32_t now_us);

uint8_t bx72_grid_cell(int32_t x_mm, int32_t y_mm);
uint8_t bx72_grid_cell_at(int32_t world_x_cell, int32_t world_y_cell);
/* 仅对已判定为UNKNOWN的格调用；仅显式确认的固定起点圆可以豁免。 */
int bx72_grid_unknown_blocking(int32_t world_x_cell, int32_t world_y_cell,
                               const bx72_grid_unknown_exemption_t *exemption);

/* 返回上述检查位的或；保守检查圆与每个格方块的交集。exemption由
 * 导航模块指定，仅当未知格全部位于起点圆内，或整次待查扫掠圆
 * 都位于起点圆内时豁免；占用和越界永不豁免。NULL表示无豁免。
 * 前方可观测通道是否充足由导航层判断；本接口不会把未知改为空闲。
 * 半径上限3200 mm，扫掠两端在各轴的差各不超过3200 mm。 */
uint32_t bx72_grid_check_circle(int32_t x_mm, int32_t y_mm, uint32_t radius_mm,
                                const bx72_grid_unknown_exemption_t *exemption);
uint32_t bx72_grid_check_sweep(int32_t x0_mm, int32_t y0_mm,
                               int32_t x1_mm, int32_t y1_mm, uint32_t radius_mm,
                               const bx72_grid_unknown_exemption_t *exemption);
/* 仅检测占用和越界；未知的通行性必须另由导航的前向观测通道判断。
 * 空闲/未知格跳过几何计算，适用于控制循环中的少量候选扫掠。 */
uint32_t bx72_grid_sweep_occupied(int32_t x0_mm, int32_t y0_mm,
                                  int32_t x1_mm, int32_t y1_mm,
                                  uint32_t radius_mm);

/* 以yaw_mrad为参考朝向沿线段扫掠；max_yaw_delta_mrad是整段内
 * 相对参考航向的最大偏差，margin_mm覆盖轨迹和量化误差。只检查
 * 已知占用和越界，未知通行性仍由导航模块另行判断。 */
uint32_t bx72_grid_sweep_body_occupied(int32_t x0_mm, int32_t y0_mm,
                                       int32_t x1_mm, int32_t y1_mm,
                                       int32_t yaw_mrad, uint32_t max_yaw_delta_mrad,
                                       uint32_t margin_mm);
/* 同一车体扫掠内的未观测格；调用方仍需单独检查已知占用。 */
uint32_t bx72_grid_sweep_body_unknown(int32_t x0_mm, int32_t y0_mm,
                                      int32_t x1_mm, int32_t y1_mm,
                                      int32_t yaw_mrad, uint32_t max_yaw_delta_mrad,
                                      uint32_t margin_mm,
                                      const bx72_grid_unknown_exemption_t *exemption);

/* 每字节编码4格、低位先行，按y再x排列：00未知、01空闲、10占用。
 * 单线程分片导出时，调用方应在前后检查generation相同。 */
uint32_t bx72_grid_export(uint32_t byte_offset, uint8_t *out, uint32_t capacity);
void bx72_grid_info(bx72_grid_info_t *out);

#ifdef __cplusplus
}
#endif
#endif
