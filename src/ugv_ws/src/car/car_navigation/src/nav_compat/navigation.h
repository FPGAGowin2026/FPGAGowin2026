#ifndef BX72_LITE_NAVIGATION_H
#define BX72_LITE_NAVIGATION_H

#include "bx72_lite.h"
#include "local_grid.h"

/* BX72_IO_NAV_STATUS的低8位仍沿用任务状态。 */
#define BX72_NAV_STATUS_IDLE       0u
#define BX72_NAV_STATUS_RECEIVING  1u
#define BX72_NAV_STATUS_PLANNING   2u
#define BX72_NAV_STATUS_FOLLOWING  3u
#define BX72_NAV_STATUS_ARRIVED    4u
#define BX72_NAV_STATUS_BLOCKED    5u
#define BX72_NAV_STATUS_REJECTED   6u
#define BX72_NAV_STATUS_CANCELLED  7u
#define BX72_NAV_STATUS_STALE      8u
#define BX72_NAV_STATUS_WAIT_AUTO  9u
#define BX72_NAV_TASK_QUEUE_SIZE   4u
#define BX72_LIDAR_SAMPLE_MAX_AGE_US 120000u

/* 高位为本版本的导航诊断；bit15区分旧极坐标固件。 */
#define BX72_NAV_DIAG_MAP_FRESH        (1u << 8)
#define BX72_NAV_DIAG_MAP_READY        (1u << 9)
#define BX72_NAV_DIAG_GAP_LATCHED      (1u << 10)
#define BX72_NAV_DIAG_PLANNING         (1u << 11)
#define BX72_NAV_DIAG_PROGRESS_BLOCKED (1u << 12)
#define BX72_NAV_DIAG_UNKNOWN_PATH     (1u << 13)
#define BX72_NAV_DIAG_RECOVERY_REQUIRED (1u << 14)
#define BX72_NAV_DIAG_GRID_V1          (1u << 15)
#define BX72_NAV_DIAG_GAP_REASON_SHIFT 16u
#define BX72_NAV_DIAG_START_LOST       (1u << 21)

/* 最近一次输入缺口原因；完整新圈恢复后保留，便于实板诊断。 */
#define BX72_NAV_GAP_NONE              0u
#define BX72_NAV_GAP_FIFO              1u
#define BX72_NAV_GAP_UPSTREAM          2u
#define BX72_NAV_GAP_STALE             3u
#define BX72_NAV_GAP_POSE              4u
#define BX72_NAV_GAP_ANGLE             5u
#define BX72_NAV_GAP_INVALID           6u
#define BX72_NAV_GAP_RESET             7u

/* 进程内航点接口。ROS 服务入口负责参数、会话和授权校验；这里保留
 * 任务水位、有效期、地图重置屏障和队列状态。坐标和角度使用米/弧度 Q16。 */
typedef struct {
    uint32_t action, mode, flags, task_id;
    int32_t goal_x_q16, goal_y_q16, hold_yaw_q16, final_yaw_q16;
    uint32_t max_speed_mm_s, max_accel_mm_s2, tolerance_mm, yaw_tolerance_mrad;
    uint32_t sent_at_us, expiry_us;
} bx72_nav_task_request_t;

/* replace_existing 在任务通过水位/时间校验后原子地替换活动任务。 */
int bx72_nav_submit_task(const bx72_nav_task_request_t *request, uint32_t now_us,
                         int replace_existing);

/* 一次性冻结的局部图及其odom到栅格坐标变换；每格的2位顺序见local_grid.h。 */
typedef struct {
    uint32_t boot_id, odom_domain;
    uint32_t generation, scan_generation;
    int32_t origin_x_cell, origin_y_cell;
    uint32_t complete_scan_stamp_us, last_ray_stamp_us;
    int32_t map_anchor_x_mm, map_anchor_y_mm;
    int32_t odom_anchor_x_mm, odom_anchor_y_mm, map_yaw_offset_mrad;
    uint32_t grid_frame_epoch, transform_revision, flags;
} bx72_nav_grid_snapshot_t;

/* 单线程调用，成功时1024字节与元数据属于同一帧；截止前不足1ms放弃整帧。 */
int bx72_nav_grid_snapshot(bx72_nav_grid_snapshot_t *snapshot,
                           uint8_t cells[BX72_GRID_EXPORT_BYTES], uint32_t deadline_us);

/* 仅接收入口已验证的样本：顺时针角度小于36000，时间不晚于now。
 * 原 FPGA 逐束接口使用120ms上限和逐束位姿；当前 ROS 整圈适配使用
 * 起始时刻及其近似位姿、500ms上限，不具有逐束精度。无回波或超出
 * 地图量程保留为未知，不视为扫描缺口。 */
void bx72_nav_lidar_validated(uint32_t angle_cdeg, uint32_t distance_mm, uint32_t hit,
                              const bx72_pose_t *pose, uint32_t sample_stamp_us,
                              uint32_t now_us);
void bx72_nav_lidar_gap(uint32_t now_us, uint32_t reason);
/* 无条件清空任务，用于网桥失联、撤权和重启。 */
void bx72_nav_force_cancel(uint32_t now_us);
/* 网桥报告人工或其他非导航运动来源接管时撤销旧任务和固定起点许可。 */
void bx72_nav_external_takeover(uint32_t now_us);
/* CSM将整段融合历史做同一个SE(2)校正；用同一时刻前后位姿保持图坐标连续。 */
void bx72_nav_rebase(const bx72_pose_t *old_odom, const bx72_pose_t *new_odom);
/* 无法建立刚体校正时，废弃旧图和任务；新任务须在重置后发送，
 * 且新扫描及当前位姿恢复后才能重新确认固定起点区域。 */
void bx72_nav_map_reset(const bx72_pose_t *pose, uint32_t now_us);

#endif
