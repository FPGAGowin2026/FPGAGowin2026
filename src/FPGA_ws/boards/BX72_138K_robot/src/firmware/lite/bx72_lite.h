#ifndef BX72_LITE_H
#define BX72_LITE_H
#include <stdint.h>

/* 雷达原点在车体几何中心前方，方向与车体坐标轴一致。 */
#define BX72_LIDAR_FORWARD_OFFSET_MM 130u
#define BX72_BODY_LENGTH_MM 260u
#define BX72_BODY_WIDTH_MM 250u

/* 按车体航向分解雷达偏移，返回整数毫米。 */
static inline int32_t bx72_lidar_offset_component(int32_t q15)
{
    int64_t value = (int64_t)BX72_LIDAR_FORWARD_OFFSET_MM * q15;
    return (int32_t)((value + (value >= 0 ? 16384 : -16384)) / 32768);
}

/* 主线程维护的软硬件合同。地址均为 0xf0000000 的字节偏移。 */
#define BX72_IO_BASE 0xf0000000u
#define BX72_IO_NOW 0x100u
#define BX72_IO_STATUS 0x104u
#define BX72_IO_SNAPSHOT 0x108u
#define BX72_IO_IMU_SEQ 0x110u
#define BX72_IO_GX_GY 0x114u
#define BX72_IO_GZ_AX 0x118u
#define BX72_IO_AY_AZ 0x11cu
#define BX72_IO_WHEEL_SEQ 0x120u
#define BX72_IO_VX 0x124u
#define BX72_IO_VY 0x128u
#define BX72_IO_WZ 0x12cu
#define BX72_IO_WHEEL_STAMP 0x130u /* 与轮速快照同批的采集时间，us。 */
#define BX72_IO_IMU_STAMP 0x134u /* 与IMU快照同批的采集时间，us。 */
#define BX72_IO_LIDAR_HEAD 0x140u /* 建图与CSM共用5度最近回波； 高16位角度(0.01度)，低16位距离(mm)。 */
#define BX72_IO_LIDAR_META 0x144u /* bit0可读，bit1有效回波，bit15:8强度。 */
#define BX72_IO_LIDAR_POP 0x148u
#define BX72_IO_LIDAR_STAMP 0x14cu /* 当前队首采集时间，us；pop前保持稳定。 */
#define BX72_IO_LIDAR_DROPS 0x150u
#define BX72_IO_LIDAR_ERRORS 0x154u /* CRC/串口/跨域错误，扫描纠偏据此中止不完整输入。 */
#define BX72_IO_LIDAR_RAW_COUNT 0x158u
#define BX72_IO_LIDAR_REDUCED_COUNT 0x15cu
#define BX72_IO_TASK_META 0x160u /* bit0可读；8字依次为fragment[255:224]至[31:0]。 */
#define BX72_IO_TASK_WORD0 0x164u
#define BX72_IO_TASK_POP 0x184u /* 写入读取分片时的TASK_EPOCH，切换后的新队首不会被误弹出。 */
#define BX72_IO_TASK_OVERFLOW_ACK 0x188u /* 清除已丢弃任务的溢出，须先提交零速。 */
#define BX72_IO_TASK_EPOCH 0x18cu /* 控制来源切换或任务溢出时递增。 */
#define BX72_IO_COMMAND_SEQ 0x190u
#define BX72_IO_COMMAND_XY 0x194u /* vx为高16位，vy为低16位，单位mm/s。 */
#define BX72_IO_COMMAND_WZ 0x198u /* 低16位，单位mrad/s。 */
#define BX72_IO_MODE_SEQ 0x19cu
#define BX72_IO_BOOT_ID 0x1a0u
#define BX72_IO_ODOM_DOMAIN 0x1a4u
#define BX72_IO_SESSION_LO 0x1a8u
#define BX72_IO_SESSION_HI 0x1acu
#define BX72_IO_MANUAL_MOTION_SEQ 0x1b0u /* 与采集快照同批，仅非零人工命令递增。 */
#define BX72_IO_OUT_VX 0x200u
#define BX72_IO_OUT_VY 0x204u
#define BX72_IO_OUT_WZ 0x208u
#define BX72_IO_OUT_COMMIT 0x20cu
#define BX72_IO_POSE_X 0x210u
#define BX72_IO_POSE_Y 0x214u
#define BX72_IO_POSE_YAW 0x218u
#define BX72_IO_POSE_ROLL 0x21cu
#define BX72_IO_POSE_PITCH 0x220u
#define BX72_IO_POSE_FLAGS 0x224u
#define BX72_IO_POSE_STAMP 0x228u
#define BX72_IO_POSE_COMMIT 0x22cu
#define BX72_IO_NAV_STATUS 0x230u
#define BX72_IO_NAV_TASK 0x234u
#define BX72_IO_NAV_PROGRESS 0x238u
#define BX72_IO_NAV_COMMIT 0x23cu
#define BX72_IO_COV_DIAG0 0x240u /* polar-v2: 前3字为误差估计float，字3/4为raw/reduced计数uint32。 */
#define BX72_IO_GYRO_BIAS 0x25cu /* mrad/s，有符号。 */
#define BX72_IO_EKF_VX 0x260u
#define BX72_IO_EKF_VY 0x264u
#define BX72_IO_EKF_WZ 0x268u
#define BX72_IO_MAX_CONTROL_GAP 0x26cu
#define BX72_IO_CSM_ACCEPTED 0x270u
#define BX72_IO_CSM_REJECTED 0x274u
#define BX72_IO_CSM_STATUS 0x278u
#define BX72_IO_SELFTEST 0x27cu /* bit31已执行，低31位故障掩码；完整状态不等于实车验收。 */
#define BX72_IO_NAV_GOAL_X 0x284u /* 米Q16，与任务线协议一致。 */
#define BX72_IO_NAV_GOAL_Y 0x288u
#define BX72_IO_NAV_EXPIRY 0x28cu /* 本次会话本地微秒时基，0表示无活动任务。 */
#define BX72_IO_MAP_VERSION 0x290u
#define BX72_IO_SEARCH_EXPANSIONS 0x294u
#define BX72_IO_MAP_AGE 0x298u
#define BX72_IO_LAST_TASK_ID 0x29cu
#define BX72_IO_OUT_AUTONOMOUS 0x2a0u
#define BX72_IO_OUT_COMMAND_SEQ 0x2a4u
#define BX72_IO_OUT_MODE_SEQ 0x2a8u
#define BX72_IO_NAV_MODE 0x2acu /* 1人工、2协同任务、3独立任务。 */
#define BX72_IO_PLAN_STAMP 0x2b0u
#define BX72_IO_PLAN_START_X 0x2b4u
#define BX72_IO_PLAN_START_Y 0x2b8u
#define BX72_IO_PLAN_END_X 0x2bcu
#define BX72_IO_PLAN_END_Y 0x2c0u
#define BX72_IO_PLAN_VALID 0x2c4u
#define BX72_IO_GRID_WORD0 0x600u /* 20字元数据与32字栅格，每片共52字。 */
#define BX72_IO_GRID_COMMIT 0x6d0u
#define BX72_IO_GRID_READY 0x6d4u
#define BX72_COMMIT_MAGIC 0x42583732u

#define BX72_STATUS_LINK_DOWN (1u<<0)
#define BX72_STATUS_IMU_OK (1u<<1)
#define BX72_STATUS_LIDAR_OK (1u<<2)
#define BX72_STATUS_GUARD_STOP (1u<<3)
#define BX72_STATUS_WHEEL_OK (1u<<4)
#define BX72_STATUS_TASK (1u<<5)
#define BX72_STATUS_LIDAR_DATA (1u<<6)
#define BX72_STATUS_AUTO (1u<<7)
#define BX72_POSE_VALID 1u

typedef struct {
    int32_t x_mm,y_mm,yaw_mrad,roll_mrad,pitch_mrad;
    int32_t vx_mm_s,vy_mm_s,wz_mrad_s;
    uint32_t stamp_us,flags;
} bx72_pose_t;
typedef struct {
    int32_t vx_mm_s,vy_mm_s,wz_mrad_s;
    uint32_t status,task_id,progress;
    uint32_t expiry_us;
    int32_t goal_x_q16,goal_y_q16;
    uint32_t planner_expansions,map_age_us,map_version,last_task_id,autonomous,task_mode;
    uint32_t plan_stamp_us,plan_valid;
    int32_t plan_start_x_mm,plan_start_y_mm,plan_end_x_mm,plan_end_y_mm;
} bx72_nav_output_t;

/* A线程提供并维护，B线程调用。正弦/余弦返回Q15，角度mrad。 */
int32_t bx72_sin_q15(int32_t angle_mrad);
int32_t bx72_cos_q15(int32_t angle_mrad);
void bx72_sincos_cdeg_q15(uint32_t angle_cdeg,
                         int32_t *sine_q15, int32_t *cosine_q15);
/* B线程实现。控制tick由A按20ms周期调用；地图和路径长期保存在CPU RAM。 */
void bx72_nav_init(void);
uint32_t bx72_nav_selftest(void);
uint32_t bx72_nav_diagnostic_status(void);
void bx72_nav_step(const bx72_pose_t *pose,uint32_t now_us);
void bx72_nav_fragment(const uint32_t words[8],uint32_t now_us);
void bx72_nav_task_input_lost(uint32_t now_us);
void bx72_nav_control(const bx72_pose_t *pose,uint32_t now_us,uint32_t status,
    uint32_t command_seq,int32_t vx,int32_t vy,int32_t wz,uint32_t mode_seq,bx72_nav_output_t *out);
#endif
