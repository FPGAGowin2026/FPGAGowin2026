#ifndef ROBOT_BOARD_IO_H
#define ROBOT_BOARD_IO_H

#include <stdint.h>

#define IO_BASE 0xf0000000u
#define IO_READY 0x008u
#define IO_HEARTBEAT 0x00cu
#define IO_FAULT 0x010u
#define IO_NOW 0x100u
#define IO_STATUS 0x104u
#define IO_SNAPSHOT 0x108u
#define IO_IMU_SEQ 0x110u
#define IO_GZ_AX 0x118u
#define IO_WHEEL_SEQ 0x120u
#define IO_WHEEL_STAMP 0x130u
#define IO_IMU_STAMP 0x134u
#define IO_LIDAR_HEAD 0x140u
#define IO_LIDAR_META 0x144u
#define IO_LIDAR_POP 0x148u
#define IO_LIDAR_STAMP 0x14cu
#define IO_LIDAR_DROPS 0x150u
#define IO_LIDAR_ERRORS 0x154u
#define IO_TASK_META 0x160u
#define IO_TASK_WORD0 0x164u
#define IO_TASK_POP 0x184u
#define IO_TASK_OVERFLOW_ACK 0x188u
#define IO_TASK_EPOCH 0x18cu
#define IO_COMMAND_SEQ 0x190u
#define IO_COMMAND_XY 0x194u
#define IO_COMMAND_WZ 0x198u
#define IO_MODE_SEQ 0x19cu
#define IO_BOOT_ID 0x1a0u
#define IO_ODOM_DOMAIN 0x1a4u
#define IO_SESSION_LO 0x1a8u
#define IO_SESSION_HI 0x1acu
#define IO_WHEEL_0 0x1c0u
#define IO_SET_ODOM_DOMAIN 0x1d0u
#define IO_FRONT_STOP_MM 0x1d4u
#define IO_FRONT_SLOW_MM 0x1d8u
#define IO_OUT_VX 0x200u
#define IO_OUT_VY 0x204u
#define IO_OUT_WZ 0x208u
#define IO_OUT_COMMIT 0x20cu
#define IO_POSE_X 0x210u
#define IO_POSE_Y 0x214u
#define IO_POSE_YAW 0x218u
#define IO_POSE_ROLL 0x21cu
#define IO_POSE_PITCH 0x220u
#define IO_POSE_FLAGS 0x224u
#define IO_POSE_STAMP 0x228u
#define IO_POSE_COMMIT 0x22cu
#define IO_NAV_STATUS 0x230u
#define IO_NAV_TASK 0x234u
#define IO_NAV_PROGRESS 0x238u
#define IO_NAV_COMMIT 0x23cu
#define IO_COV_DIAG0 0x240u
#define IO_GYRO_BIAS 0x25cu
#define IO_EKF_VX 0x260u
#define IO_EKF_VY 0x264u
#define IO_EKF_WZ 0x268u
#define IO_MAX_CONTROL_GAP 0x26cu
#define IO_SELFTEST 0x27cu
#define IO_NAV_GOAL_X 0x284u
#define IO_NAV_GOAL_Y 0x288u
#define IO_NAV_EXPIRY 0x28cu
#define IO_MAP_VERSION 0x290u
#define IO_SEARCH_EXPANSIONS 0x294u
#define IO_MAP_AGE 0x298u
#define IO_LAST_TASK_ID 0x29cu
#define IO_OUT_AUTONOMOUS 0x2a0u
#define IO_OUT_COMMAND_SEQ 0x2a4u
#define IO_OUT_MODE_SEQ 0x2a8u
#define IO_NAV_MODE 0x2acu
#define IO_GRID_WORD0 0x600u
#define IO_GRID_COMMIT 0x6d0u
#define IO_GRID_READY 0x6d4u
#define IO_COMMIT_MAGIC 0x42583732u

#define STATUS_LINK_DOWN (1u << 0)
#define STATUS_IMU_OK (1u << 1)
#define STATUS_LIDAR_OK (1u << 2)
#define STATUS_GUARD_STOP (1u << 3)
#define STATUS_WHEEL_OK (1u << 4)
#define STATUS_AUTO (1u << 7)
#define STATUS_GUARD_SIDE (1u << 8)

static inline uint32_t io_read(uint32_t address)
{
    return ((volatile uint32_t *)IO_BASE)[address >> 2];
}

static inline void io_write(uint32_t address, uint32_t value)
{
    ((volatile uint32_t *)IO_BASE)[address >> 2] = value;
}

#endif
