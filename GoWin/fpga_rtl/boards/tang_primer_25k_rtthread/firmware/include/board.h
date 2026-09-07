#ifndef BOARD_H__
#define BOARD_H__

#include <stdint.h>

#define GORV32_CPU_HZ           50000000UL
#define GORV32_UART1_BASE       0xF0200000UL
#define GORV32_CLINT_BASE       0xE6000000UL
#define GORV32_MTIMECMP_BASE    (GORV32_CLINT_BASE + 0x4000UL)
#define GORV32_MTIME_BASE       (GORV32_CLINT_BASE + 0xBFF8UL)

void rt_hw_board_init(void);

#endif /* BOARD_H__ */
