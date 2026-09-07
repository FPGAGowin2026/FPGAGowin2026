#include <rthw.h>
#include <rtthread.h>

#include "board.h"

typedef struct
{
    volatile const uint32_t IDREV;
    uint32_t reserved0[3];
    volatile const uint32_t CFG;
    volatile uint32_t OSCR;
    uint32_t reserved1[2];
    volatile uint32_t DATA;
    volatile uint32_t IER_DLM;
    volatile uint32_t IIR_FCR;
    volatile uint32_t LCR;
    volatile uint32_t MCR;
    volatile uint32_t LSR;
    volatile uint32_t MSR;
    volatile uint32_t SCR;
} gorv32_uart_t;

#define UART1               ((gorv32_uart_t *)GORV32_UART1_BASE)
#define UART_LCR_DLAB       (1UL << 7)
#define UART_LSR_DATA_READY (1UL << 0)
#define UART_LSR_TX_EMPTY   (1UL << 5)

static uint64_t next_tick_deadline;

static uint64_t gorv32_mtime_read(void)
{
    volatile uint32_t *mtime_lo = (volatile uint32_t *)GORV32_MTIME_BASE;
    volatile uint32_t *mtime_hi = mtime_lo + 1;
    uint32_t hi0;
    uint32_t lo;
    uint32_t hi1;

    do
    {
        hi0 = *mtime_hi;
        lo = *mtime_lo;
        hi1 = *mtime_hi;
    } while (hi0 != hi1);

    return ((uint64_t)hi1 << 32) | lo;
}

static void gorv32_mtimecmp_write(uint64_t value)
{
    volatile uint32_t *cmp_lo = (volatile uint32_t *)GORV32_MTIMECMP_BASE;
    volatile uint32_t *cmp_hi = cmp_lo + 1;

    /* RV32 规范写序，避免更新 64 位比较值时短暂触发错误中断。 */
    *cmp_lo = 0xffffffffUL;
    *cmp_hi = (uint32_t)(value >> 32);
    *cmp_lo = (uint32_t)value;
}

static void gorv32_uart_init(void)
{
    const uint32_t baud = 115200UL;
    const uint32_t divisor = (GORV32_CPU_HZ + 8UL * baud) / (16UL * baud);

    UART1->IER_DLM = 0;
    UART1->LCR = 3UL | UART_LCR_DLAB;
    UART1->DATA = divisor & 0xffUL;
    UART1->IER_DLM = (divisor >> 8) & 0xffUL;
    UART1->LCR = 3UL;                  /* 8 data bits, no parity, 1 stop bit */
    UART1->IIR_FCR = 0x07UL;           /* FIFO enable + clear RX/TX FIFO */
    UART1->MCR = 0;
}

static void gorv32_uart_putc(char ch)
{
    while ((UART1->LSR & UART_LSR_TX_EMPTY) == 0)
    {
    }
    UART1->DATA = (uint32_t)(uint8_t)ch;
}

void rt_hw_console_output(const char *str)
{
    while (*str != '\0')
    {
        if (*str == '\n')
        {
            gorv32_uart_putc('\r');
        }
        gorv32_uart_putc(*str++);
    }
}

char rt_hw_console_getchar(void)
{
    if ((UART1->LSR & UART_LSR_DATA_READY) != 0)
    {
        return (char)(UART1->DATA & 0xffUL);
    }

    rt_thread_mdelay(5);
    return (char)-1;
}

void rt_hw_board_init(void)
{
    gorv32_uart_init();

    /* 50 MHz / 100 Hz = 500000 个 MTIME 计数。 */
    next_tick_deadline = gorv32_mtime_read() +
                         (GORV32_CPU_HZ / RT_TICK_PER_SECOND);
    gorv32_mtimecmp_write(next_tick_deadline);
    __asm__ volatile ("csrs mie, %0" :: "r"(0x80UL));

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

void gorv32_tick_reload(void)
{
    uint64_t now = gorv32_mtime_read();

    /* 使用绝对截止时间消除 ISR 延迟造成的长期 tick 漂移。 */
    do
    {
        next_tick_deadline += (GORV32_CPU_HZ / RT_TICK_PER_SECOND);
    } while ((int64_t)(next_tick_deadline - now) <= 0);

    gorv32_mtimecmp_write(next_tick_deadline);
}
