#include <rthw.h>
#include <rtthread.h>

#define MCAUSE_INTERRUPT_BIT   (1UL << 31)
#define MCAUSE_CODE_MASK       0x7fffffffUL
#define MCAUSE_MACHINE_TIMER   7UL

extern void gorv32_tick_reload(void);

void handle_trap(rt_ubase_t mcause, rt_ubase_t mepc, rt_ubase_t interrupt_sp)
{
    (void)interrupt_sp;

    if (((mcause & MCAUSE_INTERRUPT_BIT) != 0) &&
        ((mcause & MCAUSE_CODE_MASK) == MCAUSE_MACHINE_TIMER))
    {
        gorv32_tick_reload();
        rt_tick_increase();
        return;
    }

    /* 未接管的异常不能静默返回，否则会在同一条指令上持续陷入。 */
    rt_kprintf("\nFATAL: mcause=0x%08x mepc=0x%08x\n",
               (unsigned int)mcause,
               (unsigned int)mepc);
    while (1)
    {
        __asm__ volatile ("wfi");
    }
}
