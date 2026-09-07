/*
 * GoRV32 XIP 启动入口。
 * 逻辑执行地址固定为 0x80000000；IP 的 Flash Burn Address 负责映射物理偏移。
 */

    .section .text.entry, "ax", @progbits
    .align  2
    .global crtStart
    .type   crtStart, @function

crtStart:
    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop

    la      sp, _sp

    /* 调度器接管前保持全局中断关闭。 */
    csrw    mie, zero
    li      t0, 8
    csrc    mstatus, t0

    /* 将 .data 从 XIP Flash 的 LMA 复制到片上 RAM。 */
    la      a0, __data_load
    la      a1, __data_start
    la      a2, __data_end
1:
    bgeu    a1, a2, 2f
    lw      t0, 0(a0)
    sw      t0, 0(a1)
    addi    a0, a0, 4
    addi    a1, a1, 4
    j       1b

    /* 清零 .bss。链接脚本保证边界按 4 字节对齐。 */
2:
    la      a0, __bss_start
    la      a1, __bss_end
3:
    bgeu    a0, a1, 4f
    sw      zero, 0(a0)
    addi    a0, a0, 4
    j       3b

4:
    la      t0, trap_entry
    csrw    mtvec, t0
    call    entry

5:
    wfi
    j       5b

    .size crtStart, . - crtStart
