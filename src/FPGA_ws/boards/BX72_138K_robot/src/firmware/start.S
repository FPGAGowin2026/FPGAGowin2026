    .section .text.start,"ax",@progbits
    .align 2
    .globl _start
    .type _start,@function
_start:
    /* 尽早安装直接模式陷阱入口，非法F指令也能留下粘滞诊断。 */
    la      t0, _trap
    csrw    mtvec, t0

    /* 打开mstatus.FS，清除浮点控制状态后再进入任何C代码。 */
    li      t0, 0x6000
    csrs    mstatus, t0
    csrw    fcsr, zero

    .option push
    .option norelax
    la      gp, __global_pointer$
    .option pop
    la      sp, __stack_top

    /* 把ROM中的.data复制到RAM，再清零所有BSS。 */
    la      t0, __data_load_start
    la      t1, __data_start
    la      t2, __data_end
.Lcopy_data:
    bgeu    t1, t2, .Lclear_bss_begin
    lw      t3, 0(t0)
    sw      t3, 0(t1)
    addi    t0, t0, 4
    addi    t1, t1, 4
    j       .Lcopy_data

.Lclear_bss_begin:
    la      t0, __bss_start
    la      t1, __bss_end
.Lclear_bss:
    bgeu    t0, t1, .Lenter_c
    sw      zero, 0(t0)
    addi    t0, t0, 4
    j       .Lclear_bss

.Lenter_c:
    call    firmware_main
.Lhalt:
    j       .Lhalt

/* 陷阱寄存器位于lite内部MMIO窗口；诊断与故障码在复位前保持。 */
_trap:
    csrr    t0, mcause
    csrr    t1, mepc
    lui     t2, 0xf0000
    sw      t0, 0x014(t2)
    sw      t1, 0x018(t2)
    li      t3, 5
    sw      t3, 0x010(t2)
.Ltrap_halt:
    j       .Ltrap_halt

    .size _start, .-_start
