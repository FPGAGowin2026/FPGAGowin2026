OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    ROM (rx)  : ORIGIN = 0x80000000, LENGTH = 64K
    RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 32K
}

__stack_size = 0x1000;
__stack_top = ORIGIN(RAM) + LENGTH(RAM);
__stack_bottom = __stack_top - __stack_size;

SECTIONS
{
    .text ORIGIN(ROM) :
    {
        . = ALIGN(4);
        KEEP(*(.text.start))
        *(.text .text.*)
        *(.gnu.linkonce.t.*)
    } > ROM

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata .rodata.*)
        *(.srodata .srodata.*)
        *(.srodata.cst16 .srodata.cst8 .srodata.cst4 .srodata.cst2)
        *(.rdata .rdata.*)
        *(.gnu.linkonce.r.*)
        . = ALIGN(4);
    } > ROM

    .data ORIGIN(RAM) : AT(ALIGN(LOADADDR(.rodata) + SIZEOF(.rodata), 4))
    {
        . = ALIGN(4);
        __data_start = .;
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        *(.got .got.*)
        . = ALIGN(8);
        PROVIDE(__global_pointer$ = . + 0x800);
        *(.sdata .sdata.*)
        *(.sdata2 .sdata2.*)
        *(.gnu.linkonce.s.*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM
    __data_load_start = LOADADDR(.data);

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start = .;
        *(.sbss .sbss.*)
        *(.gnu.linkonce.sb.*)
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    /DISCARD/ :
    {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame.*)
        *(.gcc_except_table .gcc_except_table.*)
    }
}

ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(ROM) + LENGTH(ROM),
       "固件ROM镜像超过64KiB")
ASSERT(__bss_end <= __stack_bottom,
       "固件静态RAM数据与保留栈重叠")
ASSERT(__data_end <= __stack_bottom,
       "固件初始化数据与保留栈重叠")
ASSERT((__stack_top & 15) == 0,
       "初始栈顶必须按16字节对齐")
