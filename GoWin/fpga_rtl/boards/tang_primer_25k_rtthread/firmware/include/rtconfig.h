#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* RT-Thread 4.1.1：面向 64 KiB 片上 RAM 的最小静态配置。 */
#define RT_NAME_MAX                     12
/* RISC-V psABI 要求栈指针始终保持 16 字节对齐。 */
#define RT_ALIGN_SIZE                   16
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX          32
#define RT_TICK_PER_SECOND              100
#define RT_USING_OVERFLOW_CHECK
#define IDLE_THREAD_STACK_SIZE          256

/* FinSH 的 shell 对象始终包含一个静态信号量。 */
#define RT_USING_SEMAPHORE

/* 不启用 heap/device 框架，FinSH 直接轮询板级 UART。 */
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE              128

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE       1024
#define RT_MAIN_THREAD_PRIORITY         10

#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_USING_MSH_ONLY
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define MSH_USING_BUILT_IN_COMMANDS
#define FINSH_THREAD_NAME               "tshell"
#define FINSH_THREAD_PRIORITY           20
#define FINSH_THREAD_STACK_SIZE         2048
#define FINSH_CMD_SIZE                  80
#define FINSH_ARG_MAX                   8

#define RT_VER_NUM                      0x40101

#define SOC_GOWIN_GORV32
#define ARCH_RISCV

#endif /* RT_CONFIG_H__ */
