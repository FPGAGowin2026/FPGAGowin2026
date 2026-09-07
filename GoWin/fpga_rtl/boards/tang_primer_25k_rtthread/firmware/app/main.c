#include <rtthread.h>

static int hello(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    rt_kprintf("hello from Tang Primer 25K / Gowin GoRV32\n");
    return 0;
}
MSH_CMD_EXPORT(hello, print GoRV32 BSP greeting);

int main(void)
{
    rt_kprintf("GoRV32 BSP: 50MHz, UART1 115200 8N1, tick 100Hz\n");
    return 0;
}
