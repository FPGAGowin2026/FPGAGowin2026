#include <stddef.h>

/* 裸机freestanding运行时所需的字节复制与清零函数。 */
void *memcpy(void *destination, const void *source, size_t length)
{
    volatile unsigned char *out = (volatile unsigned char *)destination;
    const volatile unsigned char *in = (const volatile unsigned char *)source;
    size_t index;
    for (index = 0; index < length; ++index) out[index] = in[index];
    return destination;
}

void *memset(void *destination, int value, size_t length)
{
    volatile unsigned char *out = (volatile unsigned char *)destination;
    size_t index;
    for (index = 0; index < length; ++index) out[index] = (unsigned char)value;
    return destination;
}

/* freestanding构建不用libm；若编译器选择sqrtf符号，直接映射到F扩展指令。 */
float sqrtf(float value)
{
    float result;
    __asm__ volatile("fsqrt.s %0, %1" : "=f"(result) : "f"(value));
    return result;
}
