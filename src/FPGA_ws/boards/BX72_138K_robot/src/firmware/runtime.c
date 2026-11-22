#include <stddef.h>

/* The freestanding compiler may emit memcpy for structure assignments. */
void *memcpy(void *destination, const void *source, size_t length)
{
    volatile unsigned char *out = (volatile unsigned char *)destination;
    const volatile unsigned char *in = (const volatile unsigned char *)source;
    size_t i;
    for (i = 0; i < length; ++i) out[i] = in[i];
    return destination;
}
