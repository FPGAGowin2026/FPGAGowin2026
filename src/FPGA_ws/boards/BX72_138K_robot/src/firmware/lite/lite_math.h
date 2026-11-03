#ifndef BX72_LITE_MATH_H
#define BX72_LITE_MATH_H

#include <stdint.h>

/* Shared numerical primitives; caller-specific validation stays at each use. */
#define BX72_LITE_PI 3.14159265358979323846f
#define BX72_LITE_TWO_PI 6.28318530717958647692f

static inline float abs_f(float value)
{
    return value < 0.0f ? -value : value;
}

static inline float max_f(float a, float b)
{
    return a > b ? a : b;
}

static inline float wrap_angle(float angle)
{
    while (angle > BX72_LITE_PI) angle -= BX72_LITE_TWO_PI;
    while (angle < -BX72_LITE_PI) angle += BX72_LITE_TWO_PI;
    return angle;
}

static inline float sqrt_approx(float value)
{
    if (!(value > 0.0f)) return 0.0f;
#if defined(BX72_HOST_TEST_NO_RV32_ASM)
    {
        extern float sqrtf(float);
        return sqrtf(value);
    }
#else
    {
        float result;
        __asm__ volatile("fsqrt.s %0, %1" : "=f"(result) : "f"(value));
        return result;
    }
#endif
}

static inline int32_t float_to_i32(float value)
{
    if (!(value == value)) return 0;
    if (value > 2147483000.0f) return 2147483000;
    if (value < -2147483000.0f) return -2147483000;
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

#endif
