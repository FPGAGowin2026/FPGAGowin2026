/*
 * Q15公共三角接口误差测试源；不在代码阶段运行。
 * 从boards/BX72_138K_robot目录：
 * cc -std=c11 sim/lite/a_trig_error_host_test.c src/firmware/lite/trig.c \
 *   -lm -o /tmp/a_trig_error_host_test
 * cdeg查表+Q9线性插值上界按2 Q15，mrad转换/周期归约上界按12 Q15。
 */
#include "../../src/firmware/lite/bx72_lite.h"
#include <math.h>
#include <stdint.h>

static double abs_double(double value)
{
    return value < 0.0 ? -value : value;
}

int main(void)
{
    const double pi = 3.1415926535897932384626433832795;
    double maximum_cdeg_error = 0.0;
    double maximum_mrad_error = 0.0;
    int32_t sine, cosine;
    uint32_t angle;
    int32_t angle_mrad;
    static const uint32_t axis_angle[] = {0u, 9000u, 18000u, 27000u, 36000u};
    static const int32_t axis_sine[] = {0, 32767, 0, -32767, 0};
    static const int32_t axis_cosine[] = {32767, 0, -32767, 0, 32767};
    unsigned int i;

    for (i = 0; i < sizeof(axis_angle) / sizeof(axis_angle[0]); ++i) {
        bx72_sincos_cdeg_q15(axis_angle[i], &sine, &cosine);
        if (sine != axis_sine[i] || cosine != axis_cosine[i]) return 1;
    }
    bx72_sincos_cdeg_q15(4500u, &sine, &cosine);
    if (sine <= 0 || cosine <= 0) return 2;
    bx72_sincos_cdeg_q15(13500u, &sine, &cosine);
    if (sine <= 0 || cosine >= 0) return 3;
    bx72_sincos_cdeg_q15(22500u, &sine, &cosine);
    if (sine >= 0 || cosine >= 0) return 4;
    bx72_sincos_cdeg_q15(31500u, &sine, &cosine);
    if (sine >= 0 || cosine <= 0) return 5;

    for (angle = 0; angle < 36000u; ++angle) {
        double radians = (double)angle * pi / 18000.0;
        double sin_error, cos_error;
        bx72_sincos_cdeg_q15(angle, &sine, &cosine);
        sin_error = abs_double((double)sine - sin(radians) * 32767.0);
        cos_error = abs_double((double)cosine - cos(radians) * 32767.0);
        if (sin_error > maximum_cdeg_error) maximum_cdeg_error = sin_error;
        if (cos_error > maximum_cdeg_error) maximum_cdeg_error = cos_error;
    }
    if (maximum_cdeg_error > 2.0) return 6;

    for (angle_mrad = -12567; angle_mrad <= 12567; ++angle_mrad) {
        double radians = (double)angle_mrad / 1000.0;
        double sin_error, cos_error;
        sine = bx72_sin_q15(angle_mrad);
        cosine = bx72_cos_q15(angle_mrad);
        sin_error = abs_double((double)sine - sin(radians) * 32767.0);
        cos_error = abs_double((double)cosine - cos(radians) * 32767.0);
        if (sin_error > maximum_mrad_error) maximum_mrad_error = sin_error;
        if (cos_error > maximum_mrad_error) maximum_mrad_error = cos_error;
    }
    if (maximum_mrad_error > 12.0) return 7;

    /* 近似±pi、整圈及跨圈输入须保持符号和全圈连续性。 */
    for (i = 0; i < 6u; ++i) {
        static const int32_t angles[] = {3141, 3142, -3141, -3142, 6283, -6283};
        double radians = (double)angles[i] / 1000.0;
        sine = bx72_sin_q15(angles[i]);
        cosine = bx72_cos_q15(angles[i]);
        if (abs_double((double)sine - sin(radians) * 32767.0) > 12.0 ||
            abs_double((double)cosine - cos(radians) * 32767.0) > 12.0) return 8;
    }
    sine = bx72_sin_q15(INT32_MAX);
    cosine = bx72_cos_q15(INT32_MIN);
    if (sine < -32767 || sine > 32767 || cosine < -32767 || cosine > 32767)
        return 9;
    return 0;
}
