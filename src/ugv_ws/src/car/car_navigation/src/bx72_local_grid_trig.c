#include "bx72_local_grid_trig.h"

/* 与原栅格相同的定点量化，用于保持碰撞边界一致。 */
/* 0.5度采样的四分之一波正弦表，端点固定为0和32767。
 * Q9线性分数使百分之一度接口误差不超过约2个Q15单位；毫弧度接口
 * 另含整mrad到百分之一度的舍入误差，主机测试限值为12个Q15单位。 */
static const int16_t sine_quarter_q15[181] = {
    0, 286, 572, 858, 1144, 1429, 1715, 2000, 2286, 2571, 2856, 3141,
    3425, 3709, 3993, 4277, 4560, 4843, 5126, 5408, 5690, 5971, 6252, 6533,
    6813, 7092, 7371, 7649, 7927, 8204, 8481, 8757, 9032, 9306, 9580, 9853,
    10126, 10397, 10668, 10938, 11207, 11475, 11743, 12009, 12275, 12539, 12803, 13066,
    13328, 13588, 13848, 14107, 14364, 14621, 14876, 15130, 15383, 15635, 15886, 16135,
    16383, 16631, 16876, 17121, 17364, 17606, 17846, 18085, 18323, 18559, 18794, 19028,
    19260, 19491, 19720, 19947, 20173, 20398, 20621, 20842, 21062, 21280, 21497, 21712,
    21925, 22137, 22347, 22555, 22762, 22967, 23170, 23371, 23571, 23768, 23964, 24158,
    24351, 24541, 24730, 24916, 25101, 25284, 25465, 25644, 25821, 25996, 26169, 26340,
    26509, 26676, 26841, 27004, 27165, 27324, 27481, 27635, 27788, 27938, 28087, 28233,
    28377, 28519, 28659, 28796, 28932, 29065, 29196, 29324, 29451, 29575, 29697, 29817,
    29934, 30049, 30162, 30273, 30381, 30487, 30591, 30692, 30791, 30888, 30982, 31074,
    31163, 31250, 31335, 31418, 31498, 31575, 31650, 31723, 31794, 31862, 31927, 31990,
    32051, 32109, 32165, 32218, 32269, 32318, 32364, 32407, 32448, 32487, 32523, 32556,
    32587, 32616, 32642, 32666, 32687, 32706, 32722, 32736, 32747, 32756, 32762, 32766,
    32767
};

static int32_t cached_angle;
static int32_t cached_sine;
static int32_t cached_cosine;
static uint8_t cached_valid;

/* 以整数线性插值求四分之一波，返回Q15值；输入范围含两个端点。 */
static int32_t quarter_sine_q15(uint32_t position, uint32_t quarter_units)
{
    uint32_t scaled;
    uint32_t index, fraction;
    int32_t low, high;
    if (position == 0u) return 0;
    if (position >= quarter_units) return 32767;
    scaled = (position * (180u * 512u)) / quarter_units;
    index = scaled >> 9;
    fraction = scaled & 511u;
    low = sine_quarter_q15[index];
    high = sine_quarter_q15[index + 1u];
    return low + ((high - low) * (int32_t)fraction + 256) / 512;
}

static void cache_pair(int32_t angle,
                       int32_t sine, int32_t cosine)
{
    cached_angle = angle;
    cached_valid = 1u;
    cached_sine = sine;
    cached_cosine = cosine;
}

static uint32_t mrad_to_cdeg(int32_t angle_mrad)
{
    int32_t angle_cdeg;
    if (angle_mrad >= -6282 && angle_mrad <= 6282) {
        int32_t numerator = angle_mrad * 57296;
        numerator += numerator >= 0 ? 5000 : -5000;
        angle_cdeg = numerator / 10000;
    } else {
        int64_t phase_nrad = (int64_t)angle_mrad * 1000000LL;
        int64_t numerator;
        phase_nrad %= 6283185307LL;
        if (phase_nrad > 3141592654LL) phase_nrad -= 6283185307LL;
        if (phase_nrad < -3141592654LL) phase_nrad += 6283185307LL;
        numerator = phase_nrad * 18000LL;
        numerator += numerator >= 0 ? 1570796327LL : -1570796327LL;
        angle_cdeg = (int32_t)(numerator / 3141592654LL);
    }
    if (angle_cdeg < 0) angle_cdeg += 36000;
    else if (angle_cdeg >= 36000) angle_cdeg -= 36000;
    return (uint32_t)angle_cdeg;
}

static void compute_cdeg_q15(uint32_t angle_cdeg,
                             int32_t *sine_q15, int32_t *cosine_q15)
{
    /* 两个入口均已归一化到百分之一度，缓存键和查表单位一致。 */
    uint32_t angle = angle_cdeg;
    uint32_t quadrant, position;
    int32_t sine, cosine;

    quadrant = angle / 9000u;
    position = angle % 9000u;
    if (quadrant == 0u) {
        sine = quarter_sine_q15(position, 9000u);
        cosine = quarter_sine_q15(9000u - position, 9000u);
    } else if (quadrant == 1u) {
        sine = quarter_sine_q15(9000u - position, 9000u);
        cosine = -quarter_sine_q15(position, 9000u);
    } else if (quadrant == 2u) {
        sine = -quarter_sine_q15(position, 9000u);
        cosine = -quarter_sine_q15(9000u - position, 9000u);
    } else {
        sine = -quarter_sine_q15(9000u - position, 9000u);
        cosine = quarter_sine_q15(position, 9000u);
    }
    *sine_q15 = sine;
    *cosine_q15 = cosine;
}

static void sincos_q15(int32_t angle_mrad)
{
    uint32_t angle_cdeg = mrad_to_cdeg(angle_mrad);
    int32_t sine, cosine;
    if (cached_valid && (uint32_t)cached_angle == angle_cdeg) return;
    compute_cdeg_q15(angle_cdeg, &sine, &cosine);
    cache_pair((int32_t)angle_cdeg, sine, cosine);
}

int32_t bx72_grid_sin_q15(int32_t angle_mrad)
{
    sincos_q15(angle_mrad);
    return cached_sine;
}

int32_t bx72_grid_cos_q15(int32_t angle_mrad)
{
    sincos_q15(angle_mrad);
    return cached_cosine;
}

/* CSM直接使用百分之一度输入，避免先量化到整毫弧度。 */
void bx72_grid_sincos_cdeg_q15(uint32_t angle_cdeg,
                          int32_t *sine_q15, int32_t *cosine_q15)
{
    uint32_t angle = angle_cdeg % 36000u;
    int32_t sine, cosine;
    if (sine_q15 == 0 || cosine_q15 == 0) return;
    if (cached_valid && (uint32_t)cached_angle == angle) {
        *sine_q15 = cached_sine;
        *cosine_q15 = cached_cosine;
        return;
    }
    compute_cdeg_q15(angle, &sine, &cosine);
    cache_pair((int32_t)angle, sine, cosine);
    *sine_q15 = sine;
    *cosine_q15 = cosine;
}
