#include "car_navigation/bx72_local_grid.h"
#include "bx72_local_grid_trig.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
        exit(1); \
    } \
} while (0)

static bx72_grid_pose_t pose_at(int32_t x, int32_t y, int32_t yaw, uint32_t stamp)
{
    bx72_grid_pose_t pose = {0};
    pose.x_mm = x;
    pose.y_mm = y;
    pose.yaw_mrad = yaw;
    (void)stamp;
    return pose;
}

static int ray(uint32_t angle, uint32_t range, const bx72_grid_pose_t *pose,
               uint32_t stamp, uint32_t now)
{
    /* 这些栅格几何用例给定的是射线原点，先还原对应的车体位姿。 */
    bx72_grid_pose_t sample = *pose;
    int32_t cosine = bx72_grid_cos_q15(pose->yaw_mrad);
    int32_t sine = bx72_grid_sin_q15(pose->yaw_mrad);
    int32_t dx = (BX72_GRID_LIDAR_FORWARD_OFFSET_MM * (int64_t)cosine +
                  (cosine >= 0 ? 16384 : -16384)) / 32768;
    int32_t dy = (BX72_GRID_LIDAR_FORWARD_OFFSET_MM * (int64_t)sine +
                  (sine >= 0 ? 16384 : -16384)) / 32768;
    sample.x_mm -= dx;
    sample.y_mm -= dy;
    return bx72_grid_lidar_validated(angle, range, 1u, &sample, stamp, now);
}

static void test_lidar_forward_offset(void)
{
    bx72_grid_pose_t p = pose_at(0, 0, 0, 0);
    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(bx72_grid_lidar_validated(0, 500, 1, &p, 0, 0));
    CHECK(bx72_grid_cell(0, 0) == BX72_GRID_UNKNOWN);
    CHECK(bx72_grid_cell(130, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell(630, 0) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(650, 0) == BX72_GRID_UNKNOWN);

    /* 相同固定目标在纯旋转前后仍落在同一个世界栅格。 */
    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(bx72_grid_lidar_validated(31106, 1340, 1, &p, 0, 0));
    CHECK(bx72_grid_cell(1010, 1010) == BX72_GRID_OCCUPIED);
    bx72_grid_scan_begin();
    p.yaw_mrad = 1571;
    CHECK(bx72_grid_lidar_validated(4894, 1340, 1, &p, 1, 1));
    CHECK(bx72_grid_cell(1010, 1010) == BX72_GRID_OCCUPIED);
}

static void test_rays(void)
{
    bx72_grid_pose_t p = pose_at(0, 0, 0, 1000u);
    bx72_grid_info_t before, after;
    bx72_grid_reset(0, 0, 1000u);
    bx72_grid_scan_begin();
    CHECK(ray(0u, 500u, &p, 1000u, 1000u));
    CHECK(bx72_grid_cell(0, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell(450, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell(500, 0) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(550, 0) == BX72_GRID_UNKNOWN);
    /* 同一圈的后续长射线不能擦除先前命中。 */
    CHECK(ray(0u, 1000u, &p, 1001u, 1001u));
    CHECK(bx72_grid_cell(500, 0) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(1000, 0) == BX72_GRID_OCCUPIED);
    bx72_grid_scan_begin();
    CHECK(ray(0u, 1000u, &p, 1002u, 1002u));
    CHECK(bx72_grid_cell(500, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell(1000, 0) == BX72_GRID_OCCUPIED);

    bx72_grid_reset(0, 0, 2000u);
    bx72_grid_scan_begin();
    bx72_grid_info(&before);
    CHECK(!bx72_grid_lidar_validated(0, 1000, 0, &p, 2000, 2000));
    CHECK(!ray(9000, 1000, &p, 2000, 2000));
    CHECK(!ray(18000, 1000, &p, 2000, 2000));
    CHECK(!ray(27000, 1000, &p, 2000, 2000));
    CHECK(!ray(0, 49, &p, 2000, 2000));
    CHECK(!ray(0, 8001, &p, 2000, 2000));
    bx72_grid_info(&after);
    CHECK(before.generation == after.generation);
    CHECK(bx72_grid_cell(0, 0) == BX72_GRID_UNKNOWN);

    /* 原始雷达角顺时针，位姿偏航逆时针。 */
    bx72_grid_reset(0, 0, 0u);
    bx72_grid_scan_begin();
    CHECK(ray(4500, 1000, &p, 0, 0));
    CHECK(bx72_grid_cell(707, -707) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(707, 707) == BX72_GRID_UNKNOWN);
    bx72_grid_reset(0, 0, 0u);
    bx72_grid_scan_begin();
    p.yaw_mrad = 1571;
    CHECK(ray(0, 1000, &p, 0, 0));
    CHECK(bx72_grid_cell(0, 1000) == BX72_GRID_OCCUPIED);

    /* 地图外的实际回波，只能把边界内的射线标为空闲。 */
    p.yaw_mrad = 0;
    bx72_grid_reset(0, 0, 0u);
    bx72_grid_scan_begin();
    CHECK(ray(0, 4000, &p, 0, 0));
    CHECK(bx72_grid_cell(1599, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell(1600, 0) == BX72_GRID_OUTSIDE);
}

static void test_rolling_and_negative(void)
{
    bx72_grid_info_t info;
    bx72_grid_pose_t p = pose_at(-1, -1, 0, 0);
    bx72_grid_reset(-1, -1, 0);
    bx72_grid_info(&info);
    CHECK(info.origin_x_cell == -33 && info.origin_y_cell == -33);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, 0, 0));
    CHECK(bx72_grid_cell(499, -1) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell_at(9, -1) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(499, 0) == BX72_GRID_UNKNOWN);
    bx72_grid_recenter(149, -151, 1);
    CHECK(bx72_grid_cell(499, -1) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(-1601, -1) == BX72_GRID_OUTSIDE);
    bx72_grid_recenter(-1, -1, 2);
    CHECK(bx72_grid_cell(499, -1) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_cell(-1601, -1) == BX72_GRID_UNKNOWN);

    /* 移出窗口再移回不能恢复旧格；跨整幅地图一次清除。 */
    bx72_grid_recenter(2200, 0, 3);
    CHECK(bx72_grid_cell(499, -1) == BX72_GRID_OUTSIDE);
    bx72_grid_recenter(0, 0, 4);
    CHECK(bx72_grid_cell(499, -1) == BX72_GRID_UNKNOWN);
    bx72_grid_scan_begin();
    p = pose_at(0, 0, 0, 5);
    CHECK(ray(0, 500, &p, 5, 5));
    bx72_grid_recenter(4000, 4000, 6);
    bx72_grid_recenter(0, 0, 7);
    CHECK(bx72_grid_cell(500, 0) == BX72_GRID_UNKNOWN);

    bx72_grid_reset(INT_MIN, INT_MAX, 0);
    bx72_grid_info(&info);
    CHECK(info.origin_x_cell == -42949705);
    CHECK(info.origin_y_cell == 42949640);
    CHECK(bx72_grid_cell(INT_MIN, INT_MAX) == BX72_GRID_UNKNOWN);
    p = pose_at(INT_MIN, INT_MAX, 0, 0);
    bx72_grid_scan_begin();
    CHECK(bx72_grid_lidar_validated(0, 100, 1, &p, 0, 0));
    CHECK(bx72_grid_cell_at(-42949669, 42949672) == BX72_GRID_OCCUPIED);
    CHECK(bx72_grid_sweep_occupied(INT_MIN, INT_MAX, INT_MIN, INT_MAX, 231) &
          BX72_GRID_CHECK_OCCUPIED);
}

static void test_rolling_windows(void)
{
    static const int32_t shifts[] = {-63, -32, -5, -1, 0, 1, 5, 32, 63};
    uint8_t before[64u * 64u];
    bx72_grid_info_t old_info, new_info;
    bx72_grid_pose_t p = pose_at(0, 0, 0, 0);
    uint32_t a, b, i, x, y;
    for (a = 0u; a < sizeof(shifts) / sizeof(shifts[0]); ++a) {
        for (b = 0u; b < sizeof(shifts) / sizeof(shifts[0]); ++b) {
            bx72_grid_reset(0, 0, 0);
            bx72_grid_scan_begin();
            for (i = 0u; i <= 17u; ++i) {
                CHECK(ray(i * 500u, 500u + i * 20u, &p, i * 2u, i * 2u));
                CHECK(ray((36000u - i * 500u) % 36000u, 800u + i * 20u,
                          &p, i * 2u + 1u, i * 2u + 1u));
            }
            bx72_grid_info(&old_info);
            for (y = 0u; y < 64u; ++y)
                for (x = 0u; x < 64u; ++x)
                    before[y * 64u + x] = bx72_grid_cell_at(old_info.origin_x_cell + (int32_t)x,
                                                              old_info.origin_y_cell + (int32_t)y);
            bx72_grid_recenter(shifts[a] * 50, shifts[b] * 50, 100u);
            bx72_grid_info(&new_info);
            CHECK(new_info.origin_x_cell == old_info.origin_x_cell + shifts[a]);
            CHECK(new_info.origin_y_cell == old_info.origin_y_cell + shifts[b]);
            for (y = 0u; y < 64u; ++y) {
                for (x = 0u; x < 64u; ++x) {
                    int32_t wx = new_info.origin_x_cell + (int32_t)x;
                    int32_t wy = new_info.origin_y_cell + (int32_t)y;
                    int32_t ox = wx - old_info.origin_x_cell;
                    int32_t oy = wy - old_info.origin_y_cell;
                    uint8_t expected = (ox >= 0 && ox < 64 && oy >= 0 && oy < 64) ?
                        before[(uint32_t)oy * 64u + (uint32_t)ox] : BX72_GRID_UNKNOWN;
                    CHECK(bx72_grid_cell_at(wx, wy) == expected);
                    if (expected == BX72_GRID_OCCUPIED)
                        CHECK(bx72_grid_sweep_occupied(wx * 50 + 25, wy * 50 + 25,
                              wx * 50 + 25, wy * 50 + 25, 0) & BX72_GRID_CHECK_OCCUPIED);
                }
            }
            if (bx72_grid_cell(0, 0) != BX72_GRID_OUTSIDE &&
                bx72_grid_cell(500, 0) == BX72_GRID_OCCUPIED) {
                CHECK(ray(0, 1000, &p, 101, 101));
                CHECK(bx72_grid_cell(500, 0) == BX72_GRID_OCCUPIED);
            }
        }
    }
}

static void test_age_gap_and_wrap(void)
{
    uint32_t now = UINT32_MAX - 500000u, t;
    bx72_grid_pose_t p = pose_at(0, 0, 0, now);
    bx72_grid_info_t info, completed;
    bx72_grid_reset(0, 0, now);
    bx72_grid_scan_begin();
    bx72_grid_gap();
    bx72_grid_info(&info);
    CHECK(!info.fresh && info.scan_generation == 0u);
    bx72_grid_reset(0, 0, now);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, now, now));
    bx72_grid_scan_complete(now + 100000u);
    bx72_grid_info(&info);
    CHECK(info.fresh && info.scan_generation != 0u);
    completed = info;
    bx72_grid_scan_begin();
    bx72_grid_gap();
    bx72_grid_info(&info);
    CHECK(info.fresh && info.scan_generation == completed.scan_generation &&
          info.complete_scan_stamp_us == completed.complete_scan_stamp_us);
    CHECK(ray(0, 1000, &p, now + 100001u, now + 100001u));
    bx72_grid_scan_complete(now + 100002u);
    bx72_grid_info(&info);
    CHECK(info.fresh && info.scan_generation == completed.scan_generation &&
          info.complete_scan_stamp_us == completed.complete_scan_stamp_us);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, now + 200000u, now + 200000u));
    bx72_grid_scan_complete(now + 300000u);
    bx72_grid_info(&info);
    CHECK(info.fresh);
    bx72_grid_advance(now + 1800001u);
    bx72_grid_info(&info);
    CHECK(!info.fresh);
    CHECK(bx72_grid_cell(500, 0) == BX72_GRID_OCCUPIED);

    /* 连续不完整圈不能刷新完整圈时标。 */
    bx72_grid_scan_begin();
    CHECK(ray(0, 1000, &p, now + 1800001u, now + 1800001u));
    bx72_grid_gap();
    bx72_grid_scan_complete(now + 1800001u);
    bx72_grid_info(&info);
    CHECK(!info.fresh && info.complete_scan_stamp_us == now + 200000u);

    /* 连续跨32位微秒回绕、以及64个年龄时刻回绕：旧格绝不复活。 */
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, now + 1800002u, now + 1800002u));
    for (t = 1; t <= 90; ++t) {
        uint32_t stamp = now + 1800002u + t * 1000000u;
        bx72_grid_advance(stamp);
        /* 同圈命中掩码阻止长射线刷新500mm处，便于观察原始格老化。 */
        CHECK(ray(0, 1000, &p, stamp, stamp));
        if (t >= 5) CHECK(bx72_grid_cell(500, 0) == BX72_GRID_UNKNOWN);
    }
    CHECK(bx72_grid_cell(1000, 0) == BX72_GRID_OCCUPIED);
    /* 一次长暂停后所有观测立即变未知，也不留fresh。 */
    bx72_grid_advance(now + 1800002u + 100000000u);
    CHECK(bx72_grid_cell(1000, 0) == BX72_GRID_UNKNOWN);
    bx72_grid_info(&info);
    CHECK(!info.fresh && info.scan_generation == 0u && info.complete_scan_stamp_us == 0u);

    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, 0, 0));
    bx72_grid_scan_complete(1);
    bx72_grid_info(&info);
    CHECK(info.fresh && info.scan_generation != 0u);
    bx72_grid_reset(0, 0, 2);
    bx72_grid_info(&info);
    CHECK(!info.fresh && info.scan_generation == 0u && info.complete_scan_stamp_us == 0u);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, 3, 3));
    bx72_grid_scan_complete(4);
    bx72_grid_recenter(3200, 0, 5);
    bx72_grid_info(&info);
    CHECK(!info.fresh && info.scan_generation == 0u && info.complete_scan_stamp_us == 0u);
}

static void test_circle_and_sweep(void)
{
    bx72_grid_pose_t p = pose_at(0, 0, 0, 0);
    bx72_grid_unknown_exemption_t permitted = {0, 0, BX72_GRID_START_CONFIRM_RADIUS_MM};
    uint32_t flags;
    bx72_grid_reset(0, 0, 0);
    CHECK(bx72_grid_check_circle(0, 0, 231, 0) & BX72_GRID_CHECK_UNKNOWN);
    /* 人工确认的同一圆可豁免未知，但并不写入自由格。 */
    CHECK(bx72_grid_check_circle(0, 0, 231, &permitted) == 0u);
    CHECK(bx72_grid_cell(-1, -1) == BX72_GRID_UNKNOWN);
    CHECK(bx72_grid_check_sweep(0, 0, 10, 0, 231, &permitted) &
          BX72_GRID_CHECK_UNKNOWN);
    bx72_grid_scan_begin();
    CHECK(ray(0, 100, &p, 0, 0));
    CHECK(bx72_grid_check_circle(0, 0, 231, &permitted) & BX72_GRID_CHECK_OCCUPIED);

    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, 0, 0));
    CHECK((bx72_grid_check_circle(267, 0, 231, 0) & BX72_GRID_CHECK_OCCUPIED) == 0);
    CHECK(bx72_grid_check_circle(269, 0, 231, 0) & BX72_GRID_CHECK_OCCUPIED);
    /* 两端均不在障碍内，线段穿过格中间也必须命中。 */
    CHECK(bx72_grid_check_sweep(400, 25, 600, 25, 0, 0) & BX72_GRID_CHECK_OCCUPIED);
    CHECK(bx72_grid_check_sweep(400, 51, 600, 51, 1, 0) & BX72_GRID_CHECK_OCCUPIED);
    CHECK(!(bx72_grid_check_sweep(400, 52, 600, 52, 1, 0) & BX72_GRID_CHECK_OCCUPIED));
    CHECK(!(bx72_grid_check_sweep(0, 0, 1000, 1000, 10, 0) & BX72_GRID_CHECK_OCCUPIED));
    CHECK(bx72_grid_sweep_occupied(400, 25, 600, 25, 0) == BX72_GRID_CHECK_OCCUPIED);
    CHECK(bx72_grid_sweep_occupied(400, 51, 600, 51, 1) == BX72_GRID_CHECK_OCCUPIED);
    CHECK(bx72_grid_sweep_occupied(400, 52, 600, 52, 1) == 0u);
    CHECK(bx72_grid_sweep_occupied(0, 0, 1000, 1000, 10) == 0u);
    CHECK(bx72_grid_sweep_occupied(1550, 0, 1550, 0, 50) == BX72_GRID_CHECK_OUTSIDE);
    flags = bx72_grid_check_circle(1550, 0, 50, &permitted);
    CHECK(flags & BX72_GRID_CHECK_OUTSIDE);
    CHECK(bx72_grid_check_sweep(INT_MIN, 0, INT_MAX, 0, 231, 0) & BX72_GRID_CHECK_OUTSIDE);
}

static void test_directional_body_sweep(void)
{
    bx72_grid_pose_t p = pose_at(0, 0, 0, 0);
    bx72_grid_unknown_exemption_t permitted = {0, 0, BX72_GRID_START_CONFIRM_RADIUS_MM};
    bx72_grid_reset(0, 0, 0);
    CHECK(bx72_grid_sweep_body_unknown(0, 0, 0, 0, 0, 0, 10, 0) & BX72_GRID_CHECK_UNKNOWN);
    CHECK(bx72_grid_sweep_body_unknown(0, 0, 0, 0, 0, 0, 10, &permitted) == 0u);
    CHECK(bx72_grid_sweep_body_unknown(250, 0, 250, 0, 0, 0, 10, &permitted) &
          BX72_GRID_CHECK_UNKNOWN);

    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 150, &p, 0, 0));
    CHECK(!(bx72_grid_sweep_body_occupied(0, 0, 18, 0, 0, 0, 0) & BX72_GRID_CHECK_OCCUPIED));
    CHECK(bx72_grid_sweep_body_occupied(0, 0, 20, 0, 0, 0, 0) & BX72_GRID_CHECK_OCCUPIED);
    CHECK(bx72_grid_sweep_body_occupied(0, 0, 100, 0, 0, 0, 10) & BX72_GRID_CHECK_OCCUPIED);
    CHECK(bx72_grid_sweep_body_occupied(1469, 0, 1469, 0, 0, 0, 0) & BX72_GRID_CHECK_OUTSIDE);

    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(28000u, 180u, &p, 0, 0));
    CHECK(!(bx72_grid_sweep_body_occupied(0, 0, 0, 0, 0, 0, 10) & BX72_GRID_CHECK_OCCUPIED));
    CHECK(bx72_grid_sweep_body_occupied(0, 0, 0, 0, 0, 300, 10) & BX72_GRID_CHECK_OCCUPIED);
    CHECK(bx72_grid_sweep_body_occupied(0, 0, 0, 0, 785, 0, 10) & BX72_GRID_CHECK_OCCUPIED);
    bx72_grid_advance(5000001u);
    CHECK(!(bx72_grid_sweep_body_occupied(0, 0, 0, 0, 785, 0, 10) & BX72_GRID_CHECK_OCCUPIED));

    /* 实体间隙小于250mm车宽，不能把矩形近似退化为点或忽略占用。 */
    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(8000u, 122u, &p, 0, 0));
    CHECK(ray(28000u, 122u, &p, 0, 0));
    CHECK(bx72_grid_sweep_body_occupied(0, 0, 0, 0, 0, 0, 0) & BX72_GRID_CHECK_OCCUPIED);

    /* 环形位图的31/32列和负坐标边界不能漏掉相交的占用格。 */
    for (int target = 1550; target <= 1600; target += 50) {
        bx72_grid_pose_t beam = pose_at(1300, 0, 0, 0);
        bx72_grid_reset(400, 0, 0);
        bx72_grid_scan_begin();
        CHECK(ray(0, (uint32_t)(target - 1300), &beam, 0, 0));
        CHECK(bx72_grid_sweep_body_occupied(target - 130, 0, target - 130, 0,
                                            0, 0, 0) & BX72_GRID_CHECK_OCCUPIED);
    }
    p = pose_at(-300, -1, 0, 0);
    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 150, &p, 0, 0));
    CHECK(bx72_grid_sweep_body_occupied(-20, -1, -20, -1, 0, 0, 0) & BX72_GRID_CHECK_OCCUPIED);
}

typedef struct { double x, y; } test_point_t;

static double orientation(test_point_t a, test_point_t b, test_point_t p)
{
    return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

static int point_in_segment_box(test_point_t a, test_point_t b, test_point_t p)
{
    return p.x >= (a.x < b.x ? a.x : b.x) && p.x <= (a.x > b.x ? a.x : b.x) &&
           p.y >= (a.y < b.y ? a.y : b.y) && p.y <= (a.y > b.y ? a.y : b.y);
}

static int segments_cross(test_point_t a, test_point_t b, test_point_t c, test_point_t d)
{
    double ab_c = orientation(a, b, c), ab_d = orientation(a, b, d);
    double cd_a = orientation(c, d, a), cd_b = orientation(c, d, b);
    if ((ab_c == 0 && point_in_segment_box(a, b, c)) ||
        (ab_d == 0 && point_in_segment_box(a, b, d)) ||
        (cd_a == 0 && point_in_segment_box(c, d, a)) ||
        (cd_b == 0 && point_in_segment_box(c, d, b))) return 1;
    return ((ab_c < 0 && ab_d > 0) || (ab_c > 0 && ab_d < 0)) &&
           ((cd_a < 0 && cd_b > 0) || (cd_a > 0 && cd_b < 0));
}

static double point_segment_squared(test_point_t p, test_point_t a, test_point_t b)
{
    double vx = b.x - a.x, vy = b.y - a.y, len = vx * vx + vy * vy;
    double t = len ? ((p.x - a.x) * vx + (p.y - a.y) * vy) / len : 0;
    double dx, dy;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    dx = p.x - (a.x + t * vx);
    dy = p.y - (a.y + t * vy);
    return dx * dx + dy * dy;
}

static double segment_distance_squared(test_point_t a, test_point_t b,
                                       test_point_t c, test_point_t d)
{
    double result, v;
    if (segments_cross(a, b, c, d)) return 0;
    result = point_segment_squared(a, c, d);
    v = point_segment_squared(b, c, d); if (v < result) result = v;
    v = point_segment_squared(c, a, b); if (v < result) result = v;
    v = point_segment_squared(d, a, b); if (v < result) result = v;
    return result;
}

/* 独立几何参考：计算候选线段到占用方格四条边的最短距离。 */
static int reference_sweep_hits(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                                uint32_t radius)
{
    test_point_t a = {x0, y0}, b = {x1, y1};
    static const test_point_t corners[5] = {{500, 0}, {550, 0}, {550, 50},
                                            {500, 50}, {500, 0}};
    uint32_t i;
    if ((x0 >= 500 && x0 <= 550 && y0 >= 0 && y0 <= 50) ||
        (x1 >= 500 && x1 <= 550 && y1 >= 0 && y1 <= 50)) return 1;
    for (i = 0u; i < 4u; ++i)
        if (segment_distance_squared(a, b, corners[i], corners[i + 1u]) <=
            (double)radius * radius + 1e-7) return 1;
    return 0;
}

static uint32_t random_word(uint32_t *state)
{
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    return *state;
}

static void test_age_variable_intervals(void)
{
    uint32_t state = 0x31cc071bu, now = UINT32_MAX - 1000000u, i;
    uint64_t elapsed = 0u;
    bx72_grid_pose_t p = pose_at(0, 0, 0, now);
    bx72_grid_reset(0, 0, now);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, now, now));
    for (i = 0u; i < 5000u; ++i) {
        uint32_t step = random_word(&state) % 4750000u + 1u;
        elapsed += step;
        now += step;
        bx72_grid_advance(now);
        if (elapsed >= BX72_GRID_TTL_US)
            CHECK(bx72_grid_cell(500, 0) == BX72_GRID_UNKNOWN);
    }
    CHECK(elapsed > 2ull * 0x100000000ull);
}

static void test_sweep_against_independent_geometry(void)
{
    uint32_t state = 0x97a316d5u, i;
    bx72_grid_pose_t p = pose_at(0, 0, 0, 0);
    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, 0, 0));
    for (i = 0u; i < 3000u; ++i) {
        int32_t x0 = (int32_t)(random_word(&state) % 2001u) - 1000;
        int32_t y0 = (int32_t)(random_word(&state) % 2001u) - 1000;
        int32_t x1 = (int32_t)(random_word(&state) % 2001u) - 1000;
        int32_t y1 = (int32_t)(random_word(&state) % 2001u) - 1000;
        uint32_t radius = random_word(&state) % 301u;
        int expected = reference_sweep_hits(x0, y0, x1, y1, radius);
        CHECK(((bx72_grid_sweep_occupied(x0, y0, x1, y1, radius) &
                BX72_GRID_CHECK_OCCUPIED) != 0u) == expected);
        CHECK(((bx72_grid_check_sweep(x0, y0, x1, y1, radius, 0) &
                BX72_GRID_CHECK_OCCUPIED) != 0u) == expected);
    }
}

static uint8_t unpack_cell(const uint8_t *packed, const bx72_grid_info_t *info,
                           int32_t x, int32_t y)
{
    uint32_t i = (uint32_t)(y - info->origin_y_cell) * 64u +
                 (uint32_t)(x - info->origin_x_cell);
    return (uint8_t)((packed[i / 4u] >> (2u * (i % 4u))) & 3u);
}

static void check_all_exported_cells(void)
{
    uint8_t data[BX72_GRID_EXPORT_BYTES], split[BX72_GRID_EXPORT_BYTES];
    bx72_grid_info_t info;
    uint32_t x,y;
    bx72_grid_info(&info);
    CHECK(bx72_grid_export(0u,data,sizeof(data))==sizeof(data));
    CHECK(bx72_grid_export(0u,split,127u)==127u);
    CHECK(bx72_grid_export(127u,split+127u,sizeof(split)-127u)==sizeof(split)-127u);
    CHECK(memcmp(data,split,sizeof(data))==0);
    for (y=0u;y<64u;++y) for (x=0u;x<64u;++x) {
        int32_t wx=info.origin_x_cell+(int32_t)x, wy=info.origin_y_cell+(int32_t)y;
        CHECK(unpack_cell(data,&info,wx,wy)==bx72_grid_cell_at(wx,wy));
    }
}

static void test_export_and_generations(void)
{
    uint8_t data[BX72_GRID_EXPORT_BYTES];
    bx72_grid_info_t before, after;
    bx72_grid_pose_t p = pose_at(-1, -1, 0, 0);
    bx72_grid_reset(-1, -1, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 500, &p, 0, 0));
    bx72_grid_info(&before);
    CHECK(bx72_grid_export(0, data, 127) == 127u);
    CHECK(bx72_grid_export(127, data + 127, sizeof(data) - 127) == sizeof(data) - 127u);
    CHECK(bx72_grid_export(1024, data, sizeof(data)) == 0u);
    bx72_grid_info(&after);
    CHECK(before.generation == after.generation);
    CHECK(unpack_cell(data, &before, 9, -1) == BX72_GRID_OCCUPIED);
    CHECK(unpack_cell(data, &before, 8, -1) == BX72_GRID_FREE);
    CHECK(unpack_cell(data, &before, 10, -1) == BX72_GRID_UNKNOWN);
    check_all_exported_cells();
    bx72_grid_recenter(49, -1, 10);
    bx72_grid_info(&after);
    CHECK(after.generation != before.generation);
    check_all_exported_cells();
    bx72_grid_recenter(-1601, 1499, 11);
    check_all_exported_cells();
    bx72_grid_advance(5000012u);
    check_all_exported_cells();
}

static void test_unknown_is_not_inferred_from_neighbors(void)
{
    bx72_grid_unknown_exemption_t start = {0, 0, BX72_GRID_START_CONFIRM_RADIUS_MM};
    bx72_grid_pose_t left = pose_at(-25, 25, 3142, 1);
    bx72_grid_pose_t right = pose_at(75, 25, 0, 2);
    bx72_grid_pose_t top = pose_at(25, 75, 1571, 3);
    bx72_grid_reset(0, 0, 0);
    bx72_grid_scan_begin();
    CHECK(ray(0, 200, &left, 1, 1));
    CHECK(ray(0, 200, &right, 2, 2));
    CHECK(ray(0, 200, &top, 3, 3));
    CHECK(bx72_grid_cell_at(-1, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell_at(1, 0) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell_at(0, 1) == BX72_GRID_FREE);
    CHECK(bx72_grid_cell_at(0, 0) == BX72_GRID_UNKNOWN);
    CHECK(bx72_grid_unknown_blocking(0, 0, NULL));
    CHECK(!bx72_grid_unknown_blocking(0, 0, &start));
}

int main(void)
{
    test_lidar_forward_offset();
    test_rays();
    test_rolling_and_negative();
    test_rolling_windows();
    test_age_gap_and_wrap();
    test_circle_and_sweep();
    test_unknown_is_not_inferred_from_neighbors();
    test_directional_body_sweep();
    test_age_variable_intervals();
    test_sweep_against_independent_geometry();
    test_export_and_generations();
    puts("local grid tests passed");
    return 0;
}
