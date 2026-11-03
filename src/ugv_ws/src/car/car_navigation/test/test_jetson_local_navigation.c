/* 迁移原局部航点回归，直接包含 Jetson 规划器并连接唯一共享栅格。 */
#include "navigation.c"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 原用例直接把完整位姿传给栅格；共享库只需要前3个坐标字段。 */
static int test_grid_lidar_validated(uint32_t angle, uint32_t distance, uint32_t hit,
                                     const bx72_pose_t *pose, uint32_t stamp, uint32_t now)
{
    bx72_grid_pose_t grid_pose = {pose->x_mm, pose->y_mm, pose->yaw_mrad};
    return bx72_grid_lidar_validated(angle, distance, hit, &grid_pose, stamp, now);
}
#define bx72_grid_lidar_validated test_grid_lidar_validated

uint32_t bx72_nav_test_mmio[0x2b0u / 4u];
static bx72_pose_t pose;
static uint32_t now_us, task_test_flags;
static unsigned performance_regressions;
#ifdef BX72_NAV_TEST_TIMED
static uint32_t timer_test_now, timer_test_step, timer_test_reads, timer_test_jump_at, timer_test_jump_us;
uint32_t bx72_nav_test_live_time(void)
{
    if (!timer_test_step && !timer_test_jump_at) return last_now_low;
    timer_test_now += timer_test_step;
    ++timer_test_reads;
    if (timer_test_reads == timer_test_jump_at) timer_test_now += timer_test_jump_us;
    return timer_test_now;
}
#endif
#define SENSORS (BX72_STATUS_AUTO | BX72_STATUS_IMU_OK | BX72_STATUS_LIDAR_OK | BX72_STATUS_WHEEL_OK)

static void check(int ok, const char *message)
{
    if (!ok) { fprintf(stderr, "FAIL %s\n", message); exit(1); }
}
static void send_task_config_at(uint32_t id, uint32_t action, uint32_t mode,
                                int32_t gx, int32_t gy, uint32_t ttl, uint32_t speed,
                                uint32_t accel, uint32_t tolerance, int32_t final_yaw_q16,
                                uint32_t yaw_tolerance, uint32_t sent_at)
{
    bx72_nav_task_request_t request = {0};
    request.action=action; request.mode=mode; request.flags=task_test_flags; request.task_id=id;
    request.goal_x_q16=gx; request.goal_y_q16=gy; request.final_yaw_q16=final_yaw_q16;
    request.max_speed_mm_s=speed; request.max_accel_mm_s2=accel;
    request.tolerance_mm=tolerance; request.yaw_tolerance_mrad=yaw_tolerance;
    request.expiry_us=now_us+ttl; request.sent_at_us=sent_at;
    (void)bx72_nav_submit_task(&request,now_us,0);
}
static void send_task_config_yaw_tolerance(uint32_t id, uint32_t action, uint32_t mode,
                             int32_t gx, int32_t gy, uint32_t ttl, uint32_t speed,
                             uint32_t accel, uint32_t tolerance, int32_t final_yaw_q16,
                             uint32_t yaw_tolerance)
{
    send_task_config_at(id, action, mode, gx, gy, ttl, speed, accel, tolerance,
                        final_yaw_q16, yaw_tolerance, now_us);
}
static void send_task_config(uint32_t id, uint32_t action, uint32_t mode,
                             int32_t gx, int32_t gy, uint32_t ttl, uint32_t speed,
                             uint32_t accel, uint32_t tolerance, int32_t final_yaw_q16)
{
    send_task_config_yaw_tolerance(id, action, mode, gx, gy, ttl, speed, accel,
                                   tolerance, final_yaw_q16, 100u);
}
static void send_task_speed(uint32_t id, uint32_t action, uint32_t mode,
                            int32_t gx, int32_t gy, uint32_t ttl, uint32_t speed, uint32_t accel)
{
    send_task_config(id, action, mode, gx, gy, ttl, speed, accel, 100u, 0);
}
static void send_task(uint32_t id, uint32_t action, uint32_t mode, int32_t gx, int32_t gy, uint32_t ttl)
{
    send_task_speed(id, action, mode, gx, gy, ttl, 200u, 500u);
}
static void init_fixture(void)
{
    memset(&pose,0,sizeof(pose));
    memset(bx72_nav_test_mmio,0,sizeof(bx72_nav_test_mmio));
    bx72_nav_test_mmio[BX72_IO_BOOT_ID / 4u]=7;
    bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN / 4u]=1;
    bx72_nav_test_mmio[BX72_IO_SESSION_HI / 4u]=0x12345678u;
    bx72_nav_test_mmio[BX72_IO_SESSION_LO / 4u]=0x9abcdef0u;
    now_us=1000000u; task_test_flags=0u;
    pose.flags=BX72_POSE_VALID; pose.stamp_us=now_us;
    bx72_nav_init(); bx72_nav_step(&pose,now_us);
}
static void feed(int sparse, uint32_t delay_us)
{
    uint32_t i;
    /* 两圈：第一圈找回卷边界，第二圈形成完整扫描。后方265mm模拟车体回波。 */
    for (i=0;i<=720u;++i) {
        uint32_t a=(i%360u)*100u;
        int front = a < 9000u || a > 27000u;
        int hit = !sparse || (a>5000u && a<9000u);
        now_us += delay_us == 120000u ? 200u : 400u; pose.stamp_us=now_us-delay_us;
        bx72_nav_lidar_validated(a, front ? 6000u : 265u, hit, &pose,
                                 pose.stamp_us, now_us);
    }
    pose.stamp_us=now_us;
    bx72_nav_step(&pose,now_us);
}
/* 对应硬件每5度保留一个回波的最小覆盖。 */
static void feed_reduced(void)
{
    uint32_t i;
    for (i=0u;i<=144u;++i) {
        uint32_t a=(i%72u)*500u;
        int front=a<9000u || a>27000u;
        now_us+=2000u; pose.stamp_us=now_us;
        bx72_nav_lidar_validated(a,front?6000u:265u,1u,&pose,now_us,now_us);
    }
    bx72_nav_step(&pose,now_us);
}
static bx72_nav_output_t control(uint32_t status, uint32_t sequence, int32_t x, int32_t y, int32_t w, uint32_t mode)
{
    bx72_nav_output_t out;
    now_us += 20000u; pose.stamp_us=now_us;
    bx72_nav_control(&pose,now_us,status,sequence,x,y,w,mode,&out);
    return out;
}
static bx72_nav_output_t plan_mode(uint32_t status, uint32_t mode)
{
    bx72_nav_output_t out=control(status,1,0,0,0,mode);
    uint32_t i, round;
    /* A* 捕获栅格并分时扩展，允许多个切片完成后才检查动作候选。 */
    for (round=0;round<4u;++round) {
        for (i=0;i<200u;++i) { now_us += 100u; pose.stamp_us=now_us; bx72_nav_step(&pose,now_us); }
        out=control(status,1,0,0,0,mode);
        if (out.vx_mm_s || out.vy_mm_s || out.wz_mrad_s ||
            out.status != BX72_NAV_STATUS_PLANNING) break;
    }
    return out;
}
static bx72_nav_output_t plan(uint32_t status) { return plan_mode(status,1u); }

static void test_start_recheck_gap_and_unknown(void)
{
    bx72_nav_output_t out;
    uint32_t version, i;
    init_fixture(); feed(0,0);
    check((bx72_nav_diagnostic_status() & (BX72_NAV_DIAG_MAP_FRESH|BX72_NAV_DIAG_MAP_READY|BX72_NAV_DIAG_GRID_V1)) ==
          (BX72_NAV_DIAG_MAP_FRESH|BX72_NAV_DIAG_MAP_READY|BX72_NAV_DIAG_GRID_V1), "complete raw scan and architecture marker");
    send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0 && out.vy_mm_s==0 && out.plan_valid && out.task_id==1, "front scan starts forward despite rear vehicle echo");
    check(start_exemption_valid && start_exemption.x_mm==0 && start_exemption.y_mm==0 && start_exemption.radius_mm==231,
          "one fixed operator-confirmed start circle");
    /* 模拟已加速的车体遇到雷达前方50mm新障碍，旧候选必须复核。 */
    for (i = 0u; i < 12u; ++i) {
        bx72_nav_step(&pose, now_us);
        out = control(SENSORS, 1, 0, 0, 0, 1);
        pose.vx_mm_s = out.vx_mm_s;
    }
    check(pose.vx_mm_s >= 60, "live obstacle fixture has a real braking tail");
    now_us+=100; pose.stamp_us=now_us;
    bx72_nav_lidar_validated(100u,50u,1,&pose,now_us,now_us);
    out=control(SENSORS,1,0,0,0,1);
    check(out.vx_mm_s==0 && out.vy_mm_s==0 && !out.plan_valid, "live obstacle invalidates candidate");
    pose.vx_mm_s = 0;
    feed(0,0); out=plan(SENSORS);
    check(out.vx_mm_s>0, "obstacle removal resumes after a fresh scan");
    version=out.map_version;
    bx72_nav_lidar_gap(now_us, BX72_NAV_GAP_FIFO);
    out=control(SENSORS,1,0,0,0,1);
    check(out.autonomous && out.vx_mm_s>0 &&
          (bx72_nav_diagnostic_status()&BX72_NAV_DIAG_GAP_LATCHED) &&
          (bx72_nav_diagnostic_status()&BX72_NAV_DIAG_RECOVERY_REQUIRED),
          "short gap retains previously complete scan without refreshing timestamp");
    now_us+=500001u;
    out=control(SENSORS,1,0,0,0,1);
    check(out.autonomous && out.vx_mm_s==0 && out.status==BX72_NAV_STATUS_STALE,
          "raw lidar input expires after 500ms despite retained map");
    feed(0,0); out=plan(SENSORS);
    check(out.map_version!=version && !(bx72_nav_diagnostic_status()&BX72_NAV_DIAG_GAP_LATCHED), "fresh complete scan recovers gap");
    init_fixture(); feed(1,0); send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s==0 && out.vy_mm_s==0, "unobserved front cannot be driven");
    out=plan(SENSORS);
    check(out.vx_mm_s==0 && out.status != BX72_NAV_STATUS_ARRIVED,
          "unknown forward corridor cannot be treated as a completed or clear path");
    feed(0,0); out=plan(SENSORS);
    check(out.vx_mm_s>0, "newly observed clear corridor releases the waiting path");
}

static void test_turn_and_tail(void)
{
    bx72_nav_output_t out;
    int32_t ex,ey;
    uint8_t unknown;
    init_fixture(); feed(0,0); send_task(1,1,3,-2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s==0 && out.vy_mm_s==0 && out.wz_mrad_s!=0 && abs(out.wz_mrad_s)<=30 && !out.plan_valid,
          "rear goal turns to observe without blind reversing or side translation");
    pose.yaw_mrad=observe_target_yaw_mrad; pose.wz_mrad_s=80;
    out=control(SENSORS,1,0,0,0,1);
    check(out.vx_mm_s==0 && out.wz_mrad_s==0 && !observe_settled,
          "observation must stop and wait for measured angular speed");
    pose.wz_mrad_s=0;
    (void)control(SENSORS,1,0,0,0,1);
    (void)control(SENSORS,1,0,0,0,1);
    (void)control(SENSORS,1,0,0,0,1);
    out=control(SENSORS,1,0,0,0,1);
    feed(0,120000u);
    out=control(SENSORS,1,0,0,0,1);
    check(out.vx_mm_s==0 && observe_settled && observe_active,
          "delayed complete scan that began before settling cannot release observation");
    feed(0,0);
    out=control(SENSORS,1,0,0,0,1);
    check(out.vx_mm_s==0 && !observe_active,
          "new complete scan after settling completes observation before replanning");
    out=plan(SENSORS);
    check(out.vx_mm_s>0 && out.vy_mm_s==0,
          "freshly observed direction permits forward travel");
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u); out=plan(SENSORS);
    check(out.vx_mm_s>0, "braking fixture starts");
    /* 500mm/s实测速度的制动尾段，障碍在当前车体圆之外但在刹停扫掠内。 */
    pose.vx_mm_s=500; previous_vx_mm_s=500; previous_vy_mm_s=previous_wz_mrad_s=0;
    now_us+=100; pose.stamp_us=now_us;
    bx72_nav_lidar_validated(100u,260u,1,&pose,now_us,now_us);
    check(evaluate_motion(&pose,300,0,0,&ex,&ey,&unknown,current_time_us())==0,
          "actual 500mm/s and ten per tick braking tail intersects obstacle");
    out=control(SENSORS,1,0,0,0,1);
    check(out.vx_mm_s==0 && !out.plan_valid, "unsafe braking tail commands stop");
}

static void test_observe_requires_new_complete_scan(void)
{
    bx72_grid_info_t info;
    uint32_t i, settled_at;
    init_fixture(); feed(0, 0);
    observe_active = 1u; observe_settled = 0u; observe_target_yaw_mrad = pose.yaw_mrad;
    bx72_grid_info(&info);
    check(update_observation(&pose, 0, 0, now_us, info.scan_generation) && observe_settled,
          "observation begins only after measured motion settles");
    settled_at = now_us;
    for (i = 1u; i <= 144u; ++i) {
        uint32_t angle = (i % 72u) * 500u;
        now_us += 1000u; pose.stamp_us = now_us;
        bx72_nav_lidar_validated(angle, 6000u, 1u, &pose, now_us, now_us);
        if (i == 20u || i == 72u || i == 144u) {
            bx72_grid_info(&info);
            (void)update_observation(&pose, 0, 0, now_us, info.scan_generation);
            if (i < 144u)
                check(observe_active, "partial or pre-settle scan cannot release observation");
        }
    }
    check(!observe_active && now_us - settled_at < 180000u,
          "a complete scan started after settling releases without a fixed 180ms delay");
}

static void test_curve_envelope(void)
{
    bx72_pose_t beam;
    init_fixture(); feed(0,0);
    /* 将雷达原点放在原曲线几何样例的位置，而不是把车体中心当雷达。 */
    beam=pose; beam.x_mm-=(int32_t)BX72_LIDAR_FORWARD_OFFSET_MM; beam.y_mm=16; beam.stamp_us=now_us;
    bx72_grid_scan_begin();
    check(bx72_grid_lidar_validated(28000u,140u,1,&beam,now_us,now_us),
          "curve obstacle fixture is observed from the front");
    check(!(bx72_grid_sweep_body_occupied(0,16,100,16,0,0,0)&BX72_GRID_CHECK_OCCUPIED) &&
           (bx72_grid_sweep_body_occupied(0,16,50,25,0,0,0)&BX72_GRID_CHECK_OCCUPIED) &&
           (bx72_grid_sweep_body_occupied(0,16,100,16,0,0,NAV_SWEEP_MARGIN_MM)&BX72_GRID_CHECK_OCCUPIED),
          "extra 10mm catches obstacle on curved path outside the straight chord");
}

#ifdef BX72_NAV_TEST_TIMED
static void test_live_budget(void)
{
    bx72_nav_output_t out;
    bx72_pose_t old_pose,new_pose;
    int32_t end_x,end_y;
    uint8_t unknown;
    uint32_t i;

    /* 静止预测旧版有两次中途检查：第三次恰是成功返回前的最终检查。 */
    init_fixture(); feed(0,0);
    timer_test_now=now_us; timer_test_step=0u; timer_test_reads=0u;
    timer_test_jump_at=3u; timer_test_jump_us=3999u;
    check(evaluate_motion(&pose,0,0,0,&end_x,&end_y,&unknown,now_us)==1 && timer_test_reads==3u,
          "3999us at the final successful-return check remains within budget");
    timer_test_now=now_us; timer_test_reads=0u; timer_test_jump_us=4000u;
    check(evaluate_motion(&pose,0,0,0,&end_x,&end_y,&unknown,now_us)==-1 && timer_test_reads==3u,
          "exactly 4000us at the final successful-return check times out");
    timer_test_jump_at=0u;

    /* 输出复核超时的当拍和下一控制拍均提交零速，不可由后台抢先填回。 */
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u); out=plan(SENSORS);
    check(out.vx_mm_s>0,"budget fixture begins motion");
    timer_test_now=now_us; timer_test_step=4000u;
    out=control(SENSORS,1,0,0,0,1);
    timer_test_step=0;
    check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "exact 4000us live evaluation returns a zero command");
    for (i=0u;i<20u;++i) bx72_nav_step(&pose,now_us);
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s,
          "output timeout fences the next control tick despite intervening background steps");
    out=plan(SENSORS);
    check(out.vx_mm_s>0,"planner recovers when budget is available again");

    /* 后台超时也必须清已保留候选，且必须先出现一个零速控制拍。 */
    old_pose=new_pose=pose; new_pose.x_mm+=12;
    bx72_nav_rebase(&old_pose,&new_pose); pose=new_pose;
    timer_test_now=now_us; timer_test_step=4000u;
    bx72_nav_step(&pose,now_us);
    timer_test_step=0u;
    for (i=0u;i<20u;++i) bx72_nav_step(&pose,now_us);
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s,
          "background timeout fences the next control tick despite intervening background steps");
    out=plan(SENSORS);
    check(out.vx_mm_s>0,"planning recovers after the background timeout zero tick");

    /* 已知占用使候选提前返回时，同一次4ms预算仍须清除旧候选。 */
    init_fixture(); feed(0, 0); send_task(1, 1, 3, 2 * 65536, 0, 60000000u);
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "collision timeout fixture begins with a valid candidate");
    now_us += 100u; pose.stamp_us = now_us;
    bx72_nav_lidar_validated(0u, 130u, 1u, &pose, now_us, now_us);
    planner_active = 1u; planner_complete = 0u; planner_cursor = 0u;
    planner_pose = pose; planner_task_id = task_queue[task_head].task_id;
    planner_best_valid = selected_valid = 1u;
    timer_test_now = now_us; timer_test_reads = 0u;
    timer_test_jump_at = 2u; timer_test_jump_us = 4000u;
    bx72_nav_step(&pose, now_us);
    timer_test_jump_at = 0u;
    check(timer_test_reads >= 2u && planner_timeout_pending && !planner_best_valid && !selected_valid,
          "collision early return still times out and invalidates retained candidates");
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s, "collision timeout fences the next command");
}
#endif

static void test_rebase_and_delayed_scan(void)
{
    bx72_pose_t old_pose, new_pose, before, after;
    bx72_grid_info_t a,b;
    init_fixture(); feed(0,50000u);
    bx72_grid_info(&a);
    check(a.fresh && a.scan_generation!=0u && bx72_grid_cell(500,0)==BX72_GRID_FREE,
          "delayed FIFO wrap advances live clock without clearing map");
    old_pose=pose; new_pose=pose;
    new_pose.x_mm += 35; new_pose.y_mm -= 20; new_pose.yaw_mrad += 50;
    odom_to_grid(&old_pose,&before);
    bx72_nav_rebase(&old_pose,&new_pose);
    odom_to_grid(&new_pose,&after);
    bx72_grid_info(&b);
    check(abs(after.x_mm-before.x_mm)<=1 && abs(after.y_mm-before.y_mm)<=1 &&
          abs(after.yaw_mrad-before.yaw_mrad)<=1 && b.generation==a.generation && b.fresh,
          "CSM correction composes continuous grid transform without clearing map");
    pose=new_pose;
    now_us+=100; pose.stamp_us=now_us;
    bx72_nav_lidar_validated(100u,1000u,1,&pose,now_us,now_us);
    check(bx72_grid_cell(1130,-18)==BX72_GRID_OCCUPIED || bx72_grid_cell(1129,-18)==BX72_GRID_OCCUPIED,
          "subsequent beam uses rebased history and same grid frame");
}

static void test_out_of_range_returns(void)
{
    uint32_t i, generation;
    bx72_grid_info_t info;
    init_fixture(); feed(0,0);
    bx72_grid_info(&info); generation=info.scan_generation;
    for (i=1;i<=360u;++i) {
        uint32_t angle=(i%360u)*100u;
        uint32_t range=i%3u==0u ? 12000u : 6000u;
        now_us+=400u; pose.stamp_us=now_us;
        bx72_nav_lidar_validated(angle,range,1,&pose,now_us,now_us);
    }
    bx72_grid_info(&info);
    check(info.fresh && info.scan_generation!=generation && !map_gap_latched,
          "out-of-range returns stay unknown without breaking a mixed complete scan");
}

static void test_task_safety(void)
{
    bx72_nav_output_t out;
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u); out=plan(SENSORS);
    check(out.vx_mm_s>0,"task safety fixture");
    bx72_nav_force_cancel(now_us);
    out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_CANCELLED && !out.autonomous && !out.vx_mm_s, "forced cancel stops task");
    out=control(SENSORS & ~BX72_STATUS_AUTO,1,100,0,0,1);
    check(out.vx_mm_s==0,"old manual command inhibited after cancellation");
    out=control(SENSORS & ~BX72_STATUS_AUTO,2,0,0,0,1);
    out=control(SENSORS & ~BX72_STATUS_AUTO,3,100,0,0,1);
    check(out.vx_mm_s>0 && out.vx_mm_s<=10,"new manual epoch and acceleration slope");
    send_task(2,1,3,2*65536,0,60000000u);
    bx72_nav_test_mmio[BX72_IO_SESSION_LO/4u]^=1u;
    out=control(SENSORS,3,0,0,0,1);
    check(!out.autonomous && out.status==BX72_NAV_STATUS_CANCELLED,"session change cancels tasks");
    init_fixture(); feed(0,0); send_task(1,1,2,2*65536,0,60000000u);
    out=plan(SENSORS);
    out=control(SENSORS|BX72_STATUS_LINK_DOWN,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_STALE && !out.autonomous && !out.vx_mm_s,"cooperative task link loss");
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,100000u);
    now_us+=100000u; out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_STALE && !out.autonomous, "task TTL");

    /* 网桥撤权后的无条件取消不能让旧活动航点重新驱动车辆。 */
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.autonomous && out.vx_mm_s>0, "forced cancel fixture");
    bx72_nav_force_cancel(now_us);
    out=plan(SENSORS);
    check(!out.autonomous && !out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s &&
          out.last_task_id==1u, "forced cancel clears task but retains transaction watermark");
}

static void test_limits_and_epochs(void)
{
    bx72_nav_output_t out;
    uint32_t i, sample_time;
    init_fixture(); feed(0,0);
    sample_time=last_lidar_us;
    now_us=sample_time+479999u;
    out=control(SENSORS & ~BX72_STATUS_AUTO,1,400,0,800,1);
    check(out.vx_mm_s>0 && out.wz_mrad_s>0, "manual accepts input within 500ms");
    now_us=sample_time+480001u;
    out=control(SENSORS & ~BX72_STATUS_AUTO,1,400,0,800,1);
    check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s, "manual stops after 500ms lidar timeout");
    feed(0,0);
    for (i=0;i<45u;++i) out=control(SENSORS & ~BX72_STATUS_AUTO,1,400,0,800,1);
    /* 重新供给传感输入后检查限速与斜率，且不要求地图覆盖人工运动。 */
    feed(0,0);
    for (i=0;i<20u;++i) out=control(SENSORS & ~BX72_STATUS_AUTO,1,400,0,800,1);
    check(out.vx_mm_s<=400 && out.wz_mrad_s<=800 && out.vy_mm_s==0, "manual velocity limits retained");
    out=control((SENSORS & ~BX72_STATUS_AUTO)|BX72_STATUS_GUARD_STOP,1,400,0,800,1);
    check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s,"independent guard stops manual output");

    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0, "mode/session fixture starts");
    send_task(1,1,3,3*65536,0,60000000u);
    check(task_count==1 && last_task_id==1, "duplicate task sequence rejected");
    out=control(SENSORS|BX72_STATUS_GUARD_STOP,1,0,0,0,1);
    check(out.autonomous && !out.vx_mm_s && !out.wz_mrad_s, "guard holds autonomous task without deleting it");
    pose.flags=0;
    out=control(SENSORS,1,0,0,0,1);
    check(out.autonomous && out.status==BX72_NAV_STATUS_STALE && !out.vx_mm_s, "invalid pose stops and retains task");
    pose.flags=BX72_POSE_VALID;
    out=control(SENSORS,1,0,0,0,2);
    check(!out.autonomous && out.status==BX72_NAV_STATUS_CANCELLED && !out.vx_mm_s,
          "new mode epoch cancels task");

    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u);
    send_task(2,1,3,3*65536,0,60000000u);
    out=plan(SENSORS|BX72_STATUS_LINK_DOWN);
    check(out.autonomous && out.task_id==1 && out.vx_mm_s>0, "independent task survives link down");
    pose.x_mm=1900;
    out=control(SENSORS,1,0,0,0,1);
    check(out.autonomous && out.task_id==1 && !out.vx_mm_s,
          "waypoint arrival first stops the previously commanded motion");
    (void)control(SENSORS,1,0,0,0,1);
    (void)control(SENSORS,1,0,0,0,1);
    out=control(SENSORS,1,0,0,0,1);
    check(out.task_id==2 && out.autonomous && start_exemption.x_mm==0,
          "arrival advances waypoint without moving fixed start circle");
}

static void test_independent_waypoint_start_authorization(void)
{
    bx72_nav_output_t out;
    uint32_t reset_stamp;
    init_fixture(); feed(0,0);
    send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0 && start_exemption.x_mm==0, "first waypoint sets confirmed startup area");
    pose.x_mm=1900;
    out=control(SENSORS,1,0,0,0,1);
    check(out.autonomous && out.status!=BX72_NAV_STATUS_ARRIVED && !out.vx_mm_s,
          "independent waypoint first stops before reporting arrival");
    (void)control(SENSORS,1,0,0,0,1);
    (void)control(SENSORS,1,0,0,0,1);
    out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_ARRIVED && !out.autonomous && start_exemption_valid,
          "arrival without prequeued waypoint retains startup authorization");
    out=control(SENSORS & ~BX72_STATUS_AUTO,1,0,0,0,2);
    check(start_exemption_valid && start_exemption.x_mm==0,
          "bridge-generated zero and mode-off cannot renew startup authorization");
    bx72_nav_step(&pose,now_us);
    feed(0,0);
    send_task(2,1,3,3*65536,0,60000000u);
    out=plan_mode(SENSORS,2u);
    check(out.autonomous && out.task_id==2 && start_exemption.x_mm==0 && start_exemption.y_mm==0,
          "separately submitted second waypoint cannot mint a new startup circle");
    send_task(3,2,3,0,0,60000000u);
    check(start_exemption_valid && start_exemption.x_mm==0, "cancel does not renew authorization");
    out=control(SENSORS & ~BX72_STATUS_AUTO,2,0,0,0,2);
    out=control(SENSORS & ~BX72_STATUS_AUTO,3,100,0,0,2);
    check(out.vx_mm_s>0 && start_exemption_valid && start_exemption.x_mm==0,
          "manual control does not renew authorization");
    bx72_nav_test_mmio[BX72_IO_BOOT_ID/4u]=9u;
    bx72_nav_test_mmio[BX72_IO_SESSION_LO/4u]^=1u;
    out=control(SENSORS,3,0,0,0,2);
    check(start_exemption_valid && start_exemption.x_mm==0 && !start_confirmation_lost,
          "protocol boot/session epoch change retains the original startup circle");
    send_task(4,1,3,3*65536,0,60000000u);
    out=plan_mode(SENSORS,2u);
    check(out.autonomous && start_exemption.x_mm==0, "new session retains original startup circle");
    send_task(5,1,3,4*65536,0,60000000u);
    check(task_count==2u, "reset fixture has an active and a queued old task");
    reset_stamp=now_us;
    bx72_nav_map_reset(&pose,now_us);
    check(!task_count && start_confirmation_lost && !start_exemption_valid,
          "map reset cancels all old tasks and removes the previous start circle");
    feed(0,0);
    out=plan_mode(SENSORS,2u);
    check(start_confirmation_lost && !start_exemption_valid && !out.autonomous && !out.vx_mm_s,
          "fresh map alone cannot reactivate old tasks or authorize a new start");
    send_task_config_at(6,1,3,3*65536,0,60000000u,200u,500u,100u,0,100u,reset_stamp-1u);
    check(!task_count && event_status==BX72_NAV_STATUS_REJECTED && start_confirmation_lost,
          "a complete task sent before reset cannot be replayed after map recovery");
    send_task_config_at(6,1,3,3*65536,0,60000000u,200u,500u,100u,0,100u,reset_stamp);
    check(!task_count && event_status==BX72_NAV_STATUS_REJECTED && start_confirmation_lost,
          "a task sent in the same microsecond as reset is not proof of a later action");
    send_task(7,2,3,0,0,60000000u);
    out=plan_mode(SENSORS,3u);
    check(!out.autonomous && !out.vx_mm_s && start_confirmation_lost && !start_exemption_valid,
          "cancel and mode changes cannot renew lost authorization");
    send_task(8,1,3,3*65536,0,60000000u);
    out=plan_mode(SENSORS,3u);
    check(out.vx_mm_s>0 && out.task_id==8 && start_exemption.x_mm==1900 && !start_confirmation_lost,
          "a validated new task after reset establishes a new fixed start circle");

    bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN/4u]=2u;
    out=control(SENSORS,1,0,0,0,3);
    check(!task_count && start_confirmation_lost && !start_exemption_valid,
          "odometry domain change cancels the old task and start circle");
    ++now_us;
    send_task(9,1,3,3*65536,0,60000000u);
    out=control(SENSORS,1,0,0,0,3);
    check(out.autonomous && !out.vx_mm_s && start_confirmation_lost && !start_exemption_valid,
          "new task waits for the new map before confirming its start");
    send_task(10,2,3,0,0,60000000u);
    feed(0,0);
    out=plan_mode(SENSORS,3u);
    check(!out.autonomous && !out.vx_mm_s && start_confirmation_lost && !start_exemption_valid,
          "cancelling a waiting task cannot authorize the car when the map recovers");
    send_task(11,1,3,3*65536,0,60000000u);
    out=control(SENSORS | BX72_STATUS_GUARD_STOP,1,0,0,0,3);
    check(out.autonomous && !out.vx_mm_s && start_confirmation_lost && !start_exemption_valid,
          "guard stop prevents confirmation even with a fresh map and a new task");
    out=control(SENSORS & ~BX72_STATUS_IMU_OK,1,0,0,0,3);
    check(out.autonomous && !out.vx_mm_s && start_confirmation_lost && !start_exemption_valid,
          "unhealthy sensors prevent confirmation");
    out=plan_mode(SENSORS,3u);
    check(out.vx_mm_s>0 && out.task_id==11 && start_exemption.x_mm==1900 && !start_confirmation_lost,
          "post-domain-change task recovers when scan, pose and sensors are healthy");

    reset_stamp=now_us;
    bx72_nav_map_reset(&pose,now_us);
    check(!task_count && start_confirmation_lost,
          "reset clears the active task before another task can be submitted");
    feed(0,0);
    send_task_config_at(12,1,3,4*65536,0,60000000u,200u,500u,100u,0,100u,reset_stamp);
    out=plan_mode(SENSORS,3u);
    check(!out.autonomous && !out.vx_mm_s && start_confirmation_lost,
          "a task issued at the reset cannot reactivate it after recovery");
    send_task(13,1,3,3*65536,0,60000000u);
    out=plan_mode(SENSORS,3u);
    check(out.task_id==13 && out.vx_mm_s>0 && !start_confirmation_lost,
          "a fresh task recovers after the reset");
}

static void test_reset_discards_cached_pose(void)
{
    bx72_nav_output_t out;
    init_fixture(); feed(0,0);
    send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0 && latest_pose.x_mm==0, "cached pose fixture starts at origin");
    pose.x_mm=1000; /* The new odometry frame cannot use the cached old pose. */
    bx72_nav_map_reset(&pose,now_us);
    ++now_us;
    send_task(2,1,3,2*65536,0,60000000u);
    check(task_count==1u && !latest_pose_valid && !task_start_initialized,
          "a task received just after reset cannot initialize from the old pose");
    feed(0,0);
    out=plan(SENSORS);
    check(out.vx_mm_s>0 && task_start_distance_q16==1000u*NAV_Q16_ONE,
          "the recovered task uses the new current pose for progress");
}

static void test_reset_barrier_after_long_wait(void)
{
    bx72_nav_output_t out;
    init_fixture(); feed(0,0);
    send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0, "long-wait fixture starts a task");
    bx72_nav_map_reset(&pose,now_us);
    /* 以小于半个32位周期的步长推进，让扩展时钟跨过35分钟边界。 */
    now_us+=0x7ffffff0u;
    pose.stamp_us=now_us;
    bx72_nav_step(&pose,now_us);
    feed(0,0);
    send_task(2,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(clock_valid && out.task_id==2 && out.vx_mm_s>0 && !start_confirmation_lost,
          "post-reset task is accepted even after waiting more than half a timestamp cycle");
}

static void test_near_goal_speed_and_live_recheck(void)
{
    bx72_nav_output_t out;
    int32_t candidate_vx, candidate_wz;
    init_fixture(); feed(0,0);
    /* 100 Q16 metres is about 1.526 mm: outside a 1 mm tolerance. */
    send_task_config(1,1,3,100,0,60000000u,200u,500u,1u,0);
    out=plan(SENSORS);
    candidate_command(0,&pose,&task_queue[task_head],&candidate_vx,&candidate_wz);
    check(!goal_position_reached(&pose,&task_queue[task_head]) && candidate_vx==1 &&
          out.status==BX72_NAV_STATUS_FOLLOWING && out.vx_mm_s==1 && out.plan_valid,
          "sub-two-millimetre remaining distance keeps a checked 1 mm/s command");
    now_us+=100u; pose.stamp_us=now_us;
    /* 前置雷达侧前方的近回波落在与车体前缘相交的栅格。 */
    bx72_nav_lidar_validated(8000u,50u,1u,&pose,now_us,now_us);
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.plan_valid,
          "near-goal command is rechecked against a live obstacle");

    init_fixture(); feed(0,0);
    send_task_config(1,1,3,100,0,60000000u,200u,500u,1u,0);
    out=plan(SENSORS);
    pose.x_mm=1;
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s, "entering the requested tolerance stops near-goal translation");
}

static void test_first_domain_handshake(void)
{
    bx72_nav_output_t out;
    init_fixture();
    bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN/4u]=0u;
    bx72_nav_init();
    bx72_nav_test_mmio[BX72_IO_ODOM_DOMAIN/4u]=1u;
    out=control(SENSORS,1,0,0,0,1);
    check(!start_confirmation_lost && !start_exemption_valid,
          "first domain handshake without a prior startup circle is permitted");
    feed(0,0);
    send_task(1,1,3,2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0 && start_exemption_valid, "first handshake can establish its initial confirmed circle");
}

static void test_closed_loop(void)
{
    bx72_nav_output_t out;
    int32_t wheels[4]={0,0,0,0}, target[4];
    double x=0.0, y=0.0, yaw=0.0;
    uint32_t i,j,angle=0u,moving=0u;
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u);
    for (i=0;i<180u;++i) {
        for (j=0;j<9u;++j) bx72_nav_step(&pose,now_us);
        out=control(SENSORS,1,0,0,0,1);
        check(out.vx_mm_s>=0 && out.vy_mm_s==0 &&
              abs(out.wz_mrad_s)<=(out.vx_mm_s ? 300 : 800),
              "closed loop never commands blind translation or excessive yaw");
        if (out.vx_mm_s>0) ++moving;
        wheel_vector(out.vx_mm_s,out.vy_mm_s,out.wz_mrad_s,target);
        wheel_approach(wheels,target);
        pose.vx_mm_s=(wheels[0]+wheels[1]+wheels[2]+wheels[3])/4;
        pose.vy_mm_s=(-wheels[0]+wheels[1]+wheels[2]-wheels[3])/4;
        pose.wz_mrad_s=(-wheels[0]+wheels[1]-wheels[2]+wheels[3])*1000/(4*NAV_WHEEL_GEOMETRY_MM);
        x+=(pose.vx_mm_s*cos(yaw/1000.0)-pose.vy_mm_s*sin(yaw/1000.0))*0.02;
        y+=(pose.vx_mm_s*sin(yaw/1000.0)+pose.vy_mm_s*cos(yaw/1000.0))*0.02;
        yaw+=pose.wz_mrad_s*0.02;
        if (yaw>3142.0) yaw-=6283.0;
        if (yaw< -3142.0) yaw+=6283.0;
        pose.x_mm=(int32_t)x;
        pose.y_mm=(int32_t)y;
        pose.yaw_mrad=(int32_t)yaw;
        for (j=0;j<18u;++j) {
            angle=(angle+100u)%36000u;
            pose.stamp_us=now_us;
            bx72_nav_lidar_validated(angle,6000,1,&pose,now_us,now_us);
        }
    }
    check(moving>100u && x>300.0,"rolling grid and repeated 20ms control make forward progress");
}

/* 使用真实候选、实时栅格和轮速模型；Jetson 最多每周期执行八个后台切片。 */
static void test_continuous_motion_with_rebase(void)
{
    uint32_t variant;
    uint32_t failures = 0u;
    for (variant=0u; variant<2u; ++variant) {
        bx72_nav_output_t out;
        int32_t wheels[4]={0}, target[4], peak=0;
        double x=0.0,y=0.0,yaw=0.0;
        uint32_t i,j,angle=0u,stops=0u;
        init_fixture(); feed_reduced();
        send_task_speed(1,1,3,4*65536,0,60000000u,400u,500u);
        out=plan(SENSORS);
        check(out.vx_mm_s>0, "continuity fixture starts");
        for (i=0u;i<220u;++i) {
            if (variant && i%25u==12u) {
                bx72_pose_t old_pose=pose, new_pose=pose;
                int32_t correction=((i/25u)&1u) ? -12 : 12;
                new_pose.x_mm+=correction;
                new_pose.yaw_mrad+=correction;
                bx72_nav_rebase(&old_pose,&new_pose); pose=new_pose;
                x+=correction; yaw+=correction;
            }
            for (j=0u;j<8u;++j) bx72_nav_step(&pose,now_us);
            out=control(SENSORS,1,0,0,0,1);
            if (!out.vx_mm_s) {
                ++stops;
                check(out.status == BX72_NAV_STATUS_PLANNING || out.status == BX72_NAV_STATUS_BLOCKED ||
                      out.status == BX72_NAV_STATUS_FOLLOWING,
                      "a temporary stop keeps a valid active navigation state");
            }
            if (out.vx_mm_s>peak) peak=out.vx_mm_s;
            wheel_vector(out.vx_mm_s,out.vy_mm_s,out.wz_mrad_s,target);
            wheel_approach(wheels,target);
            pose.vx_mm_s=(wheels[0]+wheels[1]+wheels[2]+wheels[3])/4;
            pose.vy_mm_s=(-wheels[0]+wheels[1]+wheels[2]-wheels[3])/4;
            pose.wz_mrad_s=(-wheels[0]+wheels[1]-wheels[2]+wheels[3])*1000/
                            (4*NAV_WHEEL_GEOMETRY_MM);
            x+=(pose.vx_mm_s*cos(yaw/1000.0)-pose.vy_mm_s*sin(yaw/1000.0))*0.02;
            y+=(pose.vx_mm_s*sin(yaw/1000.0)+pose.vy_mm_s*cos(yaw/1000.0))*0.02;
            yaw+=pose.wz_mrad_s*0.02;
            pose.x_mm=(int32_t)lround(x); pose.y_mm=(int32_t)lround(y);
            pose.yaw_mrad=(int32_t)lround(yaw);
            for (j=0u;j<3u+(i%5u<3u);++j) {
                angle=(angle+500u)%36000u; pose.stamp_us=now_us;
                bx72_nav_lidar_validated(angle,6000u,1u,&pose,now_us,now_us);
            }
        }
        printf("TRACE moving rebase=%u stops=%u peak=%ld end_x=%ld\n",
               variant,stops,(long)peak,(long)pose.x_mm);
        if (!(stops<=4u && (!variant || stops==0u) && peak==400 && pose.x_mm>1000)) ++failures;
    }
    if (failures) {
        fprintf(stderr, "FAIL continuous movement survives rolling replans and 12mm/12mrad CSM corrections\n");
        performance_regressions += failures;
    }
}

static void test_turn_measured_drift(void)
{
    bx72_nav_output_t out;
    bx72_pose_t beam;
    int32_t end_x,end_y;
    uint8_t unknown;
    uint32_t i,direction;
    init_fixture(); feed(0,0); send_task(1,1,3,-2*65536,0,60000000u);
    out=plan(SENSORS);
    check(out.wz_mrad_s>0 && !out.vx_mm_s, "drift fixture starts a rear-goal observation");
    pose.vx_mm_s=-1; pose.vy_mm_s=1; pose.wz_mrad_s=20;
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.vy_mm_s && out.wz_mrad_s>0,
          "bounded measured wheel asymmetry does not cancel a safe observation turn");
    pose.x_mm=40; pose.y_mm=-9; pose.vx_mm_s=-1; pose.vy_mm_s=-1;
    check(drift_inside_start(40,-7) && drift_inside_start(40,-9) &&
          drift_inside_start(48,0) && !drift_inside_start(48,1),
          "the fixed 48mm allowance is Euclidean and bounded at its true edge");
    check(evaluate_motion(&pose,0,60,0,&end_x,&end_y,&unknown,current_time_us())==1,
          "observed axis-skewed measured drift inside the fixed circle permits a pivot");
    check(evaluate_motion(&pose,-1,0,0,&end_x,&end_y,&unknown,current_time_us())==0,
          "small negative measurement never authorizes a reverse command");
    pose.vx_mm_s=-21;
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s,
          "larger measured reverse motion must stop before autonomous takeover");

    init_fixture(); feed(0,0); send_task(1,1,3,-2*65536,0,60000000u);
    out=plan(SENSORS);
    pose.x_mm=70; pose.vx_mm_s=-1; pose.vy_mm_s=1; pose.wz_mrad_s=20;
    check(evaluate_motion(&pose,0,60,0,&end_x,&end_y,&unknown,current_time_us())==1,
          "small turning drift outside the fixed circle does not stop on unknown cells");
    pose.vx_mm_s=1; pose.vy_mm_s=0;
    check(evaluate_motion(&pose,0,60,0,&end_x,&end_y,&unknown,current_time_us())==1,
          "small forward drift during a pivot does not stop on unknown cells");
    pose.vx_mm_s=-1; pose.vy_mm_s=1;
    out=control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.vy_mm_s && out.wz_mrad_s>0 &&
          start_exemption.x_mm==0 && start_exemption.y_mm==0,
          "small drift permits turning without moving the confirmed circle");
    pose.x_mm=-120; pose.vx_mm_s=100; pose.vy_mm_s=5;
    check(evaluate_motion(&pose,0,60,0,&end_x,&end_y,&unknown,current_time_us())==0 && unknown,
          "larger drift outside the fixed circle still requires observed space");
    pose.vx_mm_s=-1; pose.vy_mm_s=1;

    /* 分别从两个朝向采集真实栅格射线，证明全已知区域可转、占用区域仍停。 */
    beam=pose;
    for (direction=0u;direction<2u;++direction) {
        beam.yaw_mrad=direction ? 3142 : 0;
        bx72_grid_scan_begin();
        for (i=0u;i<360u;++i)
            (void)bx72_grid_lidar_validated(i*100u,6000u,1u,&beam,now_us,now_us);
        bx72_grid_scan_complete(now_us);
    }
    check(evaluate_motion(&pose,0,60,0,&end_x,&end_y,&unknown,current_time_us())==1,
          "bounded drift outside the confirmed circle is permitted only in observed free cells");
    bx72_grid_scan_begin();
    beam.yaw_mrad=0;
    beam.x_mm -= (int32_t)BX72_LIDAR_FORWARD_OFFSET_MM; /* 从更早的车体位姿观测同一障碍格。 */
    (void)bx72_grid_lidar_validated(0u,150u,1u,&beam,now_us,now_us);
    check(evaluate_motion(&pose,0,60,0,&end_x,&end_y,&unknown,current_time_us())==0,
          "observed occupied cells still block the physical footprint during a pivot");
}

/* 50Hz被动记录seq30609..30632的实测速度；地图与起点许可为可控测试夹具，
 * 启动位姿的3mm偏置用于覆盖许可圆内的偏轴轨迹，并非实车锚点实测。 */
static void test_recorded_stationary_drift(void)
{
    static const int8_t measured[][3] = {
        {0,0,0},{0,0,1},{0,0,1},{0,0,0},{3,3,0},{0,0,-1},
        {-3,-3,4},{-3,-3,-1},{-1,-1,-3},{0,0,-2},{0,0,2},{0,0,0},
        {0,0,-1},{0,0,-1},{0,0,2},{0,0,1},{0,0,2},{2,2,1},
        {2,2,-5},{0,0,3},{0,0,-3},{0,0,0},{0,0,1},{0,0,0}
    };
    bx72_nav_output_t out;
    uint32_t i,j,angle=0u;
    init_fixture(); pose.y_mm=3; feed_reduced();
    pose.yaw_mrad=1796;
    send_task(1,1,3,-47448,14025,60000000u);
    out=plan(SENSORS);
    /* 固定一个需要向左观察的引导方向；A* 不再保证复现旧猜测方向。 */
    observe_active=1u;
    observe_settled=0u;
    observe_target_yaw_mrad=wrap_yaw_mrad(pose.yaw_mrad+1000);
    detour_active=1u; detour_x_mm=-400; detour_y_mm=100; path_target_final=0u;
    planner_active=planner_complete=selected_valid=0u;
    out=plan(SENSORS);
    check(out.wz_mrad_s>0 && !out.vx_mm_s,"recorded pivot fixture begins observing");
    pose.x_mm=40; pose.y_mm=-8;
    for (i=0u;i<sizeof(measured)/sizeof(measured[0]);++i) {
        pose.vx_mm_s=measured[i][0]; pose.vy_mm_s=measured[i][1];
        pose.wz_mrad_s=measured[i][2];
        for (j=0u;j<2u;++j) bx72_nav_step(&pose,now_us);
        out=control(SENSORS,1,0,0,0,1);
        check(!out.vx_mm_s && !out.vy_mm_s && out.wz_mrad_s>0,
              "recorded millimetre-scale feedback does not cancel safe in-circle turning");
        for (j=0u;j<3u+(i%5u<3u);++j) {
            angle=(angle+500u)%36000u; pose.stamp_us=now_us;
            bx72_nav_lidar_validated(angle,6000u,1u,&pose,now_us,now_us);
        }
    }
}

static void test_forward_feedback_noise(void)
{
    int32_t end_x, end_y;
    uint8_t unknown;
    init_fixture(); feed(0,0);
    send_task_speed(1,1,3,2*65536,0,60000000u,100u,200u);
    (void)plan(SENSORS);
    pose.x_mm=70; pose.vx_mm_s=4; pose.vy_mm_s=20;
    previous_vx_mm_s=4; previous_vy_mm_s=previous_wz_mrad_s=0;
    check(evaluate_motion(&pose,60,0,1,&end_x,&end_y,&unknown,current_time_us())==1,
          "lateral feedback up to 20mm/s does not trigger drift-only blocking");
    pose.x_mm=-120; pose.vy_mm_s=21;
    check(evaluate_motion(&pose,60,0,0,&end_x,&end_y,&unknown,current_time_us())==0 && unknown,
          "larger lateral drift into genuinely unknown space remains blocked");
    pose.x_mm=70; pose.vx_mm_s=-20; pose.vy_mm_s=0;
    check(evaluate_motion(&pose,60,0,1,&end_x,&end_y,&unknown,current_time_us())==1,
          "reverse feedback at the 20mm/s noise boundary does not block forward motion");
    pose.vx_mm_s=-21;
    check(evaluate_motion(&pose,60,0,1,&end_x,&end_y,&unknown,current_time_us())==0,
          "reverse feedback beyond the noise boundary still inhibits forward planning");
}

static void test_observed_lateral_feedback(void)
{
    bx72_nav_output_t out;
    bx72_pose_t beam;
    int32_t end_x, end_y;
    uint8_t unknown;
    uint32_t direction, i;
    init_fixture(); feed(0,0);
    beam=pose;
    for (direction=0u; direction<2u; ++direction) {
        beam.yaw_mrad=direction ? 3142 : 0;
        bx72_grid_scan_begin();
        for (i=0u; i<360u; ++i)
            (void)bx72_grid_lidar_validated(i*100u,6000u,1u,&beam,now_us,now_us);
        bx72_grid_scan_complete(now_us);
    }
    send_task_speed(1,1,3,2*65536,0,60000000u,100u,200u);
    out=plan(SENSORS);
    check(out.vx_mm_s>0,"lateral feedback fixture starts forward");
    pose.vy_mm_s=40;
    out=control(SENSORS,1,0,0,0,1);
    check(out.vx_mm_s>0,"observed lateral feedback above 20mm/s is not an unconditional stop");
    pose.vy_mm_s=400;
    beam=pose; beam.yaw_mrad=0;
    beam.x_mm -= (int32_t)BX72_LIDAR_FORWARD_OFFSET_MM; /* 重现从先前位姿观察到的同一侧障格。 */
    bx72_grid_scan_begin();
    check(bx72_grid_lidar_validated(28000u,350u,1u,&beam,now_us,now_us),
          "side obstacle fixture is observed");
    check(!(bx72_grid_sweep_occupied(0,0,0,0,BX72_GRID_BODY_RADIUS_MM)&BX72_GRID_CHECK_OCCUPIED),
          "side obstacle starts outside the stationary footprint");
    check(evaluate_motion(&pose,4,0,0,&end_x,&end_y,&unknown,current_time_us())==0 && !unknown,
          "lateral braking sweep still rejects a known obstacle");
}

/* 左右近侧障碍仍属于已知占用，但不再让额外的起点确认余量禁止原地转向。 */
static void test_near_side_physical_envelope(void)
{
    static const uint32_t distances[] = {200u, 220u, 240u, 250u};
    uint32_t i, side;
    for (side = 0u; side < 2u; ++side) for (i = 0u; i < 4u; ++i) {
        bx72_nav_output_t out;
        int32_t end_x, end_y;
        uint8_t unknown;
        init_fixture(); feed(0, 0);
        /* 20cm 回波靠近栅格边界；保留2mm偏置以覆盖两侧对称布局。 */
        pose.y_mm = side ? -2 : 2;
        now_us += 100u; pose.stamp_us = now_us;
        bx72_nav_lidar_validated(side ? 8500u : 27500u, distances[i],
                                 1u, &pose, now_us, now_us);
        if (distances[i] <= 220u)
            check(bx72_grid_sweep_occupied(pose.x_mm, pose.y_mm, pose.x_mm, pose.y_mm,
                                          BX72_GRID_START_CONFIRM_RADIUS_MM) & BX72_GRID_CHECK_OCCUPIED,
                  "near-side obstacle intersects the old 231mm collision circle");
        check(!(bx72_grid_sweep_body_occupied(pose.x_mm, pose.y_mm, pose.x_mm, pose.y_mm,
                     pose.yaw_mrad, 0u, NAV_SWEEP_MARGIN_MM) & BX72_GRID_CHECK_OCCUPIED),
              "near-side obstacle is outside the physical swept footprint");
        check(evaluate_motion(&pose, 0, 30, 0, &end_x, &end_y, &unknown, current_time_us()) == 1 &&
              evaluate_motion(&pose, 0, -30, 0, &end_x, &end_y, &unknown, current_time_us()) == 1,
              "both stationary turns can be evaluated beside a 200-250mm return");
        send_task(1, 1, 3, 65536, 0, 60000000u);
        out = plan(SENSORS);
        check(out.autonomous && out.task_id == 1u,
              "near-side obstacle keeps the task active even if circular search clearance blocks it");
        check(start_exemption.radius_mm == BX72_GRID_START_CONFIRM_RADIUS_MM,
              "physical footprint does not shrink the fixed unknown confirmation circle");
    }
    for (i = 0u; i < 2u; ++i) {
        int32_t end_x, end_y;
        uint8_t unknown;
        bx72_pose_t beam;
        init_fixture(); feed(0, 0);
        now_us += 100u; pose.stamp_us = now_us;
        beam = pose; beam.x_mm -= (int32_t)BX72_LIDAR_FORWARD_OFFSET_MM;
        bx72_grid_scan_begin();
        (void)bx72_grid_lidar_validated(0u, i ? 130u : 100u, 1u, &beam, now_us, now_us);
        check(evaluate_motion(&pose, 0, 30, 0, &end_x, &end_y, &unknown, current_time_us()) == 0 &&
              evaluate_motion(&pose, 10, 0, 1, &end_x, &end_y, &unknown, current_time_us()) == 0,
              "known returns inside the physical body stop candidates");
    }
}

static void test_goal_tolerance_consistency(void)
{
    nav_task_t task = {0};
    bx72_pose_t current = {0};
    bx72_nav_output_t out;
    uint64_t distance_q16;
    int32_t yaw_error, vx, wz;
    task.tolerance_mm=30u; task.yaw_tolerance_mrad=100u;
    task.max_speed_mm_s=100u;
    task.goal_x_mm_q16=25ll*NAV_Q16_ONE;
    task.goal_y_mm_q16=-13ll*NAV_Q16_ONE;
    check(goal_arrived(&current,&task,&distance_q16,&yaw_error),
          "diagonal point within 30mm circle is reported arrived");
    task.goal_x_mm_q16=30ll*NAV_Q16_ONE + NAV_Q16_ONE/2;
    task.goal_y_mm_q16=0;
    check(!goal_arrived(&current,&task,&distance_q16,&yaw_error),
          "position outside the tolerance is not arrived");
    observe_active=0; selected_valid=0;
    candidate_command(0,&current,&task,&vx,&wz);
    check(vx>0,"position outside tolerance does not switch prematurely to final yaw");
    task.goal_x_mm_q16=30ll*NAV_Q16_ONE;
    check(goal_arrived(&current,&task,&distance_q16,&yaw_error),
          "point exactly on the tolerance boundary is arrived");
    task.goal_x_mm_q16=18ll*NAV_Q16_ONE; task.goal_y_mm_q16=24ll*NAV_Q16_ONE;
    check(goal_arrived(&current,&task,&distance_q16,&yaw_error),
          "diagonal point exactly on the tolerance circle is arrived");
    task.goal_y_mm_q16+=NAV_Q16_ONE/100;
    check(!goal_arrived(&current,&task,&distance_q16,&yaw_error),
          "diagonal point outside the tolerance circle is not arrived");
    task.goal_x_mm_q16=2000000000ll*NAV_Q16_ONE;
    check(!goal_arrived(&current,&task,&distance_q16,&yaw_error),
          "distant positions are rejected before squared-distance overflow");
    init_fixture(); feed(0,0);
    send_task_config(1,1,3,1638,-852,60000000u,100u,200u,30u,0);
    out=control(SENSORS,1,0,0,0,1);
    check(out.autonomous && out.status!=BX72_NAV_STATUS_ARRIVED && !out.vx_mm_s,
          "already-reached waypoint waits for stable feedback");
    (void)control(SENSORS,1,0,0,0,1);
    out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_ARRIVED && !out.autonomous,
          "diagonal waypoint inside requested radius completes the real task");

    init_fixture(); feed(0,0);
    send_task_config(1,1,3,65536,0,60000000u,100u,200u,300u,0);
    check(task_count==1u && task_queue[task_head].tolerance_mm==300u,
          "task entry preserves a requested 300 mm position tolerance");
    pose.x_mm=699;
    check(!goal_position_reached(&pose,&task_queue[task_head]),
          "outside the requested 300 mm tolerance is not reached");
    pose.x_mm=701;
    check(goal_position_reached(&pose,&task_queue[task_head]),
          "inside the requested 300 mm tolerance is reached");
    (void)control(SENSORS,1,0,0,0,1);
    (void)control(SENSORS,1,0,0,0,1);
    out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_ARRIVED && !out.autonomous,
          "arrival honors the requested tolerance after settling");
}

/* 四方向目标从同一朝向开始；A* 可先沿已观测区域前往引导点。 */
static void test_directional_closed_loop(void)
{
    static const int goals[4][2]={{2,0},{0,2},{0,-2},{-2,0}};
    uint32_t direction;
    for (direction=0u;direction<4u;++direction) {
        bx72_nav_output_t out;
        int32_t wheels[4]={0},target[4];
        double x=0.0,y=0.0,yaw=0.0;
        uint32_t i,j,angle=0u,turned=0u,settled=0u,forward=0u;
        init_fixture(); feed_reduced();
        send_task(1,1,3,goals[direction][0]*65536,goals[direction][1]*65536,60000000u);
        out=plan(SENSORS);
        for (i=0u;i<1600u;++i) {
            for (j=0u;j<2u;++j) bx72_nav_step(&pose,now_us);
            out=control(SENSORS,1,0,0,0,1);
            check(out.vx_mm_s>=0 && !out.vy_mm_s &&
                  abs(out.wz_mrad_s)<=(out.vx_mm_s ? 300 : 800),
                  "directional closed loop never commands blind translation");
            if (observe_settled) settled=1u;
            if (out.wz_mrad_s && !out.vx_mm_s) ++turned;
            if (out.vx_mm_s) {
                ++forward;
                if (direction)
                    check(!observe_active,
                          "translation does not resume before a required observation completes");
            }
            wheel_vector(out.vx_mm_s,out.vy_mm_s,out.wz_mrad_s,target);
            wheel_approach(wheels,target);
            pose.vx_mm_s=(wheels[0]+wheels[1]+wheels[2]+wheels[3])/4;
            pose.vy_mm_s=(-wheels[0]+wheels[1]+wheels[2]-wheels[3])/4;
            pose.wz_mrad_s=(-wheels[0]+wheels[1]-wheels[2]+wheels[3])*1000/
                            (4*NAV_WHEEL_GEOMETRY_MM);
            if (!out.vx_mm_s && out.wz_mrad_s) --pose.vx_mm_s;
            x+=(pose.vx_mm_s*cos(yaw/1000.0)-pose.vy_mm_s*sin(yaw/1000.0))*0.02;
            y+=(pose.vx_mm_s*sin(yaw/1000.0)+pose.vy_mm_s*cos(yaw/1000.0))*0.02;
            yaw+=pose.wz_mrad_s*0.02;
            if (yaw>3142.0) yaw-=6283.0;
            if (yaw<-3142.0) yaw+=6283.0;
            pose.x_mm=(int32_t)lround(x); pose.y_mm=(int32_t)lround(y);
            pose.yaw_mrad=(int32_t)lround(yaw);
            for (j=0u;j<3u+(i%5u<3u);++j) {
                angle=(angle+500u)%36000u; pose.stamp_us=now_us;
                bx72_nav_lidar_validated(angle,6000u,1u,&pose,now_us,now_us);
            }
            if (x*goals[direction][0]+y*goals[direction][1]>=2800.0) break;
        }
        printf("TRACE direction=%u ticks=%u turn=%u forward=%u pose=%ld,%ld\n",
               direction,i,turned,forward,(long)pose.x_mm,(long)pose.y_mm);
        check(i<1600u && forward>100u && (!direction || (turned>100u && settled)),
              "front left right and rear targets make 1.4m of protected progress");
    }
}

typedef struct { double x, y, radius; } test_obstacle_t;
static test_obstacle_t obstacles[4];
static uint32_t obstacle_count;

/* 在真实局部栅格中模拟障碍；每条射线按当前姿态计算首个命中。 */
static uint32_t obstacle_range(uint32_t angle)
{
    double ray = pose.yaw_mrad / 1000.0 - angle * 3.141592653589793 / 18000.0;
    double dx = cos(ray), dy = sin(ray), nearest = 6000.0;
    double sensor_x = pose.x_mm + BX72_LIDAR_FORWARD_OFFSET_MM * cos(pose.yaw_mrad / 1000.0);
    double sensor_y = pose.y_mm + BX72_LIDAR_FORWARD_OFFSET_MM * sin(pose.yaw_mrad / 1000.0);
    uint32_t i;
    for (i = 0u; i < obstacle_count; ++i) {
        double ox = obstacles[i].x - sensor_x, oy = obstacles[i].y - sensor_y;
        double along = ox * dx + oy * dy;
        double cross_squared = ox * ox + oy * oy - along * along;
        double radius_squared = obstacles[i].radius * obstacles[i].radius;
        if (along > 0.0 && cross_squared <= radius_squared) {
            double distance = along - sqrt(radius_squared - cross_squared);
            if (distance > 0.0 && distance < nearest) nearest = distance;
        }
    }
    return (uint32_t)lround(nearest);
}

static void test_obstacle_closed_loop(void)
{
    uint32_t scenario;
    for (scenario = 0u; scenario < 12u; ++scenario) {
        bx72_nav_output_t out = {0};
        int32_t wheels[4] = {0}, target[4], first_side = 0;
        double x = 0.0, y = 0.0, yaw = 0.0;
        uint32_t i, j, angle = 0u, added = 0u, rebased = 0u, removed = 0u;
        init_fixture();
        obstacle_count = 1u;
        obstacles[0] = (test_obstacle_t){800.0, 0.0, 150.0};
        if (scenario == 1u || scenario == 2u) {
            obstacles[1] = (test_obstacle_t){360.0, scenario == 1u ? 480.0 : -480.0, 150.0};
            obstacle_count = 2u;
        } else if (scenario == 4u) {
            obstacles[0] = (test_obstacle_t){550.0, 0.0, 200.0};
            obstacles[1] = (test_obstacle_t){0.0, 550.0, 200.0};
            obstacles[2] = (test_obstacle_t){0.0, -550.0, 200.0};
            obstacles[3] = (test_obstacle_t){-550.0, 0.0, 200.0};
            obstacle_count = 4u;
        } else if (scenario == 6u || scenario == 7u) {
            obstacles[1] = (test_obstacle_t){40.0, scenario == 6u ? 300.0 : -300.0, 80.0};
            obstacle_count = 2u;
        } else if (scenario == 10u || scenario == 11u) {
            /* 回到目标方向时贴近侧障；覆盖左右镜像和真实前半圈观测。 */
            obstacles[0] = (test_obstacle_t){750.0, scenario == 10u ? -100.0 : 100.0, 150.0};
        }
        for (i = 0u; i <= 144u; ++i) {
            angle = (i % 72u) * 500u;
            now_us += 2000u; pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, obstacle_range(angle), 1u, &pose, now_us, now_us);
        }
        bx72_nav_step(&pose, now_us);
        if (scenario >= 10u) task_test_flags = NAV_TASK_POSITION_ONLY;
        send_task_config(1, 1, 3, scenario >= 10u ? 78643 : 2 * 65536,
                         scenario == 10u ? 5243 : scenario == 11u ? -5243 : 0,
                         120000000u, 200u, 500u, 30u, 0);
        for (i = 0u; i < 5800u; ++i) {
            for (j = 0u; j < 2u; ++j) bx72_nav_step(&pose, now_us);
            out = control(SENSORS, 1, 0, 0, 0, 1);
            /* 网格中心可能有25mm偏移；绕行侧以实际模拟位置确定。 */
            if (!first_side && fabs(y) > 120.0) first_side = y > 0 ? 1 : -1;
            if (scenario == 3u && !added && detour_active && fabs(y) > 120.0) {
                obstacles[1] = (test_obstacle_t){x, y + first_side * 750.0, 100.0};
                obstacle_count = 2u;
                added = 1u;
            }
            if (scenario == 8u && !removed && detour_active && fabs(y) > 120.0) {
                obstacle_count = 0u;
                removed = 1u;
            }
            if (scenario == 9u && !added && detour_active && fabs(y) > 120.0) {
                obstacles[0] = (test_obstacle_t){x, y + first_side * 750.0, 100.0};
                added = 1u;
            }
            if (scenario == 5u && !rebased && detour_active && fabs(y) > 120.0) {
                bx72_pose_t old_pose = pose, new_pose = pose;
                double c = cos(0.020), s = sin(0.020);
                for (j = 0u; j < obstacle_count; ++j) {
                    double ox = obstacles[j].x - x, oy = obstacles[j].y - y;
                    obstacles[j].x = x + 12.0 + ox * c - oy * s;
                    obstacles[j].y = y - 7.0 + ox * s + oy * c;
                }
                new_pose.x_mm += 12; new_pose.y_mm -= 7;
                new_pose.yaw_mrad = wrap_yaw_mrad(new_pose.yaw_mrad + 20);
                bx72_nav_rebase(&old_pose, &new_pose);
                pose = new_pose; x += 12.0; y -= 7.0; yaw += 20.0;
                rebased = 1u;
            }
            wheel_vector(out.vx_mm_s, out.vy_mm_s, out.wz_mrad_s, target);
            wheel_approach(wheels, target);
            pose.vx_mm_s = (wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4;
            pose.vy_mm_s = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4;
            pose.wz_mrad_s = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) * 1000 /
                             (4 * NAV_WHEEL_GEOMETRY_MM);
            x += (pose.vx_mm_s * cos(yaw / 1000.0) - pose.vy_mm_s * sin(yaw / 1000.0)) * 0.02;
            y += (pose.vx_mm_s * sin(yaw / 1000.0) + pose.vy_mm_s * cos(yaw / 1000.0)) * 0.02;
            yaw += pose.wz_mrad_s * 0.02;
            if (yaw > 3142.0) yaw -= 6283.0;
            if (yaw < -3142.0) yaw += 6283.0;
            pose.x_mm = (int32_t)lround(x); pose.y_mm = (int32_t)lround(y);
            pose.yaw_mrad = (int32_t)lround(yaw);
            for (j = 0u; j < obstacle_count; ++j) {
                double ox = obstacles[j].x - x, oy = obstacles[j].y - y;
                double forward = fabs(ox * cos(yaw / 1000.0) + oy * sin(yaw / 1000.0));
                double lateral = fabs(-ox * sin(yaw / 1000.0) + oy * cos(yaw / 1000.0));
                double dx = fmax(0.0, forward - BX72_GRID_BODY_HALF_LENGTH_MM);
                double dy = fmax(0.0, lateral - BX72_GRID_BODY_HALF_WIDTH_MM);
                if (hypot(dx, dy) <= obstacles[j].radius)
                    printf("TRACE physical contact scenario=%u tick=%u position=%.3f,%.3f yaw=%.3f obstacle=%.3f,%.3f,%.3f distances=%.3f,%.3f cmd=%ld,%ld\n",
                           scenario, i, x, y, yaw, obstacles[j].x, obstacles[j].y,
                           obstacles[j].radius, dx, dy, (long)out.vx_mm_s, (long)out.wz_mrad_s);
                check(hypot(dx, dy) > obstacles[j].radius,
                      "closed loop remains outside every obstacle and physical rectangle");
            }
            for (j = 0u; j < 3u + (i % 5u < 3u); ++j) {
                angle = (angle + 500u) % 36000u; pose.stamp_us = now_us;
                bx72_nav_lidar_validated(angle, obstacle_range(angle), 1u, &pose, now_us, now_us);
            }
            if (out.status == BX72_NAV_STATUS_ARRIVED) break;
        }
        printf("TRACE obstacle=%u ticks=%u pose=%ld,%ld yaw=%ld status=%u side=%ld added=%u removed=%u detour=%u observe=%u unknown=%u target=%ld,%ld\n",
               scenario, i, (long)pose.x_mm, (long)pose.y_mm, (long)pose.yaw_mrad,
               out.status, (long)first_side, added, removed, detour_active, observe_active,
               unknown_in_swept_corridor, (long)detour_x_mm, (long)detour_y_mm);
        if (scenario == 4u) {
            check(out.status != BX72_NAV_STATUS_ARRIVED && hypot(x, y) < 300.0,
                  "enclosed robot remains stopped without a false arrival");
        } else {
            check(out.status == BX72_NAV_STATUS_ARRIVED,
                  "robot reaches the goal by passing the observed obstacles");
            if (scenario == 1u || scenario == 2u)
                check(first_side == (scenario == 1u ? -1 : 1),
                      "mirrored side obstacles select the clear side");
            if (scenario == 3u) check(added, "new obstacle appears while following the detour");
            if (scenario == 5u) check(rebased, "detour continues after a CSM rebase");
            if (scenario == 8u) check(removed, "obstacle is removed while following the detour");
            if (scenario == 9u) check(added, "obstacle moves while following the detour");
            if (scenario == 10u) check(first_side, "side-obstacle recovery uses the detour");
        }
    }
}

typedef struct {
    int32_t peak_vx;
    uint32_t zero_ticks;
} rebase_ramp_result_t;

/* 固定机身模拟台架：每20ms控制，每100ms注入小幅同一时刻CSM修正。 */
static rebase_ramp_result_t run_rebase_ramp(int32_t goal_q16, int rebase)
{
    rebase_ramp_result_t result = {0, 0u};
    bx72_nav_output_t out;
    uint32_t i, j, angle = 0u;
    int32_t last_vx;

    init_fixture();
    feed(0, 0);
    send_task_config(1, 1, 3, goal_q16, 0, 60000000u, 400u, 200u, 30u, 0);
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "rebase ramp starts with a valid forward candidate");
    last_vx = out.vx_mm_s;
    for (i = 0u; i < 120u; ++i) {
        if (rebase && i % 5u == 0u) {
            bx72_pose_t old_pose = pose, new_pose = pose;
            int32_t correction = ((i / 5u) & 1u) ? -1 : 1;
            new_pose.x_mm += correction;
            new_pose.yaw_mrad += correction;
            bx72_nav_rebase(&old_pose, &new_pose);
            pose = new_pose;
        }
        /* 分片后台预算：每个控制周期完成两个候选，不伪造整体瞬时完成。 */
        for (j = 0u; j < 2u; ++j) bx72_nav_step(&pose, now_us);
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.vx_mm_s >= 0 && out.vx_mm_s <= last_vx + 4,
              "rebase ramp respects the requested 200 mm/s2 acceleration");
        if (out.vx_mm_s > result.peak_vx) result.peak_vx = out.vx_mm_s;
        if (!out.vx_mm_s) ++result.zero_ticks;
        last_vx = out.vx_mm_s;
        for (j = 0u; j < 18u; ++j) {
            angle = (angle + 100u) % 36000u;
            pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, 6000u, 1u, &pose, now_us, now_us);
        }
    }
    return result;
}

static void test_periodic_rebase_ramp(void)
{
    rebase_ramp_result_t near_plain = run_rebase_ramp(9830, 0);
    rebase_ramp_result_t near_rebase = run_rebase_ramp(9830, 1);
    rebase_ramp_result_t far_plain = run_rebase_ramp(2 * 65536, 0);
    rebase_ramp_result_t far_rebase = run_rebase_ramp(2 * 65536, 1);
    printf("TRACE rebase ramp near=%ld/%ld zero=%u/%u far=%ld/%ld zero=%u/%u\n",
           (long)near_plain.peak_vx, (long)near_rebase.peak_vx,
           near_plain.zero_ticks, near_rebase.zero_ticks,
           (long)far_plain.peak_vx, (long)far_rebase.peak_vx,
           far_plain.zero_ticks, far_rebase.zero_ticks);
    /* 固定位姿不会跨过第一个中间点；该用例只比较修正前后的加速表现。 */
    check(near_plain.peak_vx >= 60 && far_plain.peak_vx >= 60,
          "fixed-pose clear paths reach a nonzero controlled speed");
    check(near_rebase.peak_vx == near_plain.peak_vx && far_rebase.peak_vx == far_plain.peak_vx,
          "small periodic CSM corrections do not reduce fixed-pose acceleration");
    check(near_rebase.zero_ticks == near_plain.zero_ticks &&
          far_rebase.zero_ticks == far_plain.zero_ticks,
          "small periodic CSM corrections do not introduce forward stop ticks");
}

static rebase_ramp_result_t run_rebase_turn(int observe, int rebase)
{
    rebase_ramp_result_t result = {0, 0u};
    bx72_nav_output_t out;
    uint32_t i, j, angle = 0u;
    int32_t last_wz;

    init_fixture(); feed(0, 0);
    if (observe)
        send_task_config(1, 1, 3, -2 * 65536, 0, 60000000u, 400u, 200u, 30u, 0);
    else
        send_task_config(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u, 32768);
    out = plan(SENSORS);
    check(!out.vx_mm_s && out.wz_mrad_s != 0, "turn fixture starts rotating");
    last_wz = abs(out.wz_mrad_s);
    for (i = 0u; i < 120u; ++i) {
        if (rebase && i % 5u == 0u) {
            bx72_pose_t old_pose = pose, new_pose = pose;
            int32_t correction = ((i / 5u) & 1u) ? -1 : 1;
            new_pose.x_mm += correction;
            new_pose.yaw_mrad += correction;
            bx72_nav_rebase(&old_pose, &new_pose);
            pose = new_pose;
        }
        for (j = 0u; j < 2u; ++j) bx72_nav_step(&pose, now_us);
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(!out.vx_mm_s && !out.vy_mm_s && abs(out.wz_mrad_s) <= last_wz + 30,
              "rebased turn respects angular acceleration");
        if (abs(out.wz_mrad_s) > result.peak_vx) result.peak_vx = abs(out.wz_mrad_s);
        if (!out.wz_mrad_s) ++result.zero_ticks;
        last_wz = abs(out.wz_mrad_s);
        for (j = 0u; j < 18u; ++j) {
            angle = (angle + 100u) % 36000u;
            pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, 6000u, 1u, &pose, now_us, now_us);
        }
    }
    return result;
}

static void test_periodic_rebase_turn(void)
{
    rebase_ramp_result_t final_plain = run_rebase_turn(0, 0);
    rebase_ramp_result_t final_rebase = run_rebase_turn(0, 1);
    rebase_ramp_result_t observe_plain = run_rebase_turn(1, 0);
    rebase_ramp_result_t observe_rebase = run_rebase_turn(1, 1);
    printf("TRACE rebase turn final=%ld/%ld zero=%u/%u observe=%ld/%ld zero=%u/%u\n",
           (long)final_plain.peak_vx, (long)final_rebase.peak_vx,
           final_plain.zero_ticks, final_rebase.zero_ticks,
           (long)observe_plain.peak_vx, (long)observe_rebase.peak_vx,
           observe_plain.zero_ticks, observe_rebase.zero_ticks);
    check(final_plain.peak_vx >= 200 && observe_plain.peak_vx >= 200,
          "final and observation turns accelerate without rebase");
    check(final_rebase.peak_vx >= 200 && observe_rebase.peak_vx >= 200,
          "small periodic CSM corrections do not prevent turn acceleration");
    check(final_rebase.zero_ticks == final_plain.zero_ticks &&
          observe_rebase.zero_ticks == observe_plain.zero_ticks,
          "small periodic CSM corrections do not introduce turn stop ticks");
}

static void test_manual_during_rebase(void)
{
    bx72_nav_output_t out;
    uint32_t i, j, angle = 0u;
    int32_t previous = 0;
    init_fixture(); feed(0, 0);
    for (i = 0u; i < 60u; ++i) {
        if (i % 5u == 0u) {
            bx72_pose_t old_pose = pose, new_pose = pose;
            int32_t correction = ((i / 5u) & 1u) ? -1 : 1;
            new_pose.x_mm += correction;
            new_pose.yaw_mrad += correction;
            bx72_nav_rebase(&old_pose, &new_pose);
            pose = new_pose;
        }
        out = control(SENSORS & ~BX72_STATUS_AUTO, 1, 400, 0, 0, 1);
        check(out.vx_mm_s >= previous && out.vx_mm_s <= previous + 10,
              "manual acceleration remains continuous during rebase");
        previous = out.vx_mm_s;
        for (j = 0u; j < 18u; ++j) {
            angle = (angle + 100u) % 36000u;
            pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, 6000u, 1u, &pose, now_us, now_us);
        }
    }
    check(out.vx_mm_s == 400, "manual command reaches 400 despite periodic rebase");
    out = control((SENSORS & ~BX72_STATUS_AUTO) | BX72_STATUS_GUARD_STOP, 1, 400, 0, 0, 1);
    check(!out.vx_mm_s, "manual guard stop remains immediate after rebase");
}

static bx72_nav_output_t rebase_forward_fixture(void)
{
    bx72_nav_output_t out;
    init_fixture(); feed(0, 0);
    send_task_speed(1, 1, 3, 2 * 65536, 0, 60000000u, 400u, 200u);
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "rebase safety fixture starts forward");
    return out;
}

static void test_rebase_safety(void)
{
    bx72_nav_output_t out;
    bx72_pose_t old_pose, new_pose;
    uint32_t i;

    (void)rebase_forward_fixture();
    old_pose = new_pose = pose; new_pose.x_mm += 11;
    bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.vx_mm_s>0 && out.plan_valid && !planner_complete,
          "position correction keeps a live-checked candidate while replanning");

    (void)rebase_forward_fixture();
    old_pose = new_pose = pose; new_pose.yaw_mrad += 11;
    bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.vx_mm_s>0 && out.plan_valid && !planner_complete,
          "bounded angular correction keeps a live-checked candidate while replanning");

    (void)rebase_forward_fixture();
    for (i = 0u; i < 11u; ++i) {
        old_pose = new_pose = pose; new_pose.x_mm += 10;
        bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
        out = control(SENSORS, 1, 0, 0, 0, 1);
        if (i < 10u) check(out.vx_mm_s > 0, "bounded cumulative correction retains motion");
    }
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "large cumulative goal reprojection invalidates the old guide");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "a changed goal replans without discarding the task");

    (void)rebase_forward_fixture();
    for (i = 0u; i < 26u; ++i) {
        old_pose = new_pose = pose; new_pose.yaw_mrad += 10;
        bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
        out = control(SENSORS, 1, 0, 0, 0, 1);
        if (!out.vx_mm_s) break;
        bx72_nav_lidar_validated((i + 1u) * 500u, 6000u, 1u,
                                 &pose, now_us, now_us);
    }
    check(i > 0u && i < 26u && !out.wz_mrad_s && !out.plan_valid,
          "cumulative yaw that moves a distant goal sufficiently triggers replanning");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "goal reprojection resumes after replanning");

    init_fixture(); feed(0, 0);
    send_task_config(1, 1, 3, 9830, 0, 60000000u, 400u, 200u, 30u, 0);
    out = plan(SENSORS);
    for (i = 0u; i < 4u; ++i) {
        old_pose = new_pose = pose; new_pose.y_mm += 10;
        bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
        out = control(SENSORS, 1, 0, 0, 0, 1);
        if (i < 3u) check(out.vx_mm_s > 0, "small goal-bearing changes retain motion");
    }
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "large goal-bearing changes invalidate an outdated guide");

    (void)rebase_forward_fixture();
    old_pose = new_pose = pose; new_pose.x_mm += 1;
    bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
    now_us += 100u; pose.stamp_us = now_us;
    bx72_nav_lidar_validated(8000u, 50u, 1u, &pose, now_us, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "latest obstacle stops a retained candidate after rebase");

    (void)rebase_forward_fixture();
    out = control(SENSORS | BX72_STATUS_GUARD_STOP, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s, "guard stop clears motion");
    for (i = 0u; i < 10u; ++i) bx72_nav_step(&pose, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "guard recovery must obtain a new candidate");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "planning resumes after guard recovery");

    (void)rebase_forward_fixture();
    out = control(SENSORS & ~BX72_STATUS_LIDAR_OK, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s, "sensor loss clears motion");
    for (i = 0u; i < 10u; ++i) bx72_nav_step(&pose, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "sensor recovery must obtain a new candidate");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "planning resumes after sensor recovery");

    (void)rebase_forward_fixture();
    pose.flags = 0;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s, "pose loss clears motion");
    pose.flags = BX72_POSE_VALID;
    for (i = 0u; i < 10u; ++i) bx72_nav_step(&pose, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "pose recovery must obtain a new candidate");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "planning resumes after pose recovery");

    (void)rebase_forward_fixture();
    now_us += 1600000u; pose.stamp_us = now_us;
    bx72_nav_lidar_validated(100u, 6000u, 1u, &pose, now_us, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s, "expired complete scan stops motion");
    feed(0, 0);
    for (i = 0u; i < 10u; ++i) bx72_nav_step(&pose, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "fresh scan after expiry must obtain a new candidate");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "planning resumes after scan recovery");

#ifdef BX72_NAV_TEST_TIMED
    (void)rebase_forward_fixture();
    old_pose = new_pose = pose; new_pose.x_mm += 1;
    bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
    timer_test_now = now_us; timer_test_step = 6000u;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    timer_test_step = 0u;
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "rebase cannot bypass the real evaluation timeout");
#endif
}

static void test_retained_goal_geometry(void)
{
    bx72_nav_output_t out;
    uint32_t i;
    int32_t before;

    init_fixture(); feed(0, 0);
    send_task_config(1, 1, 3, 9830, 0, 60000000u, 400u, 200u, 30u, 0);
    out = plan(SENSORS);
    for (i = 0u; i < 10u; ++i) out = control(SENSORS, 1, 0, 0, 0, 1);
    before = out.vx_mm_s;
    pose.x_mm = 90;
    for (i = 0u; i < 5u; ++i) {
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.vx_mm_s <= before, "new goal distance prevents further acceleration");
        before = out.vx_mm_s;
    }
    check(out.vx_mm_s <= 30, "current goal distance limits a retained candidate");

    init_fixture(); feed(0, 0);
    send_task_config(1, 1, 3, 5243, 0, 60000000u, 400u, 200u, 30u, 32768);
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "final yaw fixture starts forward");
    pose.x_mm = 60; pose.yaw_mrad = 399;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid && out.autonomous,
          "entering position tolerance stops forward motion before final yaw");
    out = plan(SENSORS);
    check(!out.vx_mm_s && out.wz_mrad_s > 0,
          "final yaw is planned after position tolerance");
    pose.yaw_mrad = 601;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "final yaw crossing stops an outdated turn");
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    out = plan(SENSORS);
    check(!out.vx_mm_s && out.wz_mrad_s < 0,
          "final yaw crossing replans the reverse turn");

    init_fixture(); feed(0, 0);
    pose.x_mm = 52; pose.stamp_us = now_us;
    bx72_nav_step(&pose, now_us);
    send_task_config(1, 1, 3, 5243, 0, 60000000u, 400u, 200u, 30u, 32768);
    out = plan(SENSORS);
    check(!out.vx_mm_s && out.wz_mrad_s > 0, "inside tolerance starts final yaw");
    {
        bx72_pose_t old_pose = pose, new_pose = pose;
        new_pose.x_mm = 48;
        bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
    }
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "leaving position tolerance stops the old final yaw candidate");
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "leaving position tolerance replans toward the goal");

    init_fixture(); feed(0, 0);
    send_task_config(1, 1, 3, -2 * 65536, 0, 60000000u, 400u, 200u, 30u, 0);
    (void)plan(SENSORS);
    check(observe_active, "rear-goal search starts an observation before checking its target crossing");
    pose.yaw_mrad = wrap_yaw_mrad(observe_target_yaw_mrad - 101);
    out = plan(SENSORS);
    check(out.wz_mrad_s > 0, "observation crossing fixture begins positive turn");
    pose.yaw_mrad = wrap_yaw_mrad(observe_target_yaw_mrad + 101);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.wz_mrad_s,
          "observation target crossing stops the outdated turn");
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    out = plan(SENSORS);
    check(out.wz_mrad_s < 0, "observation target crossing replans the reverse turn");

    init_fixture(); feed(0, 0);
    send_task_config(1, 1, 3, -2 * 65536, 0, 60000000u, 400u, 200u, 30u, 0);
    /* 覆盖精确正后方的观察目标；A* 栅格中心通常不恰好落在 ±π。 */
    observe_active = 1u;
    observe_target_yaw_mrad = -NAV_PI_MRAD;
    out = plan(SENSORS);
    check(out.wz_mrad_s < 0, "negative pi boundary starts a negative turn");
    {
        bx72_pose_t old_pose = pose, new_pose = pose;
        new_pose.yaw_mrad += 1;
        bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
    }
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.wz_mrad_s < 0, "small correction retains the pi-boundary turn");
    for (i = 0u; i < 3u; ++i) {
        bx72_pose_t old_pose = pose, new_pose = pose;
        new_pose.yaw_mrad -= 1;
        bx72_nav_rebase(&old_pose, &new_pose); pose = new_pose;
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.wz_mrad_s < 0, "negative-to-positive pi wrap retains turn direction");
    }
}

/* 斜向进入欧氏圆后仍需转向，出圆时不能沿用终点转向候选。 */
static void test_diagonal_goal_transition(void)
{
    bx72_nav_output_t out;

    init_fixture();
    pose.x_mm = 47; pose.y_mm = -17; pose.yaw_mrad = 200;
    feed(0, 0);
    send_task_config(1, 1, 3, 5243, 0, 60000000u, 400u, 200u, 30u, 32768);
    out = plan(SENSORS);
    check(out.autonomous && out.task_id == 1u && out.vx_mm_s > 0,
          "diagonal outside-circle fixture starts forward");

    /* 离目标约 (25, 13) mm：欧氏距离约 28.2 mm，旧近似距离约 30.4 mm。 */
    pose.x_mm = 55; pose.y_mm = -13;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.autonomous && out.task_id == 1u && out.status != BX72_NAV_STATUS_ARRIVED &&
          !out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "diagonal entry invalidates forward candidate while final yaw remains");
    out = plan(SENSORS);
    check(out.autonomous && out.task_id == 1u && out.status != BX72_NAV_STATUS_ARRIVED &&
          !out.vx_mm_s && out.wz_mrad_s > 0,
          "diagonal inside-circle position plans the requested final yaw");

    /* 离目标约 (22, 21) mm：欧氏距离约 30.4 mm，旧近似截断后为 30 mm。 */
    pose.x_mm = 58; pose.y_mm = -21;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.autonomous && out.task_id == 1u && !out.vx_mm_s &&
          !out.wz_mrad_s && !out.plan_valid,
          "diagonal exit invalidates the final-yaw candidate");
    out = plan(SENSORS);
    check(out.autonomous && out.task_id == 1u && out.vx_mm_s > 0,
          "diagonal outside-circle position replans toward the goal");

    pose.x_mm = 55; pose.y_mm = -13;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.autonomous && !out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
          "diagonal re-entry invalidates the renewed forward candidate");
    out = plan(SENSORS);
    check(out.autonomous && !out.vx_mm_s && out.wz_mrad_s > 0,
          "diagonal re-entry resumes the final-yaw plan");
    pose.yaw_mrad = 500;
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.autonomous && out.status != BX72_NAV_STATUS_ARRIVED && !out.wz_mrad_s,
          "entering final yaw tolerance commands zero before confirming arrival");
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.autonomous && out.task_id == 1u && out.status == BX72_NAV_STATUS_ARRIVED &&
          !out.vx_mm_s && !out.wz_mrad_s,
          "diagonal position and completed final yaw report arrival");
}

static void test_task_speed_400(void)
{
    bx72_nav_output_t out;
    int32_t wheels[4] = {0, 0, 0, 0}, old_wheels[4], target[4];
    int32_t end_x, end_y, peak_output = 0, previous_output;
    uint8_t unknown;
    uint32_t i, j, angle = 0u;
    double x = 0.0, y = 0.0, yaw = 0.0;

    init_fixture(); feed(0,0);
    send_task_speed(1,1,3,4*65536,0,60000000u,400u,1000u);
    out = plan(SENSORS);
    check(task_count == 1u && task_queue[task_head].max_speed_mm_s == 400u &&
          out.autonomous && out.task_id == 1u && out.vx_mm_s > 0,
          "a complete 400 mm/s task is accepted and begins moving");
    check(selected_valid && selected_vx > 0 && selected_vx <= 400,
          "the first A* guide respects the accepted 400 mm/s limit");
    check(evaluate_motion(&pose, 400, 300, 0, &end_x, &end_y, &unknown, current_time_us()) == 1 &&
          evaluate_motion(&pose, 501, 0, 0, &end_x, &end_y, &unknown, current_time_us()) == 0,
          "the real motion predictor accepts bounded turning and rejects wheels above 500 mm/s");

    for (i = 0u; i < 160u; ++i) {
        for (j = 0u; j < 9u; ++j) bx72_nav_step(&pose, now_us);
        previous_output = out.vx_mm_s;
        out = control(SENSORS,1,0,0,0,1);
        check(out.vx_mm_s >= 0 && out.vx_mm_s <= 400 && out.vy_mm_s == 0 &&
              abs(out.wz_mrad_s) <= NAV_AUTO_MAX_YAW_MRAD_S && out.vx_mm_s <= previous_output + 20,
              "400 mm/s task output respects speed, yaw and requested acceleration");
        if (out.vx_mm_s > peak_output) peak_output = out.vx_mm_s;
        wheel_vector(out.vx_mm_s, out.vy_mm_s, out.wz_mrad_s, target);
        for (j = 0u; j < 4u; ++j) {
            check(abs(target[j]) <= 500, "controlled command stays inside wheel limit");
            old_wheels[j] = wheels[j];
        }
        wheel_approach(wheels, target);
        for (j = 0u; j < 4u; ++j)
            check(abs(wheels[j] - old_wheels[j]) <= 10, "wheel prediction retains 10 mm/s step");
        pose.vx_mm_s = (wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4;
        pose.vy_mm_s = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4;
        pose.wz_mrad_s = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) * 1000 /
                           (4 * NAV_WHEEL_GEOMETRY_MM);
        x += (pose.vx_mm_s * cos(yaw / 1000.0) - pose.vy_mm_s * sin(yaw / 1000.0)) * 0.02;
        y += (pose.vx_mm_s * sin(yaw / 1000.0) + pose.vy_mm_s * cos(yaw / 1000.0)) * 0.02;
        yaw += pose.wz_mrad_s * 0.02;
        pose.x_mm = (int32_t)lround(x); pose.y_mm = (int32_t)lround(y);
        pose.yaw_mrad = (int32_t)lround(yaw);
        for (j = 0u; j < 18u; ++j) {
            angle = (angle + 100u) % 36000u;
            pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, 6000u, 1u, &pose, now_us, now_us);
        }
    }
    check(peak_output == 400, "clear long path can reach 400 mm/s");

    init_fixture(); feed(1,0);
    send_task_speed(1,1,3,4*65536,0,60000000u,400u,500u);
    out = plan(SENSORS);
    check(!out.vx_mm_s && !out.vy_mm_s, "unknown front remains blocked for a 400 mm/s task");

    init_fixture(); feed(0,0);
    send_task_speed(1,1,3,4*65536,0,60000000u,400u,500u);
    out = plan(SENSORS);
    check(out.vx_mm_s > 0, "400 mm/s obstacle fixture begins motion");
    pose.vx_mm_s = previous_vx_mm_s = 400;
    now_us += 100u; pose.stamp_us = now_us;
    bx72_nav_lidar_validated(100u, 250u, 1u, &pose, now_us, now_us);
    out = control(SENSORS,1,0,0,0,1);
    check(!out.vx_mm_s && !out.plan_valid,
          "obstacle in the 400 mm/s braking sweep forces a stop");
    puts("PASS 400 mm/s task: acceptance, controlled trajectory and safety");
}

static void test_progress_final_yaw_and_expiry(void)
{
    bx72_nav_output_t out;
    int32_t previous=0;
    uint32_t i;
    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u); out=plan(SENSORS);
    now_us+=21000000u;
    feed(0,0);
    out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_BLOCKED && (bx72_nav_diagnostic_status()&BX72_NAV_DIAG_PROGRESS_BLOCKED),
          "twenty seconds without target progress becomes blocked");

    init_fixture(); feed(0,0); pose.yaw_mrad=-1000;
    send_task(1,1,3,0,0,60000000u);
    out=control(SENSORS,1,0,0,0,1);
    for (i=0;i<10u;++i) bx72_nav_step(&pose,now_us);
    for (i=0;i<8u;++i) {
        out=control(SENSORS,1,0,0,0,1);
        check(out.vx_mm_s==0 && out.vy_mm_s==0 && abs(out.wz_mrad_s)<=300 &&
              abs(out.wz_mrad_s-previous)<=30, "final heading uses zero translation and bounded yaw slope");
        previous=out.wz_mrad_s;
    }
    check(previous>0,"final yaw drives toward the requested heading");

    init_fixture(); feed(0,0); send_task(1,1,3,2*65536,0,60000000u); out=plan(SENSORS);
    now_us+=1500001u;
    pose.stamp_us=now_us;
    bx72_nav_lidar_validated(100u,6000u,1,&pose,now_us,now_us);
    out=control(SENSORS,1,0,0,0,1);
    check(!(bx72_nav_diagnostic_status()&BX72_NAV_DIAG_MAP_FRESH) && out.vx_mm_s==0,
          "old complete scan expires without being extended by one new beam");
    for (i=0;i<3u;++i) {
        now_us+=0x60000000u;
        out=control(SENSORS & ~BX72_STATUS_AUTO,1,100,0,0,1);
    }
    check(!(bx72_nav_diagnostic_status()&BX72_NAV_DIAG_MAP_FRESH) && out.vx_mm_s==0,
          "expired scan cannot revive after timestamp wrap");
}

static void test_sealed_stop_until_expiry(void)
{
    bx72_nav_output_t out;
    bx72_pose_t beam;
    uint32_t i, turn_seen=0u;
    init_fixture();
    /* 在先前位姿观测到的障碍，如今与车体相交，不能由新外参清除。 */
    beam = pose; beam.x_mm -= (int32_t)BX72_LIDAR_FORWARD_OFFSET_MM;
    for (i=0;i<=720u;++i) {
        uint32_t angle=(i%360u)*100u;
        now_us+=400u; beam.stamp_us=now_us;
        bx72_nav_lidar_validated(angle,200u,1,&beam,now_us,now_us);
    }
    bx72_nav_step(&pose,now_us);
    send_task(1,1,3,2*65536,0,2000000u);
    for (i=0;i<20u;++i) {
        out=plan(SENSORS);
        if (out.wz_mrad_s) ++turn_seen;
        check(!out.vx_mm_s && !out.vy_mm_s && !out.plan_valid,
              "known obstacle inside initial footprint forbids translation");
    }
    now_us+=2200000u;
    out=control(SENSORS,1,0,0,0,1);
    check(out.status==BX72_NAV_STATUS_STALE && !out.autonomous && !out.vx_mm_s,
          "sealed task expires without false arrival");
    check(turn_seen==0u,"known obstacle inside footprint also forbids observation turn");

    /* 偏航后的封闭障碍仍不得产生平移、旋转或虚假的到达状态。 */
    init_fixture(); beam = pose;
    beam.x_mm -= (int32_t)BX72_LIDAR_FORWARD_OFFSET_MM;
    for (i=0;i<=720u;++i) {
        uint32_t angle=(i%360u)*100u;
        now_us+=400u; beam.stamp_us=now_us;
        bx72_nav_lidar_validated(angle,200u,1,&beam,now_us,now_us);
    }
    bx72_nav_step(&pose,now_us); pose.yaw_mrad=1000;
    send_task(1,1,3,2*65536,0,2000000u);
    for (i=0;i<20u;++i) {
        out=plan(SENSORS);
        check(!out.vx_mm_s && !out.vy_mm_s && !out.wz_mrad_s && !out.plan_valid,
              "a sealed footprint blocks every observation direction");
        check(out.autonomous && out.status != BX72_NAV_STATUS_ARRIVED,
              "sealed observation remains pending until expiry");
    }
}

/* 调用真实任务解析、候选、输出复核及轮速模型；固定航向用于观察完整爬升。 */
/* 位置航点没有终点航向要求；显式航向任务仍保留原有转向语义。 */
static void test_position_only_waypoints(void)
{
    bx72_nav_output_t out;
    uint32_t i;

    init_fixture(); feed(0, 0);
    send_task_config(1, 1, 3, 6554, 0, 60000000u, 200u, 500u, 30u, 0);
    pose.x_mm = 80; pose.yaw_mrad = 1200;
    out = plan(SENSORS);
    check(out.autonomous && !out.vx_mm_s && out.wz_mrad_s < 0,
          "legacy waypoint still turns toward its explicitly requested final heading");

    init_fixture(); feed(0, 0);
    task_test_flags = 2u;
    send_task_config_yaw_tolerance(1, 1, 3, 6554, 0, 60000000u, 200u, 500u, 30u, 0, 1u);
    check(task_count == 1u && task_queue[task_head].flags == 2u,
          "position-only waypoint flag is accepted at the task entry");
    pose.x_mm = 80; pose.yaw_mrad = 1200; pose.vx_mm_s = 50; pose.wz_mrad_s = 21;
    previous_vx_mm_s = 50; previous_wz_mrad_s = 200;
    for (i = 0u; i < 5u; ++i) {
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.autonomous && out.status != BX72_NAV_STATUS_ARRIVED &&
              !out.vx_mm_s && !out.wz_mrad_s,
              "position-only arrival first stops and waits for measured motion");
    }
    pose.vx_mm_s = 0; pose.wz_mrad_s = 10;
    for (i = 0u; i < 5u; ++i) out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.status == BX72_NAV_STATUS_ARRIVED && !out.autonomous && !out.wz_mrad_s,
          "settled position-only waypoint arrives despite heading mismatch");

    init_fixture(); feed(0, 0);
    task_test_flags = 2u;
    send_task_config(1, 1, 3, 6554, 0, 60000000u, 200u, 500u, 30u, 0);
    pose.x_mm = 69; pose.yaw_mrad = 0;
    out = plan(SENSORS);
    check(out.autonomous && out.status != BX72_NAV_STATUS_ARRIVED,
          "position-only does not accept a point outside the requested tolerance");
    pose.x_mm = 80; pose.wz_mrad_s = 0;
    for (i = 0u; i < 5u; ++i) out = control(SENSORS, 1, 0, 0, 0, 1);
    check(out.status == BX72_NAV_STATUS_ARRIVED,
          "position-only accepts the same waypoint only after entering the tolerance");

    init_fixture(); feed(0, 0);
    task_test_flags = 2u;
    send_task_config(1, 1, 3, 6554, 0, 60000000u, 200u, 500u, 30u, 0);
    task_test_flags = 0u;
    send_task_config(2, 1, 3, 32768, 0, 60000000u, 200u, 500u, 30u, 0);
    pose.x_mm = 80; pose.yaw_mrad = 1200;
    for (i = 0u; i < 4u; ++i) out = control(SENSORS, 1, 0, 0, 0, 1);
    check(task_count == 1u && task_queue[task_head].task_id == 2u &&
          out.autonomous && out.task_id == 2u,
          "position-only arrival advances the queued waypoint without terminal rotation");

    init_fixture(); feed(0, 0);
    task_test_flags = 3u;
    send_task_config(1, 1, 2, 65536, 0, 60000000u, 200u, 500u, 30u, 0);
    check(task_count == 1u && task_queue[task_head].flags == 3u,
          "position-only and disconnect permission are independent flag bits");
    out = plan(SENSORS | BX72_STATUS_LINK_DOWN);
    check(out.autonomous && out.vx_mm_s > 0,
          "the existing disconnect permission remains effective when combined");

    init_fixture(); feed(0, 0);
    task_test_flags = 2u;
    send_task_config(1, 1, 2, 65536, 0, 60000000u, 200u, 500u, 30u, 0);
    out = control(SENSORS | BX72_STATUS_LINK_DOWN, 1, 0, 0, 0, 1);
    check(!out.autonomous && out.status == BX72_NAV_STATUS_STALE && !out.vx_mm_s,
          "position-only does not itself allow a disconnected cooperative task");

}

static void test_pivot_speed_targets(void)
{
    static const int32_t requested_yaw_q16[4] = {13107, -13107, 65536, -65536};
    static const int32_t targets[4] = {600, -600, 800, -800};
    uint32_t case_index, i, j;
    for (case_index = 0u; case_index < 4u; ++case_index) {
        bx72_nav_output_t out;
        int32_t wheels[4] = {0, 0, 0, 0}, wheel_target[4], end_x, end_y;
        int32_t sign = targets[case_index] > 0 ? 1 : -1;
        int32_t target_abs = abs(targets[case_index]);
        int32_t goal_yaw;
        uint8_t unknown;

        init_fixture(); feed(0, 0);
        send_task_config(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u,
                         requested_yaw_q16[case_index]);
        out = plan(SENSORS);
        check(selected_valid && !selected_vx && selected_wz == targets[case_index],
              "requested final heading selects the signed 600 or 800 pivot target");
        goal_yaw = task_queue[task_head].final_yaw_mrad;
        for (i = 0u; i < 30u; ++i) {
            int32_t expected = (int32_t)(i + 1u) * 30;
            if (expected > target_abs) expected = target_abs;
            check(out.autonomous && !out.vx_mm_s && !out.vy_mm_s &&
                  out.wz_mrad_s == sign * expected,
                  "pivot output follows its unchanged 30 mrad/s per tick acceleration limit");
            wheel_vector(out.vx_mm_s, out.vy_mm_s, out.wz_mrad_s, wheel_target);
            for (j = 0u; j < 4u; ++j)
                check(abs(wheel_target[j]) <= NAV_WHEEL_LIMIT_MM_S,
                      "pivot target stays inside the physical wheel speed limit");
            wheel_approach(wheels, wheel_target);
            pose.wz_mrad_s = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) * 1000 /
                             (4 * NAV_WHEEL_GEOMETRY_MM);
            check(abs(pose.wz_mrad_s) <= 800,
                  "wheel feedback remains inside the motion predictor yaw bound");
            pose.stamp_us = now_us;
            bx72_nav_lidar_validated((i + 1u) * 100u, 6000u, 1u,
                                     &pose, now_us, now_us);
            if (i + 1u < 30u) out = control(SENSORS, 1, 0, 0, 0, 1);
        }
        check(out.wz_mrad_s == targets[case_index], "full signed pivot target is reached");
        check(evaluate_motion(&pose, 0, targets[case_index], 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 1,
              "full pivot and measured wheel feedback pass the real sweep predictor");

        /* 越过目标且超过到达容差：本拍停车，再朝相反方向重新规划。 */
        pose.yaw_mrad = goal_yaw + sign * 150;
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.autonomous && !out.vx_mm_s && !out.wz_mrad_s && !selected_valid,
              "crossing the requested final heading immediately invalidates the old turn");
        pose.wz_mrad_s = 0;
        (void)control(SENSORS, 1, 0, 0, 0, 1);
        (void)control(SENSORS, 1, 0, 0, 0, 1);
        (void)control(SENSORS, 1, 0, 0, 0, 1);
        out = plan(SENSORS);
        check(selected_valid && selected_wz == -sign * 600 && out.wz_mrad_s == -sign * 30,
              "after crossing, the reverse correction uses the minimum target and ramps from zero");
        pose.yaw_mrad = goal_yaw;
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.autonomous && !out.wz_mrad_s && out.status != BX72_NAV_STATUS_ARRIVED,
              "entering tolerance after a correction first requires a settled stop");
        (void)control(SENSORS, 1, 0, 0, 0, 1);
        (void)control(SENSORS, 1, 0, 0, 0, 1);
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(!out.autonomous && out.status == BX72_NAV_STATUS_ARRIVED &&
              !out.vx_mm_s && !out.wz_mrad_s,
              "exact final heading completes the task with a zero command");

        /* 旋转中的新障碍仍必须使实时全车扫掠停止。 */
        init_fixture(); feed(0, 0);
        send_task_config(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u,
                         requested_yaw_q16[case_index]);
        out = plan(SENSORS);
        check(out.wz_mrad_s == sign * 30, "obstacle fixture starts a bounded pivot");
        for (i = 0u; i < 29u; ++i) {
            pose.stamp_us = now_us;
            bx72_nav_lidar_validated((i + 1u) * 100u, 6000u, 1u,
                                     &pose, now_us, now_us);
            out = control(SENSORS, 1, 0, 0, 0, 1);
        }
        check(out.wz_mrad_s == targets[case_index],
              "obstacle fixture reaches the full signed pivot target");
        pose.wz_mrad_s = sign * 850;
        now_us += 100u; pose.stamp_us = now_us;
        bx72_nav_lidar_validated(8000u, 50u, 1u, &pose, now_us, now_us);
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.autonomous && !out.vx_mm_s && !out.wz_mrad_s && !out.plan_valid,
              "new occupied cell stops either direction of pivot");
    }
}

static void test_pivot_never_reports_unsettled_arrival(void)
{
    int32_t sign;
    for (sign = -1; sign <= 1; sign += 2) {
        bx72_nav_output_t out;
        int32_t vx, wz;
        uint32_t i;
        init_fixture(); feed(0, 0);
        send_task_config_yaw_tolerance(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u,
                                       sign * 13107, 1u);
        pose.yaw_mrad = task_queue[task_head].final_yaw_mrad;
        candidate_command(5u, &pose, &task_queue[task_head], &vx, &wz);
        check(!vx && !wz, "zero heading error does not create a positive turn target");
        candidate_command(6u, &pose, &task_queue[task_head], &vx, &wz);
        check(!vx && !wz, "zero heading error does not create a negative turn target");
        pose.wz_mrad_s = sign;
        for (i = 0u; i < 10u; ++i) {
            out = control(SENSORS, 1, 0, 0, 0, 1);
            check(out.autonomous && out.status != BX72_NAV_STATUS_ARRIVED && !out.wz_mrad_s,
                  "one-mrad tolerance cannot report arrival while yaw feedback is nonzero");
        }
        pose.wz_mrad_s = sign * 21;
        for (i = 0u; i < 10u; ++i) {
            out = control(SENSORS, 1, 0, 0, 0, 1);
            check(out.autonomous && out.status != BX72_NAV_STATUS_ARRIVED && !out.wz_mrad_s,
                  "yaw feedback that does not settle cannot report arrival");
        }
        pose.wz_mrad_s = 0;
        for (i = 0u; i < 4u; ++i) out = control(SENSORS, 1, 0, 0, 0, 1);
        check(!out.autonomous && out.status == BX72_NAV_STATUS_ARRIVED,
              "heading within tolerance completes only after measured yaw settles");
    }
}

/* 精确边界仍由生产预测器判断；超过边界不可自行放宽。 */
static void test_pivot_feedback_bounds(void)
{
    int32_t sign;
    for (sign = -1; sign <= 1; sign += 2) {
        bx72_nav_output_t out;
        int32_t end_x, end_y;
        uint8_t unknown;
        init_fixture(); feed(0, 0);
        send_task_config(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u, sign * 65536);
        out = plan(SENSORS);
        check(out.wz_mrad_s == sign * 30, "feedback-bound fixture starts pivoting");
        pose.wz_mrad_s = sign * 800;
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 1,
              "measured yaw exactly at 800 passes the existing bound");
        pose.wz_mrad_s = sign * 801;
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 1,
              "one unit of feedback overshoot does not invalidate a pivot");
        pose.wz_mrad_s = sign * 850;
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 1,
              "measured yaw at the 850 engineering margin passes the real predictor");
        check(evaluate_motion(&pose, 0, sign * 801, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 0,
              "the command itself remains limited to 800");
        pose.wz_mrad_s = sign * 851;
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 0,
              "measured yaw beyond 850 fails the feedback bound");
        out = control(SENSORS, 1, 0, 0, 0, 1);
        check(out.autonomous && !out.vx_mm_s && !out.wz_mrad_s && !selected_valid,
              "yaw feedback above the bound stops and invalidates the candidate");
        pose.wz_mrad_s = sign * 850;
        out = plan(SENSORS);
        check(selected_valid && out.wz_mrad_s == sign * 30,
              "feedback returning to the bound requires a new plan and bounded ramp");
        pose.vy_mm_s = sign * 20;
        previous_vx_mm_s = 0;
        previous_vy_mm_s = -sign * 20;
        previous_wz_mrad_s = sign * 800;
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 0 && !unknown,
              "mixed measured and prior wheels producing predicted yaw above 850 are rejected");
    }
}

/* 850反馈下的漂移仍需已知空间，并完整检查原位圆之外的运动扫掠。 */
static void test_pivot_margin_drift_sweep(void)
{
    int32_t sign;
    for (sign = -1; sign <= 1; sign += 2) {
        bx72_pose_t beam;
        int32_t end_x, end_y;
        uint8_t unknown;
        uint32_t direction, i;
        init_fixture(); feed(0, 0);
        send_task_config(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u, sign * 65536);
        (void)plan(SENSORS);
        pose.x_mm = -120;
        pose.vx_mm_s = 300;
        pose.wz_mrad_s = sign * 850;
        previous_vx_mm_s = previous_vy_mm_s = previous_wz_mrad_s = 0;
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 0 && unknown,
              "850 feedback with drift outside the fixed circle requires observed space");
        beam = pose;
        for (direction = 0u; direction < 4u; ++direction) {
            beam.yaw_mrad = wrap_yaw_mrad((int32_t)direction * 1571);
            bx72_grid_scan_begin();
            for (i = 0u; i < 360u; ++i)
                (void)bx72_grid_lidar_validated(i * 100u, 6000u, 1u,
                                             &beam, now_us, now_us);
            bx72_grid_scan_complete(now_us);
        }
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 1,
              "850 feedback and drift pass only with a clear observed sweep");
        beam.yaw_mrad = 0;
        beam.x_mm -= (int32_t)BX72_LIDAR_FORWARD_OFFSET_MM;
        bx72_grid_scan_begin();
        check(bx72_grid_lidar_validated(0u, 250u, 1u, &beam, now_us, now_us),
              "drifting pivot obstacle is observed");
        check(!(bx72_grid_sweep_occupied(pose.x_mm, pose.y_mm, pose.x_mm, pose.y_mm,
                                         BX72_GRID_BODY_RADIUS_MM) & BX72_GRID_CHECK_OCCUPIED),
              "drift obstacle begins outside the stationary footprint");
        check(evaluate_motion(&pose, 0, sign * 800, 0,
                              &end_x, &end_y, &unknown, current_time_us()) == 0 && !unknown,
              "known obstacle in the 850 feedback drift sweep stops the pivot");
    }
}

/* 轮速模型近似实车但不代替实测；到达后继续积分惯性尾段，检查最终航向。 */
static void test_pivot_heading_closed_loop(void)
{
    static const int32_t targets_q16[] = {13107, -13107, 65536, -65536, 205887, -205887};
    static const uint32_t yaw_tolerances[] = {1u, 10u, 30u, 100u};
    uint32_t scenario, tolerance_index;
    for (scenario = 0u; scenario < sizeof(targets_q16) / sizeof(targets_q16[0]); ++scenario)
      for (tolerance_index = 0u; tolerance_index < 4u; ++tolerance_index) {
        bx72_nav_output_t out;
        int32_t wheels[4] = {0}, desired[4], goal_yaw;
        double yaw = 0.0;
        uint32_t i, j, angle = 0u, arrived = 0u;
        uint32_t tolerance = yaw_tolerances[tolerance_index];
        init_fixture(); feed(0, 0);
        send_task_config_yaw_tolerance(1, 1, 3, 0, 0, 60000000u, 400u, 200u, 30u,
                                       targets_q16[scenario], tolerance);
        check(task_count == 1u && task_queue[task_head].yaw_tolerance_mrad == tolerance,
              "task parser preserves the requested heading tolerance");
        goal_yaw = task_queue[task_head].final_yaw_mrad;
        (void)control(SENSORS, 1, 0, 0, 0, 1);
        for (i = 0u; i < 500u; ++i) {
            for (j = 0u; j < 2u; ++j) bx72_nav_step(&pose, now_us);
            out = control(SENSORS, 1, 0, 0, 0, 1);
            if (out.status == BX72_NAV_STATUS_ARRIVED) {
                arrived = 1u;
                check(abs(pose.wz_mrad_s) <= NAV_TURN_SETTLED_MRAD_S &&
                      abs(wrap_yaw_mrad(goal_yaw - pose.yaw_mrad)) <= (int32_t)tolerance,
                      "arrival is emitted only in tolerance with settled yaw feedback");
            }
            wheel_vector(out.vx_mm_s, out.vy_mm_s, out.wz_mrad_s, desired);
            wheel_approach(wheels, desired);
            pose.wz_mrad_s = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) * 1000 /
                             (4 * NAV_WHEEL_GEOMETRY_MM);
            yaw += pose.wz_mrad_s * 0.02;
            pose.yaw_mrad = (int32_t)lround(yaw);
            for (j = 0u; j < 3u + (i % 5u < 3u); ++j) {
                angle = (angle + 500u) % 36000u; pose.stamp_us = now_us;
                bx72_nav_lidar_validated(angle, 6000u, 1u, &pose, now_us, now_us);
            }
            if (arrived && !wheels[0] && !wheels[1] && !wheels[2] && !wheels[3]) break;
        }
        printf("TRACE pivot target=%ld tolerance=%u yaw=%ld ticks=%u arrived=%u\n",
               (long)goal_yaw, tolerance, (long)pose.yaw_mrad, i, arrived);
        check(i < 500u && arrived &&
              abs(wrap_yaw_mrad(goal_yaw - pose.yaw_mrad)) <= (int32_t)tolerance,
              "modeled final heading remains within requested tolerance after wheel stop");
    }
}

/* 真实前半圈入口：两面有限长墙，雷达每5度取一束，地图按50mm落格。 */
static double corridor_center_mm;
static uint32_t corridor_lidar_range(uint32_t angle)
{
    double ray = pose.yaw_mrad / 1000.0 - angle * 3.141592653589793 / 18000.0;
    double dx = cos(ray), dy = sin(ray), nearest = 6000.0;
    double sensor_x = pose.x_mm + BX72_LIDAR_FORWARD_OFFSET_MM * cos(pose.yaw_mrad / 1000.0);
    double sensor_y = pose.y_mm + BX72_LIDAR_FORWARD_OFFSET_MM * sin(pose.yaw_mrad / 1000.0);
    int side;
    for (side = -1; side <= 1; side += 2) {
        double distance, hit_x;
        if (fabs(dy) < 1e-12) continue;
        distance = (corridor_center_mm + side * 200.0 - sensor_y) / dy;
        hit_x = sensor_x + distance * dx;
        if (distance > 50.0 && distance < nearest && hit_x >= 0.0 && hit_x <= 2200.0)
            nearest = distance;
    }
    return (uint32_t)lround(nearest);
}

static void test_narrow_corridor_closed_loop(void)
{
    static const int phases[] = {0, 10, 25, 40, 49};
    static const int headings[] = {0, -50, 50, -150, 150};
    uint32_t scenario;
    for (scenario = 0u; scenario < 25u; ++scenario) {
        bx72_nav_output_t out = {0};
        int32_t wheels[4] = {0}, target[4];
        double x = -500.0, y = phases[scenario / 5u], yaw = headings[scenario % 5u];
        uint32_t i, j, angle = 0u;
        corridor_center_mm = y;
        init_fixture();
        pose.x_mm = (int32_t)x; pose.y_mm = (int32_t)y; pose.yaw_mrad = (int32_t)yaw;
        bx72_nav_step(&pose, now_us);
        for (i = 0u; i <= 144u; ++i) {
            angle = (i % 72u) * 500u;
            now_us += 2000u; pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, corridor_lidar_range(angle), 1u, &pose, now_us, now_us);
        }
        bx72_nav_step(&pose, now_us);
        task_test_flags = NAV_TASK_POSITION_ONLY;
        send_task_config(1, 1, 3, 98304, (int32_t)(corridor_center_mm * 65536.0 / 1000.0),
                         120000000u, 200u, 200u, 30u, 0);
        for (i = 0u; i < 5800u; ++i) {
            double vx[4], vy[4], c, s;
            int corner, side;
            for (j = 0u; j < 2u; ++j) bx72_nav_step(&pose, now_us);
            out = control(SENSORS, 1, 0, 0, 0, 1);
            wheel_vector(out.vx_mm_s, out.vy_mm_s, out.wz_mrad_s, target);
            wheel_approach(wheels, target);
            pose.vx_mm_s = (wheels[0] + wheels[1] + wheels[2] + wheels[3]) / 4;
            pose.vy_mm_s = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4;
            pose.wz_mrad_s = (-wheels[0] + wheels[1] - wheels[2] + wheels[3]) * 1000 /
                             (4 * NAV_WHEEL_GEOMETRY_MM);
            x += (pose.vx_mm_s * cos(yaw / 1000.0) - pose.vy_mm_s * sin(yaw / 1000.0)) * .02;
            y += (pose.vx_mm_s * sin(yaw / 1000.0) + pose.vy_mm_s * cos(yaw / 1000.0)) * .02;
            yaw += pose.wz_mrad_s * .02;
            if (yaw > 3142.0) yaw -= 6283.0;
            if (yaw < -3142.0) yaw += 6283.0;
            pose.x_mm = (int32_t)lround(x); pose.y_mm = (int32_t)lround(y);
            pose.yaw_mrad = (int32_t)lround(yaw);

            /* 独立按车体四边与实体墙求交，不能因缩小模型而穿墙。 */
            c = cos(yaw / 1000.0); s = sin(yaw / 1000.0);
            for (corner = 0; corner < 4; ++corner) {
                double f = (corner == 0 || corner == 3) ? 130.0 : -130.0;
                double lateral = corner < 2 ? 125.0 : -125.0;
                vx[corner] = x + f * c - lateral * s;
                vy[corner] = y + f * s + lateral * c;
            }
            for (side = -1; side <= 1; side += 2) {
                double wall_y = corridor_center_mm + side * 200.0;
                for (corner = 0; corner < 4; ++corner) {
                    int next = (corner + 1) % 4;
                    if (((vy[corner] <= wall_y && vy[next] >= wall_y) ||
                         (vy[next] <= wall_y && vy[corner] >= wall_y)) && vy[corner] != vy[next]) {
                        double t = (wall_y - vy[corner]) / (vy[next] - vy[corner]);
                        double cross_x = vx[corner] + t * (vx[next] - vx[corner]);
                        check(cross_x < 0.0 || cross_x > 2200.0,
                              "physical body must not cross either corridor wall");
                    }
                }
            }
            for (j = 0u; j < 3u + (i % 5u < 3u); ++j) {
                angle = (angle + 500u) % 36000u; pose.stamp_us = now_us;
                bx72_nav_lidar_validated(angle, corridor_lidar_range(angle), 1u, &pose, now_us, now_us);
            }
            if (out.status == BX72_NAV_STATUS_ARRIVED) break;
        }
        printf("TRACE corridor phase=%d initial_yaw=%d ticks=%u pose=%ld,%ld yaw=%ld status=%u unknown=%u\n",
               phases[scenario / 5u], headings[scenario % 5u], i,
               (long)pose.x_mm, (long)pose.y_mm, (long)pose.yaw_mrad, out.status,
               unknown_in_swept_corridor);
        check(out.status == BX72_NAV_STATUS_ARRIVED,
              "40cm corridor is traversable across grid phases and small heading errors");
    }
}

static void test_narrow_corridor_drift(void)
{
    int32_t end_x, end_y;
    uint8_t unknown;
    uint32_t i, position_index;
    init_fixture();
    pose.x_mm = -500; corridor_center_mm = 0.0;
    bx72_nav_step(&pose, now_us);
    for (i = 0u; i <= 144u; ++i) {
        uint32_t angle = (i % 72u) * 500u;
        now_us += 2000u; pose.stamp_us = now_us;
        bx72_nav_lidar_validated(angle, corridor_lidar_range(angle), 1u, &pose, now_us, now_us);
    }
    bx72_nav_step(&pose, now_us);
    send_task_config(1, 1, 3, 98304, 0, 120000000u, 200u, 200u, 30u, 0);
    (void)plan(SENSORS);
    /* 车体从入口行至500mm时，前置雷达沿途留下已观测区域。 */
    for (position_index = 0u; position_index < 4u; ++position_index) {
        pose.x_mm = -250 + (int32_t)position_index * 250;
        for (i = 1u; i <= 144u; ++i) {
            uint32_t angle = (i % 72u) * 500u;
            now_us += 2000u; pose.stamp_us = now_us;
            bx72_nav_lidar_validated(angle, corridor_lidar_range(angle), 1u, &pose, now_us, now_us);
        }
        bx72_nav_step(&pose, now_us);
    }
    pose.vx_mm_s = previous_vx_mm_s = 100;
    previous_vy_mm_s = previous_wz_mrad_s = 0;
    for (i = 20u; i <= 21u; ++i) {
        pose.vy_mm_s = (int32_t)i;
        check(evaluate_motion(&pose, 100, 0, 1, &end_x, &end_y, &unknown, current_time_us()) == 1,
              "20/21mm/s side feedback passes when the actual corridor sweep is observed");
    }

    /* 重建仅前向已观察地图，侧漂涉及真正未观察的后部时仍须停。 */
    bx72_grid_reset(pose.x_mm, pose.y_mm, now_us);
    bx72_grid_scan_begin();
    for (i = 0u; i < 360u; ++i)
        (void)bx72_grid_lidar_validated(i * 100u, corridor_lidar_range(i * 100u), 1u,
                                        &pose, now_us, now_us);
    bx72_grid_scan_complete(now_us);
    pose.vy_mm_s = 21;
    check(evaluate_motion(&pose, 100, 0, 1, &end_x, &end_y, &unknown, current_time_us()) == 0 && unknown,
          "side drift into an unobserved body sweep remains blocked");
}

static void test_external_takeover_invalidates_start(void)
{
    init_fixture();
    start_exemption_valid = 1u;
    start_exemption.x_mm = 0;
    start_exemption.y_mm = 0;
    start_exemption.radius_mm = BX72_GRID_START_CONFIRM_RADIUS_MM;
    task_count = 1u;
    active_task_id = 19u;
    bx72_nav_external_takeover(now_us);
    check(task_count == 0u && !start_exemption_valid,
          "external control cancels the old task and its fixed start exemption");
}

static void test_direct_guide_uses_real_sweep_and_falls_back(void)
{
    bx72_nav_output_t out;
    init_fixture(); feed(0, 0);
    send_task(1, 1, 3, 2 * 65536, 0, 60000000u);
    out = plan(SENSORS);
    check(direct_guide_active && out.vx_mm_s > 0,
          "clear aligned goal starts a direct guide");
    pose.vx_mm_s = previous_vx_mm_s = 100;
    now_us += 100u; pose.stamp_us = now_us;
    bx72_nav_lidar_validated(0u, 50u, 1u, &pose, now_us, now_us);
    out = control(SENSORS, 1, 0, 0, 0, 1);
    check(!out.vx_mm_s && !out.plan_valid && direct_guide_blocked && !direct_guide_active,
          "a newly observed obstacle stops the direct guide using the real braking sweep");
    bx72_nav_step(&pose, now_us);
    check(!direct_guide_active && bx72_local_path_state() == BX72_PATH_WORKING,
          "the next search uses A* rather than immediately repeating the blocked guide");

    init_fixture(); feed(0, 0);
    pose.yaw_mrad = 600; pose.stamp_us = now_us;
    send_task(1, 1, 3, 2 * 65536, 0, 60000000u);
    (void)control(SENSORS, 1, 0, 0, 0, 1);
    bx72_nav_step(&pose, now_us);
    check(!direct_guide_active && bx72_local_path_state() == BX72_PATH_WORKING,
          "a clearly misaligned vehicle first plans or observes instead of starting straight");
}

int main(void)
{
    test_external_takeover_invalidates_start();
    test_direct_guide_uses_real_sweep_and_falls_back();
    test_start_recheck_gap_and_unknown();
    test_turn_and_tail();
    test_observe_requires_new_complete_scan();
    test_curve_envelope();
    test_rebase_and_delayed_scan();
    test_out_of_range_returns();
    test_task_safety();
    test_limits_and_epochs();
    test_independent_waypoint_start_authorization();
    test_near_goal_speed_and_live_recheck();
    test_reset_discards_cached_pose();
    test_reset_barrier_after_long_wait();
    test_first_domain_handshake();
    test_closed_loop();
    test_turn_measured_drift();
    test_recorded_stationary_drift();
    test_forward_feedback_noise();
    test_observed_lateral_feedback();
    test_near_side_physical_envelope();
    test_goal_tolerance_consistency();
    test_directional_closed_loop();
    test_obstacle_closed_loop();
    test_continuous_motion_with_rebase();
    test_periodic_rebase_ramp();
    test_periodic_rebase_turn();
    test_manual_during_rebase();
    test_rebase_safety();
    test_retained_goal_geometry();
    test_diagonal_goal_transition();
    test_task_speed_400();
    test_progress_final_yaw_and_expiry();
    test_sealed_stop_until_expiry();
    test_position_only_waypoints();
    test_pivot_speed_targets();
    test_pivot_feedback_bounds();
    test_pivot_margin_drift_sweep();
    test_pivot_heading_closed_loop();
    test_pivot_never_reports_unsettled_arrival();
    test_narrow_corridor_closed_loop();
    test_narrow_corridor_drift();
#ifdef BX72_NAV_TEST_TIMED
    test_live_budget();
#endif
    if (performance_regressions) return 1;
    puts("PASS local navigation: startup, blind sectors, unknown, braking, CSM, FIFO delay and task safety");
    return 0;
}
