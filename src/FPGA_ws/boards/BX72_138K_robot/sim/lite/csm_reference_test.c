/* Host-only regression; include implementation to exercise actual ICP state. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../src/firmware/lite/csm.c"
static bx72_csm_state_t s;
static bx72_lite_fusion_state_t f;
static void run_match(float predicted_x, uint32_t stamp) {
 bx72_pose_t pose={0}; unsigned i;
 pose.x_mm=(int32_t)predicted_x; pose.stamp_us=stamp; pose.flags=BX72_POSE_VALID;
 s.candidate_pose=pose; selftest_seed_fusion(&f,predicted_x,0,0,stamp);
 s.icp_x_mm=predicted_x-s.reference_x_mm; s.icp_y_mm=0; s.icp_yaw_rad=0;
 s.icp_active=1; s.icp_iteration=0; s.best_rms_mm=0; reset_normal_equations(&s);
 for(i=0;i<36 && s.icp_active;i++) bx72_csm_step(&s,&pose,stamp,&f,12);
 assert(!s.icp_active && (s.status & BX72_CSM_STATUS_ACCEPTED));
 assert(abs_f(s.reference_x_mm) < 0.001f);
 assert(abs_f(s.best_x_mm-f.x[0])>2.0f);
}
static void test_excitation_boundary(int displacement, int expect_icp) {
 unsigned i;
 bx72_csm_init(&s);
 selftest_fill_scene(&s,0,0,0,0);
 s.status |= BX72_CSM_STATUS_REFERENCE;
 s.last_accepted_stamp_us=90000;
 for (i=0;i<72;i++) {
  s.collect_seen[i]=1; s.collect_valid[i]=1;
  s.collect_scan_x[i]=s.reference_x[i]; s.collect_scan_y[i]=s.reference_y[i];
 }
 s.collect_seen_count=72; s.collect_count=72; s.collect_pose_valid=1;
 s.collect_last_pose.flags=BX72_POSE_VALID;
 s.collect_last_pose.stamp_us=100000; s.collect_last_pose.x_mm=displacement;
 finish_collected_scan(&s);
 assert(s.last_accepted_stamp_us==90000 && s.reference_x_mm==0);
 assert((s.icp_active != 0)==expect_icp);
 assert(s.rejected_count==(expect_icp ? 0u : 1u));
}
static void test_stationary_residual_convergence(void) {
 unsigned i,k; float predicted=20.0f, first=predicted;
 bx72_pose_t pose={0}; bx72_csm_init(&s); selftest_fill_scene(&s,0,0,0,0);
 s.status|=BX72_CSM_STATUS_REFERENCE; s.last_accepted_stamp_us=100000;
 for(k=0;k<5;k++) {
  for(i=0;i<72;i++) { s.collect_seen[i]=1; s.collect_valid[i]=1;
   s.collect_scan_x[i]=s.reference_x[i]; s.collect_scan_y[i]=s.reference_y[i]; }
  s.collect_seen_count=72; s.collect_count=72; s.collect_pose_valid=1;
  pose.flags=BX72_POSE_VALID; pose.x_mm=(int32_t)predicted; pose.stamp_us=110000+k*10000;
  s.collect_last_pose=pose; selftest_seed_fusion(&f,predicted,0,0,pose.stamp_us);
  finish_collected_scan(&s); assert(s.icp_active);
  for(i=0;i<36 && s.icp_active;i++) bx72_csm_step(&s,&pose,pose.stamp_us,&f,12);
  assert(!s.icp_active && s.accepted_count==k+1); predicted=f.x[0]; if(k==0) { finish_collected_scan(&s); assert(!s.icp_active && s.rejected_count==1 && s.accepted_count==1); }
 }
 assert(predicted < first-5.0f && predicted> -5.0f);
}
static void test_keyframe_update(float dx, float yaw, uint32_t age, int change) {
 bx72_pose_t p={0}; float reference_point; bx72_csm_init(&s); selftest_fill_scene(&s,0,0,0,0);
 reference_point=s.reference_x[0]; s.candidate_x[0]=reference_point+1.0f;
 s.status|=BX72_CSM_STATUS_REFERENCE; s.last_accepted_stamp_us=100000;
 p.flags=BX72_POSE_VALID; p.stamp_us=100000+age; p.x_mm=(int32_t)dx; p.yaw_mrad=(int32_t)(yaw*1000);
 s.candidate_pose=p; s.icp_x_mm=dx; s.icp_yaw_rad=yaw; s.best_rms_mm=0;
 selftest_seed_fusion(&f,dx,0,yaw,p.stamp_us);
 assert(finish_icp(&s,&p,p.stamp_us,&f));
 assert(s.last_accepted_stamp_us==p.stamp_us);
 assert(s.reference_x[0]==reference_point+(change?1.0f:0.0f));
 bx72_csm_discontinuity(&s); assert(s.last_accepted_stamp_us==0);
}
static void test_timestamp_wrap(void) {
 unsigned i;
 bx72_csm_init(&s); selftest_fill_scene(&s,0,0,0,0);
 s.status|=BX72_CSM_STATUS_REFERENCE; s.last_accepted_stamp_us=100000;
 s.last_accepted_stamp_us=0xfffffff0u;
 for(i=0;i<72;i++) { s.collect_seen[i]=1; s.collect_valid[i]=1;
  s.collect_scan_x[i]=s.reference_x[i]; s.collect_scan_y[i]=s.reference_y[i]; }
 s.collect_seen_count=72; s.collect_count=72; s.collect_pose_valid=1;
 s.collect_last_pose.flags=BX72_POSE_VALID; s.collect_last_pose.stamp_us=0x1000u;
 finish_collected_scan(&s); assert(s.icp_active && s.rejected_count==0);
}

/* 固定矩形环境以真实前置雷达发射，车体中心只旋转，不产生平移。 */
static unsigned wall_range(double yaw, unsigned angle) {
 double sx=130.0*cos(yaw), sy=130.0*sin(yaw);
 double a=yaw-(double)angle*3.141592653589793/18000.0;
 double dx=cos(a), dy=sin(a), best=6000.0, t;
 if(dx>0.000001) { t=(1800.0-sx)/dx; if(t<best) best=t; }
 if(dx< -0.000001) { t=(-1800.0-sx)/dx; if(t<best) best=t; }
 if(dy>0.000001) { t=(1200.0-sy)/dy; if(t<best) best=t; }
 if(dy< -0.000001) { t=(-1200.0-sy)/dy; if(t<best) best=t; }
 return (unsigned)lround(best);
}
static void test_forward_lidar_pure_rotation(void) {
 static const int yaw_mrad[]={200,-200,1570,-1570};
 unsigned k,i;
 for(k=0;k<sizeof(yaw_mrad)/sizeof(yaw_mrad[0]);++k) {
  bx72_pose_t p={0}; double yaw=(double)yaw_mrad[k]/1000.0;
  bx72_csm_init(&s); p.flags=BX72_POSE_VALID; p.stamp_us=100000u;
  for(i=0;i<72;i++) bx72_csm_point(&s,i*500u,wall_range(0.0,i*500u),1,&p);
  finish_collected_scan(&s); assert(s.status & BX72_CSM_STATUS_REFERENCE);
  clear_collect(&s); s.angle_ready=0; p.stamp_us=200000u; p.yaw_mrad=yaw_mrad[k];
  for(i=0;i<72;i++) bx72_csm_point(&s,i*500u,wall_range(yaw,i*500u),1,&p);
  selftest_seed_fusion(&f,0,0,(float)yaw,200000u);
  finish_collected_scan(&s); assert(s.icp_active);
  for(i=0;i<100 && s.icp_active;i++) bx72_csm_step(&s,&p,200000u,&f,12);
  assert(!s.icp_active && s.accepted_count==1 && s.rejected_count==0);
  assert(abs_f(s.best_x_mm)<8.0f && abs_f(s.best_y_mm)<8.0f);
  assert(abs_f(wrap_angle(s.best_yaw_rad-(float)yaw))<0.01f);
 }
}
static void test_body_return_filter(void) {
 bx72_pose_t p={0}; p.flags=BX72_POSE_VALID; p.stamp_us=1000u;
 bx72_csm_init(&s);
 /* 现场约简桶的 129°/110mm 点位于车体内，仍记作覆盖而不参与匹配。 */
 bx72_csm_point(&s,12900u,110u,1u,&p);
 assert(s.collect_seen[25] && !s.collect_valid[25]);
 /* 同一角度更远的点在车体侧面之外，必须保留为环境。 */
 bx72_csm_point(&s,12900u,180u,1u,&p);
 assert(s.collect_valid[25] && s.collect_scan_y[25] < -125.0f);
}
int main(void) {
 bx72_pose_t p={0}; unsigned i;
 test_body_return_filter(); test_forward_lidar_pure_rotation(); test_timestamp_wrap(); test_keyframe_update(99,0,10000,0); test_keyframe_update(101,0,10000,1); test_keyframe_update(0,0.101f,10000,1); test_keyframe_update(0,0,10000000,0); test_keyframe_update(0,0,3000000000u,0); test_stationary_residual_convergence(); test_excitation_boundary(0,1); test_excitation_boundary(9,1); test_excitation_boundary(11,1); assert(bx72_csm_selftest(&s,&f)); bx72_csm_init(&s); p.flags=BX72_POSE_VALID; p.stamp_us=1000;
 p.x_mm=500; p.y_mm=300; p.yaw_mrad=1000; bx72_csm_point(&s,1000,1000,1,&p); assert(s.collect_scan_x[2] > 1114 && s.collect_scan_x[2] < 1116 && s.collect_scan_y[2] < -170); p.x_mm=-500; p.yaw_mrad=-1000; bx72_csm_point(&s,1500,1000,1,&p); assert(s.collect_scan_x[3] > 1095 && s.collect_scan_x[3] < 1097 && s.collect_scan_y[3] < -250); bx72_csm_point(&s,12999,1000,1,&p); bx72_csm_point(&s,13000,1000,1,&p);
 bx72_csm_point(&s,23499,1000,1,&p); bx72_csm_point(&s,23500,1000,1,&p);
 assert(s.collect_seen[26] && !s.collect_valid[26]);
 assert(s.collect_seen[46] && !s.collect_valid[46]);
 assert(s.collect_seen[47] && !s.collect_valid[47]);
 assert(s.collect_valid[25]);
 bx72_csm_point(&s,23501,1000,1,&p); assert(s.collect_valid[47]);
 bx72_csm_init(&s); s.status|=BX72_CSM_STATUS_REFERENCE; s.last_accepted_stamp_us=90000;
 selftest_fill_scene(&s,40,0,0,0); run_match(30,100000);
 for(i=0;i<72;i++) { s.candidate_valid[i]=s.reference_valid[i]; s.candidate_x[i]=s.reference_x[i]-40; s.candidate_y[i]=s.reference_y[i]; }
 run_match(s.reference_x_mm+30,200000);
 assert(s.accepted_count==2); puts("two ICP matches preserve measured anchor; occlusion boundaries passed");
 return 0;
}
