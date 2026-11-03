#include <stdio.h>
#include <math.h>
#include "fusion.h"
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); return 1; } } while (0)
static void sample(bx72_lite_fusion_state_t *s,bx72_pose_t *p,bx72_lite_sample_t *a,unsigned t,int vx) {
 a->now_us=a->imu_stamp_us=a->wheel_stamp_us=t; ++a->imu_seq; ++a->wheel_seq; a->vx=vx; bx72_lite_fusion_update(s,p,a);
}
int main(void) {
 bx72_lite_fusion_state_t s; bx72_pose_t p={0}; bx72_lite_sample_t a={0}; float initial,once,measurement; unsigned t;
 bx72_lite_fusion_init(&s); a.status=BX72_STATUS_IMU_OK|BX72_STATUS_WHEEL_OK; a.az=-8192; p.x_mm=100;
 sample(&s,&p,&a,10000,0); initial=s.x[0];
 for(t=20000;t<=210000;t+=10000) {
  sample(&s,&p,&a,t,0); CHECK(bx72_lite_fusion_csm_update(&s,t,t,0,0,0,25,0.0004f));
  CHECK(s.pose_cov[0][0]>=625.0f && s.pose_cov[1][1]>=625.0f && s.pose_cov[2][2]>=0.000625f);
  CHECK(s.pose_cov[0][1]==s.pose_cov[1][0] && s.pose_cov[0][2]==s.pose_cov[2][0] && s.pose_cov[1][2]==s.pose_cov[2][1]);
 }
 CHECK(fabsf(s.x[0])<2 && fabsf(s.x[0])<fabsf(initial));
 sample(&s,&p,&a,220000,100); once=s.x[0]; measurement=once+10; CHECK(bx72_lite_fusion_csm_update(&s,220000,220000,measurement,0,0,25,0.0004f)); once=s.x[0];
 /* 同一源帧不得重复压缩协方差或叠加校正。 */
 { float pxx=s.pose_cov[0][0], x=s.x[0];
   CHECK(!bx72_lite_fusion_csm_update(&s,220000,220000,measurement,0,0,25,0.0004f));
   CHECK(s.pose_cov[0][0]==pxx && s.x[0]==x); }

 CHECK(!bx72_lite_fusion_csm_update(&s,1,300000,0,0,0,25,0.0004f));
 for(t=230000;t<=1230000;t+=10000) sample(&s,&p,&a,t,100);
  CHECK(s.x[3]==100 && s.x[0]>90);
 {bx72_pose_t old_source,old_now,new_source,new_now;
  CHECK(bx72_lite_fusion_pose_at(&s,1190000,&old_source));
  CHECK(bx72_lite_fusion_pose_at(&s,1230000,&old_now));
  CHECK(bx72_lite_fusion_csm_update(&s,1190000,1230000,old_source.x_mm-10,old_source.y_mm,0,25,0.0004f));
  CHECK(bx72_lite_fusion_pose_at(&s,1190000,&new_source));
  CHECK(bx72_lite_fusion_pose_at(&s,1230000,&new_now));
  CHECK(new_source.x_mm<old_source.x_mm && new_now.x_mm<old_now.x_mm && s.x[3]==100);
 }
 {
  bx72_lite_fusion_state_t turn; bx72_pose_t tp={0}; bx72_lite_sample_t input={0};
  unsigned i; int crossed=0;
  bx72_lite_fusion_init(&turn); tp.yaw_mrad=3100;
  input.status=BX72_STATUS_IMU_OK|BX72_STATUS_WHEEL_OK; input.az=-8192; input.gz=-1000;
  for(i=1;i<=600;i++) {
   float (*p3)[3]=turn.pose_cov; float det;
   sample(&turn,&tp,&input,i*10000u,200);
   if(i>=10 && i%10==0) {
    bx72_pose_t delayed;
    CHECK(bx72_lite_fusion_pose_at(&turn,i*10000u-20000u,&delayed));
    CHECK(bx72_lite_fusion_csm_update(&turn,i*10000u-20000u,i*10000u,
          delayed.x_mm+2.0f,delayed.y_mm-1.0f,delayed.yaw_mrad*0.001f+0.002f,100.0f,0.0004f));
   }
   if(turn.x[2]<0.0f) crossed=1;
   CHECK(turn.x[2]>=-3.141593f && turn.x[2]<=3.141593f);
   CHECK(fabsf(p3[0][1]-p3[1][0])<0.001f && fabsf(p3[0][2]-p3[2][0])<0.001f && fabsf(p3[1][2]-p3[2][1])<0.001f);
   det=p3[0][0]*(p3[1][1]*p3[2][2]-p3[1][2]*p3[2][1])-
       p3[0][1]*(p3[1][0]*p3[2][2]-p3[1][2]*p3[2][0])+
       p3[0][2]*(p3[1][0]*p3[2][1]-p3[1][1]*p3[2][0]);
   CHECK(p3[0][0]>0 && p3[1][1]>0 && p3[2][2]>0 && det>=0);
   CHECK(p3[0][0]*p3[1][1]>=p3[0][1]*p3[1][0] &&
         p3[0][0]*p3[2][2]>=p3[0][2]*p3[2][0] &&
         p3[1][1]*p3[2][2]>=p3[1][2]*p3[2][1]);
  }
  CHECK(crossed);
  { float prior=turn.pose_cov[0][0];
    input.status=0u; sample(&turn,&tp,&input,6010000u,200);
    CHECK(!(tp.flags & BX72_POSE_VALID) && turn.pose_cov[0][0]>=prior);
    input.status=BX72_STATUS_IMU_OK|BX72_STATUS_WHEEL_OK;
    sample(&turn,&tp,&input,6020000u,200);
    CHECK((tp.flags & BX72_POSE_VALID) && turn.pose_cov[0][0]>=prior);
  }
 }
 printf("PASS unique corrections converge, stale rejects, nonzero velocity remains %f\n",s.x[3]);return 0;
}
