#ifndef BX72_LITE_CSM_H
#define BX72_LITE_CSM_H

#include "fusion.h"

#define BX72_CSM_POINTS 72u /* 360 degrees at 5 degrees per reducer bin. */

#define BX72_CSM_STATUS_REFERENCE (1u << 0)
#define BX72_CSM_STATUS_CAPTURE (1u << 1)
#define BX72_CSM_STATUS_ICP (1u << 2)
#define BX72_CSM_STATUS_ACCEPTED (1u << 3)
#define BX72_CSM_STATUS_REJECTED (1u << 4)
#define BX72_CSM_STATUS_NEED_SYNC (1u << 5)
#define BX72_CSM_STATUS_DEGENERATE (1u << 6)

typedef struct {
    float reference_x[72];
    float reference_y[72];
    uint8_t reference_valid[72];
    float reference_x_mm;
    float reference_y_mm;
    float reference_yaw_rad;
    uint32_t last_accepted_stamp_us;

    /* Completed scan under ICP. Candidate coordinates are local to its pose. */
    float candidate_x[72];
    float candidate_y[72];
    uint8_t candidate_valid[72];
    bx72_pose_t candidate_pose;

    /* 当前圈保存车体坐标扫描点，不混入轮式平移去畸变。 */
    float collect_scan_x[72];
    float collect_scan_y[72];
    uint8_t collect_valid[72];
    uint8_t collect_seen[72];
    uint8_t collect_count;
    uint8_t collect_seen_count;
    uint8_t collect_pose_valid;
    bx72_pose_t collect_last_pose;

    uint32_t accepted_count;
    uint32_t rejected_count;
    uint32_t status;
    uint16_t previous_angle_cdeg;
    uint16_t icp_point_index;
    uint8_t angle_ready;
    uint8_t synchronized;
    uint8_t icp_active;
    uint8_t icp_iteration;
    uint8_t icp_inliers;
    uint8_t last_result;

    /* Relative candidate-to-reference SE(2) estimate and per-slice normal equations. */
    float icp_x_mm;
    float icp_y_mm;
    float icp_yaw_rad;
    float icp_h00, icp_h01, icp_h02;
    float icp_h11, icp_h12, icp_h22;
    float icp_g0, icp_g1, icp_g2;
    float icp_normal_xx, icp_normal_xy, icp_normal_yy;
    float icp_error_sq;
    float icp_previous_rms_mm;
    float best_x_mm;
    float best_y_mm;
    float best_yaw_rad;
    float best_rms_mm;
} bx72_csm_state_t;

void bx72_csm_init(bx72_csm_state_t *state);
int bx72_csm_selftest(bx72_csm_state_t *state,
                      bx72_lite_fusion_state_t *fusion);
/* 输入角度须小于36000，pose_at_beam须为有效位姿。 */
void bx72_csm_point(bx72_csm_state_t *state,
                    uint32_t angle_cdeg, uint32_t distance_mm, uint32_t hit,
                    const bx72_pose_t *pose_at_beam);
void bx72_csm_step(bx72_csm_state_t *state,
                   const bx72_pose_t *pose_now,
                   uint32_t now_us,
                   bx72_lite_fusion_state_t *fusion,
                   uint32_t candidate_budget);
void bx72_csm_discontinuity(bx72_csm_state_t *state);
void bx72_csm_diagnostics(const bx72_csm_state_t *state,
                          uint32_t *accepted, uint32_t *rejected,
                          uint32_t *status);

#endif
