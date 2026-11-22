#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* Restore the 2026-11-12 ground-trial profile. These braking/delay values are trial assumptions, not measured calibration. */
#ifndef ROBOT_BRAKE_X_MM_S2
#define ROBOT_BRAKE_X_MM_S2 200
#endif
#ifndef ROBOT_BRAKE_Y_MM_S2
#define ROBOT_BRAKE_Y_MM_S2 200
#endif
#ifndef ROBOT_BRAKE_YAW_MRAD_S2
#define ROBOT_BRAKE_YAW_MRAD_S2 1500
#endif
#ifndef ROBOT_REACTION_US
#define ROBOT_REACTION_US 100000u
#endif
#ifndef ROBOT_OBSERVATION_UNCERTAINTY_US
#define ROBOT_OBSERVATION_UNCERTAINTY_US 100000u
#endif

#define ROBOT_MAP_TTL_TICKS 16u
/* At 800 mm/s, the stopping sweep needs over 4 s and 1.8 m ahead of the center. */
#define ROBOT_FAST_MAP_TTL_TICKS 20u
#define ROBOT_FAST_MAP_LEAD_MM 400
#define ROBOT_LIDAR_X_MM 130
#define ROBOT_BODY_LENGTH_MM 260
#define ROBOT_BODY_WIDTH_MM 250
#define ROBOT_BODY_MARGIN_MM 20
#define ROBOT_GEOMETRY_MM 178
#define ROBOT_MAX_WHEEL_MM_S 800
#define ROBOT_CRUISE_MM_S 500
#define ROBOT_MIN_LATERAL_MM_S 300
#define ROBOT_EXPLORATION_FORWARD_MM_S 200
#define ROBOT_UNOBSERVED_FORWARD_MM_S 100
/* Legacy installation uses Rx(pi): the sensor Z axis points down. */
#define ROBOT_IMU_YAW_SIGN (-1)

#endif
