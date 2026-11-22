#ifndef ROBOT_LOCAL_MAP_H
#define ROBOT_LOCAL_MAP_H

#include <stdint.h>

#define LOCAL_MAP_SIDE 64
#define LOCAL_MAP_CELL_MM 50
#define LOCAL_MAP_TICK_US 250000u
#define LOCAL_MAP_CELLS (LOCAL_MAP_SIDE * LOCAL_MAP_SIDE)

enum local_map_state {
    LOCAL_MAP_UNKNOWN = 0,
    LOCAL_MAP_FREE = 1,
    LOCAL_MAP_OCCUPIED = 2
};

/* One byte per cell: two state bits and six time-bucket bits. */
typedef struct {
    uint8_t cells[LOCAL_MAP_CELLS];
    uint8_t scan_hits[LOCAL_MAP_CELLS / 8];
    int32_t origin_x_cell;
    int32_t origin_y_cell;
    uint32_t last_update_us;
    uint32_t tick_phase_us;
    uint32_t generation;
    int32_t confirmed_x_mm;
    int32_t confirmed_y_mm;
    uint32_t confirmed_until_us;
    uint8_t confirmed_valid;
    uint8_t current_tick;
    uint8_t ttl_ticks;
    uint8_t initialized;
} local_map_t;

/*
 * Coordinates are in continuous odom, in millimetres. Time values are from
 * the same monotonic 32-bit microsecond clock, including across its wrap.
 * Call at least once per 2^31 microseconds; a longer gap invalidates the map.
 * TTL is specified in 250 ms ticks, from 1 to 31; expiry is conservative.
 */
int local_map_init(local_map_t *map, int32_t center_x_mm, int32_t center_y_mm,
                   uint8_t ttl_ticks, uint32_t now_us);
void local_map_advance(local_map_t *map, uint32_t now_us);
/* Update the lifetime of retained observations, in 250 ms ticks (1 to 31). */
void local_map_set_ttl(local_map_t *map, uint8_t ttl_ticks, uint32_t now_us);
void local_map_recenter(local_map_t *map, int32_t center_x_mm, int32_t center_y_mm,
                        uint32_t now_us);
void local_map_begin_scan(local_map_t *map, uint32_t now_us);

/*
 * A ray must be a valid, visible observation transformed using its sample
 * pose before this call. An endpoint marked as a hit is occupied. A ray
 * without a hit may clear space only when the sensor explicitly confirms
 * free range; invalid and unobserved rays must not be submitted. Submit
 * observations in sample-time order, including within each time bucket.
 */
int local_map_observe_ray(local_map_t *map, int32_t sensor_x_mm, int32_t sensor_y_mm,
                          int32_t end_x_mm, int32_t end_y_mm, int endpoint_is_hit,
                          uint32_t sample_us, uint32_t now_us);

/* Out-of-map and expired cells are unknown; remaining time is conservative. */
enum local_map_state local_map_get(local_map_t *map, int32_t world_x_cell,
                                   int32_t world_y_cell, uint32_t now_us,
                                   uint32_t *remaining_us);

/* Capture row-major two-bit states after one map-age update. */
int local_map_snapshot(local_map_t *map, uint32_t now_us, uint32_t required_fresh_us,
                       uint8_t packed[LOCAL_MAP_CELLS / 4]);

/* Assumed startup area, fixed in odom and never published as sensor data. */
void local_map_confirm_start(local_map_t *map, int32_t x_mm, int32_t y_mm,
                             uint32_t until_us);
int local_map_confirmed_free(const local_map_t *map, int32_t x_cell, int32_t y_cell,
                             uint32_t now_us, uint32_t required_us);
int local_map_confirmed_footprint(const local_map_t *map, int32_t center_x_mm,
                                  int32_t center_y_mm, int32_t cos_q15, int32_t sin_q15,
                                  int32_t half_length_mm, int32_t half_width_mm,
                                  uint32_t now_us, uint32_t required_us);
void local_map_overlay_confirmed(const local_map_t *map, uint32_t now_us,
                                 uint32_t required_us, uint8_t packed[LOCAL_MAP_CELLS / 4]);

#endif
