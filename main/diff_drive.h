#pragma once
#include "esp_err.h"

typedef struct {
    float wheel_radius_m;   // wheel radius, metres
    float track_width_m;    // distance between wheel contact patches, metres
    float steps_per_rev;    // full steps per rev * microsteps (e.g. 200 * 16)
} diff_drive_config_t;

// Call once after ramp_start(). Stores the geometry only.
esp_err_t diff_drive_init(const diff_drive_config_t *cfg);

// Thread-safe. v in m/s (+ forward), w in rad/s (+ CCW / left turn).
// If a wheel would exceed the ramp's max_rate, both are scaled together,
// so the turning radius (v/w) is kept and only speed drops.
void diff_drive_set(float v, float w);

// Thread-safe. Body velocity the ramp is currently commanding (post-ramp).
void diff_drive_get(float *v, float *w);
