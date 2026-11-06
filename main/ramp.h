#pragma once
#include "esp_err.h"
#include "stepper_motion.h"

#define RAMP_HZ  100

typedef struct {
    stepper_t *left;
    stepper_t *right;
    float accel;      // steps/s^2, applied to the wheel with the larger change
    float max_rate;   // steps/s
} ramp_config_t;

// Starts the ramp task. Call once, after stepper_init() on both wheels.
esp_err_t ramp_start(const ramp_config_t *cfg);

// Thread-safe. Targets in steps/s, signed. If either exceeds max_rate,
// both are scaled down together so the turn ratio is preserved.
void ramp_set_target(float left, float right);

// Thread-safe. The ramp's current (unquantized) commanded rates.
void ramp_get_current(float *left, float *right);