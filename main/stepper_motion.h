#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"

#define STEPPER_MCPWM_GROUP  0          // 3 timers per group, enough for 2 wheels
#define STEPPER_TIMER_HZ     1000000    // 1 MHz tick
#define STEPPER_MAX_PERIOD   65000      // 16-bit timer -> ~15.4 Hz minimum
#define STEPPER_MIN_RATE     ((float)STEPPER_TIMER_HZ / STEPPER_MAX_PERIOD)
#define STEPPER_MAX_RATE     50000.0f   // steps/s clamp

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmp;
    mcpwm_gen_handle_t   gen;
    pcnt_unit_handle_t   pcnt;
    pcnt_channel_handle_t pcnt_ch;
    int      dir_pin;
    bool     running;
    bool     forward;   // DIR high = forward = count up
    uint32_t period;    // current period in ticks
    float    rate;      // actually applied rate (steps/s, signed, quantized)
} stepper_t;

esp_err_t stepper_init(stepper_t *s, int step_pin, int dir_pin);

// Signed steps/s. |rate| below STEPPER_MIN_RATE stops the motor.
// Applies immediately -- no acceleration limiting here.
esp_err_t stepper_set_rate(stepper_t *s, float steps_per_s);

// Rate actually being output after quantization/clamping.
float stepper_get_rate(const stepper_t *s);

// Cumulative signed step count, read from the STEP/DIR pins themselves.
esp_err_t stepper_get_position(stepper_t *s, int32_t *steps);
esp_err_t stepper_reset_position(stepper_t *s);