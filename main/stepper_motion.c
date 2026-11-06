#include "stepper_motion.h"
#include <math.h>
#include "driver/gpio.h"
#include "esp_check.h"

static const char *TAG = "stepper";

#define PCNT_LIMIT 30000   // hardware counter is 16-bit; driver accumulates past this

esp_err_t stepper_init(stepper_t *s, int step_pin, int dir_pin)
{
    ESP_RETURN_ON_FALSE(s, ESP_ERR_INVALID_ARG, TAG, "null");
    *s = (stepper_t){ .dir_pin = dir_pin, .forward = true, .period = STEPPER_MAX_PERIOD };

    // ---- DIR pin (input also enabled so PCNT can read it) ----
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << dir_pin,
        .mode = GPIO_MODE_INPUT_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "dir gpio");
    gpio_set_level(dir_pin, 1);

    // ---- MCPWM: timer -> operator -> comparator -> generator ----
    const mcpwm_timer_config_t tcfg = {
        .group_id      = STEPPER_MCPWM_GROUP,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = STEPPER_TIMER_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = s->period,
        .flags.update_period_on_empty = true,   // speed changes only at period boundary
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&tcfg, &s->timer), TAG, "timer");

    const mcpwm_operator_config_t ocfg = { .group_id = STEPPER_MCPWM_GROUP };
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&ocfg, &s->oper), TAG, "operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s->oper, s->timer), TAG, "connect");

    const mcpwm_comparator_config_t ccfg = { .flags.update_cmp_on_tez = true };
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s->oper, &ccfg, &s->cmp), TAG, "comparator");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s->cmp, s->period / 2), TAG, "cmp");

    const mcpwm_generator_config_t gcfg = { .gen_gpio_num = step_pin };
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(s->oper, &gcfg, &s->gen), TAG, "generator");

    // LOW at period start, HIGH at mid-period. Rising edge (the step) is mid-period,
    // so stopping "at empty" always finishes with the pin low.
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(s->gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_LOW)), TAG, "gen tez");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(s->gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s->cmp,
                                       MCPWM_GEN_ACTION_HIGH)), TAG, "gen cmp");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s->timer), TAG, "timer enable");

    // ---- PCNT: count rising edges on STEP, direction from DIR level ----
    const pcnt_unit_config_t ucfg = {
        .low_limit  = -PCNT_LIMIT,
        .high_limit =  PCNT_LIMIT,
        .flags.accum_count = true,   // extend to 32-bit in software
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&ucfg, &s->pcnt), TAG, "pcnt unit");

    const pcnt_glitch_filter_config_t filt = { .max_glitch_ns = 1000 };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(s->pcnt, &filt), TAG, "pcnt filter");

    const pcnt_chan_config_t chcfg = { .edge_gpio_num = step_pin, .level_gpio_num = dir_pin };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(s->pcnt, &chcfg, &s->pcnt_ch), TAG, "pcnt chan");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(s->pcnt_ch,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,      // rising edge
        PCNT_CHANNEL_EDGE_ACTION_HOLD), TAG, "edge");   // falling edge
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(s->pcnt_ch,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,         // DIR high: count up
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "level"); // DIR low: count down

    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s->pcnt,  PCNT_LIMIT), TAG, "wp+");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(s->pcnt, -PCNT_LIMIT), TAG, "wp-");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(s->pcnt), TAG, "pcnt enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s->pcnt), TAG, "pcnt clear");
    return pcnt_unit_start(s->pcnt);
}

esp_err_t stepper_set_rate(stepper_t *s, float rate)
{
    float mag = fabsf(rate);

    if (mag < STEPPER_MIN_RATE) {
        if (s->running) {
            ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s->timer, MCPWM_TIMER_STOP_EMPTY),
                                TAG, "stop");
            s->running = false;
        }
        s->rate = 0.0f;
        return ESP_OK;
    }
    if (mag > STEPPER_MAX_RATE) mag = STEPPER_MAX_RATE;

    bool fwd = rate > 0.0f;
    if (fwd != s->forward) {
        gpio_set_level(s->dir_pin, fwd);
        s->forward = fwd;
    }

    uint32_t period = (uint32_t)(STEPPER_TIMER_HZ / mag + 0.5f);
    if (period > STEPPER_MAX_PERIOD) period = STEPPER_MAX_PERIOD;
    if (period < 2) period = 2;

    if (period != s->period) {
        // Order the two writes so compare < period holds at every moment,
        // even if a period boundary falls between them.
        if (period < s->period) {
            ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s->cmp, period / 2), TAG, "cmp");
            ESP_RETURN_ON_ERROR(mcpwm_timer_set_period(s->timer, period), TAG, "period");
        } else {
            ESP_RETURN_ON_ERROR(mcpwm_timer_set_period(s->timer, period), TAG, "period");
            ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s->cmp, period / 2), TAG, "cmp");
        }
        s->period = period;
    }

    if (!s->running) {
        ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s->timer, MCPWM_TIMER_START_NO_STOP),
                            TAG, "start");
        s->running = true;
    }

    float applied = (float)STEPPER_TIMER_HZ / (float)period;
    s->rate = fwd ? applied : -applied;
    return ESP_OK;
}

float stepper_get_rate(const stepper_t *s) { return s->rate; }

esp_err_t stepper_get_position(stepper_t *s, int32_t *steps)
{
    int v;
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(s->pcnt, &v), TAG, "count");
    *steps = (int32_t)v;
    return ESP_OK;
}

esp_err_t stepper_reset_position(stepper_t *s)
{
    return pcnt_unit_clear_count(s->pcnt);
}