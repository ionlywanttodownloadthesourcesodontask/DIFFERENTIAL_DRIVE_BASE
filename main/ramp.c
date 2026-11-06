#include "ramp.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_check.h"

static const char *TAG = "ramp";

static ramp_config_t s_cfg;
static float s_target[2];
static float s_current[2];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void ramp_set_target(float left, float right)
{
    float biggest = fmaxf(fabsf(left), fabsf(right));
    if (biggest > s_cfg.max_rate) {
        float k = s_cfg.max_rate / biggest;
        left  *= k;
        right *= k;
    }
    taskENTER_CRITICAL(&s_lock);
    s_target[0] = left;
    s_target[1] = right;
    taskEXIT_CRITICAL(&s_lock);
}

void ramp_get_current(float *left, float *right)
{
    taskENTER_CRITICAL(&s_lock);
    *left  = s_current[0];
    *right = s_current[1];
    taskEXIT_CRITICAL(&s_lock);
}

static void ramp_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / RAMP_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();

    for (;;) {
        xTaskDelayUntil(&last_wake, period);

        // Use measured dt so acceleration stays correct even if a tick runs late
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (dt > 0.05f) dt = 0.05f;

        taskENTER_CRITICAL(&s_lock);
        float tl = s_target[0], tr = s_target[1];
        float cl = s_current[0], cr = s_current[1];
        taskEXIT_CRITICAL(&s_lock);

        float dl = tl - cl;
        float dr = tr - cr;
        float biggest  = fmaxf(fabsf(dl), fabsf(dr));
        float max_step = s_cfg.accel * dt;

        // Scale both changes by the same factor -> both wheels arrive together
        float k = (biggest > max_step) ? (max_step / biggest) : 1.0f;
        cl += dl * k;
        cr += dr * k;

        taskENTER_CRITICAL(&s_lock);
        s_current[0] = cl;
        s_current[1] = cr;
        taskEXIT_CRITICAL(&s_lock);

        stepper_set_rate(s_cfg.left,  cl);
        stepper_set_rate(s_cfg.right, cr);
    }
}

esp_err_t ramp_start(const ramp_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->left && cfg->right && cfg->accel > 0 && cfg->max_rate > 0,
                        ESP_ERR_INVALID_ARG, TAG, "bad config");
    ESP_RETURN_ON_FALSE(pdMS_TO_TICKS(1000 / RAMP_HZ) >= 1, ESP_ERR_INVALID_STATE, TAG,
                        "RAMP_HZ faster than FreeRTOS tick");
    s_cfg = *cfg;

    BaseType_t ok = xTaskCreatePinnedToCore(ramp_task, "ramp", 3072, NULL, 10, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}