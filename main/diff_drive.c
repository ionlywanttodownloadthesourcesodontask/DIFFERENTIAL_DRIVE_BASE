#include "diff_drive.h"
#include <math.h>
#include "ramp.h"
#include "esp_check.h"

static const char *TAG = "diff_drive";

static float s_steps_per_m;   // wheel steps per metre of travel
static float s_half_track;    // track_width / 2

esp_err_t diff_drive_init(const diff_drive_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->wheel_radius_m > 0 && cfg->track_width_m > 0 &&
                        cfg->steps_per_rev > 0, ESP_ERR_INVALID_ARG, TAG, "bad config");
    s_steps_per_m = cfg->steps_per_rev / (2.0f * (float)M_PI * cfg->wheel_radius_m);
    s_half_track  = 0.5f * cfg->track_width_m;
    return ESP_OK;
}

void diff_drive_set(float v, float w)
{
    float left  = (v - w * s_half_track) * s_steps_per_m;
    float right = (v + w * s_half_track) * s_steps_per_m;
    ramp_set_target(left, right);
}

void diff_drive_get(float *v, float *w)
{
    float l, r;
    ramp_get_current(&l, &r);
    l /= s_steps_per_m;   // -> m/s at each wheel
    r /= s_steps_per_m;
    *v = 0.5f * (r + l);
    *w = (r - l) / (2.0f * s_half_track);
}
