#include <stdio.h>
#include "tmc2209.h"
#include "stepper_motion.h"
#include "ramp.h"
#include "diff_drive.h"
#include "uart_cmd.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "math.h"

static const char *TAG = "main";

// Stepper Driver Configurations
#define TMC_RMS_CURRENT     1500
#define TMC_HOLD_PERCENT    50
#define TMC_MICROSTEP       16
#define TMC_SPREADCYCLE     1

#define TMC_TX_SHARED       17
#define TMC_RX_SHARED       16
#define TMC_UART            UART_NUM_1
#define TMC_EN_SHARED       21

#define LEFT_DRIVER_ADDR    1
#define LEFT_DRIVER_STEP    2
#define LEFT_DRIVER_DIR     15
#define RIGHT_DRIVER_ADDR   0
#define RIGHT_DRIVER_STEP   22
#define RIGHT_DRIVER_DIR    23

// Robot Configuration
#define WHEEL_RADIUS_M      0.0325f
#define TRACK_WIDTH_M       0.2f
#define FULL_STEPS_PER_REV  200

#define MAX_STEPS_PER_SEC   25000
#define MAX_ACCEL           50000

// UART Configuration
#define CMD_UART            UART_NUM_2
#define CMD_TX_PIN          25
#define CMD_RX_PIN          26
#define CMD_BAUD            115200
#define CMD_TIMEOUT_MS      250
#define CONTROL_HZ          100

static tmc2209_t tmc_left, tmc_right;
static stepper_t left, right;

static void configure_drivers(void) {
    ESP_ERROR_CHECK(tmc2209_init(&tmc_left,  TMC_UART, TMC_TX_SHARED, TMC_RX_SHARED, LEFT_DRIVER_ADDR));
    ESP_ERROR_CHECK(tmc2209_init(&tmc_right, TMC_UART, TMC_TX_SHARED, TMC_RX_SHARED, RIGHT_DRIVER_ADDR));

    ESP_ERROR_CHECK(tmc2209_set_current(&tmc_left,  TMC_RMS_CURRENT, TMC_HOLD_PERCENT));
    ESP_ERROR_CHECK(tmc2209_set_current(&tmc_right, TMC_RMS_CURRENT, TMC_HOLD_PERCENT));

    ESP_ERROR_CHECK(tmc2209_set_microsteps(&tmc_left,  TMC_MICROSTEP));
    ESP_ERROR_CHECK(tmc2209_set_microsteps(&tmc_right, TMC_MICROSTEP));

    ESP_ERROR_CHECK(tmc2209_set_spreadcycle(&tmc_left,  TMC_SPREADCYCLE));
    ESP_ERROR_CHECK(tmc2209_set_spreadcycle(&tmc_right, TMC_SPREADCYCLE));

    ESP_ERROR_CHECK(stepper_init(&left,  LEFT_DRIVER_STEP,  LEFT_DRIVER_DIR));
    ESP_ERROR_CHECK(stepper_init(&right, RIGHT_DRIVER_STEP, RIGHT_DRIVER_DIR));

    ESP_ERROR_CHECK(tmc2209_set_inverted(&tmc_right, true));
}

void app_main(void) {
    configure_drivers();

    gpio_set_direction(TMC_EN_SHARED, GPIO_MODE_OUTPUT);
    gpio_set_level(TMC_EN_SHARED, 1);

    const ramp_config_t rc = {
        .left = &left, .right = &right,
        .accel = MAX_ACCEL,
        .max_rate = MAX_STEPS_PER_SEC,
    };
    ESP_ERROR_CHECK(ramp_start(&rc));

    const diff_drive_config_t dd = {
        .wheel_radius_m = WHEEL_RADIUS_M,
        .track_width_m  = TRACK_WIDTH_M,
        .steps_per_rev  = FULL_STEPS_PER_REV * TMC_MICROSTEP,
    };
    ESP_ERROR_CHECK(diff_drive_init(&dd));

    const uart_cmd_config_t uc = {
        .port = CMD_UART,
        .tx_pin = CMD_TX_PIN,
        .rx_pin = CMD_RX_PIN,
        .baud = CMD_BAUD,
    };
    ESP_ERROR_CHECK(uart_cmd_start(&uc));
    
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    bool timed_out = true;
    bool drivers_on = false;

    for (;;) {
        xTaskDelayUntil(&last_wake, period);

        uart_cmd_t cmd;
        uart_cmd_get(&cmd);
        bool stale = cmd.last_rx_us == 0 ||
                     esp_timer_get_time() - cmd.last_rx_us > CMD_TIMEOUT_MS * 1000LL;

        if (stale) {
            if (!timed_out) ESP_LOGW(TAG, "command timeout, stopping");
            timed_out = true;
            diff_drive_set(0.0f, 0.0f);

            // Let the ramp bring both wheels to a stop, then cut the drivers
            if (drivers_on) {
                float l, r;
                ramp_get_current(&l, &r);
                if (fabsf(l) < STEPPER_MIN_RATE && fabsf(r) < STEPPER_MIN_RATE) {
                    gpio_set_level(TMC_EN_SHARED, 1);
                    drivers_on = false;
                    ESP_LOGI(TAG, "drivers disabled");
                }
            }
        } else {
            if (timed_out) ESP_LOGI(TAG, "command link up");
            timed_out = false;
            if (!drivers_on) {
                gpio_set_level(TMC_EN_SHARED, 0);
                drivers_on = true;
            }
            diff_drive_set(cmd.v, cmd.w);
        }
    }
    gpio_set_level(TMC_EN_SHARED, 1);
}