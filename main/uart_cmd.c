#include "uart_cmd.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_check.h"

static const char *TAG = "uart_cmd";
static uart_port_t s_port;
static uart_cmd_t s_cmd;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t crc8(const uint8_t *d, size_t n)
{
    uint8_t c = 0x00;
    while (n--) {
        c ^= *d++;
        for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

void uart_cmd_get(uart_cmd_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_cmd;
    taskEXIT_CRITICAL(&s_lock);
}

static void uart_cmd_task(void *arg)
{
    uint8_t buf[UART_CMD_FRAME_LEN];
    size_t n = 0;

    for (;;) {
        int got = uart_read_bytes(s_port, buf + n, UART_CMD_FRAME_LEN - n, pdMS_TO_TICKS(100));
        if (got > 0) n += got;
        if (n < UART_CMD_FRAME_LEN) continue;

        if (buf[0] == UART_CMD_HEADER && crc8(buf, 9) == buf[9]) {
            float v, w;
            memcpy(&v, &buf[1], 4);
            memcpy(&w, &buf[5], 4);
            if (isfinite(v) && isfinite(w)) {
                int64_t now = esp_timer_get_time();
                taskENTER_CRITICAL(&s_lock);
                s_cmd = (uart_cmd_t){ .v = v, .w = w, .last_rx_us = now };
                taskEXIT_CRITICAL(&s_lock);
            }
            n = 0;
        } else {
            // Bad frame: drop the first byte and resync to the next 0xAA in the buffer
            size_t i = 1;
            while (i < n && buf[i] != UART_CMD_HEADER) i++;
            memmove(buf, buf + i, n - i);
            n -= i;
        }
    }
}

esp_err_t uart_cmd_start(const uart_cmd_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->baud > 0, ESP_ERR_INVALID_ARG, TAG, "bad config");
    s_port = cfg->port;

    const uart_config_t ucfg = {
        .baud_rate  = cfg->baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(s_port, 512, 0, 0, NULL, 0), TAG, "install");
    ESP_RETURN_ON_ERROR(uart_param_config(s_port, &ucfg), TAG, "param");
    ESP_RETURN_ON_ERROR(uart_set_pin(s_port, cfg->tx_pin, cfg->rx_pin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "pins");

    BaseType_t ok = xTaskCreatePinnedToCore(uart_cmd_task, "uart_cmd", 3072, NULL, 9, NULL, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
