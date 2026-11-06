#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/uart.h"

// Frame: [0xAA][v float32 LE x4][w float32 LE x4][CRC8]  = 10 bytes
// CRC-8 poly 0x07, init 0x00, no reflect, no xorout (CRC-8/SMBUS),
// computed over the header + 8 data bytes (first 9 bytes).
#define UART_CMD_HEADER      0xAA
#define UART_CMD_FRAME_LEN   10

typedef struct {
    float   v;           // m/s, from the last valid frame
    float   w;           // rad/s, from the last valid frame
    int64_t last_rx_us;  // esp_timer_get_time() of the last valid frame, 0 = never
} uart_cmd_t;

typedef struct {
    uart_port_t port;   // don't use UART_NUM_0 (console) or UART_NUM_1 (TMC bus)
    int tx_pin;
    int rx_pin;
    int baud;
} uart_cmd_config_t;

// Starts the receive task, which updates the latest command on every valid frame.
esp_err_t uart_cmd_start(const uart_cmd_config_t *cfg);

// Thread-safe snapshot of the latest command.
void uart_cmd_get(uart_cmd_t *out);
