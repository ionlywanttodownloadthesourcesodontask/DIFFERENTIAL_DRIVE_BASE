#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifndef TMC2209_RSENSE
#define TMC2209_RSENSE  0.11f      // ohms, sense resistor on TMC2209 BTT V1.3
#endif
#ifndef TMC2209_BAUD
#define TMC2209_BAUD    115200
#endif

// Registers (for tmc2209_read_reg / tmc2209_write_reg)
#define TMC2209_GCONF       0x00
#define TMC2209_GSTAT       0x01
#define TMC2209_IFCNT       0x02
#define TMC2209_IOIN        0x06
#define TMC2209_IHOLD_IRUN  0x10
#define TMC2209_TPOWERDOWN  0x11
#define TMC2209_TSTEP       0x12
#define TMC2209_TPWMTHRS    0x13
#define TMC2209_VACTUAL     0x22
#define TMC2209_SGTHRS      0x40
#define TMC2209_SG_RESULT   0x41
#define TMC2209_CHOPCONF    0x6C
#define TMC2209_DRV_STATUS  0x6F
#define TMC2209_PWMCONF     0x70

typedef struct {
    uart_port_t uart;
    uint8_t     addr;       // 0..3, set by MS1/MS2 pins (addr = MS1 + 2*MS2)
    uint32_t    gconf;      // shadow copies
    uint32_t    chopconf;
} tmc2209_t;

// Installs the UART on first use of a port. Further drivers on the same port share it,
// and their tx/rx arguments are ignored. Verifies the chip responds.
esp_err_t tmc2209_init(tmc2209_t *d, uart_port_t uart, int tx_pin, int rx_pin, uint8_t addr);

esp_err_t tmc2209_set_current(tmc2209_t *d, uint16_t run_ma_rms, uint8_t hold_percent);
esp_err_t tmc2209_set_microsteps(tmc2209_t *d, uint16_t microsteps);   // 1,2,4..256
esp_err_t tmc2209_set_spreadcycle(tmc2209_t *d, bool spreadcycle);     // false = stealthChop
esp_err_t tmc2209_set_stealth_threshold(tmc2209_t *d, uint32_t tpwmthrs); // 0 = off
esp_err_t tmc2209_set_inverted(tmc2209_t *d, bool invert);             // flip direction
esp_err_t tmc2209_enable(tmc2209_t *d, bool enable);                   // software enable (TOFF)

esp_err_t tmc2209_write_reg(tmc2209_t *d, uint8_t reg, uint32_t val);
esp_err_t tmc2209_read_reg(tmc2209_t *d, uint8_t reg, uint32_t *val);