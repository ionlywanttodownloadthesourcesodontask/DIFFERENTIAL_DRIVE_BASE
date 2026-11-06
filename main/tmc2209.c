#include "tmc2209.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "tmc2209";

#define GCONF_I_SCALE_ANALOG   (1u << 0)
#define GCONF_EN_SPREADCYCLE   (1u << 2)
#define GCONF_SHAFT            (1u << 3)
#define GCONF_PDN_DISABLE      (1u << 6)
#define GCONF_MSTEP_REG_SEL    (1u << 7)
#define GCONF_MULTISTEP_FILT   (1u << 8)

#define CHOP_TOFF_MASK         0x0Fu
#define CHOP_VSENSE            (1u << 17)
#define CHOP_MRES_SHIFT        24
#define CHOP_MRES_MASK         (0x0Fu << CHOP_MRES_SHIFT)

#define TIMEOUT                pdMS_TO_TICKS(20)

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (int j = 0; j < 8; j++) {
            crc = ((crc >> 7) ^ (b & 1)) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

esp_err_t tmc2209_write_reg(tmc2209_t *d, uint8_t reg, uint32_t val)
{
    uint8_t pkt[8] = { 0x05, d->addr, reg | 0x80,
                       val >> 24, val >> 16, val >> 8, val, 0 };
    pkt[7] = crc8(pkt, 7);

    uint8_t echo[8];
    uart_flush_input(d->uart);
    uart_write_bytes(d->uart, pkt, sizeof pkt);
    // Single-wire bus: we receive our own bytes back
    if (uart_read_bytes(d->uart, echo, sizeof echo, TIMEOUT) != sizeof echo)
        return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

esp_err_t tmc2209_read_reg(tmc2209_t *d, uint8_t reg, uint32_t *val)
{
    uint8_t req[4] = { 0x05, d->addr, reg & 0x7F, 0 };
    req[3] = crc8(req, 3);

    uint8_t buf[12];  // 4 echo + 8 reply
    uart_flush_input(d->uart);
    uart_write_bytes(d->uart, req, sizeof req);
    if (uart_read_bytes(d->uart, buf, sizeof buf, TIMEOUT) != sizeof buf)
        return ESP_ERR_TIMEOUT;

    const uint8_t *r = buf + 4;
    if (r[0] != 0x05 || r[1] != 0xFF || r[2] != (reg & 0x7F) || crc8(r, 7) != r[7])
        return ESP_ERR_INVALID_CRC;

    *val = ((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) | ((uint32_t)r[5] << 8) | r[6];
    return ESP_OK;
}

esp_err_t tmc2209_init(tmc2209_t *d, uart_port_t uart, int tx_pin, int rx_pin, uint8_t addr)
{
    ESP_RETURN_ON_FALSE(d && addr <= 3, ESP_ERR_INVALID_ARG, TAG, "bad args");
    d->uart = uart;
    d->addr = addr;

    if (!uart_is_driver_installed(uart)) {
        const uart_config_t cfg = {
            .baud_rate  = TMC2209_BAUD,
            .data_bits  = UART_DATA_8_BITS,
            .parity     = UART_PARITY_DISABLE,
            .stop_bits  = UART_STOP_BITS_1,
            .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        ESP_RETURN_ON_ERROR(uart_driver_install(uart, 256, 0, 0, NULL, 0), TAG, "install");
        ESP_RETURN_ON_ERROR(uart_param_config(uart, &cfg), TAG, "config");
        ESP_RETURN_ON_ERROR(uart_set_pin(uart, tx_pin, rx_pin,
                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "pins");
    }

    uint32_t ioin;
    ESP_RETURN_ON_ERROR(tmc2209_read_reg(d, TMC2209_IOIN, &ioin), TAG,
                        "no reply from addr %u", addr);
    ESP_RETURN_ON_FALSE((ioin >> 24) == 0x21, ESP_ERR_INVALID_RESPONSE, TAG,
                        "bad version 0x%02lx", ioin >> 24);

    // UART control, MRES from register (MS pins are the address), internal current ref
    d->gconf = GCONF_PDN_DISABLE | GCONF_MSTEP_REG_SEL | GCONF_MULTISTEP_FILT;
    ESP_RETURN_ON_ERROR(tmc2209_write_reg(d, TMC2209_GCONF, d->gconf), TAG, "gconf");
    ESP_RETURN_ON_ERROR(tmc2209_write_reg(d, TMC2209_GSTAT, 0x7), TAG, "gstat"); // clear flags
    return tmc2209_read_reg(d, TMC2209_CHOPCONF, &d->chopconf);
}

esp_err_t tmc2209_set_current(tmc2209_t *d, uint16_t run_ma, uint8_t hold_percent)
{
    const float k = 32.0f * 1.41421f * (run_ma / 1000.0f) * (TMC2209_RSENSE + 0.02f);
    int cs = (int)(k / 0.325f - 1.0f);
    bool vsense = false;
    if (cs < 16) {                       // better resolution at low currents
        vsense = true;
        cs = (int)(k / 0.180f - 1.0f);
    }
    if (cs > 31) cs = 31;
    if (cs < 0)  cs = 0;
    if (hold_percent > 100) hold_percent = 100;
    uint32_t hold = (uint32_t)cs * hold_percent / 100;

    d->chopconf = vsense ? (d->chopconf | CHOP_VSENSE) : (d->chopconf & ~CHOP_VSENSE);
    ESP_RETURN_ON_ERROR(tmc2209_write_reg(d, TMC2209_CHOPCONF, d->chopconf), TAG, "chop");

    // IHOLDDELAY = 8: gradual ramp down to hold current
    return tmc2209_write_reg(d, TMC2209_IHOLD_IRUN,
                             (8u << 16) | ((uint32_t)cs << 8) | hold);
}

esp_err_t tmc2209_set_microsteps(tmc2209_t *d, uint16_t microsteps)
{
    for (uint32_t mres = 0; mres <= 8; mres++) {
        if ((256u >> mres) == microsteps) {
            d->chopconf = (d->chopconf & ~CHOP_MRES_MASK) | (mres << CHOP_MRES_SHIFT);
            return tmc2209_write_reg(d, TMC2209_CHOPCONF, d->chopconf);
        }
    }
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t gconf_bit(tmc2209_t *d, uint32_t bit, bool on)
{
    d->gconf = on ? (d->gconf | bit) : (d->gconf & ~bit);
    return tmc2209_write_reg(d, TMC2209_GCONF, d->gconf);
}

esp_err_t tmc2209_set_spreadcycle(tmc2209_t *d, bool sc) { return gconf_bit(d, GCONF_EN_SPREADCYCLE, sc); }
esp_err_t tmc2209_set_inverted(tmc2209_t *d, bool inv)   { return gconf_bit(d, GCONF_SHAFT, inv); }

esp_err_t tmc2209_set_stealth_threshold(tmc2209_t *d, uint32_t tpwmthrs)
{
    return tmc2209_write_reg(d, TMC2209_TPWMTHRS, tpwmthrs & 0xFFFFF);
}

esp_err_t tmc2209_enable(tmc2209_t *d, bool enable)
{
    d->chopconf = (d->chopconf & ~CHOP_TOFF_MASK) | (enable ? 3u : 0u);
    return tmc2209_write_reg(d, TMC2209_CHOPCONF, d->chopconf);
}