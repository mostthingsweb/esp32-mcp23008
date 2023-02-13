#include "mcp23008.h"

#include "esp_err.h"

// I2C driver
#include "driver/i2c.h"

#include <esp_log.h>

// FreeRTOS (for delay)

// registers
#define MCP23008_IODIR      0x00
#define MCP23008_IPOL       0x01
#define MCP23008_GPINTEN    0x02
#define MCP23008_DEFVAL     0x03
#define MCP23008_INTCON     0x04
#define MCP23008_IOCON      0x05
#define MCP23008_GPPU       0x06
#define MCP23008_INTF       0x07
#define MCP23008_INTCAP     0x08
#define MCP23008_GPIO       0x09
#define MCP23008_OLAT       0x0A

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "MCP23008";

esp_err_t mcp23008_read_reg(mcp23008_t *mcp, uint8_t reg, uint8_t *d) {
    CHECK_ARG(mcp);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mcp->address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(mcp->port, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if( ret != ESP_OK ) {
        ESP_LOGE(TAG,"Error reading register %d",reg);
        return ESP_FAIL;
    }
    vTaskDelay(30/portTICK_PERIOD_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mcp->address << 1) | I2C_MASTER_READ, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, d, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(mcp->port, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if( ret != ESP_OK ) {
        ESP_LOGE(TAG,"Error reading register %d",reg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mcp23008_write_reg(mcp23008_t *mcp, uint8_t reg, uint8_t d) {
    CHECK_ARG(mcp);

    ESP_LOGI(TAG,"write reg MCP port %d address %d", mcp->port, mcp->address);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (mcp->address << 1) | I2C_MASTER_WRITE, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, d, I2C_MASTER_ACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(mcp->port, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if( ret != ESP_OK ) {
        ESP_LOGE(TAG,"Error writing register %d",reg);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t mcp23008_init(mcp23008_t *mcp) {
    ESP_LOGI(TAG,"Init MCP port %d address %d", mcp->port, mcp->address);
    return mcp23008_write_reg(mcp, MCP23008_IODIR, 0x00);
}

esp_err_t mcp23008_read_port(mcp23008_t *mcp, uint8_t *d) {
    esp_err_t ret = mcp23008_read_reg(mcp, MCP23008_GPIO, d);
    if( ret == ESP_OK ) {
        mcp->current = *d;
        return ESP_OK;
    }
    return ret;
}

esp_err_t mcp23008_write_port(mcp23008_t *mcp, uint8_t d) {
    esp_err_t ret = mcp23008_write_reg(mcp, MCP23008_GPIO, d);
    if( ret == ESP_OK ) {
        mcp->current = d;
        return ESP_OK;
    }
    return ret;
}

esp_err_t mcp23008_set_port_direction(mcp23008_t *mcp, uint8_t dirs) {
    return mcp23008_write_reg(mcp, MCP23008_IODIR, dirs);
}

esp_err_t mcp23008_get_port_direction(mcp23008_t *mcp, uint8_t *dirs) {
    return mcp23008_read_reg(mcp, MCP23008_IODIR, dirs);
}

esp_err_t mcp23008_set_interrupt_enable(mcp23008_t *mcp, uint8_t intr) {
    return mcp23008_write_reg(mcp, MCP23008_GPINTEN, intr);
}

esp_err_t mcp23008_read_interrupt_enable(mcp23008_t *mcp, uint8_t *intr) {
    return mcp23008_read_reg(mcp, MCP23008_GPINTEN, intr);
}

esp_err_t mcp23008_set_default_value(mcp23008_t *mcp, uint8_t defval) {
    return mcp23008_write_reg(mcp, MCP23008_DEFVAL, defval);
}

esp_err_t mcp23008_read_default_value(mcp23008_t *mcp, uint8_t *defval) {
    return mcp23008_read_reg(mcp, MCP23008_DEFVAL, defval);
}

esp_err_t mcp23008_set_interrupt_control(mcp23008_t *mcp, uint8_t intr) {
    return mcp23008_write_reg(mcp, MCP23008_INTCON, intr);
}

esp_err_t mcp23008_read_interrupt_control(mcp23008_t *mcp, uint8_t *intr) {
    return mcp23008_read_reg(mcp, MCP23008_INTCON, intr);
}

esp_err_t mcp23008_read_interrupt_reg(mcp23008_t *mcp, uint8_t *intr) {
    return mcp23008_read_reg(mcp, MCP23008_INTF, intr);
}

esp_err_t mcp23008_read_interrupt_capture(mcp23008_t *mcp, uint8_t *intr) {
    return mcp23008_read_reg(mcp, MCP23008_INTCAP, intr);
}

esp_err_t mcp23008_set_output_latch(mcp23008_t *mcp, uint8_t olat) {
    return mcp23008_write_reg(mcp, MCP23008_OLAT, olat);
}

esp_err_t mcp23008_read_output_latch(mcp23008_t *mcp, uint8_t *olat) {
    return mcp23008_read_reg(mcp, MCP23008_OLAT, olat);
}

esp_err_t mcp23008_set_pullups(mcp23008_t *mcp, uint8_t pu) {
    return mcp23008_write_reg(mcp, MCP23008_GPPU, pu);
}

esp_err_t mcp23008_read_pullups(mcp23008_t *mcp, uint8_t *pu) {
    return mcp23008_read_reg(mcp, MCP23008_GPPU, pu);
}

esp_err_t mcp23008_port_set_bit(mcp23008_t *mcp, uint8_t b) {
    mcp->current |= (1 << b);
    return mcp23008_write_port(mcp, mcp->current);
}

esp_err_t mcp23008_port_clear_bit(mcp23008_t *mcp, uint8_t b) {
    mcp->current &= ~(1 << b);
    return mcp23008_write_port(mcp, mcp->current);
}

esp_err_t mcp23008_toggle_bit(mcp23008_t *mcp, uint8_t b) {
    mcp->current ^= (1 << b);
    return mcp23008_write_port(mcp, mcp->current);
}


