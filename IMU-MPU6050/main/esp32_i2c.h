#ifndef __ESP32_I2C__
#define __ESP32_I2C__

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define ACK_VAL      0x0
#define NACK_VAL     0x1
#define ACK_CHECK_EN 0x1

typedef struct esp_i2c {
	uint8_t ADDR;
	gpio_num_t SDA;
	gpio_num_t SCL;
	i2c_mode_t MODE;
	i2c_port_t CHANNEL;
	uint32_t  SPEED;

	esp_err_t (*write)(struct esp_i2c *self, uint8_t *data_wr, size_t size);
        esp_err_t (*writeBits)(struct esp_i2c *self, int8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
        esp_err_t (*writeBit)(struct esp_i2c *self, uint8_t regAddr, uint8_t bitNum, uint8_t data); 
	esp_err_t (*read)(struct esp_i2c *self, uint8_t reg, uint8_t *data_rd, size_t size);
        esp_err_t (*readBits)(struct esp_i2c *self, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);


}I2C_CONFIG;

void i2c_init(I2C_CONFIG *i2c);

#endif
