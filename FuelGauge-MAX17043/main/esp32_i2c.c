#include "esp32_i2c.h"

esp_err_t _write(I2C_CONFIG *self, uint8_t *data_wr, size_t size);
esp_err_t _read(I2C_CONFIG *self, uint8_t addr, uint8_t *data_rd, size_t size);

void i2c_init(I2C_CONFIG *i2c)
{
	i2c_config_t conf_i2c;
	conf_i2c.mode = i2c->MODE;
	conf_i2c.sda_io_num = i2c->SDA;
	conf_i2c.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf_i2c.scl_io_num = i2c->SCL;
	conf_i2c.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf_i2c.master.clk_speed = i2c->SPEED;
	i2c_param_config(i2c->CHANNEL, &conf_i2c);
	i2c_driver_install(i2c->CHANNEL, conf_i2c.mode, 0, 0, 0);

	i2c->write = _write;
	i2c->read = _read;
}

esp_err_t _write(I2C_CONFIG *self, uint8_t *data_wr, size_t size)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( self->ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_wr[0], ACK_CHECK_EN);
	i2c_master_write_byte(cmd, data_wr[1], ACK_VAL);
	i2c_master_write_byte(cmd, data_wr[2], NACK_VAL);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(self->CHANNEL, cmd, 500 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t _read(I2C_CONFIG *self, uint8_t reg, uint8_t *data_rd, size_t size)
{
	esp_err_t ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( self->ADDR << 1 ) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(self->CHANNEL, cmd, 500 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( self->ADDR << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
	if (size > 1) {
		i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(self->CHANNEL, cmd, 500 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

