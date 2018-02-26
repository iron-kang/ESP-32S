#include "bus.h"
#include "mpu6050.h"

void Bus_Init(Bus *bus)
{
    bus->i2c.MODE    = I2C_MODE_MASTER;
    bus->i2c.SDA     = PIN_MPU6050_SDA;
    bus->i2c.SCL     = PIN_MPU6050_SCL;
    bus->i2c.CHANNEL = MPU6050_I2C_NUM;
    bus->i2c.SPEED   = MPU6050_I2C_SPEED;
	i2c_init(&bus->i2c);

}
