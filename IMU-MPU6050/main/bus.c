#include "bus.h"
#include "mpu6050.h"

#define MPU6050_SDA 18
#define MPU6050_SCL 19

void Bus_Init(Bus *bus)
{
    bus->i2c.MODE    = I2C_MODE_MASTER;
    bus->i2c.SDA     = MPU6050_SDA;
    bus->i2c.SCL     = MPU6050_SCL;
    bus->i2c.CHANNEL = I2C_NUM_1;
    bus->i2c.SPEED   = 400000;

	i2c_init(&bus->i2c);
}
