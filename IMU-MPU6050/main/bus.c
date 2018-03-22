#include "bus.h"

#include "imu.h"

void Bus_Init(Bus *bus)
{
    bus->i2c.MODE    = I2C_MODE_MASTER;
    bus->i2c.SDA     = PIN_IMU_SDA;
    bus->i2c.SCL     = PIN_IMU_SCL;
    bus->i2c.CHANNEL = IMU_I2C_NUM;
    bus->i2c.SPEED   = IMU_I2C_SPEED;
	i2c_init(&bus->i2c);

}
