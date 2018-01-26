#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "mpu6050.h"

void _setClockSource(MPU6050 *self, uint8_t source);
void _setFullScaleGyroRange(MPU6050 *self, uint8_t range);
void _setFullScaleAccelRange(MPU6050 *self, uint8_t range);
void _setSleepEnabled(MPU6050 *self, bool enabled);
void _getMotion(MPU6050 *self);

esp_err_t MPU6050_Init(MPU6050 *mpu)
{
    uint8_t data;

    mpu->i2c.ADDR  = MPU6050_ADDRESS_AD0_LOW;
    mpu->i2c.MODE  = I2C_MODE_MASTER;
    mpu->i2c.SDA   = mpu->SDAPin;
    mpu->i2c.SCL   = mpu->SCLPin;
    mpu->i2c.SPEED = 400000;
    i2c_init(&mpu->i2c);

    mpu->i2c.readBits(&mpu->i2c, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &data);
    printf("who am i %x\n", data);

    if (data != 0x34)
        return ESP_FAIL;

    mpu->setClockSource         = _setClockSource;
    mpu->setFullScaleGyroRange  = _setFullScaleGyroRange;
    mpu->setFullScaleAccelRange = _setFullScaleAccelRange;
    mpu->setSleepEnabled        = _setSleepEnabled;
    mpu->getMotion              = _getMotion;

    mpu->setClockSource(mpu, MPU6050_CLOCK_PLL_XGYRO);
    mpu->setFullScaleGyroRange(mpu, MPU6050_GYRO_FS_250);
    mpu->setFullScaleAccelRange(mpu, MPU6050_ACCEL_FS_2);
    //setDLPFMode(MPU6050_DLPF_BW_5);
    mpu->setSleepEnabled(mpu, false);
    return ESP_OK;
}

void _setClockSource(MPU6050 *self, uint8_t source)
{
    self->i2c.writeBits(&self->i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void _setFullScaleGyroRange(MPU6050 *self, uint8_t range)
{
    self->i2c.writeBits(&self->i2c, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void _setFullScaleAccelRange(MPU6050 *self, uint8_t range)
{
    self->i2c.writeBits(&self->i2c, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void _setSleepEnabled(MPU6050 *self, bool enabled)
{
    self->i2c.writeBit(&self->i2c, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void _getMotion(MPU6050 *self)
{
    uint8_t buffer[14];
    self->i2c.read(&self->i2c, MPU6050_RA_ACCEL_XOUT_H, buffer, 14);

    self->ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    self->ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    self->az = (((int16_t)buffer[4]) << 8) | buffer[5];
    self->gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    self->gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    self->gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

