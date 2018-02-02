#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "mpu6050.h"

void _reset(MPU6050 *self);
void _setDLPFMode(MPU6050 *self, uint8_t mode);
void _setRate(MPU6050 *self, uint8_t rate);
void _setAccelDLPF(MPU6050 *self, uint8_t range);
void _setTempSensorEnabled(MPU6050 *self, bool enabled);
void _setIntEnabled(MPU6050 *self, uint8_t enabled);
void _setClockSource(MPU6050 *self, uint8_t source);
void _setFullScaleGyroRange(MPU6050 *self, uint8_t range);
void _setFullScaleAccelRange(MPU6050 *self, uint8_t range);
void _setSleepEnabled(MPU6050 *self, bool enabled);
void _getMotion(MPU6050 *self);
void _setI2CBypassEnabled(MPU6050 *self, bool enabled);
void _setI2CMasterModeEnabled(MPU6050 *self, bool enabled);
void _readAllRaw(MPU6050 *self, uint8_t *buffer);
float _getFullScaleAccelGPL(MPU6050 *self);
uint8_t _getFullScaleAccelRangeId(MPU6050 *self);
bool _testConnection(MPU6050 *self);

void MPU6050_Init(MPU6050 *mpu)
{
    mpu->reset                    = _reset;
    mpu->setClockSource           = _setClockSource;
    mpu->setFullScaleGyroRange    = _setFullScaleGyroRange;
    mpu->setFullScaleAccelRange   = _setFullScaleAccelRange;
    mpu->setSleepEnabled          = _setSleepEnabled;
    mpu->getMotion                = _getMotion;
    mpu->setI2CBypassEnabled      = _setI2CBypassEnabled;
    mpu->setI2CMasterModeEnabled  = _setI2CMasterModeEnabled;
    mpu->getFullScaleAccelRangeId = _getFullScaleAccelRangeId;
    mpu->setTempSensorEnabled     = _setTempSensorEnabled;
    mpu->setIntEnabled            = _setIntEnabled;
    mpu->setAccelDLPF             = _setAccelDLPF;
    mpu->setRate                  = _setRate;
    mpu->setDLPFMode              = _setDLPFMode;
    mpu->testConnection           = _testConnection;
    mpu->readAllRaw               = _readAllRaw;

    //mpu->setI2CMasterModeEnabled(mpu, false);
}

void _reset(MPU6050 *self)
{
	self->i2c->writeBit(self->i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

bool _testConnection(MPU6050 *self)
{
	uint8_t data;
	self->i2c->readBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &data);
	printf("who am i %x\n", data);

	return data == 0x34;
}

void _readAllRaw(MPU6050 *self, uint8_t *buffer)
{
	self->i2c->read(self->i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buffer, BUF_LEN);
}

void _setDLPFMode(MPU6050 *self, uint8_t mode)
{
	self->i2c->writeBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void _setRate(MPU6050 *self, uint8_t rate)
{
	uint8_t cmd[2];
	cmd[0] = MPU6050_RA_SMPLRT_DIV;
	cmd[1] = rate;
	self->i2c->write(self->i2c, MPU6050_ADDRESS, cmd, 2);
}

void _setAccelDLPF(MPU6050 *self, uint8_t range)
{
	self->i2c->writeBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG_2, MPU6050_ACONFIG2_DLPF_BIT, MPU6050_ACONFIG2_DLPF_LENGTH, range);
}

void _setIntEnabled(MPU6050 *self, uint8_t enabled)
{
	uint8_t cmd[2];
	cmd[0] = MPU6050_RA_INT_ENABLE;
	cmd[1] = enabled;
	self->i2c->write(self->i2c, MPU6050_ADDRESS, cmd, 2);
}

void _setClockSource(MPU6050 *self, uint8_t source)
{
    self->i2c->writeBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void _setFullScaleGyroRange(MPU6050 *self, uint8_t range)
{
    self->i2c->writeBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void _setFullScaleAccelRange(MPU6050 *self, uint8_t range)
{
    self->i2c->writeBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

uint8_t _getFullScaleAccelRangeId(MPU6050 *self)
{
	self->i2c->readBits(self->i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
      MPU6050_ACONFIG_AFS_SEL_LENGTH, self->buffer);
  return self->buffer[0];
}

void _setSleepEnabled(MPU6050 *self, bool enabled)
{
    self->i2c->writeBit(self->i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void _setI2CBypassEnabled(MPU6050 *self, bool enabled) 
{
    self->i2c->writeBit(self->i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void _setI2CMasterModeEnabled(MPU6050 *self, bool enabled) 
{
    self->i2c->writeBit(self->i2c, MPU6050_ADDRESS, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void _setTempSensorEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}

void _getMotion(MPU6050 *self)
{
    self->i2c->read(self->i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, self->buffer, 14);

    self->ax = (((int16_t)self->buffer[0]) << 8) | self->buffer[1];
    self->ay = (((int16_t)self->buffer[2]) << 8) | self->buffer[3];
    self->az = (((int16_t)self->buffer[4]) << 8) | self->buffer[5];
    self->gx = (((int16_t)self->buffer[8]) << 8) | self->buffer[9];
    self->gy = (((int16_t)self->buffer[10]) << 8) | self->buffer[11];
    self->gz = (((int16_t)self->buffer[12]) << 8) | self->buffer[13];
}

float _getFullScaleAccelGPL(MPU6050 *self)
{
  int32_t rangeId;
  float range;

  rangeId = self->getFullScaleAccelRangeId(self);
  switch (rangeId)
  {
    case MPU6050_ACCEL_FS_2:
      range = MPU6050_G_PER_LSB_2;
      break;
    case MPU6050_ACCEL_FS_4:
      range = MPU6050_G_PER_LSB_4;
      break;
    case MPU6050_ACCEL_FS_8:
      range = MPU6050_G_PER_LSB_8;
      break;
    case MPU6050_ACCEL_FS_16:
      range = MPU6050_G_PER_LSB_16;
      break;
    default:
      range = MPU6050_DEG_PER_LSB_1000;
      break;
  }

  return range;
}
