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
void _readAllRaw(MPU6050 *self, uint8_t *buffer, uint8_t len);
float _getFullScaleAccelGPL(MPU6050 *self);
uint8_t _getFullScaleAccelRangeId(MPU6050 *self);
bool _testConnection(MPU6050 *self);
void _setSlave4MasterDelay(MPU6050 *self, uint8_t delay);
void _setWaitForExternalSensorEnabled(MPU6050 *self, bool enabled);
void _setInterruptMode(MPU6050 *self, bool mode);
void _setInterruptDrive(MPU6050 *self, bool drive);
void _setInterruptLatch(MPU6050 *self, bool latch);
void _setInterruptLatchClear(MPU6050 *self, bool clear);
void _setSlaveReadWriteTransitionEnabled(MPU6050 *self, bool enabled);
void _setMasterClockSpeed(MPU6050 *self, uint8_t speed);
void _setSlaveAddress(MPU6050 *self, uint8_t num, uint8_t address);
void _setSlaveRegister(MPU6050 *self, uint8_t num, uint8_t reg);
void _setSlaveDataLength(MPU6050 *self, uint8_t num, uint8_t length);
void _setSlaveDelayEnabled(MPU6050 *self, uint8_t num, bool enabled);
void _setIntDataReadyEnabled(MPU6050 *self, bool enabled);
void _setSlaveEnabled(MPU6050 *self, uint8_t num, bool enabled);

void MPU6050_Init(MPU6050 *mpu, uint8_t addr)
{
	mpu->devAddr = addr;
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
    mpu->setSlave4MasterDelay     = _setSlave4MasterDelay;
    mpu->setWaitForExternalSensorEnabled = _setWaitForExternalSensorEnabled;
    mpu->setInterruptMode         = _setInterruptMode;
    mpu->setInterruptDrive        = _setInterruptDrive;
    mpu->setInterruptLatch        = _setInterruptLatch;
    mpu->setInterruptLatchClear   = _setInterruptLatchClear;
    mpu->setSlaveReadWriteTransitionEnabled = _setSlaveReadWriteTransitionEnabled;
    mpu->setMasterClockSpeed      = _setMasterClockSpeed;
    mpu->setSlaveAddress          = _setSlaveAddress;
    mpu->setSlaveRegister         = _setSlaveRegister;
    mpu->setSlaveDataLength       = _setSlaveDataLength;
    mpu->setSlaveDelayEnabled     = _setSlaveDelayEnabled;
    mpu->setIntDataReadyEnabled   = _setIntDataReadyEnabled;
    mpu->setSlaveEnabled          = _setSlaveEnabled;
}

void _reset(MPU6050 *self)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, 1);
}

bool _testConnection(MPU6050 *self)
{
	uint8_t data;
	self->i2c->readBits(self->i2c, self->devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &data);
	printf("who am i %x\n", data);

	return data == 0x38;
}

void _readAllRaw(MPU6050 *self, uint8_t *buffer, uint8_t len)
{
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ACCEL_XOUT_H, buffer, len);
}

void _setDLPFMode(MPU6050 *self, uint8_t mode)
{
	self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void _setRate(MPU6050 *self, uint8_t rate)
{
	uint8_t cmd[2];
	cmd[0] = MPU6050_RA_SMPLRT_DIV;
	cmd[1] = rate;
	self->i2c->write(self->i2c, self->devAddr, cmd, 2);
}

void _setAccelDLPF(MPU6050 *self, uint8_t range)
{
	self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_ACCEL_CONFIG_2, MPU6050_ACONFIG2_DLPF_BIT, MPU6050_ACONFIG2_DLPF_LENGTH, range);
}

void _setIntEnabled(MPU6050 *self, uint8_t enabled)
{
	uint8_t cmd[2];
	cmd[0] = MPU6050_RA_INT_ENABLE;
	cmd[1] = enabled;
	self->i2c->write(self->i2c, self->devAddr, cmd, 2);
}

void _setClockSource(MPU6050 *self, uint8_t source)
{
    self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void _setFullScaleGyroRange(MPU6050 *self, uint8_t range)
{
    self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void _setFullScaleAccelRange(MPU6050 *self, uint8_t range)
{
    self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

uint8_t _getFullScaleAccelRangeId(MPU6050 *self)
{
	self->i2c->readBits(self->i2c, self->devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT,
      MPU6050_ACONFIG_AFS_SEL_LENGTH, self->buffer);
  return self->buffer[0];
}

void _setSleepEnabled(MPU6050 *self, bool enabled)
{
    self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void _setI2CBypassEnabled(MPU6050 *self, bool enabled) 
{
    self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6500_RA_USER_CTRL
 * @see MPU6500_USERCTRL_I2C_MST_EN_BIT
 */
void _setI2CMasterModeEnabled(MPU6050 *self, bool enabled) 
{
    self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void _setTempSensorEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}

void _getMotion(MPU6050 *self)
{
    self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ACCEL_XOUT_H, self->buffer, 14);

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

/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU6500_RA_I2C_SLV4_CTRL
 */
void _setSlave4MasterDelay(MPU6050 *self, uint8_t delay)
{
	self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
			MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}

/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU6500_RA_I2C_MST_CTRL
 */
void _setWaitForExternalSensorEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}

/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_INT_LEVEL_BIT
 */
void _setInterruptMode(MPU6050 *self, bool mode)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}

/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_INT_OPEN_BIT
 */
void _setInterruptDrive(MPU6050 *self, bool drive)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}

/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_LATCH_INT_EN_BIT
 */
void _setInterruptLatch(MPU6050 *self, bool latch)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}

/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU6500_RA_INT_PIN_CFG
 * @see MPU6500_INTCFG_INT_RD_CLEAR_BIT
 */
void _setInterruptLatchClear(MPU6050 *self, bool clear)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU6500_RA_I2C_MST_CTRL
 */
void _setSlaveReadWriteTransitionEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}

/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU6500_RA_I2C_MST_CTRL
 */
void _setMasterClockSpeed(MPU6050 *self, uint8_t speed)
{
	self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT,
			MPU6050_I2C_MST_CLK_LENGTH, speed);
}

void _setSlaveAddress(MPU6050 *self, uint8_t num, uint8_t address)
{
  if (num > 3)
    return;

  self->buffer[0] = MPU6050_RA_I2C_SLV0_ADDR + num * 3;
  self->buffer[1] = address;
  self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
}

/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU6500_RA_I2C_SLV0_REG
 */
void _setSlaveRegister(MPU6050 *self, uint8_t num, uint8_t reg)
{
  if (num > 3)
    return;

  self->buffer[0] = MPU6050_RA_I2C_SLV0_REG + num * 3;
  self->buffer[1] = reg;
  self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
}

/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
void _setSlaveDataLength(MPU6050 *self, uint8_t num, uint8_t length)
{
  if (num > 3)
    return;

  self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_LEN_BIT,
		  MPU6050_I2C_SLV_LEN_LENGTH, length);
}

/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
void _setSlaveEnabled(MPU6050 *self, uint8_t num, bool enabled)
{
  if (num > 3)
    return;

  self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_SLV0_CTRL + num * 3, MPU6050_I2C_SLV_EN_BIT, enabled);

}

/** Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU6500_RA_I2C_MST_DELAY_CTRL
 * @see MPU6500_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
void _setSlaveDelayEnabled(MPU6050 *self, uint8_t num, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU6500_RA_INT_CFG
 * @see MPU6500_INTERRUPT_DATA_RDY_BIT
 */
void _setIntDataReadyEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}
