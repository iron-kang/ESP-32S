#include "mpu6050.h"

const unsigned short mpu6050StTb[256] = {
  2620,2646,2672,2699,2726,2753,2781,2808, //7
  2837,2865,2894,2923,2952,2981,3011,3041, //15
  3072,3102,3133,3165,3196,3228,3261,3293, //23
  3326,3359,3393,3427,3461,3496,3531,3566, //31
  3602,3638,3674,3711,3748,3786,3823,3862, //39
  3900,3939,3979,4019,4059,4099,4140,4182, //47
  4224,4266,4308,4352,4395,4439,4483,4528, //55
  4574,4619,4665,4712,4759,4807,4855,4903, //63
  4953,5002,5052,5103,5154,5205,5257,5310, //71
  5363,5417,5471,5525,5581,5636,5693,5750, //79
  5807,5865,5924,5983,6043,6104,6165,6226, //87
  6289,6351,6415,6479,6544,6609,6675,6742, //95
  6810,6878,6946,7016,7086,7157,7229,7301, //103
  7374,7448,7522,7597,7673,7750,7828,7906, //111
  7985,8065,8145,8227,8309,8392,8476,8561, //119
  8647,8733,8820,8909,8998,9088,9178,9270,
  9363,9457,9551,9647,9743,9841,9939,10038,
  10139,10240,10343,10446,10550,10656,10763,10870,
  10979,11089,11200,11312,11425,11539,11654,11771,
  11889,12008,12128,12249,12371,12495,12620,12746,
  12874,13002,13132,13264,13396,13530,13666,13802,
  13940,14080,14221,14363,14506,14652,14798,14946,
  15096,15247,15399,15553,15709,15866,16024,16184,
  16346,16510,16675,16842,17010,17180,17352,17526,
  17701,17878,18057,18237,18420,18604,18790,18978,
  19167,19359,19553,19748,19946,20145,20347,20550,
  20756,20963,21173,21385,21598,21814,22033,22253,
  22475,22700,22927,23156,23388,23622,23858,24097,
  24338,24581,24827,25075,25326,25579,25835,26093,
  26354,26618,26884,27153,27424,27699,27976,28255,
  28538,28823,29112,29403,29697,29994,30294,30597,
  30903,31212,31524,31839,32157,32479,32804,33132
};

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
bool _selfTest(MPU6050 *self);
bool _evaluateSelfTest(MPU6050 *self, float low, float high, float value, char* string);
void _calibrate(MPU6050 *self);

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
    mpu->selfTest                 = _selfTest;
    mpu->evaluateSelfTest         = _evaluateSelfTest;
    mpu->calibrate                = _calibrate;
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
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
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
 * @see MPU6050_RA_I2C_SLV4_CTRL
 */
void _setSlave4MasterDelay(MPU6050 *self, uint8_t delay)
{
	self->i2c->writeBits(self->i2c, self->devAddr, MPU6050_RA_I2C_SLV4_CTRL, MPU6050_I2C_SLV4_MST_DLY_BIT,
			MPU6050_I2C_SLV4_MST_DLY_LENGTH, delay);
}

/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void _setWaitForExternalSensorEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}

/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_LEVEL_BIT
 */
void _setInterruptMode(MPU6050 *self, bool mode)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_LEVEL_BIT, mode);
}

/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_OPEN_BIT
 */
void _setInterruptDrive(MPU6050 *self, bool drive)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_OPEN_BIT, drive);
}

/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_LATCH_INT_EN_BIT
 */
void _setInterruptLatch(MPU6050 *self, bool latch)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_LATCH_INT_EN_BIT, latch);
}

/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_INT_RD_CLEAR_BIT
 */
void _setInterruptLatchClear(MPU6050 *self, bool clear)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_INT_RD_CLEAR_BIT, clear);
}

/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void _setSlaveReadWriteTransitionEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_P_NSR_BIT, enabled);
}

/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU6050_RA_I2C_MST_CTRL
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
 * @see MPU6050_RA_I2C_SLV0_REG
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
 * @see MPU6050_RA_I2C_SLV0_CTRL
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
 * @see MPU6050_RA_I2C_SLV0_CTRL
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
 * @see MPU6050_RA_I2C_MST_DELAY_CTRL
 * @see MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
void _setSlaveDelayEnabled(MPU6050 *self, uint8_t num, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_I2C_MST_DELAY_CTRL, num, enabled);
}

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU6050_RA_INT_CFG
 * @see MPU6050_INTERRUPT_DATA_RDY_BIT
 */
void _setIntDataReadyEnabled(MPU6050 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, enabled);
}

/** Evaluate the values from a MPU6500 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
bool _evaluateSelfTest(MPU6050 *self, float low, float high, float value, char* string)
{
	if (value < low || value > high)
	{
		printf("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                string, low, high, value);
		return false;
	}
	return true;
}

/** Do a MPU6050 self test.
 * @return True if self test passed, false otherwise
 */
bool _selfTest(MPU6050 *self)
{
	uint8_t saveReg[5];
	uint8_t selfTest[6];
	int32_t gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
	int32_t factoryTrim[6];
	float aDiff[3], gDiff[3];
	uint8_t FS = 0;
	int i;

	// Save old configuration
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_SMPLRT_DIV, &saveReg[0], 1);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_CONFIG, &saveReg[1], 1);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_GYRO_CONFIG, &saveReg[2], 1);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ACCEL_CONFIG_2, &saveReg[3], 1);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ACCEL_CONFIG, &saveReg[4], 1);
	// Write test configuration
	self->buffer[0] = MPU6050_RA_SMPLRT_DIV ;
	self->buffer[1] = 0x00;// Set gyro sample rate to 1 kHz
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_CONFIG ;
	self->buffer[1] = 0x02; // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_GYRO_CONFIG ;
	self->buffer[1] = 1<<FS;// Set full scale range for the gyro to 250 dps
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG_2 ;
	self->buffer[1] = 2;// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG ;
	self->buffer[1] = 1<<FS;// Set full scale range for the accelerometer to 2 g
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);

	for(i = 0; i < 200; i++)
	{
		// get average current values of gyro and acclerometer
		self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ACCEL_XOUT_H, self->buffer, 6);// Read the six raw data registers into data array
		aAvg[0] += (int16_t)(((int16_t)self->buffer[0] << 8) | self->buffer[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)self->buffer[2] << 8) | self->buffer[3]) ;
		aAvg[2] += (int16_t)(((int16_t)self->buffer[4] << 8) | self->buffer[5]) ;

		self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_GYRO_XOUT_H, self->buffer, 6);// Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t)((int16_t)self->buffer[0] << 8) | self->buffer[1]; // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)((int16_t)self->buffer[2] << 8) | self->buffer[3];
		gAvg[2] += (int16_t)((int16_t)self->buffer[4] << 8) | self->buffer[5];
	}

	for (i = 0; i < 3; i++)
	{ // Get average of 200 values and store as average current readings
		aAvg[i] /= 200;
		gAvg[i] /= 200;
	}

	// Configure the accelerometer for self-test
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG ;
	self->buffer[1] = 0xE0;// Enable self test on all three axes and set accelerometer range to +/- 2 g
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_GYRO_CONFIG ;
	self->buffer[1] = 0xE0;// Enable self test on all three axes and set accelerometer range to +/- 2 g
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	vTaskDelay(M2T(25)); // Delay a while to let the device stabilize

	for(i = 0; i < 200; i++)
	{
		// get average self-test values of gyro and acclerometer
		self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ACCEL_XOUT_H, self->buffer, 6);// Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)self->buffer[0] << 8) | self->buffer[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)self->buffer[2] << 8) | self->buffer[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)self->buffer[4] << 8) | self->buffer[5]) ;

		self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_GYRO_XOUT_H, self->buffer, 6);// Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)self->buffer[0] << 8) | self->buffer[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)self->buffer[2] << 8) | self->buffer[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)self->buffer[4] << 8) | self->buffer[5]) ;
	}

	for (i =0; i < 3; i++)
	{ // Get average of 200 values and store as average self-test readings
		aSTAvg[i] /= 200;
		gSTAvg[i] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG ;
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_GYRO_CONFIG ;
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	vTaskDelay(M2T(25)); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_SELF_TEST_X, &selfTest[0], 1);// X-axis accel self-test results
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_SELF_TEST_Y, &selfTest[1], 1);// Y-axis accel self-test results
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_SELF_TEST_Z, &selfTest[2], 1);// z-axis accel self-test results
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_XG_OFFS_TC, &selfTest[3], 1);// X-axis gyro self-test results
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_YG_OFFS_TC, &selfTest[4], 1);// y-axis gyro self-test results
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ZG_OFFS_TC, &selfTest[5], 1);// z-axis gyro self-test results

	for (i = 0; i < 6; i++)
	{
		if (selfTest[i] != 0)
		{
			factoryTrim[i] = mpu6050StTb[selfTest[i] - 1];
		}
		else
		{
			factoryTrim[i] = 0;
		}
	}

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (i = 0; i < 3; i++)
	{
		aDiff[i] = 100.0f*((float)((aSTAvg[i] - aAvg[i]) - factoryTrim[i]))/factoryTrim[i]; // Report percent differences
		gDiff[i] = 100.0f*((float)((gSTAvg[i] - gAvg[i]) - factoryTrim[i+3]))/factoryTrim[i+3]; // Report percent differences
//   DEBUG_PRINT("a[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, aAvg[i], aSTAvg[i], aSTAvg[i] - aAvg[i], factoryTrim[i], aDiff[i]);
//   DEBUG_PRINT("g[%d] Avg:%d, StAvg:%d, Shift:%d, FT:%d, Diff:%0.2f\n", i, gAvg[i], gSTAvg[i], gSTAvg[i] - gAvg[i], factoryTrim[i+3], gDiff[i]);
	}

	// Restore old configuration
	self->buffer[0] = MPU6050_RA_SMPLRT_DIV ;
	self->buffer[1] = saveReg[0];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_CONFIG ;
	self->buffer[1] = saveReg[1];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_GYRO_CONFIG ;
	self->buffer[1] = saveReg[2];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG_2 ;
	self->buffer[1] = saveReg[3];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG ;
	self->buffer[1] = saveReg[4];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);

	// Check result
	if (self->evaluateSelfTest(self, MPU6050_ST_GYRO_LOW, MPU6050_ST_GYRO_HIGH, gDiff[0], "gyro X") &&
		self->evaluateSelfTest(self, MPU6050_ST_GYRO_LOW, MPU6050_ST_GYRO_HIGH, gDiff[1], "gyro Y") &&
		self->evaluateSelfTest(self, MPU6050_ST_GYRO_LOW, MPU6050_ST_GYRO_HIGH, gDiff[2], "gyro Z") &&
		self->evaluateSelfTest(self, MPU6050_ST_ACCEL_LOW, MPU6050_ST_ACCEL_HIGH, aDiff[0], "acc X") &&
		self->evaluateSelfTest(self, MPU6050_ST_ACCEL_LOW, MPU6050_ST_ACCEL_HIGH, aDiff[1], "acc Y") &&
		self->evaluateSelfTest(self, MPU6050_ST_ACCEL_LOW, MPU6050_ST_ACCEL_HIGH, aDiff[2], "acc Z"))
	{
		printf("MPU6050 self test [OK].\n");
		return true;
	}
	else
	{
		return false;
	}
}

void _calibrate(MPU6050 *self)
{
	uint8_t data[12];
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device
	self->buffer[0] = MPU6050_RA_PWR_MGMT_1 ;
	self->buffer[1] = 0x80;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	vTaskDelay(100 / portTICK_PERIOD_MS);

	// Configure device for bias calculation
	self->buffer[0] = MPU6050_RA_INT_ENABLE ;	// Disable all interrupts
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_FIFO_EN ;		// Disable FIFO
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_PWR_MGMT_1 ;	// Turn on internal clock source
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_I2C_MST_CTRL ;	// Disable I2C master
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_USER_CTRL ;	// Disable FIFO and I2C master modes
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_USER_CTRL ;	// Reset FIFO and DMP
	self->buffer[1] = 0x0C;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	vTaskDelay(15 / portTICK_PERIOD_MS);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	self->buffer[0] = MPU6050_RA_CONFIG ;		// Set low-pass filter to 188 Hz
	self->buffer[1] = 0x01;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_SMPLRT_DIV ;	// Set sample rate to 1 kHz
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_GYRO_CONFIG ;	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	self->buffer[1] = 0x00;
//	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->setFullScaleGyroRange(self, MPU6050_GYRO_FS_2000);
	self->buffer[0] = MPU6050_RA_ACCEL_CONFIG ;	// Set accelerometer full-scale to 2 g, maximum sensitivity
	self->buffer[1] = 0x00;
//	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->setFullScaleAccelRange(self, MPU6050_G_PER_LSB_2);

	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	self->buffer[0] = MPU6050_RA_USER_CTRL ;	// Enable FIFO
	self->buffer[1] = 0x40;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_FIFO_EN ;		// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	self->buffer[1] = 0x78;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	vTaskDelay(40 / portTICK_PERIOD_MS);// accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	self->buffer[0] = MPU6050_RA_FIFO_EN ;		// Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	self->buffer[1] = 0x00;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_FIFO_COUNTH, &data[0], 2);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	printf("packet cnt: %d\n", packet_count);

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_FIFO_R_W, &data[0], 12);
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		printf("a: %d, %d, %d, g: %d, %d, %d\n",
				accel_temp[0], accel_temp[1], accel_temp[2], gyro_temp[0], gyro_temp[1], gyro_temp[2]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	printf("bias a: %d, %d, %d, g: %d, %d, %d\n",
			accel_bias[0], accel_bias[1], accel_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	printf("bias avg a: %d, %d, %d, g: %d, %d, %d\n",
				accel_bias[0], accel_bias[1], accel_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	gyro_bias[0] = -114;
	gyro_bias[1] = -90;
	gyro_bias[2] = 0;

	accel_bias[0] = -256;
	accel_bias[1] = 131;

	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}

	accel_bias[2] = -49;
	printf("bias avg 2 a: %d, %d, %d, g: %d, %d, %d\n",
					accel_bias[0], accel_bias[1], accel_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	self->buffer[0] = MPU6050_RA_XG_OFFS_USRH ;
	self->buffer[1] = data[0];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_XG_OFFS_USRL ;
	self->buffer[1] = data[1];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_YG_OFFS_USRH ;
	self->buffer[1] = data[2];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_YG_OFFS_USRL ;
	self->buffer[1] = data[3];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
#if 0
	self->buffer[0] = MPU6050_RA_ZG_OFFS_USRH ;
	self->buffer[1] = data[4];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_ZG_OFFS_USRL ;
	self->buffer[1] = data[5];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
#endif

//	printf("gyro bias: %d, %d, %d\n", (data[0]<<8)+data[1], (data[2]<<8)+data[3], (data[4]<<8)+data[5]);

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_XA_OFFSET_H, &data[0], 2);
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_YA_OFFSET_H, &data[0], 2);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	self->i2c->read(self->i2c, self->devAddr, MPU6050_RA_ZA_OFFSET_H, &data[0], 2);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	printf("acc bias reg: %d, %d, %d\n", accel_bias_reg[0], accel_bias_reg[1], accel_bias_reg[2]);

	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
	printf("save acc bias reg: %d, %d, %d\n", accel_bias_reg[0], accel_bias_reg[1], accel_bias_reg[2]);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers


	self->buffer[0] = MPU6050_RA_XA_OFFSET_H ;
	self->buffer[1] = data[0];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_XA_OFFSET_L ;
	self->buffer[1] = data[1];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_YA_OFFSET_H ;
	self->buffer[1] = data[2];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_YA_OFFSET_L ;
	self->buffer[1] = data[3];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
#if 1
	self->buffer[0] = MPU6050_RA_ZA_OFFSET_H ;
	self->buffer[1] = data[4];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->buffer[0] = MPU6050_RA_ZA_OFFSET_L ;
	self->buffer[1] = data[5];
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
#endif
}

