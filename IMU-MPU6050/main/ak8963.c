#include "ak8963.h"

bool _TestConnection(AK8963 *self);
bool _SelfTest(AK8963 *self);
uint8_t _GetDeviceID(AK8963 *self);
uint8_t _GetInfo(AK8963 *self);
bool _GetDataReady(AK8963 *self);
void _GetHeading(AK8963 *self, int16_t *x, int16_t *y, int16_t *z);
int16_t _GetHeadingX(AK8963 *self);
int16_t _GetHeadingY(AK8963 *self);
int16_t _GetHeadingZ(AK8963 *self);
bool _GetOverflowStatus(AK8963 *self);
bool _GetDataError();
uint8_t _GetMode(AK8963 *self);
void _SetMode(AK8963 *self, uint8_t mode);
void _Reset(AK8963 *self);
void _SetSelfTest(AK8963 *self, bool enabled);
void _DisableI2C(AK8963 *self); // um, why...?
void _GetAdjustment(AK8963 *self, int8_t *x, int8_t *y, int8_t *z);
void _SetAdjustment(AK8963 *self, int8_t x, int8_t y, int8_t z);
uint8_t _GetAdjustmentX(AK8963 *self);
void _SetAdjustmentX(AK8963 *self, uint8_t x);
uint8_t _GetAdjustmentY(AK8963 *self);
void _SetAdjustmentY(AK8963 *self, uint8_t y);
uint8_t _GetAdjustmentZ(AK8963 *self);
void _SetAdjustmentZ(AK8963 *self, uint8_t z);
void _readMagData(AK8963 *self, int16_t * destination);
void _Calibrate(AK8963 *self);

/** Evaluate the values from a HMC8335L self test.
 * @param min The min limit of the self test
 * @param max The max limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within min - max limit, false otherwise
 */
bool EvaluateSelfTest(int16_t min, int16_t max, int16_t value, char* string)
{
  if (value < min || value > max)
  {
    printf("Self test %s [FAIL]. low: %d, high: %d, measured: %d\n",
                string, min, max, value);
    return false;
  }
  return true;
}


void AK8963Init(AK8963 *mag, uint8_t addr)
{
	mag->devAddr           = addr;
	mag->TestConnection    = _TestConnection;
	mag->SelfTest          = _SelfTest;
	mag->GetDeviceID       = _GetDeviceID;
	mag->GetInfo           = _GetInfo;
	mag->GetDataReady      = _GetDataReady;
	mag->GetHeading        = _GetHeading;
	mag->GetHeadingX       = _GetHeadingX;
	mag->GetHeadingY       = _GetHeadingY;
	mag->GetHeadingZ       = _GetHeadingZ;
	mag->GetOverflowStatus = _GetOverflowStatus;
	mag->GetDataError      = _GetDataError;
	mag->GetMode           = _GetMode;
	mag->SetMode           = _SetMode;
	mag->Reset             = _Reset;
	mag->SetSelfTest       = _SetSelfTest;
	mag->DisableI2C        = _DisableI2C;
	mag->GetAdjustment     = _GetAdjustment;
	mag->SetAdjustment     = _SetAdjustment;
	mag->GetAdjustmentX    = _GetAdjustmentX;
	mag->SetAdjustmentX    = _SetAdjustmentX;
	mag->GetAdjustmentY    = _GetAdjustmentY;
	mag->SetAdjustmentY    = _SetAdjustmentY;
	mag->GetAdjustmentZ    = _GetAdjustmentZ;
	mag->SetAdjustmentZ    = _SetAdjustmentZ;
	mag->readMagData       = _readMagData;
	mag->Calibrate         = _Calibrate;

	mag->mRes = 10.*4912./32760.0;
	uint8_t rawData[3];
	mag->buffer[0] = AK8963_RA_CNTL;
	mag->buffer[1] =0x00;
	mag->i2c->write(mag->i2c, mag->devAddr, mag->buffer, 2);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	mag->buffer[0] = AK8963_RA_CNTL;
	mag->buffer[1] =0x0F;
	mag->i2c->write(mag->i2c, mag->devAddr, mag->buffer, 2);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	mag->i2c->read(mag->i2c, mag->devAddr, AK8963_RA_ASAX, &rawData[0], 3);
	mag->magCalibration[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	mag->magCalibration[1] =  (float)(rawData[1] - 128)/256. + 1.;
	mag->magCalibration[2] =  (float)(rawData[2] - 128)/256. + 1.;

	mag->buffer[0] = AK8963_RA_CNTL;
	mag->buffer[1] =0x00;
	mag->i2c->write(mag->i2c, mag->devAddr, mag->buffer, 2);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	mag->buffer[0] = AK8963_RA_CNTL;
	mag->buffer[1] = MFS_16BITS << 4 | 0x06;
	mag->i2c->write(mag->i2c, mag->devAddr, mag->buffer, 2);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool _TestConnection(AK8963 *self)
{
	uint8_t data;
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_WIA, &data, 1);
	printf("ak8963: who am i %x\n", data);

	return data == 0x48;
}

bool _SelfTest(AK8963 *self)
{
	bool testStatus = true;
	int16_t mx, my, mz;  // positive magnetometer measurements
	uint8_t confSave;
	uint8_t timeout = 20;

	// Save register values
	if (self->i2c->read(self->i2c, self->devAddr, AK8963_RA_CNTL, &confSave, 1) != ESP_OK)
	{
		// TODO: error handling
		return false;
	}

	// Power down
	self->SetMode(self, AK8963_MODE_POWERDOWN);
	self->SetSelfTest(self, true);
	self->SetMode(self, AK8963_MODE_16BIT | AK8963_MODE_SELFTEST);
	// Clear ST1 by reading ST2
	self->GetOverflowStatus(self);
	while (!self->GetDataReady(self) && timeout--)
	{
		vTaskDelay(M2T(1));
	}
	self->GetHeading(self, &mx, &my, &mz);
	printf("m(%d, %d, %d)\n", mx, my, mz);
	// Power down
	self->SetMode(self, AK8963_MODE_POWERDOWN);

	if (EvaluateSelfTest(AK8963_ST_X_MIN, AK8963_ST_X_MAX, mx, "X") &&
	    EvaluateSelfTest(AK8963_ST_Y_MIN, AK8963_ST_Y_MAX, my, "Y") &&
	    EvaluateSelfTest(AK8963_ST_Z_MIN, AK8963_ST_Z_MAX, mz, "Z"))
	{
		printf("Self test [OK].\n");
	}
	else
	{
		testStatus = false;
	}

	// Power up with saved config
	self->SetMode(self, confSave);

	return testStatus;
}

uint8_t _GetDeviceID(AK8963 *self)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_WIA, &self->buffer[0], 1);
	return self->buffer[0];
}

uint8_t _GetInfo(AK8963 *self)
{
  self->i2c->read(self->i2c, self->devAddr, AK8963_RA_INFO, &self->buffer[0], 1);
  return self->buffer[0];
}

bool _GetDataReady(AK8963 *self)
{
	self->i2c->readBits(self->i2c, self->devAddr, AK8963_RA_ST1, AK8963_ST1_DRDY_BIT, 1, &self->buffer[0]);
	return self->buffer[0];
}

void _GetHeading(AK8963 *self, int16_t *x, int16_t *y, int16_t *z)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_HXL, self->buffer, 6);
	*x = (((int16_t) self->buffer[1]) << 8) | self->buffer[0];
	*y = (((int16_t) self->buffer[3]) << 8) | self->buffer[2];
	*z = (((int16_t) self->buffer[5]) << 8) | self->buffer[4];
}

int16_t _GetHeadingX(AK8963 *self)
{
	self->buffer[0] = AK8963_RA_CNTL;
	self->buffer[1] = AK8963_MODE_SINGLE;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_HXL, self->buffer, 2);

	return (((int16_t) self->buffer[1]) << 8) | self->buffer[0];
}

int16_t _GetHeadingY(AK8963 *self)
{
	self->buffer[0] = AK8963_RA_CNTL;
	self->buffer[1] = AK8963_MODE_SINGLE;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_HYL, self->buffer, 2);

	return (((int16_t) self->buffer[1]) << 8) | self->buffer[0];
}

int16_t _GetHeadingZ(AK8963 *self)
{
	self->buffer[0] = AK8963_RA_CNTL;
	self->buffer[1] = AK8963_MODE_SINGLE;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_HZL, self->buffer, 2);

	return (((int16_t) self->buffer[1]) << 8) | self->buffer[0];
}

bool _GetOverflowStatus(AK8963 *self)
{
	self->i2c->readBits(self->i2c, self->devAddr, AK8963_RA_ST2, AK8963_ST2_HOFL_BIT, 1, &self->buffer[0]);
	return self->buffer[0];
}

bool _GetDataError(AK8963 *self)
{
	self->i2c->readBits(self->i2c, self->devAddr, AK8963_RA_ST2, AK8963_ST2_DERR_BIT, 1, &self->buffer[0]);
	return self->buffer[0];
}

uint8_t _GetMode(AK8963 *self)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_CNTL, &self->buffer[0], 1);
	return self->buffer[0];
}

void _SetMode(AK8963 *self, uint8_t mode)
{
	self->buffer[0] = AK8963_RA_CNTL;
	self->buffer[1] = mode;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
}

void _Reset(AK8963 *self)
{
	self->i2c->writeBits(self->i2c, self->devAddr, AK8963_RA_CNTL, AK8963_CNTL_MODE_BIT, AK8963_CNTL_MODE_LENGTH, AK8963_MODE_POWERDOWN);
}

void _SetSelfTest(AK8963 *self, bool enabled)
{
	self->i2c->writeBit(self->i2c, self->devAddr, AK8963_RA_ASTC, AK8963_ASTC_SELF_BIT, enabled);
}

void _DisableI2C(AK8963 *self)
{
	self->i2c->writeBit(self->i2c, self->devAddr, AK8963_RA_I2CDIS, AK8963_I2CDIS_BIT, true);
}

void _GetAdjustment(AK8963 *self, int8_t *x, int8_t *y, int8_t *z)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_ASAX, self->buffer, 3);
	*x = self->buffer[0];
	*y = self->buffer[1];
	*z = self->buffer[2];
}

void _SetAdjustment(AK8963 *self, int8_t x, int8_t y, int8_t z)
{
	self->buffer[0] = AK8963_RA_ASAX;
	self->buffer[1] = x;
	self->buffer[2] = y;
	self->buffer[3] = z;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 4);
}

uint8_t _GetAdjustmentX(AK8963 *self)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_ASAX, &self->buffer[0], 1);
	return self->buffer[0];
}

void _SetAdjustmentX(AK8963 *self, uint8_t x)
{
	self->buffer[0] = AK8963_RA_ASAX;
	self->buffer[1] = x;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
}

uint8_t _GetAdjustmentY(AK8963 *self)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_ASAY, &self->buffer[0], 1);
	return self->buffer[0];
}

void _SetAdjustmentY(AK8963 *self, uint8_t y)
{
	self->buffer[0] = AK8963_RA_ASAY;
	self->buffer[1] = y;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
}

uint8_t _GetAdjustmentZ(AK8963 *self)
{
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_ASAZ, &self->buffer[0], 1);
	return self->buffer[0];
}

void _SetAdjustmentZ(AK8963 *self, uint8_t z)
{
	self->buffer[0] = AK8963_RA_ASAZ;
	self->buffer[1] = z;
	self->i2c->write(self->i2c, self->devAddr, self->buffer, 2);
}

void _Calibrate(AK8963 *self)
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};


	sample_count = 1500;
	for(ii = 0; ii < sample_count; ii++) {
		self->readMagData(self, mag_temp);
		for (int jj = 0; jj < 3; jj++) {
			if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		vTaskDelay(12 / portTICK_PERIOD_MS);
	}

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	self->magBias[0] = (float) mag_bias[0]*self->mRes*self->magCalibration[0];  // save mag biases in G for main program
	self->magBias[1] = (float) mag_bias[1]*self->mRes*self->magCalibration[1];
	self->magBias[2] = (float) mag_bias[2]*self->mRes*self->magCalibration[2];

	 // Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	self->magScale[0] = avg_rad/((float)mag_scale[0]);
	self->magScale[1] = avg_rad/((float)mag_scale[1]);
	self->magScale[2] = avg_rad/((float)mag_scale[2]);
}

void _readMagData(AK8963 *self, int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	self->i2c->read(self->i2c, self->devAddr, AK8963_RA_ST1, &self->buffer[0], 1);
	if (self->buffer[0] & 0x01)
	{
		self->i2c->read(self->i2c, self->devAddr, AK8963_RA_HXL, &rawData[0], 7);
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
	   }
	}
}
