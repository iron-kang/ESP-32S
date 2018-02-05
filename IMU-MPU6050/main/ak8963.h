#ifndef _AK8963_H_
#define _AK8963_H_

#include "esp32_i2c.h"

#define AK8963_ADDRESS_00         0x0C
#define AK8963_ADDRESS_01         0x0D
#define AK8963_ADDRESS_10         0x0E // default for InvenSense MPU-6050 evaluation board
#define AK8963_ADDRESS_11         0x0F
#define AK8963_DEFAULT_ADDRESS    AK8963_ADDRESS_00

#define AK8963_RA_WIA             0x00
#define AK8963_RA_INFO            0x01
#define AK8963_RA_ST1             0x02
#define AK8963_RA_HXL             0x03
#define AK8963_RA_HXH             0x04
#define AK8963_RA_HYL             0x05
#define AK8963_RA_HYH             0x06
#define AK8963_RA_HZL             0x07
#define AK8963_RA_HZH             0x08
#define AK8963_RA_ST2             0x09
#define AK8963_RA_CNTL            0x0A
#define AK8963_RA_CNTL2           0x0B
#define AK8963_RA_ASTC            0x0C
#define AK8963_RA_TS1             0x0D // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_TS2             0x0E // SHIPMENT TEST, DO NOT USE
#define AK8963_RA_I2CDIS          0x0F
#define AK8963_RA_ASAX            0x10
#define AK8963_RA_ASAY            0x11
#define AK8963_RA_ASAZ            0x12

#define AK8963_ST1_DRDY_BIT       0

#define AK8963_ST2_HOFL_BIT       3
#define AK8963_ST2_DERR_BIT       2

#define AK8963_CNTL_MODE_BIT      3
#define AK8963_CNTL_MODE_LENGTH   4

#define AK8963_MODE_POWERDOWN     0x00
#define AK8963_MODE_SINGLE        0x01
#define AK8963_MODE_CONT1         0x02
#define AK8963_MODE_CONT2         0x06
#define AK8963_MODE_EXTTRIG       0x04
#define AK8963_MODE_SELFTEST      0x08
#define AK8963_MODE_FUSEROM       0x0F
#define AK8963_MODE_14BIT         0x00
#define AK8963_MODE_16BIT         0x10

#define AK8963_ASTC_SELF_BIT      6

#define AK8963_I2CDIS_BIT         0

#define AK8963_ST_X_MIN           (int16_t)(-200)
#define AK8963_ST_X_MAX           (int16_t)(200)
#define AK8963_ST_Y_MIN           (int16_t)(-200)
#define AK8963_ST_Y_MAX           (int16_t)(200)
#define AK8963_ST_Z_MIN           (int16_t)(-3200)
#define AK8963_ST_Z_MAX           (int16_t)(-800)

typedef struct ak8963
{
	I2C_CONFIG *i2c;
	uint8_t devAddr;
	uint8_t buffer[6];

	bool (*TestConnection)(struct ak8963 *self);
	bool (*SelfTest)(struct ak8963 *self);

	// WIA register
	uint8_t (*GetDeviceID)(struct ak8963 *self);
	// INFO register
	uint8_t (*GetInfo)(struct ak8963 *self);
	// ST1 register
	bool (*GetDataReady)(struct ak8963 *self);
	// H* registers
	void (*GetHeading)(struct ak8963 *self, int16_t *x, int16_t *y, int16_t *z);
	int16_t (*GetHeadingX)(struct ak8963 *self);
	int16_t (*GetHeadingY)(struct ak8963 *self);
	int16_t (*GetHeadingZ)(struct ak8963 *self);
	// ST2 register
	bool (*GetOverflowStatus)(struct ak8963 *self);
	bool (*GetDataError)();

	// CNTL register
	uint8_t (*GetMode)(struct ak8963 *self);
	void (*SetMode)(struct ak8963 *self, uint8_t mode);
	void (*Reset)(struct ak8963 *self);

	// ASTC register
	void (*SetSelfTest)(struct ak8963 *self, bool enabled);

	// I2CDIS
	void (*DisableI2C)(struct ak8963 *self); // um, why...?

	// ASA* registers
	void (*GetAdjustment)(struct ak8963 *self, int8_t *x, int8_t *y, int8_t *z);
	void (*SetAdjustment)(struct ak8963 *self, int8_t x, int8_t y, int8_t z);
	uint8_t (*GetAdjustmentX)(struct ak8963 *self);
	void (*SetAdjustmentX)(struct ak8963 *self, uint8_t x);
	uint8_t (*GetAdjustmentY)(struct ak8963 *self);
	void (*SetAdjustmentY)(struct ak8963 *self, uint8_t y);
	uint8_t (*GetAdjustmentZ)(struct ak8963 *self);
	void (*SetAdjustmentZ)(struct ak8963 *self, uint8_t z);

}AK8963;

void AK8963Init(AK8963 *mag, uint8_t addr);


#endif
