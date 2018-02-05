#include "sensor.h"
#include "imu_types.h"
#include "stabilizer_types.h"
#include "mpu6050.h"
#include "ak8963.h"
#include "config.h"
#include "freertos/FreeRTOS.h"

#define SENSORS_TASK_NAME     "SENSORS"
#define MAG_GAUSS_PER_LSB     666.7f

#define SENSORS_MPU6050_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN        8

xQueueHandle accelerometerDataQueue;
xQueueHandle gyroDataQueue;
xQueueHandle magnetometerDataQueue;
xQueueHandle barometerDataQueue;

MPU6050 imu;
AK8963 mag;
sensorData_t sensors;
uint8_t buffer[BUF_LEN] = {0};

void processAccGyroMeasurements(const uint8_t *buffer);
void processMagnetometerMeasurements(const uint8_t *buffer);
void processBarometerMeasurements(const uint8_t *buffer);
void SensorTask_Init();

void Sensor_Init(Bus *bus)
{
	imu.i2c = &bus->i2c;
	mag.i2c = &bus->i2c;

	MPU6050_Init(&imu, MPU6050_ADDRESS);

	if (imu.testConnection(&imu) == true)
	{
		printf("MPU9250 I2C connection [OK].\n");
	}
	else
	{
		printf("MPU9250 I2C connection [FAIL].\n");
		return;
	}

	imu.reset(&imu);
	vTaskDelay(M2T(50));
	imu.setSleepEnabled(&imu, false);
	vTaskDelay(M2T(100));
	// Set x-axis gyro as clock source
	imu.setClockSource(&imu, MPU6050_CLOCK_PLL_XGYRO);
	// Delay until clock is set and stable
	vTaskDelay(M2T(200));
	// Enable temp sensor
	imu.setTempSensorEnabled(&imu, true);
	// Disable interrupts
	imu.setIntEnabled(&imu, false);
	// Connect the MAG and BARO to the main I2C bus
	imu.setI2CBypassEnabled(&imu, true);
	// Set gyro full scale range
	imu.setFullScaleGyroRange(&imu, MPU6050_GYRO_FS_2000);
	// Set accelerometer full scale range
	imu.setFullScaleAccelRange(&imu, MPU6050_ACCEL_FS_16);
	// Set accelerometer digital low-pass bandwidth
	imu.setAccelDLPF(&imu, MPU6050_ACCEL_DLPF_BW_41);

	imu.setRate(&imu, 0);
	imu.setDLPFMode(&imu, MPU6050_DLPF_BW_98);

	// Init second order filer for accelerometer
	for (uint8_t i = 0; i < 3; i++)
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}

	AK8963Init(&mag, AK8963_ADDRESS_00);
	if (mag.TestConnection(&mag) == true)
	{
		mag.SetMode(&mag, AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
		printf("AK8963 I2C connection [OK].\n");
	}
	else
	{
		printf("AK8963 I2C connection [FAIL].\n");
	}

	SensorTask_Init();
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
	Axis3f accScaled;

	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = (((int16_t) buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	printf("a(%d, %d, %d), g(%d, %d, %d)\n", ax, ay, az, gx, gy, gz);
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT))
	{
	    int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
	    int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
	    int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

	    sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
	    sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
	    sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
	    printf("mag(%f, %f, %f)\n", sensors.mag.x, sensors.mag.y, sensors.mag.z);
	}
}

void processBarometerMeasurements(const uint8_t *buffer)
{
#if 0
  static uint32_t rawPressure = 0;
  static int16_t rawTemp = 0;

  // Check if there is a new pressure update
  if (buffer[0] & 0x02) {
    rawPressure = ((uint32_t) buffer[3] << 16) | ((uint32_t) buffer[2] << 8) | buffer[1];
  }
  // Check if there is a new temp update
  if (buffer[0] & 0x01) {
    rawTemp = ((int16_t) buffer[5] << 8) | buffer[4];
  }

  sensors.baro.pressure = (float) rawPressure / LPS25H_LSB_PER_MBAR;
  sensors.baro.temperature = LPS25H_TEMP_OFFSET + ((float) rawTemp / LPS25H_LSB_PER_CELSIUS);
  sensors.baro.asl = lps25hPressureToAltitude(&sensors.baro.pressure);
#endif
}

void sensorsSetupSlaveRead(void)
{
	imu.setSlave4MasterDelay(&imu, 9);
	imu.setI2CBypassEnabled(&imu, false);
	imu.setWaitForExternalSensorEnabled(&imu, true);
	imu.setInterruptMode(&imu, 0);
	imu.setInterruptDrive(&imu, 0);
	imu.setInterruptLatch(&imu, 0);
	imu.setInterruptLatchClear(&imu, 1);
	imu.setSlaveReadWriteTransitionEnabled(&imu, false);
	imu.setMasterClockSpeed(&imu, 13);

	imu.setSlaveAddress(&imu, 0, 0x80 | AK8963_ADDRESS_00);
	imu.setSlaveRegister(&imu, 0, AK8963_RA_ST1);
	imu.setSlaveDataLength(&imu, 0, SENSORS_MAG_BUFF_LEN);
	imu.setSlaveDelayEnabled(&imu, 0, true);
	imu.setSlaveEnabled(&imu, 0, true);

	imu.setI2CMasterModeEnabled(&imu, true);
	imu.setIntDataReadyEnabled(&imu, true);
}

void sensorsTask(void *param)
{
	sensorsSetupSlaveRead();

	while (true)
	{
		imu.readAllRaw(&imu, buffer, BUF_LEN);
		processAccGyroMeasurements(&(buffer[0]));
		processMagnetometerMeasurements(&(buffer[SENSORS_MPU6050_BUFF_LEN]));
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void SensorTask_Init()
{
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

	xTaskCreate(sensorsTask, SENSORS_TASK_NAME, 2048, NULL, SENSORS_TASK_PRI, NULL);
}

