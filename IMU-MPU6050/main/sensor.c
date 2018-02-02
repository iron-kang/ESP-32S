#include "sensor.h"
#include "imu_types.h"
#include "stabilizer_types.h"
#include "mpu6050.h"
#include "config.h"
#include "freertos/FreeRTOS.h"

#define SENSORS_TASK_NAME       "SENSORS"

xQueueHandle accelerometerDataQueue;
xQueueHandle gyroDataQueue;
xQueueHandle magnetometerDataQueue;
xQueueHandle barometerDataQueue;

MPU6050 mpu;
sensorData_t sensors;
uint8_t buffer[BUF_LEN] = {0};

void processAccGyroMeasurements(const uint8_t *buffer);
void processMagnetometerMeasurements(const uint8_t *buffer);
void processBarometerMeasurements(const uint8_t *buffer);

void Sensor_Init(Bus *bus)
{
	mpu.i2c = &bus->i2c;
	MPU6050_Init(&mpu);

	if (mpu.testConnection(&mpu) == true)
	{
		printf("MPU9250 I2C connection [OK].\n");
	}
	else
	{
		printf("MPU9250 I2C connection [FAIL].\n");
		return;
	}

	mpu.reset(&mpu);
	vTaskDelay(M2T(50));
	mpu.setSleepEnabled(&mpu, false);
	vTaskDelay(M2T(100));
	// Set x-axis gyro as clock source
	mpu.setClockSource(&mpu, MPU6050_CLOCK_PLL_XGYRO);
	// Delay until clock is set and stable
	vTaskDelay(M2T(200));
	// Enable temp sensor
	mpu.setTempSensorEnabled(&mpu, true);
	// Disable interrupts
	mpu.setIntEnabled(&mpu, false);
	// Connect the MAG and BARO to the main I2C bus
	mpu.setI2CBypassEnabled(&mpu, false);
	// Set gyro full scale range
	mpu.setFullScaleGyroRange(&mpu, MPU6050_GYRO_FS_2000);
	// Set accelerometer full scale range
	mpu.setFullScaleAccelRange(&mpu, MPU6050_ACCEL_FS_16);
	// Set accelerometer digital low-pass bandwidth
	mpu.setAccelDLPF(&mpu, MPU6050_ACCEL_DLPF_BW_41);

	mpu.setRate(&mpu, 0);
	mpu.setDLPFMode(&mpu, MPU6050_DLPF_BW_98);

	// Init second order filer for accelerometer
	for (uint8_t i = 0; i < 3; i++)
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}
}

void sensorsTask(void *param)
{
	while (true)
	{
		mpu.readAllRaw(&mpu, buffer);
	}
}

void SensorTask_Init()
{
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

	xTaskCreate(sensorsTask, SENSORS_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, SENSORS_TASK_PRI, NULL);
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
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{

}

void processBarometerMeasurements(const uint8_t *buffer)
{
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
}

