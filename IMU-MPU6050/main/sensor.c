#include <math.h>
#include "sensor.h"
#include "imu_types.h"
#include "ak8963.h"
#include "config.h"
#include "imu.h"
#include "led.h"
#include "kalman.h"
#include "utility.h"
#include "driver/uart.h"
#include "ms5611.h"

#define SENSORS_TASK_NAME           "SENSORS"
#define BARO_TASK_NAME              "BARO"
#define MAG_GAUSS_PER_LSB           666.7f

#define SENSORS_DEG_PER_LSB_CFG     MPU6050_DEG_PER_LSB_2000
#define SENSORS_G_PER_LSB_CFG       MPU6050_G_PER_LSB_16
#define SENSORS_MPU6050_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN        8
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
#define SENSORS_ACC_SCALE_SAMPLES   200

#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)
#define GYRO_VARIANCE_BASE          5000
#define GYRO_VARIANCE_THRESHOLD_X   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z   (GYRO_VARIANCE_BASE)
#define RAD_TO_DEG                  (180/M_PI)

#define BARO

typedef struct
{
  Axis3f     bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

xQueueHandle accelerometerDataQueue;
xQueueHandle gyroDataQueue;
xQueueHandle magnetometerDataQueue;
xQueueHandle barometerDataQueue;
xSemaphoreHandle sensorsDataReady;
bool gyroBiasFound = false;
bool isMpu6500TestPassed = false;
bool  isAK8963TestPassed = false;

IMU imu;
AK8963 mag;
MS5611 baro;
BiasObj gyroBiasRunning;
Axis3f  gyroBias;
sensorData_t sensors;
uint8_t buffer[BUF_LEN] = {0};
Kalman kalRoll, kalPitch;

float accScaleSum = 0;
float accScale = 1;
float cosPitch;
float sinPitch;
float cosRoll;
float sinRoll;
double referencePressure;

void processAccGyroMeasurements(const uint8_t *buffer);
void processMagnetometerMeasurements(const uint8_t *buffer);
void processBarometerMeasurements(const uint8_t *buffer);
void sensorsBiasObjInit(BiasObj* bias);
void SensorTask_Init();
void sensorsSetupSlaveRead(void);
bool sensorsTest();

void IRAM_ATTR imu_isr_handler(void* arg)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	portYIELD_FROM_ISR();
}

void Sensor_Init(Bus *bus, uint8_t *status)
{
	gpio_config_t io_conf;
	uint8_t try_cnt = 10;

	sensorsDataReady = xSemaphoreCreateBinary();
	io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	io_conf.pin_bit_mask = (1ULL<<PIN_IMU_INT);;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_IMU_INT, imu_isr_handler, (void*) PIN_IMU_INT);

	sensorsBiasObjInit(&gyroBiasRunning);
	imu.i2c = &bus->i2c;
	mag.i2c = &bus->i2c;

	IMU_Init(&imu, MPU6050_ADDRESS);

	while (!imu.testConnection(&imu))
	{
		try_cnt--;
		if (try_cnt == 0)
		{
			LED_ON(PIN_LED_YELLOW);
			*status |= (1 << STATUS_IMU);
			printf("IMU I2C connection [FAIL].\n");
			return;
		}
		printf("IMU retry\n");
		vTaskDelay(M2T(100));
	}

	imu.reset(&imu);
	vTaskDelay(M2T(100));
	imu.calibrate(&imu);
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
#if 0
	AK8963Init(&mag, AK8963_ADDRESS_00);
	if (mag.TestConnection(&mag) == true)
	{
		mag.SetMode(&mag, AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
		printf("AK8963 I2C connection [OK].\n");
	}
	else
	{
		LED_ON(PIN_LED_YELLOW);
		printf("AK8963 I2C connection [FAIL].\n");
	}
	mag.Calibrate(&mag);
#endif
	cosPitch = cosf(0 * (float) M_PI/180);
	sinPitch = sinf(0 * (float) M_PI/180);
	cosRoll = cosf(0 * (float) M_PI/180);
	sinRoll = sinf(0 * (float) M_PI/180);

	sensorsSetupSlaveRead();
	imu.readAllRaw(&imu, buffer, BUF_LEN);
	processAccGyroMeasurements(&(buffer[0]));

#ifdef BARO
	MS5611_Init(&baro, &bus->i2c, MS5611_HIGH_RES);
	referencePressure = baro.readPressure(&baro, false);
	uint32_t rawTemp = baro.readRawTemperature();
	uint32_t rawPressure = baro.readRawPressure();

	// Read true temperature & Pressure
	double realTemperature = baro.readTemperature(&baro, false);
	long realPressure = baro.readPressure(&baro, false);

	// Calculate altitude
	float absoluteAltitude = baro.getAltitude(realPressure, 101325);
	float relativeAltitude = baro.getAltitude(realPressure, referencePressure);
	printf("rawTemp = %d, realTemp: %f*C\n", rawTemp, realTemperature);
	printf("rawPressure = %d, readPressure = %ldPa\n", rawPressure, realPressure);
	printf("absoulteAltitude = %fm, relativeAltitude = %fm\n", absoluteAltitude, relativeAltitude);
#endif

#ifdef KALMAN
	float roll = atan2(sensors.acc.y, sensors.acc.z) * RAD_TO_DEG;
	float pitch = atan(-sensors.acc.x/sqrt(sensors.acc.y*sensors.acc.y+sensors.acc.z*sensors.acc.z)) * RAD_TO_DEG;
	Kalman_Init(&kalRoll);
	Kalman_Init(&kalPitch);
	kalRoll.setAngle(&kalRoll, roll);
	kalPitch.setAngle(&kalPitch, pitch);
#endif


	*status |= (!sensorsTest() << STATUS_IMU);
	printf("imu status: %d\n", *status);

	SensorTask_Init();
}

void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
	uint32_t i;
	int64_t sum[GYRO_NBR_OF_AXES] = {0};
	int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

//	printf("sumY: %lld, sumYSq: %lld\n", sum[1], sumSq[1]);
	varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;

//	printf("gyro mean: %f, %f, %f\n", meanOut->x, meanOut->y, meanOut->z);
}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
bool sensorsFindBiasValue(BiasObj* bias)
{
	static int32_t varianceSampleTime;
	bool foundBias = false;

	if (bias->isBufferFilled)
	{
		Axis3f variance;
		Axis3f mean;

		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

//		printf("gyro var(%f, %f, %f)\n", variance.x, variance.y, variance.z);
		if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
			variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
			variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
			(varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
		{
			varianceSampleTime = xTaskGetTickCount();
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundBias = true;
			bias->isBiasValueFound = true;
		}
	}

	return foundBias;
}

bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
		if (gyroBiasRunning.isBiasValueFound)
		{
			//soundSetEffect(SND_CALIB);
			//ledseqRun(SYS_LED, seq_calibrated);
		}
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

bool sensorsAreCalibrated()
{
	return gyroBiasFound;
}

/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
	Axis3f rx;
	Axis3f ry;

	// Rotate around x-axis
	rx.x = in->x;
	rx.y = in->y * cosRoll - in->z * sinRoll;
	rx.z = in->y * sinRoll + in->z * cosRoll;

	// Rotate around y-axis
	ry.x = rx.x * cosPitch - rx.z * sinPitch;
	ry.y = rx.y;
	ry.z = -rx.x * sinPitch + rx.z * cosPitch;

	out->x = ry.x;
	out->y = ry.y;
	out->z = ry.z;
}

void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (uint8_t i = 0; i < 3; i++) {
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}

void processAccGyroMeasurements(const uint8_t *buffer)
{
	Axis3f accScaled;

	int16_t ax = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ay = (((int16_t) buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gx = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gy = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

//	gx = abs(gx) > 8 ? gx : 0;
//	gy = abs(gy) > 8 ? gy : 0;
	gz = abs(gz) > 8 ? gz : 0;

	//printf("a(%6d, %6d, %6d), g(%3d, %3d, %3d)\n", ax, ay, az-2048, gx, gy, gz);

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias) | true;

	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);
	}

	sensors.gyro.x =  (gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensors.gyro);
//	printf("g bias (%f, %f, %f)\n", gyroBias.x, gyroBias.y, gyroBias.z);

	accScaled.x =  (ax) * SENSORS_G_PER_LSB_CFG / accScale;
	accScaled.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;
	accScaled.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;
	sensorsAccAlignToGravity(&accScaled, &sensors.acc);
	applyAxis3fLpf((lpf2pData*)(&accLpf), &sensors.acc);
//	printf("a(%f, %f, %f), g(%f, %f, %f)\n", sensors.acc.x, sensors.acc.y, sensors.acc.z,
//			sensors.gyro.x, sensors.gyro.y, sensors.gyro.z);
}

void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT))
	{
	    int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
	    int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
	    int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

//	    sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
//	    sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
//	    sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
	    sensors.mag.x = headingx*mag.mRes*mag.magCalibration[0] - mag.magBias[0];
	    sensors.mag.y = headingy*mag.mRes*mag.magCalibration[1] - mag.magBias[1];
	    sensors.mag.z = headingz*mag.mRes*mag.magCalibration[2] - mag.magBias[2];
	    sensors.mag.x *= mag.magScale[0];
	    sensors.mag.y *= mag.magScale[1];
	    sensors.mag.z *= mag.magScale[2];
//	    printf("mag(%f, %f, %f)\n", sensors.mag.x, sensors.mag.y, sensors.mag.z);
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

bool sensorsReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsTest()
{
	bool testStatus = true;

	// Try for 3 seconds so the quad has stabilized enough to pass the test
	for (int i = 0; i < 300; i++)
	{
		if(imu.selfTest(&imu) == true)
		{
			isMpu6500TestPassed = true;
			break;
		}
		else
		{
			vTaskDelay(M2T(10));
		}
	}
	testStatus &= isMpu6500TestPassed;

	printf("test status: %d\n", testStatus);
#ifdef AK8963
	if (testStatus)
	{
		isAK8963TestPassed = mag.SelfTest(&mag);
		testStatus = isAK8963TestPassed;
	}
#endif

	return testStatus;
}

void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)
{
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
//	sensorsReadMag(&sensors->mag);
//	sensorsReadBaro(&sensors->baro);
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

void sensorsKalman(sensorData_t *sensors, attitude_t *attitude, float dt)
{
	float roll = atan2(sensors->acc.y, sensors->acc.z) * RAD_TO_DEG;
	float pitch = atan(-sensors->acc.x*invSqrt(sensors->acc.y*sensors->acc.y+sensors->acc.z*sensors->acc.z)) * RAD_TO_DEG;

	if ((roll < -90 && kalRoll.angle > 90) || (roll > 90 && kalRoll.angle < -90))
	{
		printf("out range: %f\n", roll);
		kalRoll.setAngle(&kalRoll, roll);
	}
	else
		kalRoll.getAngle(&kalRoll, roll, sensors->gyro.x, dt);

	if (abs(kalRoll.angle) > 90)
	{
		kalPitch.getAngle(&kalPitch, pitch, -sensors->gyro.y, dt);
	}
	kalPitch.getAngle(&kalPitch, pitch, sensors->gyro.y, dt);

	attitude->roll = kalRoll.angle;
	attitude->pitch = kalPitch.angle;
}

void sensorsTask(void *param)
{

	while (true)
	{
		if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
		{
//			LED_Toggle(PIN_LED_YELLOW);
			imu.readAllRaw(&imu, buffer, BUF_LEN);
			processAccGyroMeasurements(&(buffer[0]));
			//processMagnetometerMeasurements(&(buffer[SENSORS_MPU6050_BUFF_LEN]));

			vTaskSuspendAll();
			xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
			xQueueOverwrite(gyroDataQueue, &sensors.gyro);
			//xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
			xTaskResumeAll();

		}


	}
}

void baroTask(void *param)
{
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while (true)
	{
		vTaskDelayUntil(&lastWakeTime, 20);

		long realPressure = baro.readPressure(&baro, false);
		// Calculate altitude
		float absoluteAltitude = baro.getAltitude(realPressure, 101325);
		float relativeAltitude = baro.getAltitude(realPressure, referencePressure);
//		printf("absoulteAltitude = %fm, relativeAltitude = %fm\n", absoluteAltitude, relativeAltitude);
	}
}

void SensorTask_Init()
{
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue          = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue  = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue     = xQueueCreate(1, sizeof(float));

	xTaskCreate(baroTask, BARO_TASK_NAME, 2048, NULL, BARO_TASK_PRI, NULL);
	xTaskCreatePinnedToCore(sensorsTask, SENSORS_TASK_NAME, 2048, NULL, SENSORS_TASK_PRI, NULL, 0);
}

