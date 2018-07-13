#include "stabilizer.h"
#include "sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "estimator.h"
#include "led.h"
#include "controller.h"
#include "ultrasonic.h"
#include "gps.h"

#define STABILIZER_TASK_NAME    "STABILIZER"

xQueueHandle infoQueue;
sensorData_t sensorData;
setpoint_t setpoint;
control_t control;
state_t state;
attitude_t attitude;

void stabilizerTask(void* param);

void Stabilizer(uint8_t *status)
{
	Controller_Init();
//	Ultrasonic_Init();
	//GPS_Init(status);
	printf("gps status: %d\n", *status);
	infoQueue = xQueueCreate(1, sizeof(Info));

//	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, 8192, NULL, STABILIZER_TASK_PRI, NULL);
	xTaskCreatePinnedToCore(stabilizerTask, STABILIZER_TASK_NAME, 8192, NULL, STABILIZER_TASK_PRI, NULL, 1);
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= sensorsTest();
//	pass &= stateEstimatorTest();
//	pass &= stateControllerTest();
//  	pass &= powerDistributionTest();

  	return pass;
}

void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime;
	Info info;
	unsigned int height;
//	vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while (!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, 1);
	}
	printf("start stabilizer\n");
	while (true)
	{
		vTaskDelayUntil(&lastWakeTime, 1);
//		LED_Toggle(PIN_LED_YELLOW);
		sensorsAcquire(&sensorData, tick);
		stateEstimator(&state, &sensorData, tick);
//		sensorsKalman(&sensorData, &attitude, 0.001);
#if 1
		info.attitude.x = state.attitude.roll;
		info.attitude.y = state.attitude.pitch;
		info.attitude.z = state.attitude.yaw;
		//GPS_GetInfo(&info.gps);
//		if (RATE_DO_EXECUTE(100, tick))
//		{
//		    if (Ultrasonic_GetDistance(&height) == pdTRUE)
//		        info.height = height;
//		}
		xQueueOverwrite(infoQueue, &info);

		Controller_PID(&state, &sensorData, tick);
#endif
		tick++;
	}
}

bool stabilizer_GetState(Info *info)
{
	return (pdTRUE == xQueueReceive(infoQueue, info, 0));
}

void stabilizer_GetState_Android(Info_Android *info)
{
	Info tmp;
	if (pdTRUE == xQueueReceive(infoQueue, &tmp, 0))
	{
		info->status    = tmp.status;
		info->latitude  = tmp.gps.latitude;
		info->longitude = tmp.gps.longitude;
		info->altitude  = tmp.gps.altitude;
	}
}

attitude_t *stabilizer_GetAttitude()
{
	return &attitude;
}
