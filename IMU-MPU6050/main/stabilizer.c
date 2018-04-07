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
	Ultrasonic_Init();
	GPS_Init(status);
	printf("gps status: %d\n", *status);
	infoQueue = xQueueCreate(1, sizeof(Info));

	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, 8192, NULL, STABILIZER_TASK_PRI, NULL);
//	xTaskCreatePinnedToCore(stabilizerTask, STABILIZER_TASK_NAME, 2048, NULL, STABILIZER_TASK_PRI, NULL, 1);
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
/*
 * setpoint->mode.z = modeDisable;
 * setpoint->mode.x = modeDisable;
 * setpoint->mode.y = modeDisable;
 * setpoint->attitudeRate.roll = 0;
 * setpoint->attitudeRate.pitch = 0;
 * setpoint->attitudeRate.yaw  = commanderGetActiveYaw();
 * setpoint->attitude.pitch = commanderGetActivePitch();
 * setpoint->attitude.roll = commanderGetActiveRoll();
 * setpoint->thrust = commanderGetActiveThrust();
 * attitudeDesired.yaw -= setpoint->attitudeRate.yaw/500.0;
 * attitudeDesired.roll = setpoint->attitude.roll;
 * attitudeDesired.pitch = setpoint->attitude.pitch;
 */
void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime;
	Info info;
//	vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while (!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, 1);
	}

	while (true)
	{
		vTaskDelayUntil(&lastWakeTime, 1);

//		getExtPosition(&state);

//		LED_Toggle(PIN_LED_YELLOW);
		sensorsAcquire(&sensorData, tick);
		stateEstimator(&state, &sensorData, tick);
//		sensorsKalman(&sensorData, &attitude, 0.001);
		info.attitude.x = state.attitude.roll;
		info.attitude.y = state.attitude.pitch;
		info.attitude.z = state.attitude.yaw;
		GPS_GetInfo(&info.gps);
		Ultrasonic_GetDistance(&info.height);
		xQueueOverwrite(infoQueue, &info);
		attitude_t target;
		target.pitch = 0;
		target.roll = 0;
		target.yaw = 0;
		Controller_PID(&state, &sensorData, target, tick);

		tick++;
	}
}

bool stablizer_GetState(Info *info)
{
	return (pdTRUE == xQueueReceive(infoQueue, info, 0));
}

attitude_t *stablizer_GetAttitude()
{
	return &attitude;
}
