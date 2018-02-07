#include "stabilizer.h"
#include "stabilizer_types.h"
#include "sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "estimator.h"

#define STABILIZER_TASK_NAME    "STABILIZER"

sensorData_t sensorData;
setpoint_t setpoint;
control_t control;
state_t state;

void stabilizerTask(void* param);

void Stabilizer_Init()
{
	//xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, 2048, NULL, STABILIZER_TASK_PRI, NULL);
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
	//vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);


	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while (!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

//		getExtPosition(&state);

		sensorsAcquire(&sensorData, tick);
		stateEstimator(&state, &sensorData, tick);

//		commanderGetSetpoint(&setpoint, &state);
//
//		sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
//
//		stateController(&control, &setpoint, &sensorData, &state, tick);
//		powerDistribution(&control);

		tick++;
	}
}
