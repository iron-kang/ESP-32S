#include "stabilizer.h"
#include "stabilizer_types.h"

#define STABILIZER_TASK_NAME    "STABILIZER"

void stabilizerTask(void* param);

void Stabilizer_Init()
{
	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME, 2048, NULL, STABILIZER_TASK_PRI, NULL);
}

void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime;
	//vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);


	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while(!sensorsAreCalibrated())
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}

	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));

		getExtPosition(&state);

		sensorsAcquire(&sensorData, tick);
		stateEstimator(&state, &sensorData, tick);

		commanderGetSetpoint(&setpoint, &state);

		sitAwUpdateSetpoint(&setpoint, &sensorData, &state);

		stateController(&control, &setpoint, &sensorData, &state, tick);
		powerDistribution(&control);

		tick++;
	}
}
