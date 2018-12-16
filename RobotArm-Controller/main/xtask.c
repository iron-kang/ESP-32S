/*
 * xtask.c
 *
 *  Created on: Dec 16, 2018
 *      Author: iron
 */
#include "xtask.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "math.h"
#include "motor.h"

#define JOINT_TASK_NAME  "joint-task"

extern xQueueHandle jointQ;
extern float joints[6];
float joints_dest[6];
bool isGoal = false;
uint8_t jointOkCnt = 0;

void jointTask(void* param)
{
	uint32_t lastWakeTime;
	float delta;

	lastWakeTime = xTaskGetTickCount ();
	memcpy(joints, joint_d4, sizeof(joint_d4));
	memcpy(joints_dest, joint_d4, sizeof(joint_d4));

	while (true)
	{
		vTaskDelayUntil(&lastWakeTime, 1);
		if (pdTRUE == xQueueReceive(jointQ, joints_dest, 0))
		{
			printf("joints_dest:1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f, 5: %.2f\n",
					joints_dest[J1], joints_dest[J2], joints_dest[J3],
					joints_dest[J4], joints_dest[J5]);

			printf("joints:1: %.2f, 2: %.2f, 3: %.2f, 4: %.2f, 5: %.2f\n",
						joints[J1], joints[J2], joints[J3],
						joints[J4], joints[J5]);
			isGoal = false;
		}

		if (isGoal) continue;

		jointOkCnt = 0;

		for (int i = 0; i < 5; i++)
		{
			delta = joints_dest[i] - joints[i];
			if (fabs(delta) > 0.1)
				joints[i] += delta/fabs(delta)*20/1000.0;
			else {
				joints[i] = joints_dest[i];
				jointOkCnt++;
			}
		}

		if (jointOkCnt <= 5 && !isGoal)
		{
			Motor_SetAngle(joints);
			if (jointOkCnt == 5)
			{
				printf("Goal\n");
				isGoal = true;
			}
		}
	}
}

void XTask_Init()
{

	xTaskCreate(jointTask, JOINT_TASK_NAME, 8192, NULL, STABILIZER_TASK_PRI, NULL);
}

