#include "led.h"
#include "motor.h"
#include "network.h"
#include "xtask.h"

void app_main()
{
	LED_Init();
	Motor_Init();
	Network_Init();
	XTask_Init();

//	float joint[6] = { 105, 60, 90, 0, 90, 0};//{ 105, 90, 90, 90, 90, 90 };
	Motor_SetAngle(joint_d4);
}
