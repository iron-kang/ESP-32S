#include "led.h"
#include "motor.h"
#include "network.h"

void app_main()
{
	LED_Init();
	Motor_Init();
	Network_Init();

	float joint[6] = { 90, 90, 90, 90, 90, 90 };
	Motor_SetAngle(joint);
}
