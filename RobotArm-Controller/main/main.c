#include "led.h"
#include "motor.h"

void app_main()
{
	LED_Init();
	Motor_Init();

	Motor_SetAngle(45);
}
