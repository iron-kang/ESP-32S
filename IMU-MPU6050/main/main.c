/* Madgwick algorithm: https://github.com/kriswiner/MPU9150/blob/master/MPU9150BasicAHRS.ino
 * https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L.h
 * https://github.com/kriswiner/MPU9250/blob/master/MPU9250_MS5637_AHRS_t3.ino
 */
#include <stdio.h>
#include <math.h>
#include "sensor.h"
#include "bus.h"
#include "hmc5883l.h"
#include "sdkconfig.h"
#include "stabilizer.h"
#include "led.h"
#include "motor.h"
#include "mqtt.h"
#include "network.h"
#include "system.h"

Bus bus;
uint8_t system_state = 0;

void init()
{
	LED_Init();
    Bus_Init(&bus);
    Sensor_Init(&bus, &system_state);
//    MQTT_Init();
    Motor_Init();
    Stabilizer(&system_state);
    Network_Init(&system_state);
    System_Init(&system_state);
    printf("ALL status============= %d\n", system_state);
}
#if 0
void mqtt_task(void *pvParameters)
{
	state_t *state;
	attitude_t *attitude;
	uint32_t lastWakeTime;

	lastWakeTime = xTaskGetTickCount ();
	while (true)
	{
		state = stablizer_GetState();
//		attitude = stablizer_GetAttitude();
//		MQTT_Update(state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
		printf("    rpy: %f, %f, %f\n", state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
//		printf("kal rpy: %f, %f\n", attitude->roll, attitude->pitch);
		vTaskDelayUntil(&lastWakeTime, 1000);
	}
}
#endif

void app_main()
{
    init();
//    xTaskCreate(&mqtt_task, "mqtt_task", 8192, NULL, 3, NULL);

}

