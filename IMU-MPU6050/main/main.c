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
#include "ssd1306.h"
#include "stabilizer.h"
#include "led.h"
#include "motor.h"
#include "mqtt.h"
#include "network.h"

#define OLED_SDI  12
#define OLED_SDO  13
#define OLED_SCK  14
#define OLED_CS   15
#define OLED_DC   27
#define OLED_RES  25
#define COMPASSADDR   HMC5883L_ADDRESS
#define IMUADDR   MPU6050_ADDRESS_AD0_LOW

HMC5883L compass;
SSD1306 ssd1306;
Bus bus;

void init()
{
	bool pass = true;

	LED_Init();
    Bus_Init(&bus);
    Sensor_Init(&bus);

//    MQTT_Init();
//    Motor_Init();

//    pass &= stabilizerTest();
//
//    if (pass)
//    	printf("system OK\n");

    Stabilizer();
//    Network_Init();

//    i2c.setAddr(&i2c, COMPASSADDR);
//    compass.i2c = &i2c;
//    ret = HMC5883L_Init(&compass);
//    if (ret != ESP_OK)
//        printf("Compass fail\n");

//    compass.checkSettings(&compass);

//    ssd1306.sdi = OLED_SDI;
//    ssd1306.sdo = OLED_SDO;
//    ssd1306.sck = OLED_SCK;
//    ssd1306.cs  = OLED_CS;
//    ssd1306.dc  = OLED_DC;
//    ssd1306.res = OLED_RES;
//    SSD1306_init(&ssd1306);
//    ssd1306.SSD1306_begin(&ssd1306);
//    ssd1306.SSD1306_bitmap(&ssd1306, 0, 0, waveshare, 128, 64);
//    ssd1306.SSD1306_display(&ssd1306);
}

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
		vTaskDelayUntil(&lastWakeTime, 100);
	}
}

void app_main()
{
    init();

    xTaskCreate(&mqtt_task, "mqtt_task", 8192, NULL, 3, NULL);

}

