#include <stdio.h>
#include "Bat_MAX17043.h"
#include "sdkconfig.h"

#define I2C_SDA 25
#define I2C_SCL 26
#define I2C_INT 32

MAX17043 fuel_gague;

void bat_read_task(void *arg)
{
	fuel_gague.setAlertThreshold(&fuel_gague, 10);
	printf("Alert Threshold is set to : %d\n", fuel_gague.getAlertThreshold(&fuel_gague));
	printf("Version: %d\n", fuel_gague.getVersion(&fuel_gague));

	while (true)
	{
		printf("Percent: %.1f%%, Vcell: %.3fV\n", fuel_gague.getBatteryPercentage(&fuel_gague),
									    	  fuel_gague.getBatteryVoltage(&fuel_gague));

		vTaskDelay(5000 / portTICK_RATE_MS);
	}
}

void init()
{
	esp_err_t ret;

	fuel_gague.SDAPin = I2C_SDA;
	fuel_gague.SCLPin = I2C_SCL;
	fuel_gague.alertPin = I2C_INT;
	fuel_gague.alertThreshold = 20;
	MAX17043_Init(&fuel_gague);

	ret = fuel_gague.reset(&fuel_gague);
	if (ret == ESP_FAIL)
		printf("i2c fail\n");
}

void app_main()
{
	init();
    vTaskDelay(1000 / portTICK_RATE_MS);

    xTaskCreate(bat_read_task, "Battery Read", 2048, NULL, 2, NULL);
}

