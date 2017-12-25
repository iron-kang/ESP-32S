#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"

#define PIN_SDI  12
#define PIN_SDO  13
#define PIN_SCK  14
#define PIN_CS   15
#define PIN_DC   27
#define PIN_RES  25

SSD1306 ssd1306;

void app_main()
{
    ssd1306.sdi = PIN_SDI;
	ssd1306.sdo = PIN_SDO;
	ssd1306.sck = PIN_SCK;
	ssd1306.cs  = PIN_CS;
	ssd1306.dc  = PIN_DC;
	ssd1306.res = PIN_RES;

	SSD1306_init(&ssd1306);
	ssd1306.SSD1306_begin(&ssd1306);
	ssd1306.SSD1306_bitmap(&ssd1306, 0, 0, waveshare, 128, 64);
	ssd1306.SSD1306_display(&ssd1306);
    vTaskDelay(2000 / portTICK_RATE_MS);
    ssd1306.SSD1306_clear(&ssd1306);

	while (1)
	{
	    ssd1306.SSD1306_bitmap(&ssd1306, 0, 2, Signal816, 16, 8); 
		ssd1306.SSD1306_bitmap(&ssd1306, 24, 2, Bluetooth88, 8, 8); 
		ssd1306.SSD1306_bitmap(&ssd1306, 40, 2, Msg816, 16, 8); 
		ssd1306.SSD1306_bitmap(&ssd1306, 64, 2, GPRS88, 8, 8); 
		ssd1306.SSD1306_bitmap(&ssd1306, 90, 2, Alarm88, 8, 8); 
		ssd1306.SSD1306_bitmap(&ssd1306, 112, 2, Bat816, 16, 8); 

		ssd1306.SSD1306_string(&ssd1306, 0, 52, "MUSIC", 12, 0); 
		ssd1306.SSD1306_string(&ssd1306, 52, 52, "MENU", 12, 0); 
		ssd1306.SSD1306_string(&ssd1306, 98, 52, "PHONE", 12, 0);
		ssd1306.SSD1306_char3216(&ssd1306, 0, 16, '2');
		ssd1306.SSD1306_char3216(&ssd1306, 16, 16, '1');
		ssd1306.SSD1306_char3216(&ssd1306, 32, 16, ':');
		ssd1306.SSD1306_char3216(&ssd1306, 48, 16, '3');
		ssd1306.SSD1306_char3216(&ssd1306, 64, 16, '2');
		ssd1306.SSD1306_char3216(&ssd1306, 80, 16, ':');
		ssd1306.SSD1306_char3216(&ssd1306, 96, 16, '4');
		ssd1306.SSD1306_char3216(&ssd1306, 112, 16, '5');

	    ssd1306.SSD1306_display(&ssd1306);
	    vTaskDelay(1000 / portTICK_RATE_MS);
		printf("loop\n");
	}


}
