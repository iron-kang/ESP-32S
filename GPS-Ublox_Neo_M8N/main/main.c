#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "gps.h"
#include "ssd1306.h"

#define PIN_SDI  12
#define PIN_SDO  13
#define PIN_SCK  14
#define PIN_CS   15
#define PIN_DC   27
#define PIN_RES  25

#define TXD  (17)
#define RXD  (16)

#define BUF_SIZE (1024)

GPS_M8N gps;
SSD1306 ssd1306;

void echo_task()
{
    uint8_t data;
    char buf_display[3][20];
    char str[128];
    int i, len;

    while(1) 
    {
        i = 0;
        memset(str, 0, 128);
        do {
            len = uart_read_bytes(gps.uart_num, &data, 1, 20 / portTICK_RATE_MS);
            if (len > 0) str[i++] = data;
        } while (data != '\n');

        //printf("%s", str);
        if (gps.parse(&gps, str)) 
        {
            //sprintf(buf_display[0], "altitude: %.2f", gps.altitude);
            //ssd1306.SSD1306_string(&ssd1306, 0, 0, buf_display[0], 16, 1); 
            sprintf(buf_display[1], "N:%f", gps.latitude);
            ssd1306.SSD1306_string(&ssd1306, 0, 16, buf_display[1], 16, 1); 
            sprintf(buf_display[2], "E:%f", gps.longitude);
            ssd1306.SSD1306_string(&ssd1306, 0, 32, buf_display[2], 16, 1); 
            ssd1306.SSD1306_display(&ssd1306);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

void init()
{
    gps.pin_Tx = TXD;
    gps.pin_Rx = RXD;
	gps.baud = 9600;
    gps.uart_num = UART_NUM_2;
    GPS_init(&gps);

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

}

void app_main()
{
    init();

    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
}
