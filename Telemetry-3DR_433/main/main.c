#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define Telemetry_TX	  17
#define Telemetry_RX      16

#define header_H    0x55 //Header
#define header_L    0xAA //Header
#define device_Addr 0x11 //Address
#define data_Length 0x00 //Data length
#define get_Dis_CMD 0x02 //Command: Read Distance
#define checksum    (header_H+header_L+device_Addr+data_Length+get_Dis_CMD) //Checksum

typedef struct {
	 float x;
	 float y;
	 float z;
} AXIS;

typedef struct _gps_data {
	 float utc_time;
	 float latitude;
	 char  latitude_ch;
	 float longitude;
	 char  longitude_ch;
	 int   status;
	 int   num;
	 float hdop;
	 float altitude;
	 float height;
} GPS_Data;

typedef struct _info {
 	AXIS attitude;
 	float thrust[4];
 	float bat;
 	unsigned int height;
// 	uint8_t status;
// 	GPS_Data gps;
 }Info;


void app_main()
{
    int len = 0;
    Info info;
    char buf[100];

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1,
                 Telemetry_TX,
                 Telemetry_RX,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    vTaskDelay(100 / portTICK_RATE_MS);

    info.attitude.x = 1;
    info.attitude.y = 0.3;
    info.attitude.z = 0.4;

    uint8_t size = sizeof(Info);
    char cmd[2] = {'#', '$'};
    printf("init finish\n");

    while(1) {
        buf[0] = 0;
        uart_read_bytes(UART_NUM_1, (uint8_t *)&buf[0], 1, 1000 / portTICK_RATE_MS);
        if (buf[0] == '@')
        {
            info.attitude.x++;
            buf[size] = 0;
            uart_write_bytes(UART_NUM_1, cmd, 2);
            memcpy(buf, &info, sizeof(Info));
            for (int i = 0; i < size; i++)
                buf[size] += buf[i];

            uart_write_bytes(UART_NUM_1, buf, size+1);

        }
        else vTaskDelay(50 / portTICK_RATE_MS);
        uart_flush(UART_NUM_1);
        


    }
}

