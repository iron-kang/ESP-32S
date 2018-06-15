#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define Telemetry_TX	  12
#define Telemetry_RX      27

#define header_H    0x55 //Header
#define header_L    0xAA //Header
#define device_Addr 0x11 //Address
#define data_Length 0x00 //Data length
#define get_Dis_CMD 0x02 //Command: Read Distance
#define checksum    (header_H+header_L+device_Addr+data_Length+get_Dis_CMD) //Checksum
unsigned char i=0;
unsigned int  Distance=0;
unsigned char Rx_DATA[8];
char CMD[6]={
  header_H,header_L,device_Addr,data_Length,get_Dis_CMD,checksum}; //Distance command package

void app_main()
{
    int len = 0;

    uart_config_t uart_config = {
        .baud_rate = 57600,
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
    printf("init finish\n");

    while(1) {
        uart_write_bytes(UART_NUM_1, CMD, 6);
        vTaskDelay(150 / portTICK_RATE_MS);


    }
}

