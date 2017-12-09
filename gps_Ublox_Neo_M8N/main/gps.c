#include "gps.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h" 

bool _parse(GPS_M8N *self, const char *str);

void GPS_init(GPS_M8N *gps)
{
    uart_config_t uart_config = {                                          
        .baud_rate = gps->baud,                                               
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,                                     
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,                             
        .rx_flow_ctrl_thresh = 122,
    };

    uart_param_config(gps->uart_num, &uart_config);
    uart_set_pin(gps->uart_num, 
                 gps->pin_Tx, 
                 gps->pin_Rx, 
                 UART_PIN_NO_CHANGE, 
                 UART_PIN_NO_CHANGE);
    uart_driver_install(gps->uart_num, 1024 * 2, 0, 0, NULL, 0);

    gps->parse = _parse;
}

bool _parse(GPS_M8N *self, const char *str)
{
    char *pos_N, *pos_E;
    uint8_t pos;
    char substr[20];

    if (strncmp(str, "$GNGLL", 6))
       return false;

    //printf("%s\n", str);
    pos_N = strrchr(str, 'N');
    if (pos_N == NULL || (pos_N-str) < 8) return false;

    pos = pos_N - str;
    strncpy(substr, &str[7], pos-1-7);
    //printf("N: %s\n", substr);
    self->latitude = atof(substr);
    pos_E = strrchr(str, 'E');
    if (pos_E == NULL) return false;
    memset(substr, 0, 20);
    strncpy(substr, &str[pos+2], pos_E-pos-3-str);
    //printf("E: %s\n", substr);
    //printf("posN: %d\n", pos_N - str);
    self->longitude = atof(substr);

    return true;
}

