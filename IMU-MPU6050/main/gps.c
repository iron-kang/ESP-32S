//http://zzzzllll2006y.blog.163.com/blog/static/3326981420142163313919/
//http://b8807053.pixnet.net/blog/post/3610870-gps%E8%B3%87%E6%96%99%E6%A0%BC%E5%BC%8F
#include "gps.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h" 
#include "config.h"

bool _parse(GPS *self);

void GPS_Init(GPS *gps, uint8_t *status)
{
	int len;
	uint8_t tmp;
    uart_config_t uart_config = {                                          
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,                                     
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,                             
        .rx_flow_ctrl_thresh = 122,
    };

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM,
    			 PIN_GPS_TX,
				 PIN_GPS_RX,
                 UART_PIN_NO_CHANGE, 
                 UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, 1024 * 2, 0, 0, NULL, 0);

    gps->parse = _parse;
    len = uart_read_bytes(GPS_UART_NUM, &tmp, 1, 20 / portTICK_RATE_MS);
    if (len <= 0)
    	*status |= (1 << STATUS_GPS);
}

bool _parse(GPS *self)
{
    char *pos_N, *pos_E;
    uint8_t tmp;
    char substr[20];

    int i = 0, len;
    memset(self->str, 0, 128);
    do {
    	len = uart_read_bytes(GPS_UART_NUM, &tmp, 1, 20 / portTICK_RATE_MS);
    	if (len > 0) self->str[i++] = tmp;
    	else
    		return false;
    } while (tmp != '\n');

    if (strncmp(self->str, "$GNGLL", 6))
       return false;

//    printf("%s\n", self->str);
    pos_N = strrchr(self->str, 'N');
    if (pos_N == NULL || (pos_N-self->str) < 8) return false;

    tmp = pos_N - self->str;
    strncpy(substr, &self->str[7], tmp-1-7);
    //printf("N: %s\n", substr);
    self->data.latitude = atof(substr);
    pos_E = strrchr(self->str, 'E');
    if (pos_E == NULL) return false;
    memset(substr, 0, 20);
    strncpy(substr, &self->str[tmp+2], pos_E-tmp-3-self->str);
    //printf("E: %s\n", substr);
    //printf("posN: %d\n", pos_N - str);
    self->data.longitude = atof(substr);

    return true;
}

