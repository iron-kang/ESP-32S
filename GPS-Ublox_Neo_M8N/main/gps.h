#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct gps {
    uint8_t pin_Tx;
    uint8_t pin_Rx;
    unsigned int baud;
    int uart_num;

    char utc_time[30];
    float latitude;
    char latitude_ch;
    float longitude;
    char longitude_ch;
    int status;
    int num;
    float precision;
    float altitude;
    float height;

    bool (*parse)(struct gps *self, const char *str);

} GPS_M8N;

void GPS_init(GPS_M8N *gps);

#endif 
