#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct gps {
    uint8_t pin_Tx;
    uint8_t pin_Rx;
    unsigned int baud;
    int uart_num;

    float latitude, longitude, altitude;

    bool (*parse)(struct gps *self, const char *str);

} GPS_M8N;

void GPS_init(GPS_M8N *gps);

#endif 
