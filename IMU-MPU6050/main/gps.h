#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct gps {
	char str[128];
    float latitude, longitude, altitude;

    bool (*parse)(struct gps *self);

} GPS_M8N;

void GPS_init(GPS_M8N *gps);

#endif 
