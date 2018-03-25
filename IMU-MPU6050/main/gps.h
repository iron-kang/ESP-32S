#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"

typedef struct gps {
	char str[128];
	GPS_Data data;

    bool (*parse)(struct gps *self);

} GPS;

void GPS_Init(GPS *gps, uint8_t *status);

#endif 
