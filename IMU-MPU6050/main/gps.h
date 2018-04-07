#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"

void GPS_Init(uint8_t *status);
void GPS_GetInfo(GPS_Data *info);

#endif 
