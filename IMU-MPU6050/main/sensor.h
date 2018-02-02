#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "bus.h"
#include "filter.h"

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

void Sensor_Init(Bus *bus);


#endif
