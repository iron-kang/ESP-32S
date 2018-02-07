#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "bus.h"
#include "filter.h"
#include "stabilizer_types.h"

// Low Pass filtering
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
lpf2pData accLpf[3];
lpf2pData gyroLpf[3];

void Sensor_Init(Bus *bus);
bool sensorsAreCalibrated();
bool sensorsTest();
void sensorsAcquire(sensorData_t *sensors, const uint32_t tick);

#endif
