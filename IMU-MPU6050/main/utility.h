#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "common.h"
#include "imu_types.h"

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

#undef abs
#define abs(a) ((a) > 0 ? (a) : (-1*(a)))

#undef isnan
#define isnan(n) ((n != n) ? 1 : 0)


typedef struct _info {
	AXIS attitude;
	float thrust[4];
}Info;

float invSqrt(float x);
uint16_t single2half(float number);
float half2single(uint16_t number);

uint16_t limitUint16(int32_t value);
float constrain(float value, const float minVal, const float maxVal);
float deadband(float value, const float threshold);

#endif
