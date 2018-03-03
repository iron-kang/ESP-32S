#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "common.h"
#include "imu_types.h"

typedef struct _info {
	Axis3f attitude;
	float thrust[4];
}Info;

float invSqrt(float x);

#endif
