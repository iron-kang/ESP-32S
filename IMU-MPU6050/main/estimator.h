#ifndef _ESTIMATOR_H_
#define _ESTIMATOR_H_

#include "stabilizer_types.h"
#include "common.h"

void stateEstimator(state_t *state, const sensorData_t *sensorData, const uint32_t tick);

#endif
