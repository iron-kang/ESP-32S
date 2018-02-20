#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include "common.h"
#include "stabilizer_types.h"

void Stabilizer();
bool stabilizerTest(void);
state_t *stablizer_GetState();
attitude_t *stablizer_GetAttitude();

#endif
