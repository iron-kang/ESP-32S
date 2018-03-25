#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include "common.h"
#include "stabilizer_types.h"

void Stabilizer();
bool stabilizerTest(void);
bool stablizer_GetState(Info *info);
attitude_t *stablizer_GetAttitude();

#endif
