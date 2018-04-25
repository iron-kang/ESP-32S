#ifndef _STABILIZER_H_
#define _STABILIZER_H_

#include "common.h"
#include "stabilizer_types.h"

void Stabilizer(uint8_t *status);
bool stabilizerTest(void);
bool stabilizer_GetState(Info *info);
void stabilizer_GetState_Android(Info_Android *info);
attitude_t *stabilizer_GetAttitude();

#endif
