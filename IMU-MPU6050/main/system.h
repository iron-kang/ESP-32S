#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "common.h"

void System_Init(uint8_t *status);
void System_GetBatVal(float *bat);
void System_ClearNetTimout();
uint16_t System_GetCPUUsage (void);

#endif
