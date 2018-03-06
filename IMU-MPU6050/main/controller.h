#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "stabilizer_types.h"

void Controller_Init();
void Controller_PID(state_t *state);


#endif
