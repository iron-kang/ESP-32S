#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "stabilizer_types.h"
#include "pid.h"

PID pidRollRate;
PID pidPitchRate;
PID pidYawRate;
PID pidRoll;
PID pidPitch;
PID pidYaw;

void Controller_Init();
void Controller_PID(state_t *state, sensorData_t *sensors, attitude_t target, uint32_t tick);


#endif
