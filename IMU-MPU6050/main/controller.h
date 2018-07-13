#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "stabilizer_types.h"
#include "pid.h"

typedef enum {
	PID_ROLL_KP = 0,
	PID_ROLL_KI,
	PID_ROLL_KD,
	PID_PITCH_KP,
	PID_PITCH_KI,
	PID_PITCH_KD,
	PID_YAW_KP,
	PID_YAW_KI,
	PID_YAW_KD,
	PID_ROLL_RATE_KP,
	PID_ROLL_RATE_KI,
	PID_ROLL_RATE_KD,
	PID_PITCH_RATE_KP,
	PID_PITCH_RATE_KI,
	PID_PITCH_RATE_KD,
	PID_YAW_RATE_KP,
	PID_YAW_RATE_KI,
	PID_YAW_RATE_KD,
} PID_PARA_t;

#define PID_NUM 18

PID pidRollRate;
PID pidPitchRate;
PID pidYawRate;
PID pidRoll;
PID pidPitch;
PID pidYaw;

void Controller_Init();
void Controller_PID(state_t *state, sensorData_t *sensors, uint32_t tick);
void Controller_SetPID(PidParam pid_atti, PidParam pid_rate);
void Controller_SetAttitude(attitude_t *attitude);

#endif
