#include "controller.h"
#include "pid.h"

#define KP_ROLL_RATE  70.0
#define KI_ROLL_RATE  0.0
#define KD_ROLL_RATE  0.0

#define KP_PITCH_RATE  70.0
#define KI_PITCH_RATE  0.0
#define KD_PITCH_RATE  0.0

#define KP_YAW_RATE  70.0
#define KI_YAW_RATE  16.7
#define KD_YAW_RATE  0.0

#define KP_ROLL  3.5
#define KI_ROLL  2.0
#define KD_ROLL  0.0

#define KP_PITCH  3.5
#define KI_PITCH  2.0
#define KD_PITCH  0.0

#define KP_YAW  10.0
#define KI_YAW  1.0
#define KD_YAW  0.35

#define DT (1/500)

PID pidRollRate;
PID pidPitchRate;
PID pidYawRate;
PID pidRoll;
PID pidPitch;
PID pidYaw;

void Controller_Init()
{
	PID_Init(&pidRollRate, KP_ROLL_RATE, KI_ROLL_RATE, KD_ROLL_RATE, DT);
	PID_Init(&pidPitchRate, KP_PITCH_RATE, KI_PITCH_RATE, KD_PITCH_RATE, DT);
	PID_Init(&pidYawRate, KP_YAW_RATE, KI_YAW_RATE, KD_YAW_RATE, DT);

	PID_Init(&pidRoll, KP_ROLL, KI_ROLL, KD_ROLL, DT);
	PID_Init(&pidPitch, KP_PITCH, KI_PITCH, KD_PITCH, DT);
	PID_Init(&pidYaw, KP_YAW, KI_YAW, KD_YAW, DT);

}

void Controller_PID(state_t *state)
{
	attitude_t rateDesired;

}

