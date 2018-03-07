#include "controller.h"
#include "pid.h"
#include "motor.h"

#define KP_ROLL_RATE  7.0
#define KI_ROLL_RATE  0.0
#define KD_ROLL_RATE  0.0

#define KP_PITCH_RATE  7.0
#define KI_PITCH_RATE  0.0
#define KD_PITCH_RATE  0.0

#define KP_YAW_RATE  0.7
#define KI_YAW_RATE  0.167
#define KD_YAW_RATE  0.0

#define KP_ROLL  0.035
#define KI_ROLL  0.020
#define KD_ROLL  0.0

#define KP_PITCH  0.035
#define KI_PITCH  0.20
#define KD_PITCH  0.0

#define KP_YAW  0.100
#define KI_YAW  0.010
#define KD_YAW  0.0035

#define DT (1/500.0)

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

void Controller_PID(state_t *state, sensorData_t *sensors, attitude_t target, uint32_t tick)
{
	attitude_t rateDesired;
	float error;

	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		rateDesired.roll  = PID_Exe(&pidRoll, target.roll - state->attitude.roll);
		rateDesired.pitch = PID_Exe(&pidPitch, target.pitch - state->attitude.pitch);
		error = target.yaw - state->attitude.yaw;
		if (error > 180.0)
			error -= 360.0;
		  else if (error < -180.0)
			error += 360.0;
		rateDesired.yaw = PID_Exe(&pidYaw, error);

//		printf("out1 pid: %f, %f, %f\n", target.roll - state->attitude.roll, target.pitch - state->attitude.pitch, error);
		float thrust_roll  = rateDesired.roll;//PID_Exe(&pidRollRate, rateDesired.roll - sensors->gyro.x);
		float thrust_pitch = rateDesired.pitch;//PID_Exe(&pidPitchRate, rateDesired.pitch - sensors->gyro.y);
		float thrust_yaw   = 0;//PID_Exe(&pidYawRate, rateDesired.yaw - sensors->gyro.z);


		motor_LF.thrust_extra =  thrust_pitch + thrust_roll - thrust_yaw;
		motor_RF.thrust_extra =  thrust_pitch - thrust_roll + thrust_yaw;
		motor_LB.thrust_extra = -thrust_pitch + thrust_roll + thrust_yaw;
		motor_RB.thrust_extra = -thrust_pitch - thrust_roll - thrust_yaw;

//		printf("thrust: %f, %f, %f, %f\n", motor_LF.thrust_extra, motor_LB.thrust_extra, motor_RF.thrust_extra, motor_RB.thrust_extra);

		motor_LF.update(&motor_LF);
		motor_LB.update(&motor_LB);
		motor_RF.update(&motor_RF);
		motor_RB.update(&motor_RB);
	}
}

