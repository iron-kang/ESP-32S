#include "pid.h"

void PID_Reset(PID *pid)
{
	pid->u_1 = 0;
	pid->error_1 = 0;
	pid->error_2 = 0;
}

void PID_Init(PID *pid, float kp, float ki, float kd, float dt)
{
	PID_Reset(pid);
	pid->dt = dt;
	PID_Set(pid, kp, ki, kd);
}

void PID_Set(PID *pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki * pid->dt;
	pid->kd = kd / pid->dt;
}

float PID_Exe(PID *pid, float error)
{
	float output = 0.0;
	/* 增量式PID
	 * ∆u(k) = kp*(e(k)-e(k-1)) + ki*e(k) + kd(e(k)-2*e(k-1)+e(k-2))
	 * u(k) = u(k-1) + ∆u(k)
	 */
	output = pid->u_1*0 + pid->kp*(error - pid->error_1) + pid->ki*error + pid->kd*(error - 2*pid->error_1 + pid->error_2);
	pid->u_1 = output;
	pid->error_2 = pid->error_1;
	pid->error_1 = error;

	return output;
}
