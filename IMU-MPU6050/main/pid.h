#ifndef PID_H_
#define PID_H_

typedef struct pid {
	float kp;
	float ki;
	float kd;
	float dt;
	float error_1;
	float error_2;
	float u_1;
}PID;

float PID_Exe(PID *pid, float error);
void  PID_Reset(PID *pid);
void  PID_Init(PID *pid, float kp, float ki, float kd, float dt);
void  PID_Set(PID *pid, float kp, float ki, float kd);

#endif
