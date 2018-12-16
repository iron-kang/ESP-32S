#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "sdkconfig.h"

#define PWM_DUTY_MAX  10.55//12
#define PWM_DUTY_MIN  2.75//3
#define DEGREE_MAX    180.0

extern const float joint_config[6];
extern const float joint_d4[6];

void Motor_Init();
void Motor_SetAngle(const float *joint);


#endif
