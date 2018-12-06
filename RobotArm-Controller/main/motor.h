#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#define PWM_DUTY_MAX  12
#define PWM_DUTY_MIN  3
#define DEGREE_MAX    180.0

enum {
	J1 = 0,
	J2,
	J3,
	J4,
	J5,
	J6
};

void Motor_Init();
void Motor_SetAngle(float *joint);


#endif
