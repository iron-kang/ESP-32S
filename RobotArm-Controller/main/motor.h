#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#define PWM_DUTY_MAX  12
#define PWM_DUTY_MIN  3
#define DEGREE_MAX    180.0

void Motor_Init();
void Motor_SetAngle(float angle);


#endif
