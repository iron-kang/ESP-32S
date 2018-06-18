#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


typedef enum {
	LEFT_FORWARD = 0,
	LEFT_BACK,
	RIGHT_FORWARD,
	RIGHT_BACK
} motor_t;

typedef struct _motor {
	uint8_t id;
	double thrust_base;
	double thrust_extra;
	double thrust;
	double thrust_base_ex;
	xSemaphoreHandle mutex;

	void (*update)(struct _motor *this);
	void (*d4)(struct _motor *this);
	void (*setBaseThrust)(struct _motor *this, double val);
	void (*setBaseEXThrust)(struct _motor *this, double val);

}Motor;

Motor motor_LF, motor_LB, motor_RF, motor_RB;

void Motor_Init();


#endif
