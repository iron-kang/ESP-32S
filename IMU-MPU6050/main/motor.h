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
	float thrust_base;
	float thrust_extra;
	float thrust;
	xSemaphoreHandle mutex;

	void (*update)(struct _motor *this);
}Motor;

Motor motor_LF, motor_LB, motor_RF, motor_RB;

void Motor_Init();


#endif
