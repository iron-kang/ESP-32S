#include "motor.h"
#include "driver/mcpwm.h"
#include "common.h"

void _Thrust(Motor *this);
void _SetDuty(Motor *this, float duty);

void Motor_Init()
{
	mcpwm_pin_config_t pin_config = {
			.mcpwm0a_out_num = PIN_MOTOR_1,
			.mcpwm0b_out_num = PIN_MOTOR_2,
			.mcpwm1a_out_num = PIN_MOTOR_3,
			.mcpwm1b_out_num = PIN_MOTOR_4,
			.mcpwm2a_out_num = -1,
			.mcpwm2b_out_num = -1,
			.mcpwm_cap0_in_num   = -1,
			.mcpwm_cap1_in_num   = -1,
			.mcpwm_cap2_in_num   = -1,
			.mcpwm_sync0_in_num  = -1,
			.mcpwm_sync1_in_num  = -1,
			.mcpwm_sync2_in_num  = -1,
			.mcpwm_fault0_in_num = -1,
			.mcpwm_fault1_in_num = -1,
			.mcpwm_fault2_in_num = -1
	};
	mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);

	mcpwm_config_t pwm_config;
	pwm_config.frequency = 428;    //frequency = 1000Hz
	pwm_config.cmpr_a = 42.8;    //duty cycle of PWMxA = 50.0%
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

	float duty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
	motor_LF.id = LEFT_FORWARD;
	motor_LB.id = LEFT_BACK;
	motor_RF.id = RIGHT_FORWARD;
	motor_RB.id = RIGHT_BACK;
	motor_LF.duty = duty;
	motor_LB.duty = duty;
	motor_RF.duty = duty;
	motor_RB.duty = duty;

	motor_LF.mutex = xSemaphoreCreateMutex();
	motor_LB.mutex = xSemaphoreCreateMutex();
	motor_RF.mutex = xSemaphoreCreateMutex();
	motor_RB.mutex = xSemaphoreCreateMutex();

	motor_LF.update = _Thrust;
	motor_LB.update = _Thrust;
	motor_RF.update = _Thrust;
	motor_RB.update = _Thrust;

	motor_LF.setDuty = _SetDuty;
	motor_LB.setDuty = _SetDuty;
	motor_RF.setDuty = _SetDuty;
	motor_RB.setDuty = _SetDuty;

	motor_LF.update(&motor_LF);
	motor_LB.update(&motor_LB);
	motor_RF.update(&motor_RF);
	motor_RB.update(&motor_RB);
//	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
//	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);
//	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
//	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty);
	printf("pwm duty: %f\n", duty);
}

void _SetDuty(Motor *this, float duty)
{
	xSemaphoreTake(this->mutex, portMAX_DELAY);
	this->duty = duty;
	xSemaphoreGive(this->mutex);
}

void _Thrust(Motor *this)
{
	switch (this->id)
	{
	case LEFT_FORWARD:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, this->duty);
		break;
	case LEFT_BACK:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, this->duty);
		break;
	case RIGHT_FORWARD:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, this->duty);
		break;
	case RIGHT_BACK:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, this->duty);
		break;
	}
}
