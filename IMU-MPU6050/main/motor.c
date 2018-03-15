#include "motor.h"
#include "driver/mcpwm.h"
#include "common.h"

void _update(Motor *this);
void _d4(Motor *this);
void _setBaseThrust(Motor *this, float val);

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
	pwm_config.cmpr_a = 43.0;    //duty cycle of PWMxA = 50.0%
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

	float duty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
	motor_LF.id = RIGHT_FORWARD;
	motor_LB.id = RIGHT_BACK;
	motor_RF.id = LEFT_FORWARD;
	motor_RB.id = LEFT_BACK;
	motor_LF.thrust_base = duty;
	motor_LB.thrust_base = duty;
	motor_RF.thrust_base = duty;
	motor_RB.thrust_base = duty;

	motor_LF.thrust_extra = 0;
	motor_LB.thrust_extra = 0;
	motor_RF.thrust_extra = 0;
	motor_RB.thrust_extra = 0;

	motor_LF.mutex = xSemaphoreCreateMutex();
	motor_LB.mutex = xSemaphoreCreateMutex();
	motor_RF.mutex = xSemaphoreCreateMutex();
	motor_RB.mutex = xSemaphoreCreateMutex();

	motor_LF.update = _update;
	motor_LB.update = _update;
	motor_RF.update = _update;
	motor_RB.update = _update;

	motor_LF.d4 = _d4;
	motor_LB.d4 = _d4;
	motor_RF.d4 = _d4;
	motor_RB.d4 = _d4;

	motor_LF.setBaseThrust = _setBaseThrust;
	motor_LB.setBaseThrust = _setBaseThrust;
	motor_RF.setBaseThrust = _setBaseThrust;
	motor_RB.setBaseThrust = _setBaseThrust;

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

void _setBaseThrust(Motor *this, float val)
{
	if ((this->thrust_base+val) > 80)
		this->thrust_base = 80;
	else if ((this->thrust_base+val) < 42)
		this->thrust_base = 42;
	else
		this->thrust_base += val;

	printf("thrust: %f\n", this->thrust_base);

	this->update(this);
}

void _d4(Motor *this)
{
	this->thrust_base = 42.0;
	this->thrust_extra = 0;
	this->update(this);
}


void _update(Motor *this)
{
	xSemaphoreTake(this->mutex, portMAX_DELAY);
	this->thrust = this->thrust_base + this->thrust_extra;
	if (this->thrust > 80 || this->thrust <42)
	{
		this->thrust = this->thrust > 80 ? 80 : this->thrust;
		this->thrust = this->thrust < 42 ? 42 : this->thrust;
		xSemaphoreGive(this->mutex);
		return;
	}
#if 0
	switch (this->id)
	{
	case LEFT_FORWARD:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, this->thrust);
		break;
	case LEFT_BACK:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, this->thrust);
		break;
	case RIGHT_FORWARD:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, this->thrust);
		break;
	case RIGHT_BACK:
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, this->thrust);
		break;
	}
#endif
	xSemaphoreGive(this->mutex);
}
