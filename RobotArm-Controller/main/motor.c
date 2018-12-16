#include "motor.h"
#include "config.h"
#include "driver/mcpwm.h"

#define PWM_MAX  2.2
#define PWM_MIN  0.8
#define PWM_PERIOD 20.0

const float joint_config[6] = { 105, 180, 180, 0, 90, 0 };
const float joint_d4[6]     = { 105,  60,  90, 0, 90, 90 };
bool pwmEn = false;

void Motor_Init()
{
	mcpwm_pin_config_t pin_config = {
			.mcpwm0a_out_num = CONFIG_PIN_MOTOR_1,
			.mcpwm0b_out_num = CONFIG_PIN_MOTOR_2,
			.mcpwm1a_out_num = CONFIG_PIN_MOTOR_3,
			.mcpwm1b_out_num = CONFIG_PIN_MOTOR_4,
			.mcpwm2a_out_num = CONFIG_PIN_MOTOR_5,
			.mcpwm2b_out_num = CONFIG_PIN_MOTOR_6,
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
	pwm_config.frequency = 50;
	pwm_config.cmpr_a = 4;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
	mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_2);

#if 0
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 12);  // min:3(0 deg) max:15 mid:7.5 12(180 deg) y = x/180*9 + 3
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 10);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 15);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 20);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 25);
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, PWM_MAX/PWM_PERIOD*100);
	duty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
	printf("pwm duty: %f\n", duty);
#endif
}

void Motor_SetAngle(const float *joint)
{
	float duty = joint[J1]/DEGREE_MAX*(PWM_DUTY_MAX - PWM_DUTY_MIN) + PWM_DUTY_MIN;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);

	duty = joint[J2]/DEGREE_MAX*(PWM_DUTY_MAX - PWM_DUTY_MIN) + PWM_DUTY_MIN;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty);

	duty = joint[J3]/DEGREE_MAX*(PWM_DUTY_MAX - PWM_DUTY_MIN) + PWM_DUTY_MIN;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);

	duty = joint[J4]/DEGREE_MAX*(PWM_DUTY_MAX - PWM_DUTY_MIN) + PWM_DUTY_MIN;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, duty);

	duty = joint[J5]/DEGREE_MAX*(PWM_DUTY_MAX - PWM_DUTY_MIN) + PWM_DUTY_MIN;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty);

	duty = joint[J6]/DEGREE_MAX*(PWM_DUTY_MAX - PWM_DUTY_MIN) + PWM_DUTY_MIN;
	mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, duty);

	if (!pwmEn)
	{
		mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
		mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
		mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_2);
		pwmEn = true;
	}
}

