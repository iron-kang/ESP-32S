#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ble_server.h"
#include "driver/mcpwm.h"
#include "sdkconfig.h"

#define GPIO_PWM0A_OUT 15
#define GPIO_BUTTON 0

static xQueueHandle gpio_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_queue, &gpio_num, NULL);
}

void PWM_Init()
{
	mcpwm_pin_config_t pin_config = {
	        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
	        .mcpwm0b_out_num = -1,
	        .mcpwm1a_out_num = -1,
	        .mcpwm1b_out_num = -1,
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
    pin_config.mcpwm0a_out_num = GPIO_PWM0A_OUT;
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
}

static void PWM_Control(void *arg)
{
	mcpwm_config_t pwm_config;
    uint32_t io_num;
	float duty;
	pwm_config.frequency = 428;    //frequency = 1000Hz
	pwm_config.cmpr_a = 42.8;    //duty cycle of PWMxA = 50.0%
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	duty = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);

	while (true)
	{
		if (speed != 0)
		{
			duty += speed;
			mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);

			printf("PWM Change: %f\n", duty);
			speed = 0;
		}
//		printf("Speed: %d\n", speed);
//		if(xQueueReceive(gpio_queue, &io_num, portMAX_DELAY)) {
//			printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
//		}
		vTaskDelay(100);
	}
}

void Button_Init()
{
    gpio_config_t io_conf;
    //interrupt of rising edge
	io_conf.intr_type = GPIO_PIN_INTR_LOLEVEL;
	//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = 1 << GPIO_BUTTON;
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

    gpio_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*) GPIO_BUTTON);
}

void app_main()
{
#if 1
	esp_err_t ret = BLE_Server_Init();
	if (ret)
	{
		ESP_LOGE(GATTS_TAG, "BLE Server Init fail!\n");
	}


#endif
	//Button_Init();
	PWM_Init();

	speed = 1;
	printf("start schedule\n");
	xTaskCreate(PWM_Control, "PWM Task", 4096, NULL, 2, NULL);

	return;
}
