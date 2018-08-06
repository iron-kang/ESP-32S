#include "led.h"
#include "common.h"
#include "driver/ledc.h"

void LED_Init()
{
#if 1
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL<<PIN_LED_YELLOW);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

//	gpio_set_level(PIN_LED_YELLOW, 1);
#endif

#if 0
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz = 10,                      // frequency of PWM signal
		.speed_mode = LEDC_LOW_SPEED_MODE,   // timer mode
		.timer_num = LEDC_TIMER_1             // timer index
	};

	ledc_timer_config(&ledc_timer);
	ledc_channel_config_t ledc_channel =
	{
		.channel    = LEDC_CHANNEL_0,
		.duty       = 0,
		.gpio_num   = PIN_LED_YELLOW,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.timer_sel  = LEDC_LOW_SPEED_MODE
	};
	ledc_channel_config(&ledc_channel);
	ledc_fade_func_install(0);

	ledc_set_fade_with_time(ledc_channel.speed_mode,
							ledc_channel.channel, 4096, 300);
	ledc_fade_start(ledc_channel.speed_mode,
					ledc_channel.channel, LEDC_FADE_NO_WAIT);
#endif
}

void LED_Toggle(gpio_num_t led)
{
	static int toggle = 0;
	toggle = !toggle;

//	printf("led tootle: %d\n", toggle);
	gpio_set_level(led, toggle);
}

void LED_ON(gpio_num_t led)
{
	gpio_set_level(led, 1);
}

void LED_OFF(gpio_num_t led)
{
	gpio_set_level(led, 0);
}
