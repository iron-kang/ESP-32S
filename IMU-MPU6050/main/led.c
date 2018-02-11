#include "led.h"
#include "common.h"

void LED_Init()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL<<PIN_LED_YELLOW);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_set_level(PIN_LED_YELLOW, 0);
}

void LED_Toggle(gpio_num_t led)
{
	static int toggle = 0;
	toggle = !toggle;

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
