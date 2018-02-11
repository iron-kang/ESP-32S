#ifndef _LED_H_
#define _LED_H_

#include "driver/gpio.h"

void LED_Init();
void LED_Toggle(gpio_num_t led);
void LED_ON(gpio_num_t led);
void LED_OFF(gpio_num_t led);


#endif
