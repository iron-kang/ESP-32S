#include "ultrasonic.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led.h"

#define TIMER_DIVIDER         16
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define ESP_INTR_FLAG_DEFAULT 0
#define TIMER                 TIMER_0
#define TIMER_GROUP           TIMER_GROUP_0

static xQueueHandle ultrsonicQueue = NULL;
uint64_t time_high, time_low, time_escape;
float distance;

static void IRAM_ATTR ultrasonic_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_get_level(gpio_num) == 1)
        timer_get_counter_value(0, TIMER, &time_high);
    else if (gpio_get_level(gpio_num) == 0)
    {
        timer_get_counter_value(0, TIMER, &time_low);
//        timer_set_counter_value(TIMER_GROUP, TIMER, 0x00000000ULL);
    }

    time_escape = time_low - time_high;
    if (time_escape > 0 && time_escape < 200000)
    {
    	xQueueOverwriteFromISR(ultrsonicQueue, &time_escape, NULL);

    }
//    LED_Toggle(PIN_LED_YELLOW);

}

void ultrasonic_task(void* arg)
{
	uint32_t lastWakeTime;

	lastWakeTime = xTaskGetTickCount ();
	while (true)
	{
//		printf("ultrasonic\n");
		gpio_set_level(PIN_Ultrasonic_TRIG, 1);
		vTaskDelay(1 / portTICK_RATE_MS);
		gpio_set_level(PIN_Ultrasonic_TRIG, 0);

		vTaskDelayUntil(&lastWakeTime, 80);
	}
}

void Ultrasonic_GetDistance(float *dist)
{
	uint64_t tmp;
	if (pdTRUE == xQueueReceive(ultrsonicQueue, &tmp, 0))
	{
		distance = tmp * 1000000.0 / TIMER_SCALE / 2 /29;
		*dist = distance;
//		printf("dist: %f, %lld\n", distance, tmp);
	}
}

void Ultrasonic_Init()
{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL<<PIN_Ultrasonic_TRIG);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.pin_bit_mask = (1ULL<<PIN_Ultrasonic_ECHO);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);
//	gpio_set_intr_type(PIN_Ultrasonic_ECHO, GPIO_INTR_ANYEDGE);
	ultrsonicQueue = xQueueCreate(1, sizeof(uint64_t));

//	gpio_install_isr_service(0);
	gpio_isr_handler_add(PIN_Ultrasonic_ECHO, ultrasonic_isr_handler, (void*) PIN_Ultrasonic_ECHO);

	timer_config_t config_tim;
	config_tim.divider     = TIMER_DIVIDER;
	config_tim.counter_dir = TIMER_COUNT_UP;
	config_tim.counter_en  = TIMER_PAUSE;
	config_tim.alarm_en    = TIMER_ALARM_EN;
	config_tim.intr_type   = TIMER_INTR_LEVEL;
	config_tim.auto_reload = 1;
	timer_init(TIMER_GROUP, TIMER, &config_tim);
	timer_set_counter_value(TIMER_GROUP, TIMER, 0x00000000ULL);
	timer_start(TIMER_GROUP, TIMER);
	printf("ultrasonic init\n");

	xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, ULTRASONIC_TASK_PRI, NULL);

}

