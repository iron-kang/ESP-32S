#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/timer.h"

#define Ultrasonic_TRIG     27
#define Ultrasonic_ECHO     35

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<Ultrasonic_TRIG)
#define GPIO_INPUT_PIN_SEL   (1ULL<<Ultrasonic_ECHO)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;
uint64_t time_high, time_low, time_escape;

static void IRAM_ATTR ultrasonic_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_get_level(gpio_num) == 1)
        timer_get_counter_value(0, TIMER_0, &time_high);
    else
    {
        timer_get_counter_value(0, TIMER_0, &time_low);
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    }

    time_escape = time_low - time_high;
    if (time_escape > 0)
        xQueueSendFromISR(gpio_evt_queue, &time_escape, NULL);

}

static void ultrasonic_task_example(void* arg)
{
    uint64_t time_escape;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &time_escape, portMAX_DELAY*0)) {
            printf("Time escape  : %.8f s\n", (double) time_escape / TIMER_SCALE);
        }
    }
}

void app_main()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_set_intr_type(Ultrasonic_ECHO, GPIO_INTR_ANYEDGE);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint64_t));
    xTaskCreate(ultrasonic_task_example, "ultrasonic_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(Ultrasonic_ECHO, ultrasonic_isr_handler, (void*) Ultrasonic_ECHO);

    timer_config_t config_tim;
    config_tim.divider     = TIMER_DIVIDER;
    config_tim.counter_dir = TIMER_COUNT_UP;
    config_tim.counter_en  = TIMER_PAUSE;
    config_tim.alarm_en    = TIMER_ALARM_EN;
    config_tim.intr_type   = TIMER_INTR_LEVEL;
    config_tim.auto_reload = 1;
    timer_init(TIMER_GROUP_0, TIMER_0, &config_tim);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    while(1) {
        printf("ultrasonic\n");
        gpio_set_level(Ultrasonic_TRIG, 1);
        vTaskDelay(10 / portTICK_RATE_MS);
        gpio_set_level(Ultrasonic_TRIG, 0);

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

