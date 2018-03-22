#include "system.h"
#include "common.h"
#include <driver/adc.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "freertos/queue.h"

#define SAMPLE_NUM 20
#define BATTERY_SCALE (750+240)/240.0

esp_adc_cal_characteristics_t *adc_chars;
int32_t bat[SAMPLE_NUM];
xQueueHandle batDataQueue;

void BatRead_Init()
{
	 adc1_config_width(ADC_WIDTH_12Bit);
	 adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_11db);
	 adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	 esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_BIT_12, 1100, adc_chars);

	 batDataQueue = xQueueCreate(1, sizeof(int32_t));
}

void system_task(void *pvParameters)
{
	uint32_t lastWakeTime;
	int32_t voltage, avg;
	uint8_t cnt = 0, i;

	lastWakeTime = xTaskGetTickCount ();
	while (true)
	{
		bat[cnt++] = adc1_get_raw(ADC1_CHANNEL_4);
		cnt %= SAMPLE_NUM;
		avg = 0;

		for (i = 0; i < SAMPLE_NUM; i++ )
			avg += bat[i];
		if (cnt == 0)
		{
			voltage = esp_adc_cal_raw_to_voltage(avg/SAMPLE_NUM, adc_chars);
//			printf("bat vol: %d\n", voltage);
			xQueueOverwrite(batDataQueue, &voltage);
		}
		vTaskDelayUntil(&lastWakeTime, 400);
	}
}

void System_Init()
{
	BatRead_Init();

	xTaskCreate(&system_task, "system_task", 4096, NULL, SYSTEM_TASK_PRI, NULL);
}

void System_GetBatVal(float *bat)
{
	int32_t val;

	if (pdTRUE == xQueueReceive(batDataQueue, &val, 0))
		*bat = val/1000.0*BATTERY_SCALE;
}

