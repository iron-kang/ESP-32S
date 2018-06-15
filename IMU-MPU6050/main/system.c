#include "system.h"
#include "common.h"
#include <driver/adc.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "freertos/queue.h"
#include "esp_freertos_hooks.h"
#include "led.h"
#include "motor.h"

#define CALCULATION_PERIOD    1000
#define SAMPLE_NUM 20
#define BATTERY_SCALE (750+240)/240.0
#define NETWORK_TIMEOUT_MAX   8

xTaskHandle    xIdleHandle = NULL;
uint32_t osCPU_Usage = 0;
uint32_t osCPU_IdleStartTime = 0;
uint32_t osCPU_IdleSpentTime = 0;
uint32_t osCPU_TotalIdleTime = 0;
uint16_t net_timeout = 0;
uint8_t *sys_status = NULL;
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
		net_timeout++;
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

		if (net_timeout >= NETWORK_TIMEOUT_MAX)
		{
			*sys_status |= (1 << STATUS_NET);
			motor_LF.d4(&motor_LF);
			motor_LB.d4(&motor_LB);
			motor_RF.d4(&motor_RF);
			motor_RB.d4(&motor_RB);
		}
		else
			*sys_status &= ~(1UL << STATUS_NET);

//		printf("CPU usage: %d\n", System_GetCPUUsage());
		vTaskDelayUntil(&lastWakeTime, 500);
	}
}

bool vApplicationIdleHook(void)
{
	if( xIdleHandle == NULL )
	{
		/* Store the handle to the idle task. */
		xIdleHandle = xTaskGetCurrentTaskHandle();
	}
	return true;
}

void vApplicationTickHook(void)
{
	//LED_Toggle(PIN_LED_YELLOW);
	static int tick = 0;
	if (net_timeout < 2*NETWORK_TIMEOUT_MAX)
	    net_timeout++;

	if(tick ++ > CALCULATION_PERIOD)
	{
		tick = 0;

		if(osCPU_TotalIdleTime > 1000)
		{
		  osCPU_TotalIdleTime = 1000;
		}
		osCPU_Usage = (100 - (osCPU_TotalIdleTime * 100) / CALCULATION_PERIOD);
		osCPU_TotalIdleTime = 0;
	}
}

void StartIdleMonitor(void)
{
	if( xTaskGetCurrentTaskHandle() == xIdleHandle )
	{
		osCPU_IdleStartTime = xTaskGetTickCountFromISR();
	}
}

void EndIdleMonitor(void)
{
	if( xTaskGetCurrentTaskHandle() == xIdleHandle )
	{
		/* Store the handle to the idle task. */
		osCPU_IdleSpentTime = xTaskGetTickCountFromISR() - osCPU_IdleStartTime;
		osCPU_TotalIdleTime += osCPU_IdleSpentTime;
	}
}

void System_Init(uint8_t *status)
{
	sys_status = status;
	BatRead_Init();
	//esp_register_freertos_idle_hook(vApplicationIdleHook);
	//esp_register_freertos_tick_hook(vApplicationTickHook);

	xTaskCreate(&system_task, "system_task", 4096, NULL, SYSTEM_TASK_PRI, NULL);
}

void System_GetBatVal(float *bat)
{
	int32_t val;

	if (pdTRUE == xQueueReceive(batDataQueue, &val, 0))
		*bat = val/1000.0*BATTERY_SCALE;
}

void System_ClearNetTimout()
{
	net_timeout = 0;
}

uint16_t System_GetCPUUsage(void)
{
  return (uint16_t)osCPU_Usage;
}

