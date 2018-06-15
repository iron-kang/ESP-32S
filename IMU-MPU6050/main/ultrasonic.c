#include "ultrasonic.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "led.h"

#define TIMER_DIVIDER         16
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)
#define ESP_INTR_FLAG_DEFAULT 0
#define TIMER                 TIMER_0
#define TIMER_GROUP           TIMER_GROUP_0
#define BETA                  0.2
#define DIST_SIZE             10

static xQueueHandle ultrsonicQueue = NULL;
unsigned int distance;
unsigned char i=0;
unsigned int Distance = 0;
unsigned int out_dist = 0;
unsigned int buf_dist[DIST_SIZE] = { 0 };
float Dist = 0;
unsigned char Rx_DATA[8];
char CMD[6]={
  header_H,header_L,device_Addr,data_Length,get_Dis_CMD,checksum};

void ultrasonic_task(void* arg)
{
	uint32_t lastWakeTime;
	int len = 0;
	uint8_t cnt = 0;
	int8_t id_prv = 0;
	int16_t slope_prv = 0;
	int16_t slope_cur = 0;

	lastWakeTime = xTaskGetTickCount ();
	while (true)
	{
		uart_write_bytes(UART_NUM_2, CMD, 6);
		vTaskDelay(1 / portTICK_RATE_MS);
		len = uart_read_bytes(UART_NUM_2, Rx_DATA, 8, 20 / portTICK_RATE_MS);
		if (len == 8)
		{
			Distance = ((Rx_DATA[5]<<8)|Rx_DATA[6]);
			//Dist = Dist - BETA*(Dist - Distance);
			id_prv = ((cnt-1)+DIST_SIZE) % DIST_SIZE;
			if (buf_dist[id_prv] != 0)
			{
				slope_cur = Distance - buf_dist[id_prv];
			}
			if (slope_cur*slope_prv < -10000)
			{
				//printf("high freq: %d, %d\n", slope_prv, slope_cur);
				buf_dist[id_prv] = Distance;
			}
			slope_prv = slope_cur;
			//printf("%d cm\n", Distance);
			//xQueueOverwrite(ultrsonicQueue, &Distance);
			buf_dist[cnt++] = Distance;
		}
#if 1
		if (buf_dist[DIST_SIZE-1] != 0)
		{
			for (len = 0; len < DIST_SIZE; len++)
			    Dist += buf_dist[len];
			Dist /= 10;
			out_dist = (int)Dist;
			//printf("avg:(%d) cm\n", out_dist);
			xQueueOverwrite(ultrsonicQueue, &out_dist);
			Dist = 0;
		}
		if (cnt == 10) cnt = 0;
#endif

		vTaskDelayUntil(&lastWakeTime, 50);
	}
}

bool Ultrasonic_GetDistance(unsigned int *dist)
{
	return (pdTRUE == xQueueReceive(ultrsonicQueue, dist, 0));
}

void Ultrasonic_Init()
{
	uart_config_t uart_config = {
		.baud_rate = Ultrasonic_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122,
	};

	uart_param_config(UART_NUM_2, &uart_config);
	uart_set_pin(UART_NUM_2,
				 PIN_Ultrasonic_TX,
				 PIN_Ultrasonic_RX,
				 UART_PIN_NO_CHANGE,
				 UART_PIN_NO_CHANGE);
	uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0);
	ultrsonicQueue = xQueueCreate(1, sizeof(unsigned int));
	printf("ultrasonic init\n");

	xTaskCreate(ultrasonic_task, "ultrasonic_task", 2048, NULL, ULTRASONIC_TASK_PRI, NULL);

}

