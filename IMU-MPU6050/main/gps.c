//http://zzzzllll2006y.blog.163.com/blog/static/3326981420142163313919/
//http://b8807053.pixnet.net/blog/post/3610870-gps%E8%B3%87%E6%96%99%E6%A0%BC%E5%BC%8F
#include "gps.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h" 
#include "config.h"

char str[128];
GPS_Data gps_data;

bool parse();
void gps_task(void* arg);

void GPS_Init(uint8_t *status)
{
	int len = 0;
	uint8_t tmp;
    uart_config_t uart_config = {                                          
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,                                     
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,                             
        .rx_flow_ctrl_thresh = 122,
    };

    uart_param_config(GPS_UART_NUM, &uart_config);
    uart_set_pin(GPS_UART_NUM,
    			 PIN_GPS_TX,
				 PIN_GPS_RX,
                 UART_PIN_NO_CHANGE, 
                 UART_PIN_NO_CHANGE);
    uart_driver_install(GPS_UART_NUM, 1024 * 2, 0, 0, NULL, 0);
    vTaskDelay(1000 / portTICK_RATE_MS);
    for (int i = 0; i < 10; i++)
    {
        len = uart_read_bytes(GPS_UART_NUM, &tmp, 1, 20 / portTICK_RATE_MS);
        printf("gps len..: %d\n", len);
        if (len > 0) break;
    }
    if (len <= 0)
    	*status |= (1 << STATUS_GPS);
    else
    	xTaskCreate(gps_task, "gps_task", 4096, NULL, GPS_TASK_PRI, NULL);
}

void GPS_GetInfo(GPS_Data *info)
{
	memcpy(info, &gps_data, sizeof(GPS_Data));
}

void gps_task(void* arg)
{
	uint32_t lastWakeTime;

	lastWakeTime = xTaskGetTickCount ();
	while (true)
	{
		parse();
		vTaskDelayUntil(&lastWakeTime, 100);
	}
}
//$GNGGA,061824.00,2410.87115,N,12036.99107,E,1,05,1.55,172.7,M,16.3,M,,*45

bool parse()
{
    char *pos_N, *pos_E;
    uint8_t tmp;
    char substr[20];

    int i = 0, len;
    memset(str, 0, 128);
    do {
    	len = uart_read_bytes(GPS_UART_NUM, &tmp, 1, 20 / portTICK_RATE_MS);
    	if (len > 0) str[i++] = tmp;
    	else
    		return false;
    } while (tmp != '\n');

//    printf("%s\n", str);
//    if (strncmp(str, "$GNGLL", 6))
	if (strncmp(str, "$GNGGA", 6))
       return false;

	sscanf(str, "$GNGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%f",
			&gps_data.utc_time, &gps_data.latitude,  &gps_data.latitude_ch,
			&gps_data.longitude, &gps_data.latitude_ch, &gps_data.status,
			&gps_data.num, &gps_data.precision, &gps_data.altitude, &gps_data.height);

//	printf("GPS: %f, %f, %f\n",
//			gps_data.latitude, gps_data.longitude, gps_data.height);
	return true;
//    printf("%s\n", str);
    pos_N = strrchr(str, 'N');
    if (pos_N == NULL || (pos_N-str) < 8) return false;

    tmp = pos_N - str;
    strncpy(substr, &str[7], tmp-1-7);
    //printf("N: %s\n", substr);
    gps_data.latitude = atof(substr);
    pos_E = strrchr(str, 'E');
    if (pos_E == NULL) return false;
    memset(substr, 0, 20);
    strncpy(substr, &str[tmp+2], pos_E-tmp-3-str);
    //printf("E: %s\n", substr);
    //printf("posN: %d\n", pos_N - str);
    gps_data.longitude = atof(substr);

    return true;
}

