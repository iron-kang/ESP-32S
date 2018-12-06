#include "network.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "config.h"
#include "string.h"

static const char *TAG = "Network";

typedef struct _tskpara {
	struct netconn *newconn;
	char buf_out[200];
}TaskPara;

int old_desired_roll = 0;
int old_desired_pitch = 0;
double thrust_base_old = 0;
uint8_t connect_num = 0;
uint8_t *system_status = NULL;

void action_reboot(char *buf_in, TaskPara *para)
{
	esp_restart();
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_AP_START:
		printf("Access point started\n");
		break;
	case SYSTEM_EVENT_AP_STACONNECTED:
		printf("connect\n");
		break;
	case SYSTEM_EVENT_AP_STADISCONNECTED:
		printf("disconnect\n");
		break;
	default:
        break;
    }

	return ESP_OK;
}

void client_task(void *pvParameters)
{
	uint8_t act;
	err_t err;
	u16_t buflen;
	struct netbuf *inbuf;
	char *buf;
	TaskPara *para = (TaskPara *) pvParameters;

	while (true) {
		err = netconn_recv(para->newconn, &inbuf);
		if (err == ERR_OK)
		{
			netbuf_data(inbuf, (void**)&buf, &buflen);

			if ((buf[0] != '@') || (buf[1] != '#'))
				continue;
			netbuf_delete(inbuf);

//			for (act = 0; act < ACT_NUM; act++)
//			{
//				if (buf[2] == actions[act].header)
//				{
//					actions[act].action(buf, para);
//					break;
//				}
//			}
		}
		else
			break;

	}
	printf("finish one client task\n");
	netconn_delete(para->newconn);
	free(para);
	vTaskDelete(NULL);
}

void server_task(void *pvParameters)
{
	struct netconn *conn;
	err_t err;

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	printf("HTTP Server listening...\n");

	while (true) {
		printf("New client connected\n");
		TaskPara *para = (TaskPara *) malloc(sizeof(TaskPara));
		err = netconn_accept(conn, &para->newconn);

		if (err != ESP_OK)
		{
			break;
		}
		xTaskCreate(&client_task, "client-task", 2048, para, NETWORK_TASK_PRI, NULL);

	}
	netconn_close(conn);
	netconn_delete(conn);
	printf("Server exit\n");
}


void Network_Init()
{
	nvs_flash_init();
//	esp_log_level_set("wifi", ESP_LOG_NONE);
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	tcpip_adapter_ip_info_t info;
	memset(&info, 0, sizeof(info));
	IP4_ADDR(&info.ip, 192, 168, 123, 1);
	IP4_ADDR(&info.gw, 192, 168, 123, 1);
	IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

	wifi_config_t ap_config = {
		.ap = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASSWORD,
			.ssid_len = 0,
			.channel = CONFIG_WIFI_CHANNEL,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = CONFIG_AP_HIDE_SSID,
			.max_connection = 10,
			.beacon_interval = CONFIG_AP_BEACON_INTERVAL,
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());

//	xTaskCreate(&server_task, "server-task", 2048, NULL, NETWORK_TASK_PRI, NULL);

}
