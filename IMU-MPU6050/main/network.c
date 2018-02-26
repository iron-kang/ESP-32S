#include "network.h"
#include "stabilizer.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "nvs_flash.h"
#include "config.h"
#include "utility.h"
#include "led.h"
#include "motor.h"

#define ACT_NUM 2

void action_getInfo();
void action_thrust();

struct netconn *conn, *newconn;
char *buf;
state_t *state;
Info data;
Action actions[] = {
	{action_getInfo, 'A'},
	{action_thrust,  'a'},
	{NULL,			'\0'}
};


void action_getInfo()
{
	char buffer[50];
	state = stablizer_GetState();
	data.attitude.x = state->attitude.roll;
	data.attitude.y = state->attitude.pitch;
	data.attitude.z = state->attitude.yaw;
	memcpy(buffer, &data, sizeof(data));
	netconn_write(newconn, buffer, sizeof(data), NETCONN_NOCOPY);
}

void action_thrust()
{
	int thrust = 0;
	if (buf[3] == '+') thrust = 1;
	else if (buf[3] == '-') thrust = -1;

	motor_LF.update(&motor_LF, motor_LF.duty+thrust);
	motor_LB.update(&motor_LB, motor_LB.duty+thrust);
	motor_RF.update(&motor_RF, motor_RF.duty+thrust);
	motor_RB.update(&motor_RB, motor_RB.duty+thrust);
	printf("thrust\n");
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

void server_task(void *pvParameters)
{
	struct netbuf *inbuf;

	u16_t buflen;
	err_t err;
	uint8_t act;

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	printf("HTTP Server listening...\n");

	while (true) {
		printf("New client connected\n");
		err = netconn_accept(conn, &newconn);

		while (true) {
//			printf("loop...");
			err = netconn_recv(newconn, &inbuf);
//			printf("ok\n");
			if (err == ERR_OK) {
				netbuf_data(inbuf, (void**)&buf, &buflen);
//				for (int i = 0; i < buflen; i++)
//					printf("%x ", buf[i]);
				if ((buf[0] != '@') || (buf[1] != '#'))
					continue;
//				printf("%s\n", &buf[2]);
				netbuf_free(inbuf);

				for (act = 0; act < ACT_NUM; act++)
				{
//					printf("head: %c\n",actions[act].header);
					if (buf[2] == actions[act].header)
					{
						actions[act].action();
						break;
					}
				}

			}
			else break;

		}
		netconn_delete(newconn);
	}
	netconn_close(conn);
	netconn_delete(conn);
	printf("Server exit\n");
}


void Network_Init()
{
	nvs_flash_init();
	esp_log_level_set("wifi", ESP_LOG_NONE);
	tcpip_adapter_init();
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	tcpip_adapter_ip_info_t info;
	memset(&info, 0, sizeof(info));
	IP4_ADDR(&info.ip, 123, 3, 2, 1);
	IP4_ADDR(&info.gw, 123, 3, 2, 1);
	IP4_ADDR(&info.netmask, 255, 255, 255, 0);
	ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));

	ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

	wifi_config_t ap_config = {
		.ap = {
			.ssid = CONFIG_AP_SSID,
			.password = CONFIG_AP_PASSWORD,
			.ssid_len = 0,
			.channel = 6,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.ssid_hidden = 0,
			.max_connection = 10,
			.beacon_interval = CONFIG_AP_BEACON_INTERVAL,
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	xTaskCreate(&server_task, "server-task", 2048, NULL, NETWORK_TASK_PRI, NULL);
}
