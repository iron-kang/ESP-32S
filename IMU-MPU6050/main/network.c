#include "network.h"
#include "stabilizer.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "lwip/api.h"
#include "lwip/err.h"
#include "nvs_flash.h"
#include "config.h"

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
	struct netconn *conn, *newconn;
	struct netbuf *inbuf;
	char *buf;
	u16_t buflen;
	err_t err;

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	printf("HTTP Server listening...\n");

	while (true) {
		err = netconn_accept(conn, &newconn);
		printf("New client connected\n");
		while (true) {
			err = netconn_recv(newconn, &inbuf);
			printf("recv: ...\n");
			if (err == ERR_OK) {
				netbuf_data(inbuf, (void**)&buf, &buflen);
//				for (int i = 0; i < buflen; i++)
//					printf("%x ", buf[i]);
				if ((buf[0] != '@') || (buf[1] != '#'))
					continue;
				printf("%s\n", &buf[2]);

			}
		}
		netconn_delete(newconn);
	}
	netconn_close(conn);
	netconn_delete(conn);
	printf("Server exit");
	printf("\n");
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
			.max_connection = CONFIG_AP_MAX_CONNECTIONS,
			.beacon_interval = CONFIG_AP_BEACON_INTERVAL,
		},
	};

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	xTaskCreate(&server_task, "server-task", 2048, NULL, NETWORK_TASK_PRI, NULL);
}
