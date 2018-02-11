#include "mqtt.h"
#include "common.h"
#include "MQTTClient.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"

#define MQTT_SERVER "mqtt.thingspeak.com"
#define MQTT_PORT 443
#define MQTT_BUF_SIZE 1000
#define MQTT_WEBSOCKET 1  // 0=no 1=yes
#define CONFIG_WIFI_SSID "170232-1"
#define CONFIG_WIFI_PASSWORD "75671697"


const int CONNECTED_BIT = BIT0;
static unsigned char mqtt_sendBuf[MQTT_BUF_SIZE];
static unsigned char mqtt_readBuf[MQTT_BUF_SIZE];
static const char *TAG = "MQTTS";
EventGroupHandle_t wifi_event_group;
Network network;
char msgbuf[100];

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

void MQTT_Init()
{
	nvs_flash_init();
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASSWORD,
		},
	};
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
	ESP_ERROR_CHECK( esp_wifi_start() );

	NetworkInit(&network);
	network.websocket = MQTT_WEBSOCKET;

}

void MQTT_Update(float roll, float pitch, float yaw)
{
	MQTTClient client;
	int ret;

	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
							false, true, portMAX_DELAY);
//	NetworkInit(&network);
//	network.websocket = MQTT_WEBSOCKET;
////	ESP_LOGI(TAG,"NetworkConnect %s:%d ...",MQTT_SERVER,MQTT_PORT);
	NetworkConnect(&network, MQTT_SERVER, MQTT_PORT);
//	ESP_LOGI(TAG,"MQTTClientInit  ...");
	MQTTClientInit(&client, &network,
		2000,            // command_timeout_ms
		mqtt_sendBuf,         //sendbuf,
		MQTT_BUF_SIZE, //sendbuf_size,
		mqtt_readBuf,         //readbuf,
		MQTT_BUF_SIZE  //readbuf_size
	);

	MQTTString clientId = MQTTString_initializer;
	clientId.cstring = "ESP32MQTT";

	MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
	data.clientID          = clientId;
	data.willFlag          = 0;
	data.MQTTVersion       = 4; // 3 = 3.1 4 = 3.1.1
	data.keepAliveInterval = 6;
	data.cleansession      = 1;

	ret = MQTTConnect(&client, &data);
	if (ret != SUCCESS) {
		ESP_LOGI(TAG, "MQTTConnect not SUCCESS: %d", ret);
	}

	MQTTMessage message;
	sprintf(msgbuf, "field1=%.3f&field2=%.3f&field3=%.3f",
			roll, pitch, yaw);

//	ESP_LOGI(TAG, "MQTTPublish  ... %s",msgbuf);
	message.qos = QOS0;
	message.retained = false;
	message.dup = false;
	message.payload = (void*)msgbuf;
	message.payloadlen = strlen(msgbuf)+1;

	ret = MQTTPublish(&client, "channels/425034/publish/S0FDP1IWSYGMINBY", &message);
	if (ret != SUCCESS) {
		ESP_LOGI(TAG, "MQTTPublish not SUCCESS: %d", ret);
	}

	MQTTDisconnect(&client);
	NetworkDisconnect(&network);
}
