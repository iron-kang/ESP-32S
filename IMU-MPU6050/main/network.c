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
#include "controller.h"

#define ACT_NUM 4

void action_getInfo();
void action_thrust();
void action_direction();
void action_getPID();

struct netconn *newconn;
char *buf;
char buf_out[100];
state_t *state;
Info data;
PidParam pid_attitude;
PidParam pid_rate;
Action actions[] = {
	{action_getInfo,   'A'},
	{action_getPID,    'a'},
	{action_thrust,    'B'},
	{action_direction, 'b'},
	{NULL,			  '\0'}
};

void action_getPID()
{
	memset(buf_out, '0', 100);
#if 1
	pid_attitude.roll[KP]  = pidRoll.kp;
	pid_attitude.roll[KI]  = pidRoll.ki;
	pid_attitude.roll[KD]  = pidRoll.kd;
	pid_attitude.pitch[KP] = pidPitch.kp;
	pid_attitude.pitch[KI] = pidPitch.ki;
	pid_attitude.pitch[KD] = pidPitch.kd;
	pid_attitude.yaw[KP]   = pidYaw.kp;
	pid_attitude.yaw[KI]   = pidYaw.ki;
	pid_attitude.yaw[KD]   = pidYaw.kd;
	pid_rate.roll[KP]  = pidRollRate.kp;
	pid_rate.roll[KI]  = pidRollRate.ki;
	pid_rate.roll[KD]  = pidRollRate.kd;
	pid_rate.pitch[KP] = pidPitchRate.kp;
	pid_rate.pitch[KI] = pidPitchRate.ki;
	pid_rate.pitch[KD] = pidPitchRate.kd;
	pid_rate.yaw[KP]   = pidYawRate.kp;
	pid_rate.yaw[KI]   = pidYawRate.ki;
	pid_rate.yaw[KD]   = pidYawRate.kd;
#endif

	buf_out[0] = 'a';
	int size = sizeof(PidParam)*2+1;
	memcpy(&buf_out[1], &size, sizeof(int));
//	memcpy(&buf_out[1], pid_para, sizeof(float)*PID_NUM);
	memcpy(&buf_out[5],&pid_attitude, sizeof(PidParam));
	memcpy(&buf_out[5+sizeof(PidParam)], &pid_rate, sizeof(PidParam));

//	printf("yaw kd: %f\n", pid_attitude.yaw[KD]);
	netconn_write(newconn, buf_out, sizeof(PidParam)*2+5, NETCONN_NOCOPY);
}


void action_getInfo()
{
	memset(buf_out, '0', 100);
	state = stablizer_GetState();

	data.attitude.x = state->attitude.roll;
	data.attitude.y = state->attitude.pitch;
	data.attitude.z = state->attitude.yaw;

	data.thrust[LEFT_FORWARD]  = motor_LF.thrust;
	data.thrust[LEFT_BACK]     = motor_LB.thrust;
	data.thrust[RIGHT_FORWARD] = motor_RF.thrust;
	data.thrust[RIGHT_BACK]    = motor_RB.thrust;
	buf_out[0] = 'A';
	memcpy(&buf_out[1], &data, sizeof(data));

//	printf("thrust: %f, %f, %f, %f\n", motor_LF.thrust, motor_LB.thrust, motor_RF.thrust, motor_RB.thrust);
//	printf("rpy: %f, %f, %f\n", data.attitude.x, data.attitude.y, data.attitude.z);

	netconn_write(newconn, buf_out, sizeof(data)+1, NETCONN_NOCOPY);
}

void action_thrust()
{
	int thrust = 0;
	if (buf[3] == '+') thrust = 1;
	else if (buf[3] == '-') thrust = -1;

	motor_LF.thrust_base += thrust;
	motor_LB.thrust_base += thrust;
	motor_RF.thrust_base += thrust;
	motor_RB.thrust_base += thrust;
	motor_LF.update(&motor_LF);
	motor_LB.update(&motor_LB);
	motor_RF.update(&motor_RF);
	motor_RB.update(&motor_RB);
	printf("thrust: %f\n", motor_LF.thrust_base);
}

void action_direction()
{
	printf("dir: %c\n", buf[3]);
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
	struct netconn *conn;
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
//			printf("ok (%d)", cnt++);
			if (err == ERR_OK)
			{
				netbuf_data(inbuf, (void**)&buf, &buflen);
//				for (int i = 0; i < buflen; i++)
//					printf("%x ", buf[i]);
				if ((buf[0] != '@') || (buf[1] != '#'))
					continue;
//				printf(" %c\n", buf[2]);
				netbuf_delete(inbuf);

				for (act = 0; act < ACT_NUM; act++)
				{
					if (buf[2] == actions[act].header)
					{
						actions[act].action();
						break;
					}
				}
			}
			else
				break;

		}
		netconn_delete(newconn);
	}
	netconn_close(conn);
	netconn_delete(conn);
	printf("Server exit\n");
}


void Network_Init()
{
//	nvs_flash_init();
	esp_log_level_set("wifi", ESP_LOG_NONE);
	tcpip_adapter_init();
	ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
	tcpip_adapter_ip_info_t info;
	memset(&info, 0, sizeof(info));
	IP4_ADDR(&info.ip, 192, 168, 123, 1);
	IP4_ADDR(&info.gw, 192, 168, 123, 1);
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
