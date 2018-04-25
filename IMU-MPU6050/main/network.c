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
#include "system.h"

#define ACT_NUM 9

typedef struct _tskpara {
	struct netconn *newconn;
	char buf_out[200];
}TaskPara;

void action_getInfo(char *buf_in, TaskPara *para);
void action_thrust(char *buf_in, TaskPara *para);
void action_direction(char *buf_in, TaskPara *para);
void action_getPID(char *buf_in, TaskPara *para);
void action_setPID(char *buf_in, TaskPara *para);
void action_reboot(char *buf_in, TaskPara *para);
void action_stop(char *buf_in, TaskPara *para);
void action_startup(char *buf_in, TaskPara *para);
void action_getInfo_android(char *buf_in, TaskPara *para);

static float bat;
uint8_t connect_num = 0;
uint8_t *system_status = NULL;
state_t *state;
Info data;
Info_Android info_android;
PidParam pid_attitude;
PidParam pid_rate;
attitude_t attitude_desired;
Action actions[] = {
	{action_getInfo,         'A'},
	{action_getPID,          'a'},
	{action_thrust,          'B'},
	{action_direction,       'b'},
	{action_setPID,          'C'},
	{action_reboot,          'c'},
	{action_stop,            'D'},
	{action_startup,         'd'},
	{action_getInfo_android, 'F'},
	{NULL,			        '\0'}
};

void action_startup(char *buf_in, TaskPara *para)
{
	motor_LF.thrust_base = 49.4;
	motor_LB.thrust_base = 49.4;
	motor_RF.thrust_base = 49.4;
	motor_RB.thrust_base = 49.4;

	motor_LF.update(&motor_LF);
	motor_LB.update(&motor_LB);
	motor_RF.update(&motor_RF);
	motor_RB.update(&motor_RB);
}

void action_stop(char *buf_in, TaskPara *para)
{
	motor_LF.d4(&motor_LF);
	motor_LB.d4(&motor_LB);
	motor_RF.d4(&motor_RF);
	motor_RB.d4(&motor_RB);
}

void action_reboot(char *buf_in, TaskPara *para)
{
	esp_restart();
}

void action_setPID(char *buf_in, TaskPara *para)
{
	printf("save PID parameter\n");
	memcpy(&pid_attitude, &buf_in[3], sizeof(PidParam));
	memcpy(&pid_rate, &buf_in[3+sizeof(pid_attitude)], sizeof(PidParam));

//	printf("yaw kp: %f\n", pid_attitude.yaw[KP]);
	PID_Set(&pidRoll,  pid_attitude.roll[KP],  pid_attitude.roll[KI],  pid_attitude.roll[KD]);
	PID_Set(&pidPitch, pid_attitude.pitch[KP], pid_attitude.pitch[KI], pid_attitude.pitch[KD]);
	PID_Set(&pidYaw,   pid_attitude.yaw[KP],   pid_attitude.yaw[KI],   pid_attitude.yaw[KD]);

	PID_Set(&pidRollRate,  pid_rate.roll[KP],  pid_rate.roll[KI],  pid_rate.roll[KD]);
	PID_Set(&pidPitchRate, pid_rate.pitch[KP], pid_rate.pitch[KI], pid_rate.pitch[KD]);
	PID_Set(&pidYawRate,   pid_rate.yaw[KP],   pid_rate.yaw[KI],   pid_rate.yaw[KD]);

	motor_LF.thrust_extra = 0;
	motor_LB.thrust_extra = 0;
	motor_RF.thrust_extra = 0;
	motor_RB.thrust_extra = 0;

	Controller_SetPID(pid_attitude, pid_rate);

	printf("yaw : %f, %f, %f\n", pid_attitude.yaw[KP], pid_attitude.yaw[KI], pid_attitude.yaw[KD]);
}

void action_getPID(char *buf_in, TaskPara *para)
{
	memset(para->buf_out, '0', 100);
	pid_attitude.roll[KP]  = pidRoll.kp;
	pid_attitude.roll[KI]  = pidRoll.ki / pidRoll.dt;
	pid_attitude.roll[KD]  = pidRoll.kd * pidRoll.dt;
	pid_attitude.pitch[KP] = pidPitch.kp;
	pid_attitude.pitch[KI] = pidPitch.ki / pidPitch.dt;
	pid_attitude.pitch[KD] = pidPitch.kd * pidPitch.dt;
	pid_attitude.yaw[KP]   = pidYaw.kp;
	pid_attitude.yaw[KI]   = pidYaw.ki / pidYaw.dt;
	pid_attitude.yaw[KD]   = pidYaw.kd * pidYaw.dt;
	pid_rate.roll[KP]  = pidRollRate.kp;
	pid_rate.roll[KI]  = pidRollRate.ki / pidRollRate.dt;
	pid_rate.roll[KD]  = pidRollRate.kd * pidRollRate.dt;
	pid_rate.pitch[KP] = pidPitchRate.kp;
	pid_rate.pitch[KI] = pidPitchRate.ki / pidPitchRate.dt;
	pid_rate.pitch[KD] = pidPitchRate.kd * pidPitchRate.dt;
	pid_rate.yaw[KP]   = pidYawRate.kp;
	pid_rate.yaw[KI]   = pidYawRate.ki / pidYawRate.dt;
	pid_rate.yaw[KD]   = pidYawRate.kd * pidYawRate.dt;

	para->buf_out[0] = 'a';
	memcpy(&para->buf_out[1],&pid_attitude, sizeof(PidParam));
	memcpy(&para->buf_out[1+sizeof(PidParam)], &pid_rate, sizeof(PidParam));

	netconn_write(para->newconn, para->buf_out, sizeof(PidParam)*2+1, NETCONN_COPY);
	printf("yaw : %f, %f, %f\n", pid_attitude.yaw[KP], pid_attitude.yaw[KI], pid_attitude.yaw[KD]);
}

void action_getInfo_android(char *buf_in, TaskPara *para)
{
	stabilizer_GetState_Android(&info_android);
	System_GetBatVal(&bat);
	info_android.bat = bat;
	info_android.status = *system_status;
	para->buf_out[0] = 'A';
	memcpy(&para->buf_out[1], &info_android, sizeof(info_android));
	printf("(%d)android get info: %d, %f, %d, %f, %f, %f\n",sizeof(info_android)+1,
			info_android.status, info_android.bat, info_android.height,
			info_android.latitude, info_android.longitude, info_android.altitude);
	netconn_write(para->newconn, para->buf_out, sizeof(Info_Android)+1, NETCONN_NOCOPY);
}


void action_getInfo(char *buf_in, TaskPara *para)
{
//	memset(para->buf_out, '0', 100);
	stabilizer_GetState(&data);
//	state = stablizer_GetState();
//
//	data.attitude.x = state->attitude.roll;
//	data.attitude.y = state->attitude.pitch;
//	data.attitude.z = state->attitude.yaw;

	data.thrust[LEFT_FORWARD]  = motor_LF.thrust;
	data.thrust[LEFT_BACK]     = motor_LB.thrust;
	data.thrust[RIGHT_FORWARD] = motor_RF.thrust;
	data.thrust[RIGHT_BACK]    = motor_RB.thrust;
	System_GetBatVal(&bat);
	data.bat = bat;
	data.status = *system_status;
	para->buf_out[0] = 'A';
	memcpy(&para->buf_out[1], &data, sizeof(data));

//	printf("status: %d\n", *system_status);
//	printf("bat: %f\n", data.bat);
//	printf("GPS: %f, %f, %f\n", data.gps.latitude, data.gps.longitude, data.gps.altitude);
//	printf("thrust: %f, %f, %f, %f\n", motor_LF.thrust, motor_LB.thrust, motor_RF.thrust, motor_RB.thrust);
//	printf("rpy: %f, %f, %f\n", data.attitude.x, data.attitude.y, data.attitude.z);

	netconn_write(para->newconn, para->buf_out, sizeof(data)+1, NETCONN_NOCOPY);
}

void action_thrust(char *buf_in, TaskPara *para)
{
	float thrust = 0;
	if (buf_in[3] == '+')
	{
		if (thrust > 62)
			thrust = 0.5;
		else
			thrust = 0.5*3;
		thrust = 0.5*2;
	}
	else if (buf_in[3] == '-') thrust = -0.5*2;
	else {
		printf("thrust: %d, %f\n", buf_in[3], thrust);
	}

	motor_LF.setBaseThrust(&motor_LF, thrust);
	motor_LB.setBaseThrust(&motor_LB, thrust);
	motor_RF.setBaseThrust(&motor_RF, thrust);
	motor_RB.setBaseThrust(&motor_RB, thrust);

//	printf("thrust: %f\n", motor_LF.thrust_base);
}

void action_direction(char *buf_in, TaskPara *para)
{
	if (buf_in[2] == 'b')
	{
		switch (buf_in[3])
		{
		case 'r':
			attitude_desired.roll = 13;
			break;
		case 'l':
			attitude_desired.roll = -13;
			break;
		case 's':
			attitude_desired.roll = 0;
			break;
		case 'f':
			attitude_desired.pitch = -13;
			break;
		case 'b':
			attitude_desired.pitch = 13;
			break;
		case 'S':
			attitude_desired.pitch = 0;
			break;
		}
	}
	Controller_SetAttitude(&attitude_desired);
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
			System_ClearNetTimout();
//				printf(" %c\n", buf[2]);
			netbuf_delete(inbuf);

			for (act = 0; act < ACT_NUM; act++)
			{
				if (buf[2] == actions[act].header)
				{
					actions[act].action(buf, para);
					break;
				}
			}
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
			*system_status |= (1 << STATUS_NET);
			break;
		}
		xTaskCreate(&client_task, "client-task", 2048, para, NETWORK_TASK_PRI, NULL);

	}
	netconn_close(conn);
	netconn_delete(conn);
	printf("Server exit\n");
}


void Network_Init(uint8_t *status)
{
	system_status = status;

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
