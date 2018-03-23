#include "controller.h"
#include "motor.h"
#include "common.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define NVS_KEY_ROLL_RATE_KP  "roll-rate-kp"
#define NVS_KEY_ROLL_RATE_KI  "roll-rate-ki"
#define NVS_KEY_ROLL_RATE_KD  "roll-rate-kd"
#define NVS_KEY_PITCH_RATE_KP "pitch-rate-kp"
#define NVS_KEY_PITCH_RATE_KI "pitch-rate-ki"
#define NVS_KEY_PITCH_RATE_KD "pitch-rate-kd"
#define NVS_KEY_YAW_RATE_KP   "yaw-rate-kp"
#define NVS_KEY_YAW_RATE_KI   "yaw-rate-ki"
#define NVS_KEY_YAW_RATE_KD   "yaw-rate-kd"

#define NVS_KEY_ROLL_KP  "roll-kp"
#define NVS_KEY_ROLL_KI  "roll-ki"
#define NVS_KEY_ROLL_KD  "roll-kd"
#define NVS_KEY_PITCH_KP "pitch-kp"
#define NVS_KEY_PITCH_KI "pitch-ki"
#define NVS_KEY_PITCH_KD "pitch-kd"
#define NVS_KEY_YAW_KP   "yaw-kp"
#define NVS_KEY_YAW_KI   "yaw-ki"
#define NVS_KEY_YAW_KD   "yaw-kd"
#define KP_ROLL_RATE  		70.0
#define KI_ROLL_RATE  		0.0
#define KD_ROLL_RATE  		0.0
#define KP_PITCH_RATE  		70.0
#define KI_PITCH_RATE  		0.0
#define KD_PITCH_RATE  		0.0
#define KP_YAW_RATE  		70.0
#define KI_YAW_RATE  		16.7
#define KD_YAW_RATE  		0.0
#define KP_ROLL  			3.5
#define KI_ROLL  			2.0
#define KD_ROLL  			0.0
#define KP_PITCH  			3.5
#define KI_PITCH  			2.0
#define KD_PITCH  			0.0
#define KP_YAW  			10.0
#define KI_YAW  			1.0
#define KD_YAW  			0.35
#define DT 					(1/500.0)


char *pid_key[PID_NUM] = {
	NVS_KEY_ROLL_KP,
	NVS_KEY_ROLL_KI,
	NVS_KEY_ROLL_KD,
	NVS_KEY_PITCH_KP,
	NVS_KEY_PITCH_KI,
	NVS_KEY_PITCH_KD,
	NVS_KEY_YAW_KP,
	NVS_KEY_YAW_KI,
	NVS_KEY_YAW_KD,
	NVS_KEY_ROLL_RATE_KP,
	NVS_KEY_ROLL_RATE_KI,
	NVS_KEY_ROLL_RATE_KD,
	NVS_KEY_PITCH_RATE_KP,
	NVS_KEY_PITCH_RATE_KI,
	NVS_KEY_PITCH_RATE_KD,
	NVS_KEY_YAW_RATE_KP,
	NVS_KEY_YAW_RATE_KI,
	NVS_KEY_YAW_RATE_KD,
};

float pid_para[PID_NUM] = {
	KP_ROLL,
	KI_ROLL,
	KD_ROLL,
	KP_PITCH,
	KI_PITCH,
	KD_PITCH,
	KP_YAW,
	KI_YAW,
	KD_YAW,
	KP_ROLL_RATE,
	KI_ROLL_RATE,
	KD_ROLL_RATE,
	KP_PITCH_RATE,
	KI_PITCH_RATE,
	KD_PITCH_RATE,
	KP_YAW_RATE,
	KI_YAW_RATE,
	KD_YAW_RATE,
};

xQueueHandle attitudeQueue;
nvs_handle nvs;
attitude_t attitude_old, attitude_desired;
const float SCALE = 1000.0;

void Controller_Init()
{
	esp_err_t err;
	uint16_t tmp_val;
//	nvs_flash_erase();

	err = nvs_flash_init();
	printf((err == ESP_ERR_NVS_NO_FREE_PAGES) ? "Flash no free space!\n" : "");
	err = nvs_open("storage", NVS_READWRITE, &nvs);
	printf((err != ESP_OK) ? "Error (%d) opening NVS handle\n" : "", err);

	for (int i = 0; i < PID_NUM; i++)
	{
		err = nvs_get_u16(nvs, pid_key[i], &tmp_val);
		if (err == ESP_OK)
		{
			printf("%s = %d\n", pid_key[i], tmp_val);
			pid_para[i] = tmp_val/SCALE;
		}
		else
			printf("%s load fail\n", pid_key[i]);
	}

//	for (int i = 0; i < PID_NUM; i++)
//		nvs_set_u16(nvs, pid_key[i], i);
//	err = nvs_commit(nvs);
//	printf((err != ESP_OK) ? "Save Failed!\n" : "Save Done\n");
//	nvs_flash_erase();
	PID_Init(&pidRollRate,  pid_para[PID_ROLL_RATE_KP],  pid_para[PID_ROLL_RATE_KI],  pid_para[PID_ROLL_RATE_KD], DT);
	PID_Init(&pidPitchRate, pid_para[PID_PITCH_RATE_KP], pid_para[PID_PITCH_RATE_KI], pid_para[PID_PITCH_RATE_KD], DT);
	PID_Init(&pidYawRate,   pid_para[PID_YAW_RATE_KP],   pid_para[PID_YAW_RATE_KI],   pid_para[PID_YAW_RATE_KD], DT);

	PID_Init(&pidRoll,  pid_para[PID_ROLL_KP],  pid_para[PID_ROLL_KI],  pid_para[PID_ROLL_KD], DT);
	PID_Init(&pidPitch, pid_para[PID_PITCH_KP], pid_para[PID_PITCH_KI], pid_para[PID_PITCH_KD], DT);
	PID_Init(&pidYaw,   pid_para[PID_YAW_KP],   pid_para[PID_YAW_KI],   pid_para[PID_YAW_KD], DT);

	attitudeQueue = xQueueCreate(1, sizeof(attitude_t));

}

float *Controller_GetPID()
{
	return pid_para;
}

void Controller_SetAttitude(attitude_t *attitude)
{
	xQueueOverwrite(attitudeQueue, attitude);
}

void Controller_SetPID(PidParam pid_atti, PidParam pid_rate)
{
	printf("roll pid: %f, %f, %f\n", pid_atti.roll[KP], pid_atti.roll[KI], pid_atti.roll[KD]);
	printf("save pid: %d, %d, %d\n", (uint16_t)(pid_atti.roll[KP]*SCALE),
			                         (uint16_t)(pid_atti.roll[KI]*SCALE),
									 (uint16_t)(pid_atti.roll[KD]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_ROLL_KP], (uint16_t)(pid_atti.roll[KP]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_ROLL_KI], (uint16_t)(pid_atti.roll[KI]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_ROLL_KD], (uint16_t)(pid_atti.roll[KD]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_PITCH_KP], (uint16_t)(pid_atti.pitch[KP]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_PITCH_KI], (uint16_t)(pid_atti.pitch[KI]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_PITCH_KD], (uint16_t)(pid_atti.pitch[KD]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_YAW_KP], (uint16_t)(pid_atti.yaw[KP]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_YAW_KI], (uint16_t)(pid_atti.yaw[KI]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_YAW_KD], (uint16_t)(pid_atti.yaw[KD]*SCALE));

	nvs_set_u16(nvs, pid_key[PID_ROLL_RATE_KP], (uint16_t)(pid_rate.roll[KP]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_ROLL_RATE_KI], (uint16_t)(pid_rate.roll[KI]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_ROLL_RATE_KD], (uint16_t)(pid_rate.roll[KD]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_PITCH_RATE_KP], (uint16_t)(pid_rate.pitch[KP]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_PITCH_RATE_KI], (uint16_t)(pid_rate.pitch[KI]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_PITCH_RATE_KD], (uint16_t)(pid_rate.pitch[KD]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_YAW_RATE_KP], (uint16_t)(pid_rate.yaw[KP]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_YAW_RATE_KI], (uint16_t)(pid_rate.yaw[KI]*SCALE));
	nvs_set_u16(nvs, pid_key[PID_YAW_RATE_KD], (uint16_t)(pid_rate.yaw[KD]*SCALE));
}

void Controller_PID(state_t *state, sensorData_t *sensors, attitude_t target, uint32_t tick)
{
	attitude_t rateDesired;
	float error;

	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick))
	{
		if (motor_LF.thrust_base < 57) return;

		if (xQueueReceive(attitudeQueue, &attitude_desired, 0))
			attitude_old = attitude_desired;
		else
			attitude_desired = attitude_old;

		rateDesired.roll  = PID_Exe(&pidRoll, target.roll*0 + attitude_desired.roll - state->attitude.roll);
		rateDesired.pitch = PID_Exe(&pidPitch, target.pitch*0 + attitude_desired.pitch - state->attitude.pitch);
		error = target.yaw*0 + attitude_desired.yaw - state->attitude.yaw;
		if (error > 180.0)
			error -= 360.0;
		  else if (error < -180.0)
			error += 360.0;
		rateDesired.yaw = PID_Exe(&pidYaw, error);

//		printf("out1 pid: %f, %f, %f\n", target.roll - state->attitude.roll, target.pitch - state->attitude.pitch, error);
		float thrust_roll  = rateDesired.roll;//PID_Exe(&pidRollRate, rateDesired.roll - sensors->gyro.x);
		float thrust_pitch = rateDesired.pitch;//PID_Exe(&pidPitchRate, rateDesired.pitch - sensors->gyro.y);
		float thrust_yaw   = rateDesired.yaw;//PID_Exe(&pidYawRate, rateDesired.yaw - sensors->gyro.z);

		motor_LF.thrust_extra =  thrust_pitch + thrust_roll + thrust_yaw;
		motor_RF.thrust_extra =  thrust_pitch - thrust_roll - thrust_yaw;
		motor_LB.thrust_extra = -thrust_pitch + thrust_roll - thrust_yaw;
		motor_RB.thrust_extra = -thrust_pitch - thrust_roll + thrust_yaw;

//		printf("rpy_u: %f, %f, %f\n", thrust_roll, thrust_pitch, thrust_yaw);

		motor_LF.update(&motor_LF);
		motor_LB.update(&motor_LB);
		motor_RF.update(&motor_RF);
		motor_RB.update(&motor_RB);
	}
}

