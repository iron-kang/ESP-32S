#include "estimator.h"
#include "sensfusion6.h"
#include "led.h"
#include "utility.h"

#define ATTITUDE_UPDATE_RATE RATE_1000_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

#define G 9.81;

struct selfState_s {
  float estimatedZ; // The current Z estimate, has same offset as asl
  float velocityZ; // Vertical speed (world frame) integrated from vertical acceleration (m/s)
  float estAlpha;
  float velocityFactor;
  float vAccDeadband; // Vertical acceleration deadband
  float velZAlpha;   // Blending factor to avoid vertical speed to accumulate error
};

static struct selfState_s state = {
  .estimatedZ = 0.0,
  .velocityZ = 0.0,
  .estAlpha = 0.99,
  .velocityFactor = 1.0,
  .vAccDeadband = 0.04,
  .velZAlpha = 0.995,
};

void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state);
void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state);

void positionEstimate(state_t* estimate, float asl, float dt)
{
	positionEstimateInternal(estimate, asl, dt, &state);
}

void positionUpdateVelocity(float accWZ, float dt)
{
	positionUpdateVelocityInternal(accWZ, dt, &state);
}

void positionEstimateInternal(state_t* estimate, float asl, float dt, struct selfState_s* state)
{
	state->estimatedZ = state->estAlpha * state->estimatedZ +
                     (1.0 - state->estAlpha) * asl +
                     state->velocityFactor * state->velocityZ * dt;

	estimate->position.x = 0.0;
	estimate->position.y = 0.0;
	estimate->position.z = state->estimatedZ;
}

void positionUpdateVelocityInternal(float accWZ, float dt, struct selfState_s* state)
{
  state->velocityZ += deadband(accWZ, state->vAccDeadband) * dt * G;
  state->velocityZ *= state->velZAlpha;
}

void stateEstimator(state_t *state, const sensorData_t *sensorData, const uint32_t tick)
{
	if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick))
	{
//		LED_Toggle(PIN_LED_YELLOW);
	    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
	                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
						   sensorData->mag.x*0, sensorData->mag.y*0, sensorData->mag.z*0,
	                       ATTITUDE_UPDATE_DT);
	   // printf("--a(%.2f, %.2f, %.2f), g(%.2f, %.2f, %.2f)\n", sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
	    //				sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z);
	    sensfusion6GetEulerRPY(&state->attitude.pitch, &state->attitude.roll, &state->attitude.yaw);

	    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
	                                                    sensorData->acc.y,
	                                                    sensorData->acc.z);
//	    printf("roll: %f, pitch: %f, yaw: %f\n", state->attitude.roll, state->attitude.pitch, state->attitude.yaw);
	    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

	}

	if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick))
	{
		// If position sensor data is preset, pass it throught
	    // FIXME: The position sensor shall be used as an input of the estimator
	    if (sensorData->position.timestamp)
	    {
	    	state->position = sensorData->position;
	    } else {
	    	positionEstimate(state, sensorData->baro.asl, POS_UPDATE_DT);
	    }
	}
}

