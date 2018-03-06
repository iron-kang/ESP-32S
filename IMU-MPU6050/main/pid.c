#include "pid.h"
#include "utility.h"
#include <float.h>

void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->desired       = desired;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->dt            = dt;
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}

float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->desired - measured;
    }

    pid->outP = pid->kp * pid->error;
    output += pid->outP;

    float deriv = (pid->error - pid->prevError) / pid->dt;
    if (pid->enableDFilter)
    {
      pid->deriv = lpf2pApply(&pid->dFilter, deriv);
    } else {
      pid->deriv = deriv;
    }
    if (isnan(pid->deriv)) {
      pid->deriv = 0;
    }
    pid->outD = pid->kd * pid->deriv;
    output += pid->outD;

    float i = pid->integ + pid->error * pid->dt;
    // Check if integral is saturated
    if (abs(i) <= pid->iLimit || abs(pid->ki * i + output) <= pid->outputLimit) {
      pid->integ = i;
    }

    pid->outI = pid->ki * pid->integ;
    output += pid->outI;

    output = constrain(output, -pid->outputLimit, pid->outputLimit);

    pid->prevError = pid->error;

    return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}


void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
  bool isActive = true;

  if (pid->kp < 0.0001 && pid->ki < 0.0001 && pid->kd < 0.0001)
  {
    isActive = false;
  }

  return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
}
