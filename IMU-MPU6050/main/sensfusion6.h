#ifndef SENSORFUSION6_H_
#define SENSORFUSION6_H_
#include "common.h"

void sensfusion6Init(void);
bool sensfusion6Test(void);

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw);
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az);
float sensfusion6GetInvThrustCompensationForTilt();

#endif /* SENSORFUSION6_H_ */
