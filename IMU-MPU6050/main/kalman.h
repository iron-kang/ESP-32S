#ifndef _KALMAN_H_
#define _KALMAN_H_

#include "common.h"

typedef struct _kalman {
	float Q_angle;
	float Q_bias;
	float R_measure;
	float angle;
	float bias;
	float rate;
	float P[2][2];

    float (*getAngle)(struct _kalman *self, float newAngle, float newRate, float dt);

    void (*setAngle)(struct _kalman *self, float angle);
    float (*getRate)(struct _kalman *self);

    void (*setQangle)(struct _kalman *self, float Q_angle);
    void (*setQbias)(struct _kalman *self, float Q_bias);
    void (*setRmeasure)(struct _kalman *self, float R_measure);

    float (*getQangle)(struct _kalman *self);
    float (*getQbias)(struct _kalman *self);
    float (*getRmeasure)(struct _kalman *self);

}Kalman;

void Kalman_Init(Kalman *kal);

#endif
