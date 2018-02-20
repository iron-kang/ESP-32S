#include "kalman.h"

void _getAngle(Kalman *this, float newAngle, float newRate, float dt);
void _setAngle(Kalman *this, float angle);
float _getRate(Kalman *this);
void _setQangle(Kalman *this, float Q_angle);
void _setQbias(Kalman *this, float Q_bias);
void _setRmeasure(Kalman *this, float R_measure);
float _getQangle(Kalman *sel);
float _getQbias(Kalman *this);
float _getRmeasure(Kalman *this);

void Kalman_Init(Kalman *this)
{
	this->Q_angle = 0.001f;
	this->Q_bias = 0.003f;
	this->R_measure = 0.03f;

	this->angle = 0.0f;
	this->bias = 0.0f;

	this->P[0][0] = 0.0f;
	this->P[0][1] = 0.0f;
	this->P[1][0] = 0.0f;
	this->P[1][1] = 0.0f;

	this->getAngle    = _getAngle;
	this->setAngle    = _setAngle;
	this->getRate     = _getRate;
	this->setQangle   = _setQangle;
	this->setQbias    = _setQbias;
	this->setRmeasure = _setRmeasure;
	this->getQangle   = _getQangle;
	this->getQbias    = _getQbias;
	this->getRmeasure = _getRmeasure;
}

void _getAngle(Kalman *this, float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
	this->rate = newRate - this->bias;
	this->angle += dt * this->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
	this->P[0][0] += dt * (dt*this->P[1][1] - this->P[0][1] - this->P[1][0] + this->Q_angle);
	this->P[0][1] -= dt * this->P[1][1];
	this->P[1][0] -= dt * this->P[1][1];
	this->P[1][1] += this->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = this->P[0][0] + this->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = this->P[0][0] / S;
    K[1] = this->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - this->angle; // Angle difference
    /* Step 6 */
    this->angle += K[0] * y;
    this->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = this->P[0][0];
    float P01_temp = this->P[0][1];

    this->P[0][0] -= K[0] * P00_temp;
    this->P[0][1] -= K[0] * P01_temp;
    this->P[1][0] -= K[1] * P00_temp;
    this->P[1][1] -= K[1] * P01_temp;

}

void _setAngle(Kalman *this, float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float _getRate(Kalman *this) { return this->rate; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void _setQangle(Kalman *this, float Q_angle) { this->Q_angle = Q_angle; };
void _setQbias(Kalman *this, float Q_bias) { this->Q_bias = Q_bias; };
void _setRmeasure(Kalman *this, float R_measure) { this->R_measure = R_measure; };

float _getQangle(Kalman *this) { return this->Q_angle; };
float _getQbias(Kalman *this) { return this->Q_bias; };
float _getRmeasure(Kalman *this) { return this->R_measure; };
