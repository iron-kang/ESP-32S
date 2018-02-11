#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "freertos/FreeRTOSConfig.h"


#define SENSORS_TASK_PRI        4
#define STABILIZER_TASK_PRI     4

#define TASK_LED_ID_NBR         1
#define TASK_RADIO_ID_NBR       2
#define TASK_STABILIZER_ID_NBR  3
#define TASK_ADC_ID_NBR         4
#define TASK_PM_ID_NBR          5
#define TASK_PROXIMITY_ID_NBR   6

// GPIO pin define
#define PIN_LED_YELLOW          23
#define PIN_MPU6050_INT         5
#define PIN_MPU6050_SDA         18
#define PIN_MPU6050_SCL         19
#define PIN_MOTOR_1             2
#define PIN_MOTOR_2             4
#define PIN_MOTOR_3             21
#define PIN_MOTOR_4             22

//Milliseconds to OS Ticks
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

#endif /* MAIN_CONFIG_H_ */
