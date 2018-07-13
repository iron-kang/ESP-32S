#ifndef _CONFIG_H_
#define _CONFIG_H_
//ESP32S spec:http://wiki.ai-thinker.com/esp32/spec/esp32s
#include "freertos/FreeRTOSConfig.h"


#define STABILIZER_TASK_PRI     5
#define SENSORS_TASK_PRI        4
#define NETWORK_TASK_PRI        3
#define SYSTEM_TASK_PRI         3
#define BARO_TASK_PRI           2
#define ULTRASONIC_TASK_PRI     2
#define GPS_TASK_PRI            2

#define TASK_LED_ID_NBR         1
#define TASK_RADIO_ID_NBR       2
#define TASK_STABILIZER_ID_NBR  3
#define TASK_ADC_ID_NBR         4
#define TASK_PM_ID_NBR          5
#define TASK_PROXIMITY_ID_NBR   6

// Bus speed
#define IMU_I2C_SPEED    	  	400000
#define GPS_UART_BAUD			9600
#define Ultrasonic_BAUD         19200
#define Telemetry_BAUD          57600

// Bus channel
#define GPS_UART_NUM            UART_NUM_2
#define Ultrasonic_UART_NUM     UART_NUM_1
#define Telemetry_UART_NUM      UART_NUM_1
#define IMU_I2C_NUM             I2C_NUM_1

// GPIO pin define
#define PIN_LED_YELLOW          23
#define PIN_IMU_INT             5
#define PIN_IMU_SDA             18
#define PIN_IMU_SCL             19
#define PIN_GPS_TX				17
#define PIN_GPS_RX				16
#define PIN_MOTOR_1             2
#define PIN_MOTOR_2             4
#define PIN_MOTOR_3             21
#define PIN_MOTOR_4             22
#define PIN_Ultrasonic_TX       13
#define PIN_Ultrasonic_RX       14
#define PIN_Telemetry_TX        12
#define PIN_Telemetry_RX        27

//Milliseconds to OS Ticks
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

#endif /* MAIN_CONFIG_H_ */
