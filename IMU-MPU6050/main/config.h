#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "freertos/FreeRTOSConfig.h"

#define SENSORS_TASK_PRI        4
#define STABILIZER_TASK_PRI     4

//Milliseconds to OS Ticks
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

#endif /* MAIN_CONFIG_H_ */
