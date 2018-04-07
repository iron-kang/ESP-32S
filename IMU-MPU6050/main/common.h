#ifndef _COMMON_H_
#define _COMMON_H_

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "config.h"
#include "esp_log.h"

enum {
	STATUS_BAT = 0,
	STATUS_IMU,
	STATUS_GPS,
	STATUS_NET
}status_t;

#endif /* MAIN_COMMON_H_ */
