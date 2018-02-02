#ifndef _BUS_H_
#define _BUS_H_

#include "esp32_i2c.h"

typedef struct _bus {
	I2C_CONFIG i2c;
}Bus;

void Bus_Init(Bus *bus);

#endif
