#ifndef _NETWORK_H_
#define _NETWORK_H_

#include "common.h"
#include "stabilizer_types.h"

typedef struct _cmd {
	void (*action)();
	char header;
}Action;

void Network_Init();

#endif
