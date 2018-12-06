#ifndef _NETWORK_H_
#define _NETWORK_H_


typedef struct _cmd {
	void (*action)();
	char header;
}Action;

void Network_Init();

#endif
