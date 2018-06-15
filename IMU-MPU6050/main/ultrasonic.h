#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include "common.h"

#define header_H    0x55 //Header
#define header_L    0xAA //Header
#define device_Addr 0x11 //Address
#define data_Length 0x00 //Data length
#define get_Dis_CMD 0x02 //Command: Read Distance
#define checksum    (header_H+header_L+device_Addr+data_Length+get_Dis_CMD) //Checksum

void Ultrasonic_Init();
bool Ultrasonic_GetDistance(unsigned int *dist);

#endif
