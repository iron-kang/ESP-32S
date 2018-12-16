/*
 * config.h
 *
 *  Created on: Dec 5, 2018
 *      Author: rws205
 */

#ifndef MAIN_CONFIG_H_
#define MAIN_CONFIG_H_

#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "string.h"

#define NETWORK_TASK_PRI        3
#define STABILIZER_TASK_PRI     2

enum {
	J1 = 0,
	J2,
	J3,
	J4,
	J5,
	J6
};

#endif /* MAIN_CONFIG_H_ */
