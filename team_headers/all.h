/*
 * all.h
 *
 *  Created on: Oct 29, 2013
 *      Author: Nathan
 */

#ifndef ALL_H_
#define ALL_H_

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "stopwatch.h"
#include "op.h"
#include "data.h"
#include "Init.h"
#include "SensorCov.h"
#include "Boot.h"
#include "PowerDown.h"
#include "common.h"
#include "main.h"
#include "clock.h"
#include "can.h"
#include <stdlib.h>
#include "adc.h"
#include "gpio.h"
#include "DSP2803x_GlobalPrototypes.h"
#include "Flash2803x_API_Library.h"

#include "MCP2515_spi.h"
#include "MCP2515_DEFS.h"
#include "MCP2515.h"

extern ops_struct ops;
extern data_struct data;

#define CPU_FREQ_MHZ	60

#endif /* ALL_H_ */
