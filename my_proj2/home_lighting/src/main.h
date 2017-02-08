/*
 * main.h
 *
 *  Created on: Feb 4, 2017
 *      Author: richa
 */

#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_acmp.h"
#include "em_timer.h"
/*************************************************************************************/
/*
 * Defines and declarations
 */
#define LOWEST_ENERGY_MODE 	EM2			//Set the default minimum Energy mode here
#define LIGHT_SENSOR_ON_TIME 		0.004		//in seconds
#define ON_DUTY_CYCLE		2.5	    //in seconds

#define ULFRCO_SELF_CALIBRATE	0		//0 is off, 1 is on
#define TIME_TO_SENSE_LIGHT		ON_DUTY_CYCLE

/******************************************************************************************/

typedef enum
{
	EM0=0, EM1, EM2, EM3
}sleepstate_t;

uint8_t sleep_block_counter[4];
/**************************************************************************************/

#endif /* SRC_MAIN_H_ */
