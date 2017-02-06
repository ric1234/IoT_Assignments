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
#define LOWEST_ENERGY_MODE 	EM0			//Set the default minimum Energy mode here
#define LED0_ON_TIME 		0.030		//in seconds
#define ON_DUTY_CYCLE		1.75	    //in seconds

/******************************************************************************************/

typedef enum
{
	EM0=0, EM1, EM2, EM3
}sleepstate_t;

uint8_t sleep_block_counter[4];
/**************************************************************************************/

#endif /* SRC_MAIN_H_ */
