#include "main.h"


/*****************************************************************************************/
/*
 * Sleep functions
 */

/***************************************************************************
 * This is the function which blocks the sleep mode to the minmode set
 * Eg. If minmode is EM2, the device cannot go below EM2
 */
/**************************************************************************************/
/*
 * This function is used for unblocking the minimum sleep level that a peripheral needs.
 * This is an atomic function as it should not be interrupted when being executed
 * Input variables: minimumMode
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void blockSleepMode(sleepstate_t minimumMode)
{
	INT_Disable();
	sleep_block_counter[minimumMode]++;
	INT_Enable();
}
/*
 * This is the function which unblocks the sleep mode to the minmode set
 * Eg. If minmode is EM2, the device cannot go below EM2
 */
/**************************************************************************************/
/*
 * This function is used for unblocking the minimum sleep level that a peripheral needs
 * Input variables: minimumMode
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void unblockSleepMode(sleepstate_t minimumMode)
{
	INT_Disable();
	if(sleep_block_counter[minimumMode] > 0)
	{
		sleep_block_counter[minimumMode]--;
	}
	INT_Enable();
}
/***************************************************************************/
/**************************************************************************************/
/*
 * This function contains the sleep routine to put the board to sleep . This routine
 * takes the board to the minimum sleep mode possible
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void sleep(void)
{
	if (sleep_block_counter[0] > 0)
	{
		return; 						// BlockedeveryingbelowEM0
	}
	else if(sleep_block_counter[1] > 0)
	{
		EMU_EnterEM1();					// BlockedeveryithingbelowEM1, enterEM1
	}
	else if(sleep_block_counter[2] > 0)
	{
		EMU_EnterEM2(true);				// BlockedeverythingbelowEM2, enterEM2
	}
	else if(sleep_block_counter[3] > 0)
	{
		EMU_EnterEM3(true);				// Block every thing below EM3, enterEM3
	}
	else
	{
		EMU_EnterEM3(true);				// Nothingisblocked, enterEM3  as EM4 is dangerous
	}
	return;

}
/***************************************************************************/
/*
 * Peripheral function
 */

/**************************************************************************************/
/*
 * This function initializes the GPIO LEDS
 * Input variables: None required
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

void GPIO_LedsInit(void)
{
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(gpioPortE,2, gpioModePushPull, 0);			//Initialize the LED0
  GPIO_PinModeSet(gpioPortE,3, gpioModePushPull, 0);			//Initialize the LED1

}
/************************************************************************************/
/**************************************************************************************/
/*
 * This function turns on the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void Turn_on_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutSet(gpioPortE,2);
	else
		GPIO_PinOutSet(gpioPortE,3);
}

/**************************************************************************************/
/*
 * This function turns off the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void Turn_off_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutClear(gpioPortE,2);		//LED0 is on portE pin2
	else
		GPIO_PinOutClear(gpioPortE,3);		//LED1 is on portE pin3
}
/**************************************************************************************/
/*
 * This function toggles the LED that is specified
 * Input variables: LED_no
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void Toggle_LED(uint8_t LED_no)
{
	if(LED_no==0)
		GPIO_PinOutToggle(gpioPortE,2);
	else
		GPIO_PinOutToggle(gpioPortE,3);
}
/**************************************************************************************/
/*
 * This function initializes the Low Energy Timer(LETIMER)
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void LETIMER0_Init(void)
{
	LETIMER_Init_TypeDef my_letimer;
	my_letimer.enable=true;
	my_letimer.debugRun=false;
	my_letimer.rtcComp0Enable=false;
	my_letimer.rtcComp1Enable=false;
	my_letimer.comp0Top=true;
	my_letimer.bufTop=false;
	my_letimer.out0Pol=0;
	my_letimer.out1Pol=0;
	my_letimer.ufoa0=letimerUFOANone;
	my_letimer.ufoa1=letimerUFOANone;
	my_letimer.repMode=letimerRepeatFree;

	LETIMER_Init(LETIMER0, &my_letimer);
	CMU->LFAPRESC0=0;				//Prescalar to the clock

	int freq_in_hz,led_time_count,led_time_duty_on;
	freq_in_hz=CMU_ClockFreqGet(cmuClock_LFA);
	led_time_count=freq_in_hz*ON_DUTY_CYCLE;
	led_time_duty_on=freq_in_hz*LED0_ON_TIME;


	LETIMER_CompareSet(LETIMER0, 0, led_time_count);
	LETIMER_CompareSet(LETIMER0, 1, led_time_duty_on);

	//Interrupts part
	LETIMER0->IFC=0xFFFF ;					//Clear all interrupts
	LETIMER0->IEN|=LETIMER_IEN_COMP0;
	LETIMER0->IEN|=LETIMER_IEN_COMP1;

	NVIC_EnableIRQ(LETIMER0_IRQn);
	LETIMER_Enable(LETIMER0, true);		//Start the timer
}
/****************************************************************************************/
void ACMP0_my_Init(void)
{
	ACMP_Init_TypeDef my_acmp0;
	GPIO_PinModeSet(gpioPortC,6, gpioModeDisabled, 0);			//Initialize the Light sense
	GPIO_PinModeSet(gpioPortD,6, gpioModeDisabled, 0);			//Initialize the Light excite

	ACMP0->INPUTSEL=0x02<<_ACMP_INPUTSEL_VDDLEVEL_SHIFT;
	//ACMP_GPIOSetup()
	//Not sure about the positive value.. go thru data sheet
	ACMP_ChannelSet(ACMP0,acmpChannelVDD,acmpChannel6);

	my_acmp0.biasProg=0x7;		/* biasProg */
	my_acmp0.enable=true;		//Enable after init
	my_acmp0.fullBias=false;		//default is false higher the bias the faster the comparison
	my_acmp0.halfBias=false;	//Lower bias better for power
	my_acmp0.hysteresisLevel=acmpHysteresisLevel1;		//Higher the better..default is 5
	my_acmp0.inactiveValue=false;
	my_acmp0.interruptOnFallingEdge=false;
	my_acmp0.interruptOnRisingEdge=false;
	my_acmp0.lowPowerReferenceEnabled=false;	/* Disabled emitting inactive value during warmup. */
	my_acmp0.vddLevel=0x3D;				/* VDD level */
	my_acmp0.warmTime=acmpWarmTime512;		/* 512 cycle warmup to be safe */

	ACMP_Init(ACMP0, &my_acmp0);

	//Interrupts part
	ACMP0->IFC=0xFFFF ;					//Clear all interrupts
	ACMP0->IEN=ACMP_IEN_WARMUP | ACMP_IEN_EDGE;

	blockSleepMode(EM2);		//Minimum is EM3 possible

	//ACMP0->CTRL=ACMP_CTRL_EN;
	NVIC_EnableIRQ(ACMP0_IRQn);

	/*
	 * Light sense: Output from the photodiode is connected to PC6 and ACMP0 Channel 6
	 * The excite pin is input to the photodiode and is connected to the PD6 Something LES_ALTEX0
	 */

}
void timer0_calibration(void)
{
	/*Run the timer 0 for 1 second and based on that value set the ulfrc0 calibration
	 *
	 */
	TIMER_Init_TypeDef my_timer0_settings;
	my_timer0_settings.enable=true;
	my_timer0_settings.debugRun=true;
	my_timer0_settings.prescale=timerPrescale32;
	my_timer0_settings.clkSel=timerClkSelHFPerClk;
	my_timer0_settings.count2x=false;
	my_timer0_settings.ati=false;
	my_timer0_settings.fallAction=timerInputActionNone;
	my_timer0_settings.riseAction=timerInputActionNone;
	my_timer0_settings.mode=timerModeUp;
	my_timer0_settings.dmaClrAct=false;
	my_timer0_settings.quadModeX4=false;
	my_timer0_settings.oneShot=false;			//Proff says keep true
	my_timer0_settings.sync=false;

	TIMER_Init(TIMER0,&my_timer0_settings);

	TIMER_InitCC_TypeDef my_timer0_channel;
	my_timer0_channel.eventCtrl=timerEventEveryEdge;
	my_timer0_channel.edge=timerEdgeRising;
	my_timer0_channel.prsSel=timerPRSSELCh0;
	my_timer0_channel.cufoa=timerOutputActionNone;
	my_timer0_channel.cofoa=timerOutputActionNone;
	my_timer0_channel.cmoa=timerOutputActionNone;
	my_timer0_channel.mode=timerCCModeOff;
	my_timer0_channel.filter=false;
	my_timer0_channel.prsInput=false;
	my_timer0_channel.coist=false;
	my_timer0_channel.outInvert=false;

	TIMER_InitCC(TIMER0,1,&my_timer0_channel);

	//You may have to select a channel
	//Interrupts part
	TIMER0->IFC=0xFFFF ;					//Clear all interrupts
	TIMER0->IEN=TIMER_IEN_OF;				//Check for overflow

	NVIC_EnableIRQ(TIMER0_IRQn);
	//You may have to do a TIMER0Enable ()
	//Since TImer0->CNT is not set, it counts to the maximum value and then overflows
}
/*******************************************************************************************/
/*
 * Interrupt handler routines
 */
/**************************************************************************************/
/*
 * This is the LETIMER interrupt handler routine
 * This is an atomic function as it should not be interrupted when being executed
 * Input variables: None required
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void LETIMER0_IRQHandler(void)
{

	int intFlags;
	INT_Disable();								//Make routine atomic
	intFlags=LETIMER_IntGet(LETIMER0);
	LETIMER_IntClear(LETIMER0,intFlags);
	Toggle_LED(0);								//Toggle the Led
	INT_Enable();
}

/**************************************************************************************/
void ACMP0_IRQHandler(void)
{
	int intFlags;
	INT_Disable();								//Make routine atomic
	intFlags=ACMP0->IF;
	Toggle_LED(1);
	if(intFlags==ACMP_IF_EDGE)
	{
		ACMP0->IFC|=ACMP_IFC_EDGE ;				//Clear the interrupt
												//Toggle the Led
	}
	else if(intFlags==ACMP_IF_WARMUP)
	{
		ACMP0->IFC|=ACMP_IFC_WARMUP ;		//Clear the interrupt
		Turn_on_LED(0);
	}
	INT_Enable();
}
/*************************************************************************************/
void TIMER0_IRQHandler(void)
{

	int intFlags;
	INT_Disable();								//Make routine atomic
	intFlags=TIMER_IntGet(TIMER0);
	TIMER_IntClear(TIMER0,intFlags);
	//ACMP0->IFC=0xFFFF;

	Toggle_LED(1);								//Toggle the Led
	//Toggle_LED(0);
	INT_Enable();
}

/**************************************************************************************/
/*
 * This function initializes the clock based on sleep mode
 * Input variables: None required
 * Global Variables: sleep_block_counter
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
void Clock_setup(void)
{
	if(sleep_block_counter[3] > 0)
			{
				CMU_OscillatorEnable(cmuOsc_ULFRCO,true, true);		//use ULFRCO for EM3
				CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_ULFRCO);
				CMU_ClockEnable(cmuClock_CORELE, true);		//Supposed to be CORELE. Do a search in the enum part to know
				CMU_ClockEnable(cmuClock_LETIMER0,true);
				CMU_ClockEnable(cmuClock_ACMP0,true);
				CMU_ClockEnable(cmuClock_TIMER0,true);
				blockSleepMode(EM3);
			}
		else
			{
				CMU_OscillatorEnable(cmuOsc_LFXO,true, true);		//use ULFRCO for EM3
				CMU_OscillatorEnable(cmuOsc_HFRCO,true, true);		//use ULFRCO for EM3
				CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
				CMU_ClockEnable(cmuClock_CORELE, true);		//Supposed to be CORELE. Do a search in the enum part to know
				CMU_ClockEnable(cmuClock_LETIMER0,true);
				CMU_ClockEnable(cmuClock_ACMP0,true);
				CMU_ClockEnable(cmuClock_TIMER0,true);
				blockSleepMode(EM0);
			}

}


/************************************************************************/
/*
 * Main function
 */
/**************************************************************************************/
/* This is the main function. The LED is turned on by default as the LED is toggled in the
 * interrupt routine
 * Input variables: None required
 * Global Variables: None required
 * Output Variables: None required
 ********************************************************************************
 * @section License
 * <b>Copyright 2016 Silicon Laboratories, Inc. http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

int main(void)
{
  /* Chip errata */
  CHIP_Init();

  GPIO_LedsInit();
  blockSleepMode(LOWEST_ENERGY_MODE);			//Set the default lowest energy mode
   Clock_setup();
   // ACMP0_my_Init();
  //LETIMER0_Init();
   //Turn_on_LED(0);
   timer0_calibration();
  								//Turn on the LED as it will start toggling in the Interrupt routine

     while(1)
     {
   	  //sleep();									//Keep putting the board to sleep
     }

  /* Infinite loop */
  while (1) {
  }
}
