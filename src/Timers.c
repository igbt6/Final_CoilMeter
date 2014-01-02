/*
 * Timers.c
 *
 *  Created on: May 23, 2013
 *      Author: lukasz
 */

#include "Timers.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_cmu.h"

#define TOP_ValueForADC /*10000*/ 9765/*1000Hz*/  //freq of the interrupt is 1024Hz , Crystal: 20MHz  prescaler 2  // it is a frequency of sampling of ADC
#define TOP_ValueForDisplayResults 3255   // freq_osc 20 MHz / presc 1024 / and freq timerUP is about 3 Hz ->   {20MHz/1024/3255 ~= 3}

volatile uint32_t msTicks; /* counts 1ms timeTicks */

///////////////////////////////////////////////////////////the start of configuration of Timer0//////////////////////////////////

void TIMER0forADC_Setup(void) {
	CMU_ClockEnable(cmuClock_TIMER0, true); // Enable clock for TIMER0 module
	// Select TIMER0 parameters
	TIMER_Init_TypeDef timerInit =
			{ .enable = true, .debugRun = true, .prescale = timerPrescale2,
					.clkSel = timerClkSelHFPerClk, .fallAction =
							timerInputActionNone, .riseAction =
							timerInputActionNone, .mode = timerModeUp,
					.dmaClrAct = false, .quadModeX4 = false, .oneShot = false,
					.sync = false, };
	TIMER_IntEnable(TIMER0, TIMER_IF_OF); // Enable overflow interrupt
	NVIC_SetPriority(TIMER0_IRQn, 0);
	NVIC_EnableIRQ(TIMER0_IRQn); // Enable TIMER0 interrupt vector in NVIC
	TIMER_TopSet(TIMER0, TOP_ValueForADC);  //Set TIMER Top value
	TIMER_Init(TIMER0, &timerInit);	// Configure TIMER
}
///////////////////////////////////////////////////////////////////////////////////////
void TIMER0forADC_Disable(void) {
	TIMER_IntDisable(TIMER0, TIMER_IF_OF);
	NVIC_DisableIRQ(TIMER0_IRQn);
}

///////////////////////////////////////////////////////////////////////////////////////
void Timer1forDisplayResults_Setup(void) {
	CMU_ClockEnable(cmuClock_TIMER1, true); // Enable clock for TIMER0 module
	// Select TIMER0 parameters
	TIMER_Init_TypeDef timerInit =
			{ .enable = true, .debugRun = true, .prescale = timerPrescale1024,
					.clkSel = timerClkSelHFPerClk, .fallAction =
							timerInputActionNone, .riseAction =
							timerInputActionNone, .mode = timerModeUp,
					.dmaClrAct = false, .quadModeX4 = false, .oneShot = false,
					.sync = false, };
	TIMER_IntEnable(TIMER1, TIMER_IF_OF); // Enable overflow interrupt
	NVIC_SetPriority(TIMER1_IRQn, 3);
	NVIC_EnableIRQ(TIMER1_IRQn); // Enable TIMER1 interrupt vector in NVIC
	TIMER_TopSet(TIMER1, TOP_ValueForDisplayResults);  //Set TIMER Top value
	TIMER_Init(TIMER1, &timerInit);	// Configure TIMER
	//INFO!!! - An Interrupt Handler is declared in main.c
}

///////////////////////////////////////////////////////////////////////////////////////
void Timer1forDisplayResults_Disable(void) {
	TIMER_IntDisable(TIMER1, TIMER_IF_OF);
	NVIC_DisableIRQ(TIMER1_IRQn);
}




///////////////////////////////////////////////////////////////////////////////////////
//   Configure Timer for the ADC using during measurement of baterry level.          //
///////////////////////////////////////////////////////////////////////////////////////
void Timer2forInternalADCSampling_Setup(void)
{
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  timerInit.prescale = timerPrescale128;  //14 Mhz/128/topValue is 0xFFFF so a measurement is invoked roughly every 1.2s
  CMU_ClockEnable(cmuClock_TIMER2, true);
  TIMER_Init(TIMER2, &timerInit);
}


void Timer2forInternalADCSampling_Disable(void){
	TIMER_Enable(TIMER2, false);
	}



/////////////////////////////////////////////////////////
void init1msSystick(void) {
	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
		while (1)
			;
}
////////////////Interrupt Handler//////////////////////////
void SysTick_Handler(void) {
	msTicks++; /* increment counter necessary in Delay()*/
}

/////////////////////////////////////////////////////////
void Delay(uint32_t dlyTicks) {
	uint32_t currentTicks;
	currentTicks = msTicks;
	while ((msTicks - currentTicks) < dlyTicks)
		;
}

///////////////////////////////////////////////////////////the end of configuration of Timer0//////////////////////////////////

