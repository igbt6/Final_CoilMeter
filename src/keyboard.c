/*
 * keyboard.c
 *
 *  Created on: Dec 5, 2013
 *      Author: lukasz
 */
#include "em_cmu.h"
#include "em_gpio.h"
#include "keyboard.h"
#include "em_rtc.h"
#include "stdbool.h"
#define OK    13
#define UP    11
#define DOWN  10
#define RIGHT 14
#define LEFT  12


// defines for RTC module
#define LFRCO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS            500
#define RTC_COUNT_BETWEEN_WAKEUP    (((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)-1)

extern volatile METER_STATE STATE;
extern bool rtcFlag;

extern  uint8_t menuPosition;


/**************************************************************************//**
 * @brief  Start LFRCO for RTC
 * Starts the low frequency RC oscillator (LFRCO) and routes it to the RTC
 *****************************************************************************/
static void startLfxoForRtc(void)
{

  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFRCO);
  CMU_ClockEnable(cmuClock_RTC, true);
  CMU_ClockEnable(cmuClock_CORELE, true);
}



/**************************************************************************//**
 * @brief  Setup RTC
 * On compare match with COMP0, clear counter and set interrupt
 *****************************************************************************/
static void setupRtc(void)
{

  startLfxoForRtc();
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

  rtcInit.enable   = true;      /* Enable RTC after init has run */
  rtcInit.comp0Top = true;      /* Clear counter on compare match */
  rtcInit.debugRun = false;     /* Counter shall keep running during debug halt. */

  /* Setting the compare value of the RTC */
  RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);

  /* Enabling Interrupt from RTC */
  RTC_IntEnable(RTC_IFC_COMP0);
  NVIC_SetPriority(RTC_IRQn,3);
  NVIC_EnableIRQ(RTC_IRQn);

  /* Initialize the RTC */
  RTC_Init(&rtcInit);
}

////static void (*rtcCallback)(uint8_t numRow, bool onORoff);//sluzy do wywolywania odpowiedniej funkcji w przerwaniu od rtc

void KeyboardGpioSetup(/*void(*Callback)(uint8_t numRow, bool onORoff)*/void) {
	/* Enable GPIO in CMU */

	/////////////////////////////////////////rtcCallback=Callback;
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* Configure OK,DOWN<UP, LEFT, RIGHT  as input */
	GPIO_PinModeSet(gpioPortE, OK, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortE, UP, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortE, DOWN, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortE, RIGHT, gpioModeInput, 0);
	GPIO_PinModeSet(gpioPortE, LEFT, gpioModeInput, 0);

	GPIO_IntConfig(gpioPortE, OK, false, true, true); // switch OK falling edge
	GPIO_IntConfig(gpioPortE, UP, false, true, true); // switch UP falling edge
	GPIO_IntConfig(gpioPortE, DOWN, false, true, true); // switch DOWN falling edge
	GPIO_IntConfig(gpioPortE, RIGHT, false, true, true); // switch RIGHT falling edge
	GPIO_IntConfig(gpioPortE, LEFT, false, true, true); // switch LEFT falling edge

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn); // set interrupts for EVEN and ODD input signals
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_SetPriority(GPIO_EVEN_IRQn,1);
	NVIC_SetPriority(GPIO_ODD_IRQn,1);
	setupRtc();  // setup RTC
}

void GPIO_ODD_IRQHandler(void) {
	if (GPIO_IntGet() == 1 << OK) {
		GPIO_IntClear(1 << OK);
		if (STATE == WAITFORENABLE) {
			STATE = ENABLING;
		}
	} else if (GPIO_IntGet() == 1 << UP) {
		GPIO_IntClear(1 << UP);
		if (STATE == MAIN_MENU) {
			if(menuPosition==0)menuPosition=5;
				menuPosition--;
			    menuPosition%=5;

						}
	}
}

void GPIO_EVEN_IRQHandler(void) {
	if (GPIO_IntGet() == 1 << DOWN) {
		GPIO_IntClear(1 << DOWN);

		if (STATE == MAIN_MENU) {
						menuPosition++;
						menuPosition%=5;

							}


	} else if (GPIO_IntGet() == 1 << LEFT) {
		GPIO_IntClear(1 << LEFT);
	} else if (GPIO_IntGet() == 1 << RIGHT) {
		GPIO_IntClear(1 << RIGHT);
	}
}



//RTC INTERRUPTS -> very useful for keyboard delays and other stuff


void RTC_IRQHandler(void)
{  rtcFlag^=1;
////////////if(rtcCallback)(*rtcCallback)(menuPosition,rtcFlag);
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
}


