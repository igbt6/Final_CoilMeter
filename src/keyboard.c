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
#include "ADS7835E.h"
#include "Timers.h"

// defines for RTC module
#define LFRCO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS            100 // 100 ms interrupt
#define RTC_COUNT_BETWEEN_WAKEUP    (((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)-1)

////////////////////////////// enums that describe a state of following modes

volatile states State;
volatile uint16_t timerCounter;
//////////////////////////
//static uint8_t main_menu_position_tab[6]{};
/**************************************************************************//**
 * @brief  Start LFRCO for RTC
 * Starts the low frequency RC oscillator (LFRCO) and routes it to the RTC
 *****************************************************************************/
static void startLfxoForRtc(void) {

	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
	CMU_ClockEnable(cmuClock_RTC, true);
	CMU_ClockEnable(cmuClock_CORELE, true);
}

/**************************************************************************//**
 * @brief  Setup RTC
 * On compare match with COMP0, clear counter and set interrupt
 *****************************************************************************/
static void setupRtc(void) {

	startLfxoForRtc();
	RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

	rtcInit.enable = true; /* Enable RTC after init has run */
	rtcInit.comp0Top = true; /* Clear counter on compare match */
	rtcInit.debugRun = false; /* Counter shall keep running during debug halt. */

	/* Setting the compare value of the RTC */
	RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);

	/* Enabling Interrupt from RTC */
	RTC_IntEnable(RTC_IFC_COMP0);
	NVIC_SetPriority(RTC_IRQn, 3);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Initialize the RTC */
	RTC_Init(&rtcInit);
}

//static void (*sleepModeCallback)(void); //it is invoked in a interrupt functione in order to go to sllep

void KeyboardGpioSetup(/*void(*Callback)(uint8_t numRow, bool onORoff)*/void) {
	/* Enable GPIO in CMU */

	/////////////////////////////////////////rtcCallback=Callback;
	CMU_ClockEnable(cmuClock_GPIO, true);
	/* Configure OK,DOWN<UP, LEFT, RIGHT  as input */
	GPIO_PinModeSet(gpioPortE, OK, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(gpioPortE, UP, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(gpioPortE, DOWN, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(gpioPortE, RIGHT, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(gpioPortE, LEFT, gpioModeInputPullFilter, 1);
	GPIO_PinModeSet(gpioPortE, BATTERY_LOW, gpioModeInputPullFilter, 1); // info about battery low level

	GPIO_IntConfig(gpioPortE, OK, false, true, true); // switch OK falling edge
	GPIO_IntConfig(gpioPortE, UP, false, true, true); // switch UP falling edge
	GPIO_IntConfig(gpioPortE, DOWN, false, true, true); // switch DOWN falling edge
	GPIO_IntConfig(gpioPortE, RIGHT, false, true, true); // switch RIGHT falling edge
	GPIO_IntConfig(gpioPortE, LEFT, false, true, true); // switch LEFT falling edge
	GPIO_IntConfig(gpioPortE, BATTERY_LOW, false, true, true); // BATTERY LOW rising edge

	NVIC_SetPriority(GPIO_EVEN_IRQn, 1);
	NVIC_SetPriority(GPIO_ODD_IRQn, 1);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn); // set interrupts for EVEN and ODD input signals
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	setupRtc();  // setup RTC
}

void GPIO_ODD_IRQHandler(void) {
	uint32_t flags = GPIO ->IF;
	GPIO ->IFC = 0xFFFFFFFF;
	if (State.MODE == SLEEP_MODE) {
		State.MODE = State.LAST_MODE;
		State.LAST_MODE=SLEEP_MODE;
		flags = 0;
		State.init = false;
	}  // wake up
	timerCounter=0;
	if (flags & (1 << OK)) {                                 	//// OK _ KEY

		if (State.MODE == WAITFORENABLE) {
			State.init = false;
			State.MODE = ENABLING;
		} else if (State.MODE == MAIN_MENU) {
			State.init = false;
			State.MODE = MAIN_MENU_OPTION;
		} else if (State.MODE == MAIN_MENU_OPTION) {

			//State.init = false;
			if (State.MAIN_MENU_CURRENT_OPTION == START) {

				//TODO SAVE NA KARTE
			} else if (State.MAIN_MENU_CURRENT_OPTION == SETTINGS) {
				State.init = false;
				State.activeFunction.settings_menu = true;
			}
		} else if (State.MODE == BATTERY_ALARM) {
			State.init = false;
			State.MODE = MAIN_MENU_OPTION;   // wracam do ostatnio uruchomionego
		}

		//GPIO ->IFC = 1 << OK;
	} else if (flags & (1 << UP)) {                      		//// UP _ KEY
		if (State.MODE == MAIN_MENU) {
			if (State.MAIN_MENU_CURRENT_OPTION == 0)
				State.MAIN_MENU_CURRENT_OPTION = NUMBER_OF_OPTIONS;
			State.MAIN_MENU_CURRENT_OPTION--;
			State.MAIN_MENU_CURRENT_OPTION %= NUMBER_OF_OPTIONS;
		}

		else if (State.MODE == MAIN_MENU_OPTION
				&& State.MAIN_MENU_CURRENT_OPTION == SETTINGS) {
			if (State.SETTINGS_CURRENT_OPTION == 0)
				State.SETTINGS_CURRENT_OPTION = NUMBER_OF_SETTINGS_OPTIONS;
			State.SETTINGS_CURRENT_OPTION--;
			State.SETTINGS_CURRENT_OPTION %= NUMBER_OF_SETTINGS_OPTIONS;
		}

		//GPIO ->IFC = 1 << UP;
	} else if (flags & (1 << BATTERY_LOW)) {          		//// BATTERY _ LOW
		State.MODE = BATTERY_ALARM;
		State.init = false;  // battery alarm occured
	}

}

void GPIO_EVEN_IRQHandler(void) {
	uint32_t flags = GPIO ->IF;
	GPIO ->IFC = 0xFFFFFFFF;

	if (State.MODE == SLEEP_MODE) {
		State.MODE = State.LAST_MODE;
		State.LAST_MODE=SLEEP_MODE;
		flags = 0;
		State.init = false;
	}
	timerCounter=0;  // kasuje sleep mode
	if (flags & (1 << DOWN)) {             					//// DOWN _ KEY

		if (State.MODE == MAIN_MENU) {
			State.MAIN_MENU_CURRENT_OPTION++;
			State.MAIN_MENU_CURRENT_OPTION %= NUMBER_OF_OPTIONS;

		} else if (State.MODE == MAIN_MENU_OPTION // tutaj priorytety wsadzamy czyli dla kolejnego podmenu jeszcze wyzej
		&& State.MAIN_MENU_CURRENT_OPTION == SETTINGS) {
			State.SETTINGS_CURRENT_OPTION++;
			State.SETTINGS_CURRENT_OPTION %= NUMBER_OF_SETTINGS_OPTIONS;
		}

	} else if (flags & (1 << RIGHT)) {       				//// RIGHT _ KEY

	} else if (flags & (1 << LEFT)) {         				//// LEFT  _ KEY

		if (State.MODE == MAIN_MENU_OPTION) {

			if (State.activeFunction.settings_menu == true) { // podmenu settings , dodac 2 strukture z polami bitowymi moze
				State.activeFunction.settings_menu = false;
				State.init = false;
			} else if (State.activeFunction.isMeasurementOn == true) { // podmenu start
				State.activeFunction.isMeasurementOn = false;
				State.init = false;
				Timer1forDisplayResults_Disable(); // turns off all timers bluetooth and others
				SPI2_disableRXInt_SW(); //
				State.MODE = MAIN_MENU;
			}

			else {
				State.init = false;   // flaga init cancel
				State.MODE = MAIN_MENU;
			}
		} else if (State.MODE == BATTERY_ALARM) {
			State.init = false;
			State.MODE = MAIN_MENU_OPTION;   // wracam do ostatnio uruchomionego
		}

	}
}

//RTC INTERRUPTS -> very useful for keyboard delays and other stuff
void RTC_IRQHandler(void) {


	if (State.MODE != SLEEP_MODE) {
		timerCounter++;
	}

	if (!(timerCounter % 3)) {
		State.rtcFlag ^= 1;
	} // menu state indicator blinks every 300ms

	if ((timerCounter / 10) == State.deepSleepTime) {
		if(!(State.activeFunction.isMeasurementOn)){ // if the mesauement is lasting
		State.LAST_MODE = State.MODE;
		State.init=false;
		State.MODE = SLEEP_MODE;
		}
		timerCounter = 0;
	}

////////////if(rtcCallback)(*rtcCallback)(menuPosition,rtcFlag);
	/* Clear interrupt source */

	RTC_IntClear(RTC_IFC_COMP0);
}

