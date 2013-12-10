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

///////////////////////////////////////////////////////////the start of configuration of Timer0//////////////////////////////////

void TIMER0forADC_Setup(void) {
	/* Enable clock for GPIO module */
///////////////////////////////////////////////////////////////////////////////////////
	/* Enable clock for TIMER0 module */
	CMU_ClockEnable(cmuClock_TIMER0, true);

	/* Select TIMER0 parameters */
	TIMER_Init_TypeDef timerInit =
			{ .enable = true, .debugRun = true, .prescale = timerPrescale2,
					.clkSel = timerClkSelHFPerClk, .fallAction =
							timerInputActionNone, .riseAction =
							timerInputActionNone, .mode = timerModeUp,
					.dmaClrAct = false, .quadModeX4 = false, .oneShot = false,
					.sync = false, };

	/* Enable overflow interrupt */
	TIMER_IntEnable(TIMER0, TIMER_IF_OF);

	/* Enable TIMER0 interrupt vector in NVIC */
	NVIC_SetPriority(TIMER0_IRQn, 0);
	NVIC_EnableIRQ(TIMER0_IRQn);

	/* Set TIMER Top value */
	TIMER_TopSet(TIMER0, TOP_Value);

	/* Configure TIMER */
	TIMER_Init(TIMER0, &timerInit);

}

void TIMER0forADC_Disable(void) {
	TIMER_IntDisable(TIMER0, TIMER_IF_OF);
	NVIC_DisableIRQ(TIMER0_IRQn);
}
///////////////////////////////////////////////////////////the end of configuration of Timer0//////////////////////////////////


