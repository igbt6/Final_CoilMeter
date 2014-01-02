/*
 * Timers.h
 *
 *  Created on: May 23, 2013
 *      Author: lukasz
 */

#ifndef TIMERS_H_
#define TIMERS_H_


#include "em_timer.h"


void TIMER0forADC_Setup(void);
void TIMER0forADC_Disable(void);

void Timer1forDisplayResults_Setup(void);
void Timer1forDisplayResults_Disable(void);


void Timer2forInternalADCSampling_Setup(void);
void Timer2forInternalADCSampling_Disable(void);


void Delay(uint32_t dlyTicks);
void init1msSystick(void) ;

#endif /* TIMERS_H_ */
