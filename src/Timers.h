/*
 * Timers.h
 *
 *  Created on: May 23, 2013
 *      Author: lukasz
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#define TOP_Value 10000 /*1000Hz*/  //freq of the interrupt is 1000Hz , Crystal: 20MHz  prescaler 2  // it is a frequency of sampling of ADC
void TIMER0forADC_Setup(void);
void TIMER0forADC_Disable(void);


#endif /* TIMERS_H_ */
