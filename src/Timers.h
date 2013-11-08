/*
 * Timers.h
 *
 *  Created on: May 23, 2013
 *      Author: lukasz
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#define TOP_Value 547 /*100Hz*/  //an interrupt appears every one second a period for osc. 14MHZ like here and prescaler is set on 256  // it is a frequency of sampling of ADC
void TIMER0forADC_Setup(void);


#endif /* TIMERS_H_ */
