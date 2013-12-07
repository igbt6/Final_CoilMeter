/*
 * keyboard.h
 *
 *  Created on: Dec 5, 2013
 *      Author: lukasz
 */

#ifndef KEYBOARD_H_
#define KEYBOARD_H_

void KeyboardGpioSetup(/*void(*Callback)(uint8_t numRow, bool onORoff)*/void);

typedef enum {
	WAITFORENABLE = 0, ENABLING, MAIN_MENU, MEASUREMENT

} METER_STATE;




#endif /* KEYBOARD_H_ */
