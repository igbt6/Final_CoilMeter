/*
 * keyboard.h
 *
 *  Created on: Dec 5, 2013
 *      Author: lukasz
 */

#ifndef KEYBOARD_H_
#define KEYBOARD_H_
#include "stdbool.h"

void KeyboardGpioSetup(/*void(*Callback)(uint8_t numRow, bool onORoff)*/void);

typedef enum {
	WAITFORENABLE = 0, ENABLING, MAIN_MENU, MAIN_MENU_OPTION

} METER_STATE;

typedef enum {
	START=0, SETTINGS, BT_STATE, BAT_LEVEL, SD_CARD, CALIBRATION,NUMBER_OF_OPTIONS
} MAIN_MENU_OPTIONS;

typedef struct{

	METER_STATE MODE;
	MAIN_MENU_OPTIONS MAIN_MENU_CURRENT_OPTION;
	bool rtcFlag;
	bool init; // flaga to indicate init functions after went to the given state
}states;


extern volatile states State; // stany dla wszystkich plikow

#endif /* KEYBOARD_H_ */
