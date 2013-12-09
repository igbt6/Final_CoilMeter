/*
 * keyboard.h
 *
 *  Created on: Dec 5, 2013
 *      Author: lukasz
 */

#ifndef KEYBOARD_H_
#define KEYBOARD_H_
#include "stdbool.h"

// defines keyboard signals
#define OK    13  // PORT E
#define UP    11
#define DOWN  10
#define RIGHT 14
#define LEFT  12
#define BATTERY_LOW 7



void KeyboardGpioSetup(/*void(*Callback)(uint8_t numRow, bool onORoff)*/void);

typedef enum {
	WAITFORENABLE = 0, ENABLING, MAIN_MENU, MAIN_MENU_OPTION, BATTERY_ALARM

} METER_STATE;

typedef enum {
	START=0, SETTINGS, BT_STATE, BAT_LEVEL, SD_CARD, CALIBRATION,NUMBER_OF_OPTIONS
} MAIN_MENU_OPTIONS;

typedef enum {
	SLAVE_MODE=0, SET_TIME, SLEEP_TIME, LCD_BRIGHTNESS ,NUMBER_OF_SETTINGS_OPTIONS
} SETTINGS_OPTIONS;



typedef struct{

	METER_STATE MODE;
	MAIN_MENU_OPTIONS MAIN_MENU_CURRENT_OPTION;
	SETTINGS_OPTIONS SETTINGS_CURRENT_OPTION;
	bool rtcFlag;
	bool init; // flaga to indicate init functions after went to the given state
	bool settings_menu; // flaga used to indicate that we chose any settings menu
}states;


extern volatile states State; // stany dla wszystkich plikow

#endif /* KEYBOARD_H_ */
