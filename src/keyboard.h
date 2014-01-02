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


#define STATES_INIT_DEFAULT                                                 \
  { .MODE=WAITFORENABLE,      /* MAIN_MENU_OPTIONS  */                      \
    .MAIN_MENU_CURRENT_OPTION=START,              /* MAIN_MENU_OPTIONS */   \
    .SETTINGS_CURRENT_OPTION=SLAVE_MODE,         /*	SETTINGS_OPTIONS  */    \
    .rtcFlag=false,              /* rtc flag */                             \
    .init=false,              /* state init flag*/                          \
    .deepSleepTime=600,             /* time to disable */           		\
    .LAST_MODE=WAITFORENABLE,       /* LAST_MODE */                         \
    .activeFunction = CHOSEN_FUNCTIONS_INIT_DEFAULT                         \
  }

#define CHOSEN_FUNCTIONS_INIT_DEFAULT                                       \
  { .settings_menu =false,  /* settings_menu_on */                          \
    .isMeasurementOn =false  /* measurement is taking place */              \
  }


void KeyboardGpioSetup(/*void(*Callback)(uint8_t numRow, bool onORoff)*/void);

typedef enum {
	WAITFORENABLE = 0, ENABLING, MAIN_MENU, MAIN_MENU_OPTION, BATTERY_ALARM,SLEEP_MODE

} METER_STATE;

typedef enum {
	START=0, SETTINGS, BT_STATE, BAT_LEVEL, SD_CARD, CALIBRATION,NUMBER_OF_OPTIONS
} MAIN_MENU_OPTIONS;

typedef enum {
	SLAVE_MODE=0, SET_TIME, SLEEP_TIME, LCD_BRIGHTNESS ,NUMBER_OF_SETTINGS_OPTIONS
} SETTINGS_OPTIONS;

struct chosenFunctions{uint8_t settings_menu:1; // flaga used to indicate that we chose any settings menu
					   uint8_t isMeasurementOn:1;
					   uint8_t isBatteryLevelVerificated:1;};

typedef struct{

	METER_STATE MODE;
	MAIN_MENU_OPTIONS MAIN_MENU_CURRENT_OPTION;
	SETTINGS_OPTIONS SETTINGS_CURRENT_OPTION;
	bool rtcFlag;
	bool init; // flaga to indicate init functions after went to the given state
	uint16_t deepSleepTime; // Time that indicates a time to go to the sleep mode
	uint8_t LAST_MODE;  // indicates which mode was after went into the current one
	struct chosenFunctions activeFunction;  // contains a lot of useful flags to indicate current mode
}states;


extern volatile states State; // stany dla wszystkich plikow

#endif /* KEYBOARD_H_ */
