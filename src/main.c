/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO
 **                Energy Micro peripheral module library for
 **                "EFM32" microcontrollers
 **
 **          Author : andrzej  z  Texas!
 **
 *****************************************************************************
 */

/* Includes */
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "splc501c.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_lcd.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "ADS7835E.h"
#include "Timers.h"
#include "BTM222.h"
#include "goertzel.h"
#include "keyboard.h"
#include "bitmaps.h"
///////////////////////////////////////////// DEFINE'y KOMPILACJI WARUNKOWEJ//////////////////////////////////////////////
#define DEBUG 1           // dla debugerra- for testing
#define COMPILATION 1   // 1 sprzetowa obsl USART, 0 programowa
#define SAMPLING_TIMER0 1 // jesli probkuje w przerwaniu od timera to 1
#define	SIGN_TEST 0

void showWhereIam(uint8_t numberMenuRow, bool onORoff);
/* OPIS DO PODL ADC TAM GDZIE SWITCHE
 //	PE12	left - clk
 //	PE14	right- conv
 //	PE13	OK- data
 *
 */
static void GLCD_Enable(void) {

	GPIO ->P[0].MODEH = (GPIO ->P[0].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
			| GPIO_P_MODEH_MODE13_PUSHPULL;
	GPIO_PinOutClear(gpioPortA, 13);
}

static void GLCD_Disable(void) {

	GPIO ->P[0].MODEH = (GPIO ->P[0].MODEH & ~_GPIO_P_MODEH_MODE13_MASK)
			| GPIO_P_MODEH_MODE13_PUSHPULL;
	GPIO_PinOutSet(gpioPortA, 13);
}

static void Set20MHzFrequency_Init(void) {
	/* Use crystal oscillator for HFXO */
	CMU ->CTRL |= CMU_CTRL_HFXOMODE_XTAL;
	/* HFXO setup */CMU ->CTRL = (CMU ->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK)
			| CMU_CTRL_HFXOBOOST_50PCENT;

	/* Enable HFXO as high frequency clock, HFCLK */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

}
//volatile METER_STATE STATE = WAITFORENABLE; // extern global enym type

volatile states State;
///////////////////////////////////////////////////////////////////////////////////////////////////

void Delay(uint32_t dlyTicks);

volatile uint32_t msTicks; /* counts 1ms timeTicks */

///////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
char receivexBuffer[10];
Results results;

/* Clearing the receive buffers */

//extern volatile struct circularBuffer;
bool endOfADCInterrupt;
//int i = 12; // liczba bitow
uint16_t FRAME = 0;
bool endOfADCInterrupt = false;
int licznik_kwiecinskigo = 0;
char buf[10];
/////////////////////////13//////////////////////////////////////////////////////////////////////////////
// value of rms
int main(void) {
	CHIP_Init();
	CircularBufferADC_Result ADC_RESULT;
	ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); // 100 samples /sampling freuency is 100Hz
	/*
	 #define SPLC501C_PAGE_BLINKING_MODE	0xD5
	 #define SPLC501C_PAGE_BLINKING_0	0x01
	 #define SPLC501C_PAGE_BLINKING_1	0x02
	 #define SPLC501C_PAGE_BLINKING_2	0x04
	 #define SPLC501C_PAGE_BLINKING_3	0x08
	 #define SPLC501C_PAGE_BLINKING_4	0x10
	 #define SPLC501C_PAGE_BLINKING_5	0x20
	 #define SPLC501C_PAGE_BLINKING_6	0x40
	 #define SPLC501C_PAGE_BLINKING_7	0x80

	 */
	KeyboardGpioSetup(/*showWhereIam*/); // init keyboard , external events
	State.MODE = WAITFORENABLE; // start
	while (1) {
		switch (State.MODE) {

		case WAITFORENABLE:
			GLCD_Disable();
			EMU_EnterEM2(true);   // deep sleep mode

			break;

		case ENABLING:
			/* Setup SysTick Timer for 1 msec interrupts  */
			if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
				while (1)
					;
			GLCD_Enable();
			GLCD_Init();
			GLCD_ClearScreen();
			GLCD_GoTo(40, 3);
			GLCD_WriteString("LOADING...");
			Delay(700);
			State.MODE = MAIN_MENU;

			///	Set20MHzFrequency_Init();
			////	SPI2_setupRXInt_SW(&FRAME);        ----Pomiar
			//////	BTM222_Init();
			//		GLCD_WriteCommand(SPLC501C_PAGE_BLINKING_MODE);
//GLCD_WriteCommand(SPLC501C_PAGE_BLINKING_2);

			char respBuf1[10];

			break;

		case MAIN_MENU:
			if (!State.init) {
				GLCD_ClearScreen();
				GLCD_bmp(main_menu);
				GLCD_GoTo(1, 1);
				/*    licznik_kwiecinskigo++;
				 snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
				 GLCD_WriteString(buf);
				 */
				State.init = true;
			}
			showWhereIam(State.MAIN_MENU_CURRENT_OPTION, State.rtcFlag);

			break;

		case MAIN_MENU_OPTION:

			//GLCD_GoTo(1, 1);
			/*    licznik_kwiecinskigo++;
			 snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
			 GLCD_WriteString(buf);
			 */

			switch (State.MAIN_MENU_CURRENT_OPTION) {

			case START:
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(measure);
					Set20MHzFrequency_Init();
					SPI2_setupRXInt_SW(&FRAME);
					//BTM222_Init();
					State.init = true;
				}
				static uint16_t BTMcounter;
				char bufoo[7]; // receive buffer

				//BTM222_ReadData(respBuf1);
				if (endOfADCInterrupt) {
					endOfADCInterrupt = false;

					ResultADC_Buf_Write(&ADC_RESULT, FRAME);
					if (ADC_RESULT.Buf_isFull) {
						BTMcounter++;
						if (BTMcounter == 1000) {

							results.avg = avg(&ADC_RESULT);
							results.max = max(&ADC_RESULT);
							results.min = min(&ADC_RESULT);
							ConvertDOUBLEtoLCD(results.max, bufoo);
							GLCD_GoTo(50, 4);
							GLCD_WriteString(bufoo);
							ConvertDOUBLEtoLCD(results.min, bufoo);
							GLCD_GoTo(50, 3);
							GLCD_WriteString(bufoo);
							ConvertDOUBLEtoLCD(results.avg, bufoo);
							GLCD_GoTo(50, 5);
							GLCD_WriteString(bufoo);
							BTMcounter = 0;
						}
						InitGoertzel();
						double goertzel = doGoertzelAlgorithm(&ADC_RESULT); // for tests
						ConvertDOUBLEtoLCD(goertzel, bufoo);
						GLCD_GoTo(50, 6);
						GLCD_WriteString(bufoo);
						results.rms = rms(&ADC_RESULT);
						ConvertDOUBLEtoLCD(results.rms, bufoo);
						GLCD_GoTo(50, 2);
						GLCD_WriteString(bufoo);

						//BTM222_SendData(ParseDataToSendThroughBTM(bufoo,'r'));
					}
				}
				//EMU_EnterEM1();

				break;
			case SETTINGS:
				break;
			case BT_STATE:
				break;
			case BAT_LEVEL:
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(battery);
					State.init = true;
				}
				GLCD_GoTo(50, 3);
				GLCD_WriteString("YES");
				GLCD_GoTo(50, 5);
				GLCD_WriteString("91 %");
				break;
			case SD_CARD:
				break;
			case CALIBRATION:
				break;
			default:
				break;
			}
			EMU_EnterEM1();
			break;

		}
	}

}

void SysTick_Handler(void) {
	msTicks++; /* increment counter necessary in Delay()*/
}

/////////////////////////////////////////////////////////
void Delay(uint32_t dlyTicks) {
	uint32_t currentTicks;
	currentTicks = msTicks;
	while ((msTicks - currentTicks) < dlyTicks)
		;
}
/////////////////////////////////////////////////////////
void showWhereIam(uint8_t numberMenuRow, bool onORoff) {
	if (numberMenuRow > NUMBER_OF_OPTIONS)
		return;
	static uint8_t lastNumberRow;

	if (lastNumberRow != numberMenuRow) {
		for (uint8_t x = 0; x < 17; x++) {
			for (uint8_t y = 0; y < 5; y++) {
				GLCD_SetPixel(x + 2, y + 17 + 8 * lastNumberRow, true);
				GLCD_SetPixel(x + 113, y + 17 + 8 * lastNumberRow, true);
			}
		}
	}
	for (uint8_t x = 0; x < 17; x++) {
		for (uint8_t y = 0; y < 5; y++) {
			GLCD_SetPixel(x + 2, y + 17 + 8 * numberMenuRow,
					onORoff ? true : false);
			GLCD_SetPixel(x + 113, y + 17 + 8 * numberMenuRow,
					onORoff ? true : false);
		}
	}

	lastNumberRow = numberMenuRow;

}

