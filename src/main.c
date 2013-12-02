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
///////////////////////////////////////////// DEFINE'y KOMPILACJI WARUNKOWEJ//////////////////////////////////////////////
#define DEBUG 1           // dla debugerra- for testing
#define COMPILATION 1   // 1 sprzetowa obsl USART, 0 programowa
#define SAMPLING_TIMER0 1 // jesli probkuje w przerwaniu od timera to 1
#define	SIGN_TEST 0
/* OPIS DO PODL ADC TAM GDZIE SWITCHE
 //	PE12	left - clk
 //	PE14	right- conv
 //	PE13	OK- data
 *
 */

static void Set20MHzFrequency_Init(void) {
	/* Use crystal oscillator for HFXO */
	CMU ->CTRL |= CMU_CTRL_HFXOMODE_XTAL;
	/* HFXO setup */CMU ->CTRL = (CMU ->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK)
			| CMU_CTRL_HFXOBOOST_50PCENT;

	/* Enable HFXO as high frequency clock, HFCLK */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Delay(uint32_t dlyTicks);

volatile uint32_t msTicks; /* counts 1ms timeTicks */

///////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
char receivexBuffer[10];
Results results;

/* Clearing the receive buffers */
char bufoo[7];

//extern volatile struct circularBuffer;
bool endOfADCInterrupt;
//int i = 12; // liczba bitow
uint16_t FRAME = 0;
bool endOfADCInterrupt = false;

///////////////////////////////////////////////////////////////////////////////////////////////////////
// value of rms
int main(void) {
	CHIP_Init();
	CircularBufferADC_Result ADC_RESULT;
	ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); // 100 samples /sampling freuency is 100Hz

	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
		while (1)
			;

	GLCD_Init();
	GLCD_ClearScreen();
	GLCD_GoTo(1, 0);

	GLCD_WriteString("JARZEBINA");

	BTM222_Init();

	char respBuf1[10];
	int BTMcounter = 0;

	Delay(500);

	Set20MHzFrequency_Init();
	SPI2_setupRXInt_SW(&FRAME);

	int licznik_kwiecinskigo = 0;
	char buf[11];
	for (int i = 0; i < 10; i++) {
		buf[i] = 0;
	}

	while (1) {
		GLCD_GoTo(4, 1);
		licznik_kwiecinskigo++;
		snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
		GLCD_WriteString(buf);
		BTMcounter++;

		//BTM222_ReadData(respBuf1);
		GLCD_GoTo(1, 6);
		GLCD_WriteString(respBuf1);

		if (endOfADCInterrupt) {
			endOfADCInterrupt = false;
			ResultADC_Buf_Write(&ADC_RESULT, FRAME);
			if (ADC_RESULT.Buf_isFull) {

				results.rms = rms(&ADC_RESULT);
				results.avg = avg(&ADC_RESULT);
				results.max = max(&ADC_RESULT);
				results.min = min(&ADC_RESULT);

				ConvertDOUBLEtoLCD(results.rms, bufoo);
				GLCD_GoTo(1, 2);
				GLCD_WriteString(bufoo);
				ConvertDOUBLEtoLCD(results.max, bufoo);
				GLCD_GoTo(1, 3);
				GLCD_WriteString(bufoo);
				ConvertDOUBLEtoLCD(results.min, bufoo);
				GLCD_GoTo(1, 4);
				GLCD_WriteString(bufoo);
				ConvertDOUBLEtoLCD(results.avg, bufoo);
				GLCD_GoTo(1, 5);
				GLCD_WriteString(bufoo);
			}
		}
		EMU_EnterEM1();
	}
}

void SysTick_Handler(void) {
	msTicks++; /* increment counter necessary in Delay()*/
}

//////////////////////////////////////////////////
void Delay(uint32_t dlyTicks) {
	uint32_t currentTicks;
	currentTicks = msTicks;
	while ((msTicks - currentTicks) < dlyTicks)
		;
}

