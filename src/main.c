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

uint16_t transmitBuffer[] ={12345};
#define            BUFFERSIZE    (sizeof(transmitBuffer) / sizeof(uint16_t))
volatile uint16_t receiveBuffer[BUFFERSIZE];

static void SPIinit(void)
{
  /* Enabling clock to USART 1 and 2*/

  CMU_ClockEnable(cmuClock_USART2, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Setup UART */
  SPI_setup();
  SPI2_setupRXInt(receiveBuffer, BUFFERSIZE);
}


static void Set20MHzFrequency_Init(void)
{
  /* Use crystal oscillator for HFXO */
  CMU->CTRL |= CMU_CTRL_HFXOMODE_XTAL;
  /* HFXO setup */
  CMU->CTRL    = (CMU->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK) | CMU_CTRL_HFXOBOOST_50PCENT;

  /* Enable HFXO as high frequency clock, HFCLK */
  CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFXO);

}


///////////////////////////////////////////////////////////////////////////////////////////////////
#define ADC_ZE_SWITCHY 1   //JAK 1 to ze switchy jak nie to znaczy ze normalnie z KONWERTERA

#if DEBUG
void gpioEXTInputSetup(void);
#endif
void USART2_setup(void);
void eADesigner_Init(void);
void Delay(uint32_t dlyTicks);
//void USART2_sendBuffer(char* txBuffer, int bytesToSend);
volatile uint32_t msTicks; /* counts 1ms timeTicks */

/* Buffers */
//circularBuffer rxBuf,
//txBuf; // externowe
///////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
char receivexBuffer[10];
double RMS_Value;
#if SAMPLING_TIMER0==1
/* Clearing the receive buffers */
char bufoo[7];

//extern volatile struct circularBuffer;

//int i = 12; // liczba bitow
volatile uint16_t FRAME = 0;
volatile bool TheEndofInterruptTImer0 = false;
volatile uint16_t DividerOfaTopValue = 1;
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////
// value of rms
int main(void) {
	/* Initialize chip */

	CHIP_Init();
	CircularBufferADC_Result ADC_RESULT;
	ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); // 64 samples / for tests , a sample freuency is 100Hz



	/* Setup SysTick Timer for 1 msec interrupts  */
	if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
		while (1)
			; // if 0 operation succed // if 1 not hang in here

	//INIT LCD

	GLCD_Init();
	GLCD_ClearScreen();
	GLCD_GoTo(1, 0);

	GLCD_WriteString("JARZEBINA");

	BTM222_Init();

	char respBuf1[10];
	int BTMcounter=0;


	Delay(500);


	Set20MHzFrequency_Init();
	/////SPI2_Init();                                  /*****/
	////SPI2_setupRXInt(receiveBuffer, BUFFERSIZE);    /****/


	CMU ->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

	/* To avoid false start, configure output US2_TX as high on PC2 */GPIO ->P[2].DOUT |=
	(1 << 2);
	/* Pin PC2 is configured to Push-pull */GPIO ->P[2].MODEL =
	(GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
	| GPIO_P_MODEL_MODE2_PUSHPULL;
	/* Pin PC3 is configured to Input enabled */GPIO ->P[2].MODEL =
	(GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE3_MASK)
	| GPIO_P_MODEL_MODE3_INPUT;
	/* Pin PC4 is configured to Push-pull */GPIO ->P[2].MODEL =
	(GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
	| GPIO_P_MODEL_MODE4_PUSHPULL;
	/* To avoid false start, configure output US2_CS as high on PC5 */GPIO ->P[2].DOUT |=
	(1 << 5);
	/* Pin PC5 is configured to Push-pull */GPIO ->P[2].MODEL =
	(GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
	| GPIO_P_MODEL_MODE5_PUSHPULL;


	int licznik_kwiecinskigo = 0;
	char buf[11];
	for (int i = 0; i < 10; i++) {
		buf[i] = 0;
	}
	///TIMER0forADC_Setup();

	while (1) {
		GLCD_GoTo(4, 1);
		licznik_kwiecinskigo++;
		snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
		GLCD_WriteString(buf);
		BTMcounter++;

		//BTM222_ReadData(respBuf1);
		GLCD_GoTo(1, 6);
		GLCD_WriteString(respBuf1);

    // SOFTWARE SPI
		if (TheEndofInterruptTImer0) {
			TheEndofInterruptTImer0 = false;
			ResultADC_Buf_Write(&ADC_RESULT, FRAME);
			if (ADC_RESULT.Buf_isFull) {

				RMS_Value = rms(&ADC_RESULT);
			}

			ConvertDOUBLEtoLCD(RMS_Value, bufoo);
					GLCD_GoTo(1, 5);
					GLCD_WriteString(bufoo);

					int x = ConvertU16_from_ADCToINT(receiveBuffer[0]);
					 ConvertDOUBLEtoLCD((double) x, bufoo);
					GLCD_GoTo(1, 3);
					GLCD_WriteString(bufoo);                //// HARDWARE SPI
				//if(BTMcounter>=1000){BTM222_SendData(bufoo);BTMcounter=0;}
				//	BTM222_SendData(bufoo);
			// Enter EM1
			EMU_EnterEM1();
		}


		/*///HARDWARE SPI
		if (TheEndofInterruptTImer0) {
					TheEndofInterruptTImer0 = false;
					ResultADC_Buf_Write(&ADC_RESULT, receiveBuffer[0]);
					if (ADC_RESULT.Buf_isFull) {

						RMS_Value = rms(&ADC_RESULT);
					}

					ConvertDOUBLEtoLCD(RMS_Value, bufoo);
							GLCD_GoTo(1, 5);
							GLCD_WriteString(bufoo);


					//		BTM222_SendData(bufoo);

					EMU_EnterEM1();
				}
*/


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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************//**
 * Interrupt Service Routine TIMER0 Interrupt Line for sampling of ADC converter
 *****************************************************************************/
void TIMER0_IRQHandler(void) {
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);

	// for TEST TODO

	FRAME = 0;
	int i = 12;

	GPIO ->P[2].DOUTCLR = 1 << 5;
	GPIO ->P[2].DOUTSET = 1 << 4;
	GPIO ->P[2].DOUTCLR = 1 << 4;
	GPIO ->P[2].DOUTSET = 1 << 4;
	GPIO ->P[2].DOUTCLR = 1 << 4;
	while (i) {
		--i;
		GPIO ->P[2].DOUTSET = 1 << 4;
		FRAME |= ((GPIO->P[2].DIN >> 3) & 0x1) << i;
		GPIO ->P[2].DOUTCLR = 1 << 4;
	}
	GPIO ->P[2].DOUTSET |= 1 << 5;


/*
    GPIO ->P[2].DOUTCLR = 1 << 5;
	receiveBuffer[0] =USART_SpiTransfer(USART2,10);
	GPIO ->P[2].DOUTSET = 1 << 5;
*/


	///GPIO ->P[2].DOUTCLR = 1 << 5;

	//////////////////////////////////////////////////////////////////////////////USART2_sendBuffer(transmitBuffer, BUFFERSIZE);
	//GPIO ->P[2].DOUTSET = 1 << 5;
/*
	  USART_TypeDef *spi = USART2;
	  uint16_t       rxdata=0;
	 if (spi->STATUS & USART_STATUS_RXFULL)
	  {
	    // Reading out data
	    rxdata = spi->RXDOUBLE;

	    if (receiveBuffer != 0)
	    {

	    	receiveBuffer[0] = rxdata;
	    }
	  }
		GPIO ->P[2].DOUTSET = 1 << 5;
*/
	TheEndofInterruptTImer0 = true;

}

