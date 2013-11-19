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
/*void RTC_init(void) {
 CMU_ClockEnable(cmuClock_RTC, true); // Enable clock to RTC module
 RTC_Init_TypeDef init;
 init.comp0Top = true; // porownanie z wartoscia ustawiona ponizej
 init.enable = true;
 RTC_Init(&init);
 RTC_CompareSet(0, 32768); // wartoæ do prównania , wtedy bede kasowal
 RTC_CounterReset(); // reset i start zegara
 RTC_IntEnable(RTC_IF_COMP0); // wlaczenie przerwan
 //RTC_IntEnable(RTC_IF_OF);
 NVIC_ClearPendingIRQ(RTC_IRQn); // Enable interrupts
 NVIC_EnableIRQ(RTC_IRQn);
 }*/
#if DEBUG
void gpioEXTInputSetup(void);
#endif
void USART2_setup(void);
void eADesigner_Init(void);
void Delay(uint32_t dlyTicks);
void USART2_sendBuffer(char* txBuffer, int bytesToSend);
volatile uint32_t msTicks; /* counts 1ms timeTicks */

/* Buffers */
//circularBuffer rxBuf,
//txBuf; // externowe
///////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
char receiveBuffer[10];
double RMS_Value;
#if SAMPLING_TIMER0==1
/* Clearing the receive buffers */
char bufoo[6];

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

		 // CMU->CTRL |= CMU_CTRL_HFXOMODE_XTAL;
		 // CMU->CTRL    = (CMU->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK) | CMU_CTRL_HFXOBOOST_100PCENT;
		 //CMU_ClockSelectSet(cmuClock_HF,cmuSelect_HFXO);

	CircularBufferADC_Result ADC_RESULT;
	ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); // 64 samples / for tests , a sample freuency is 100Hz
#if SAMPLING_TIMER0==1

	TIMER0forADC_Setup();
#endif
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
	char respBuf2[100];
	Delay(500);
	BTM222_SendData("+++\r");
	BTM222_SendData("ATE0\r");
								Delay(500);
								BTM222_SendData("ATL5\r");
																Delay(500);
	///BTM222_SendData("ATN=CMETER2\r");
	//Delay(500);
	//BTM222_SendData("ATN?\r");
	//Delay(500);
	//BTM222_SendData("ATI1\r");
//	BTM222_SendData("ATR0\r");
//	Delay(500);
//	BTM222_SendData("ATO1\r");
//	Delay(500);
	//BTM222_SendData("ATF?\r");
	//Delay(60000);

//	while(respBuf2[0]!='I'){
//		BTM222_ReadData(respBuf2);

//	}


	//BTM222_SendData("ATA1\r");
	//Delay(1000);
//	BTM222_SendData("+++\r");
//	Delay(500);
//	BTM222_SendData("ATI2\r");
//	Delay(500);

	//BTM222_SendData("ATO\r");
	Delay(500);
	/*BTM222_ReadData(respBuf2);
	GLCD_GoTo(0, 2);
	GLCD_WriteString(respBuf2);
	GLCD_GoTo(0, 3);
		GLCD_WriteString(respBuf2+20);
		GLCD_GoTo(0, 4);
			GLCD_WriteString(respBuf2+40);
			GLCD_GoTo(0, 5);
					GLCD_WriteString(respBuf2+60);
					GLCD_GoTo(0, 6);
							GLCD_WriteString(respBuf2+80);
*/

							//BTM222_SendData("ATI2\r");
								//Delay(500);
	///eADesigner_Init();



	//SPI_setup();
	//SPI2_setupRXInt(receiveBuffer, 1);


#if(1)
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

	#endif




#if SAMPLING_TIMER0==0
	/* Clearing the receive buffers */
	char bufoo[6];

	int i = 12; // liczba bitow
	uint16_t FRAME = 0;
#endif
	int licznik_kwiecinskigo = 0;
	char buf[10];
	for (int i = 0; i < 10; i++) {
		buf[i] = 0;
	}
	while (1) {
		GLCD_GoTo(4, 1);
		licznik_kwiecinskigo++;
		snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
		GLCD_WriteString(buf);
		BTM222_ReadData(respBuf1);
		GLCD_GoTo(1, 6);
		GLCD_WriteString(respBuf1);

// one time per second
Delay(900);
	BTM222_SendData("TEST"/*bufoo*/);
	ConvertDOUBLEtoLCD(RMS_Value, bufoo);
				GLCD_GoTo(1, 5);
				GLCD_WriteString(bufoo);


		if (TheEndofInterruptTImer0) {
			TheEndofInterruptTImer0 = false;
			ResultADC_Buf_Write(&ADC_RESULT, FRAME);
			if (ADC_RESULT.Buf_isFull) {

				RMS_Value = rms(&ADC_RESULT);



			}
			/*ConvertDOUBLEtoLCD(RMS_Value, bufoo);
			GLCD_GoTo(1, 5);
			GLCD_WriteString(bufoo);
			*/


			// Enter EM1
			//EMU_EnterEM1();
			}

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

/******************************************************************************
 * @brief sends data using USART2
 * @param txBuffer points to data to transmit
 * @param bytesToSend bytes will be sent
 *****************************************************************************/
void USART2_sendBuffer(char* txBuffer, int bytesToSend) {
	USART_TypeDef *uart = USART2;

	uart->TXDOUBLE = 12445;
	/* Sending the data */
	/*	int ii; for (ii = 0; ii < bytesToSend; ii++) {
	 Waiting for the usart to be ready
	 while (!(uart->STATUS & USART_STATUS_TXBL))   //Indicates the level of the transmit buffer. If TXBIL is cleared, TXBL is set whenever the transmit buffer is empty, and if TXBIL is set,
	 //TXBL is set whenever the transmit buffer is half-full or empty.
	 ;

	 if (txBuffer != 0) {
	 Writing next byte to USART
	 uart->TXDOUBLE  = *txBuffer& 0xFF;
	 txBuffer++;
	 } else {
	 uart->TXDOUBLE = 0;
	 }
	 }*/

	/*Waiting for transmission of last byte */
//	while (!(uart->STATUS & USART_STATUS_TXC))   //Set when a transmission has completed and no more data is available in the transmit buffer and shift register. Cleared when data
	//is written to the transmit buffer.
//		;
}
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/***************************************************************************//**
 * @brief
 *   Initializes USART2.
 * @note
 *   This function was autogenerated by energyAware Tools.
 ******************************************************************************/
void USART2_setup(void) {
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;

	init.baudrate = 115200;
	init.databits = usartDatabits12;
	init.msbf = 1;
	init.master = 1;
	init.clockMode = usartClockMode0;
	init.prsRxEnable = 0;
	init.autoTx = 1;

	USART_InitSync(USART2, &init);
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
	//	GPIO ->P[2].DOUTSET = 1 << 4; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! remove it after test
	//	GPIO ->P[2].DOUTCLR = 1 << 4; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! remove it after test
	GPIO ->P[2].DOUTSET = 1 << 4;
	GPIO ->P[2].DOUTCLR = 1 << 4;
	GPIO ->P[2].DOUTSET = 1 << 4;
	GPIO ->P[2].DOUTCLR = 1 << 4;
	while (i) {
		--i;
		GPIO ->P[2].DOUTSET = 1 << 4;
		FRAME |= GPIO_PinInGet(gpioPortC, 3) << i;
		GPIO ->P[2].DOUTCLR = 1 << 4;
	}
	GPIO ->P[2].DOUTSET |= 1 << 5;
	TheEndofInterruptTImer0 = true;
}

