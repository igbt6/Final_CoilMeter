/*
 * ADS7835E.c
 *
 *  Created on: May 11, 2013
 *      Author: lukasz
 */

#include "ADS7835E.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "splc501c.h"
#include "em_lcd.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
/* Buffer pointers and indexes */

char* masterRxBuffer;
char* masterRxBuffer;
int masterRxBufferSize;
volatile int masterRxBufferIndex;

void SPI_setup(void) {

	CMU_ClockEnable(cmuClock_USART2, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	USART_TypeDef *spi;
	spi = USART2;

	// Setting baudrate
	spi->CLKDIV = 128 * (SPI_PERCLK_FREQUENCY / SPI_BAUDRATE - 2);

	// Configure SPI
	// Using synchronous (SPI) mode
	spi->CTRL = USART_CTRL_SYNC;
	// Clearing old transfers/receptions, and disabling interrupts
	spi->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX;
	spi->IEN = 0;
	// Enabling pins and setting location
	spi->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN
			| USART_ROUTE_CSPEN | USART_ROUTE_LOCATION_LOC0;
	spi->FRAME = USART_FRAME_DATABITS_TWELVE;
	//Enabling Master, TX and RX and CS line
	spi->CMD |= USART_CMD_MASTEREN | USART_CMD_TXEN | USART_CMD_RXEN;
	spi->CTRL |= USART_CTRL_AUTOCS | USART_CTRL_MSBF | USART_CTRL_AUTOTX
			| USART_CTRL_TXDELAY_NONE; //MSBF- najstarszy bit 1 //If AUTOCS is set, USn_CS is activated when a transmission begins, and deactivated directly after the last bit has been transmitted and there is no more data in the  	transmit buffer.
	//spi->TXDATAX|=USART_TXDATAX_RXENAT;
//spi->CTRL|=USART_CTRL_CLKPHA;

	/* Set GPIO config to master */

	GPIO_Mode_TypeDef gpioModeMosi = gpioModePushPull;
	GPIO_Mode_TypeDef gpioModeMiso = gpioModeInput;
	GPIO_Mode_TypeDef gpioModeCs = gpioModePushPull;
	GPIO_Mode_TypeDef gpioModeClk = gpioModePushPull;
	/* Clear previous interrupts */
	spi->IFC = _USART_IFC_MASK;

	/* IO configuration (USART 2, Location #0) */
	GPIO_PinModeSet(gpioPortC, 2, gpioModeMosi, 0); /* MOSI */
	GPIO_PinModeSet(gpioPortC, 3, gpioModeMiso, 0); /* MISO */
	GPIO_PinModeSet(gpioPortC, 5, gpioModeCs, 0); /* CS */
	GPIO_PinModeSet(gpioPortC, 4, gpioModeClk, 0); /* Clock */

}

/**************************************************************************//**
 * @brief Setting up RX interrupts from USART2 RX
 * @param receiveBuffer points to where received data is to be stored
 * @param bytesToReceive indicates the number of bytes to receive
 *****************************************************************************/
void SPI2_setupRXInt(char* receiveBuffer, int bytesToReceive) {
	USART_TypeDef *spi = USART2;

	//Setting up pointer and indexes
	masterRxBuffer = receiveBuffer;
	masterRxBufferSize = bytesToReceive;
	masterRxBufferIndex = 0;

	spi->CMD = USART_CMD_CLEARRX;
//Enable interrupts
	NVIC_ClearPendingIRQ(USART2_RX_IRQn);
	NVIC_EnableIRQ(USART2_RX_IRQn);
	spi->IEN = USART_IEN_RXDATAV;	//USART_IEN_RXFULL;
// GPIO->P[2].DOUTCLR=1<<5;  // wylaczam ustawiam 0 na CS start pomiaru
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART2_RX_IRQHandler(void) {
	USART_TypeDef *spi = USART2;
	uint8_t rxdata;
//char bufoo[10];
	if (spi->STATUS & USART_STATUS_RXDATAV) { //Set when data is available in the receive buffer. Cleared when the receive buffer is empty.
		/* Reading out data */
		rxdata = spi->RXDOUBLE;
		//GPIO ->P[4].DOUTTGL = 1 << 3; // zapalam obie

		/*	if (masterRxBuffer != 0) {
		 Store Data
		 masterRxBuffer[masterRxBufferIndex] = rxdata;
		 masterRxBufferIndex++;

		 if (masterRxBufferIndex == masterRxBufferSize) {
		 masterRxBuffer = 0;
		 }
		 }*/
	}
}


/**************************************************************************//**
 * @brief USART1 TX IRQ Handler Setup
 * @param transmitBuffer points to the data to send
 * @param transmitBufferSize indicates the number of bytes to send
 *****************************************************************************/
/*
void SPI2_setupTXInt(char* transmitBuffer, int transmitBufferSize)
{
  USART_TypeDef *spi = USART1;

  // Setting up pointer and indexes
  slaveTxBuffer      = transmitBuffer;
  slaveTxBufferSize  = transmitBufferSize;
  slaveTxBufferIndex = 0;

  // Clear TX
  spi->CMD = USART_CMD_CLEARTX;

  // Enable interrupts
  NVIC_ClearPendingIRQ(USART2_TX_IRQn);
  NVIC_EnableIRQ(USART2_TX_IRQn);
  spi->IEN |= USART_IEN_TXBL;
}
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConvertU16ToINTtoLCD(uint16_t digit, char* StringOutput) {
	const float CONST_of_MULTIPLICATION = 0.00122;
	float signedDigit;

	if (digit & 0x0800) {
		digit &= 0x07FF;
		signedDigit = (0x7FF - digit) * CONST_of_MULTIPLICATION;

		snprintf(StringOutput, 5, "-%.3f", signedDigit);
	} else {
		signedDigit = digit * CONST_of_MULTIPLICATION;

		// snprintf(StringOutput, 5, "%2f,%1f", (signedDigit / 10), (signedDigit / 10));
		snprintf(StringOutput, 5, " %.3f", signedDigit);
		//  return StringOutput;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConvertDOUBLEtoLCD(double digit, char* StringOutput) {
	const float CONST_of_MULTIPLICATION = 0.00122;

	digit = digit * CONST_of_MULTIPLICATION;
	gcvt(digit, 4, StringOutput);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ConvertU16_from_ADCToINT(uint16_t digit) {

	int signedDigit;

	if (digit & 0x0800) {
		digit &= 0x07FF;
		signedDigit = digit - 0x07FF;
	} else
		signedDigit = (int) digit;

	return signedDigit;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Write an element, overwriting oldest element if buffer is full. App can
 choose to avoid the overwrite by checking cbIsFull(). */
void ResultADC_Buf_Write(CircularBufferADC_Result *cb, TYPE_OF_ADC_RESULT x) {
	switch (cb->Buf_isFull) {
	case false: {
		cb->Values[cb->end] = x;
		cb->end = (cb->end + 1); // % cb->size;

		if ((cb->end % cb->size) == (cb->start)) { // if the whole buf is full
			cb->Buf_isFull = true;
		}
//	cb->start = (cb->start + 1) % cb->size; /* full, overwrite */
		break;
	}
	case true: {
		//uint16_t tab[cb->size];
		for (uint8_t i = 0; i < cb->size; i++) {

			if (i == (cb->size - 1)) {

				cb->Values[i] = cb->Values[i - 1];
			} else
				cb->Values[cb->size - i-1] = cb->Values[cb->size - i-2];
		}
		cb->Values[0] = x;   // load a new Value from ADC
		break;
	}
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Read oldest element. App must ensure !cbIsEmpty() first. */
/*    void ResultADC_Buf_Read(CircularBufferADC_Result *cb) {
 *elem = cb->elems[cb->start];
 cb->start = (cb->start + 1) % cb->size;
 }
 */
void ResultADC_Buf_Init(CircularBufferADC_Result *cb, int size) {
	cb->size = size; /* include empty elem */
	cb->start = 0;
	cb->end = 0;
	cb->Buf_isFull = false;
	cb->Values = (TYPE_OF_ADC_RESULT*) calloc(cb->size,
			sizeof(TYPE_OF_ADC_RESULT));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ResultADC_Buf_Free(CircularBufferADC_Result *cb) {
	free(cb->Values); /* OK if null */
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double rms(CircularBufferADC_Result *v) {
	int TemporaryVariable;
	double sum = 0.0;
	for (int i = 0; i < v->size; i++) {
		TemporaryVariable=ConvertU16_from_ADCToINT(v->Values[i]);
		sum += (TemporaryVariable
				* TemporaryVariable);
	}
	return sqrt(sum / v->size);
}



