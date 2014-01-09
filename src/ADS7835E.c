/*
 * ADS7835E.c
 *
 *  Created on: May 23, 2013
 *      Author: lukasz
 */

#include "ADS7835E.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "splc501c.h"
#include "goertzel.h"
#include "em_lcd.h"
#include "em_usart.h"
#include <stdio.h>
#include "Timers.h"
#include "em_timer.h"
#include <stdlib.h>
#include <malloc.h>
#include <math.h>

#define SOFTWARE_SPI 1

uint16_t* masterRxBuffer;
uint16_t* masterRxBuffer;
int masterRxBufferSize;

uint16_t* RxFrame; // for software SPI
CircularBufferADC_Result ADC_Result;
extern bool endOfADCInterrupt;
#if !SOFTWARE_SPI
void SPI2_Init(void) {
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO ->P[2].DOUT |= (1 << 2);
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_INPUT;
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE4_MASK) | GPIO_P_MODEL_MODE4_PUSHPULL;
	GPIO ->P[2].DOUT |= (1 << 5);
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;

	CMU_ClockEnable(cmuClock_USART2, true);
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.baudrate = 1000000;
	init.databits = usartDatabits12;
	init.msbf = 1;
	init.master = 1;
	init.clockMode = usartClockMode1;
	init.prsRxEnable = 0;
	init.autoTx = 0;

	USART_InitSync(USART2, &init);
	/* Enable signals TX, RX, CLK, CS */

	USART2 ->CTRL |= USART_CTRL_AUTOCS | USART_CTRL_TXDELAY_DOUBLE;
	USART2 ->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN | USART_ROUTE_CSPEN
	| USART_ROUTE_LOCATION_LOC0;

}

/**************************************************************************//**
 * @brief Setting up RX interrupts from USART2 RX
 * @param receiveBuffer points to where received data is to be stored
 * @param bytesToReceive indicates the number of bytes to receive
 *****************************************************************************/
void SPI2_setupRXInt(uint16_t* receiveBuffer, int bytesToReceive) {
	USART_TypeDef *spi = USART2;

	masterRxBuffer = receiveBuffer;
	masterRxBufferSize = bytesToReceive;
	masterRxBufferIndex = 0;

	spi->CMD = USART_CMD_CLEARRX;
//Enable interrupts
	NVIC_ClearPendingIRQ(USART2_RX_IRQn);
	NVIC_SetPriority(USART2_RX_IRQn, 1);
	NVIC_EnableIRQ(USART2_RX_IRQn);

	spi->IEN = USART_IEN_RXFULL;//USART_IEN_RXDATAV;	//USART_IEN_RXFULL;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART2_RX_IRQHandler(void) {
	USART_TypeDef *spi = USART2;

	if (spi->IF & USART_IF_RXFULL) {
		spi->IFC = USART_IFC_RXFULL;

		uint16_t rxdata = 0;
		// if (spi->STATUS & USART_STATUS_RXFULL)
		//  {
		// Reading out data
		rxdata = spi->RXDOUBLE;

		if (masterRxBuffer != 0) {

			*masterRxBuffer = rxdata;
		}
		//  }
		//GPIO ->P[2].DOUTSET = 1 << 5;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART2_sendBuffer(uint16_t* txBuffer, int bytesToSend) {
	USART_TxDouble(USART2, 0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#else   // it means I use software SPI
//////////////////////////////////////////////////////////////////
static void SPI2_Init_SW(void) {
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO ->P[2].DOUT |= (1 << 2);
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE2_MASK)
			| GPIO_P_MODEL_MODE2_PUSHPULL;
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE3_MASK)
			| GPIO_P_MODEL_MODE3_INPUT;
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE4_MASK)
			| GPIO_P_MODEL_MODE4_PUSHPULL;
	GPIO ->P[2].DOUT |= (1 << 5);
	GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL & ~_GPIO_P_MODEL_MODE5_MASK)
			| GPIO_P_MODEL_MODE5_PUSHPULL;
}
//////////////////////////////////////////////////////////////////
void SPI2_setupRXInt_SW(uint16_t* receiveBuffer) {
	SPI2_Init_SW();
	RxFrame = receiveBuffer;
	TIMER0forADC_Setup();       // start pomiaru
}
//////////////////////////////////////////////////////////////////
void SPI2_disableRXInt_SW(void) {

	TIMER0forADC_Disable();       // zakonczenie pomiaru
}
//////////////////////////////////////////////////////////////////
static uint16_t ReadFrameFromSPI_SW(void) {
	uint16_t AdcFrame = 0;
	int i = 12;
	GPIO ->P[2].DOUTCLR = 1 << 5;
	GPIO ->P[2].DOUTSET = 1 << 4;
	GPIO ->P[2].DOUTCLR = 1 << 4;
	GPIO ->P[2].DOUTSET = 1 << 4;
	GPIO ->P[2].DOUTCLR = 1 << 4;
	while (i) {
		--i;
		GPIO ->P[2].DOUTSET = 1 << 4;
		AdcFrame |= ((GPIO ->P[2].DIN >> 3) & 0x1) << i;
		GPIO ->P[2].DOUTCLR = 1 << 4;
	}
	GPIO ->P[2].DOUTSET |= 1 << 5;
	return AdcFrame;
}

// Interrupt Service Routine TIMER0 Interrupt Line for sampling of ADC converter
void TIMER0_IRQHandler(void) {
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	*RxFrame = ReadFrameFromSPI_SW();
	endOfADCInterrupt = true;
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConvertU16ToINTtoLCD(uint16_t digit, char* StringOutput) {

	float signedDigit;
	if (digit & 0x0800) {
		digit &= 0x07FF;
		signedDigit = (0x7FF - digit) * ADC_COEFFICIENT;

		snprintf(StringOutput, 5, "-%.3f", signedDigit);
	} else {
		signedDigit = digit * ADC_COEFFICIENT;
		snprintf(StringOutput, 5, " %.3f", signedDigit);

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool checkData(double *data) {
	if (((*data) < 0.0122) && ((*data) > -0.0122)) {
		*data = 0;
		return false;
	}
	return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void ConvertDOUBLEtoLCD(double digit, char* StringOutput, bool factorEnable) {

	const double SCALLING_FACTOR = 184;
	bool valueBelowZero = false;
	if (checkData(&digit)) {
		if (factorEnable)
			digit *= SCALLING_FACTOR;
		if (digit < 0) {
			digit -= (2 * digit);
			valueBelowZero = true;
		}

		gcvt(digit, 7, StringOutput);

		if (StringOutput[1] == '.') {

			StringOutput[3] = StringOutput[0];
			StringOutput[5] = StringOutput[2];
			StringOutput[0] = ' ';
			StringOutput[1] = ' ';
			if (valueBelowZero)
				StringOutput[2] = '-';
			else
				StringOutput[2] = ' ';
		} else if (StringOutput[2] == '.') {
			StringOutput[2] = StringOutput[0];
			StringOutput[5] = StringOutput[3];
			StringOutput[3] = StringOutput[1];
			StringOutput[0] = ' ';
			if (valueBelowZero)
				StringOutput[1] = '-';
			else
				StringOutput[1] = ' ';
		} else if (StringOutput[3] == '.') {
			for (uint8_t i = 0; i < 3; i++) {
				StringOutput[3 - i] = StringOutput[2 - i];
			}
			StringOutput[5] = StringOutput[4];
			if (valueBelowZero)
				StringOutput[0] = '-';
			else
				StringOutput[0] = ' ';
		}
		StringOutput[4] = '.';

	} else {
		StringOutput[0] = ' ';
		StringOutput[1] = ' ';
		StringOutput[2] = ' ';
		StringOutput[3] = '0';
		StringOutput[4] = '.';
		StringOutput[5] = '0';

	}
	StringOutput[6] = ' ';
	StringOutput[7] = NULL;
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
char* ParseDataToSendThroughBTM(char* data, char typeOfMessage,
		uint8_t numOfHarmFFT) {
	for (uint8_t i = 0; i < 6; i++) {

		data[6 - i] = data[5 - i];
	}
	data[7] = 'x'; // end delimiter
	switch (typeOfMessage) {
	case 'r':
	case 'a':
	case 'm':
	case 'n':
		data[0] = typeOfMessage;
		break;
	case 'f':
		data[0] = typeOfMessage;
		data[1] = (char)(((int)'0')+numOfHarmFFT);
	default:
		break;
	}
	return data;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Write an element, overwriting oldest element if buffer is full. App can
 choose to avoid the overwrite by checking cbIsFull(). */
/*
 void ResultADC_Buf_Write(CircularBufferADC_Result *cb, TYPE_OF_ADC_RESULT x) {
 switch (cb->Buf_isFull) {
 case false: {
 cb->Values[cb->end] = x;
 cb->end = (cb->end + 1); // % cb->size;

 if ((cb->end % cb->size) == (cb->start)) { // if the whole buf is full
 cb->Buf_isFull = true;

 }
 //	cb->start = (cb->start + 1) % cb->size; // full, overwrite
 break;
 }
 case true: {
 //uint16_t tab[cb->size];
 for (uint8_t i = 0; i < cb->size; i++) {

 if (i == (cb->size - 1)) {

 cb->Values[i] = cb->Values[i - 1];
 } else
 cb->Values[cb->size - i - 1] = cb->Values[cb->size - i - 2];
 }
 cb->Values[0] = x;   // load a new Value from ADC
 break;
 }
 }
 }

 */
void ResultADC_Buf_Write(CircularBufferADC_Result *cb, TYPE_OF_ADC_RESULT x) {

	cb->Values[cb->end] = x;
	cb->end = (cb->end + 1); // % cb->size;

	if ((cb->end % cb->size) == (cb->start)) { // if the whole buf is full
		cb->Buf_isFull = true;
		cb->end = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ResultADC_Buf_Read(CircularBufferADC_Result *cb,
		TYPE_OF_ADC_RESULT*oldestValue) {
	*oldestValue = cb->Values[cb->start];
	cb->start = (cb->start + 1) % cb->size;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ResultADC_Buf_Init(CircularBufferADC_Result *cb, int size) {
	cb->size = size;
	cb->start = 0;
	cb->end = 0;
	cb->Buf_isFull = false;
	cb->Values = (TYPE_OF_ADC_RESULT*) calloc(cb->size,
			sizeof(TYPE_OF_ADC_RESULT));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ResultADC_Buf_Free(CircularBufferADC_Result *cb) {
	free(cb->Values);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double rms(CircularBufferADC_Result *v) {
	int TemporaryVariable;
	double sum = 0.0;
	for (int i = 0; i < v->size; i++) {
		TemporaryVariable = ConvertU16_from_ADCToINT(v->Values[i]);
		sum += (TemporaryVariable * TemporaryVariable);
	}
	return ADC_COEFFICIENT * sqrt(sum / v->size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double min(CircularBufferADC_Result *v) {
	double minValue = 0.0;
	double minTempValue = 0.0;
	double rmsValue = rms(v);
	if (!checkData(&rmsValue)) {
		minValue = 0;
		return minValue;
	}
	for (int i = 0; i < v->size; i++) {
		minTempValue = ConvertU16_from_ADCToINT(v->Values[i]);
		if (minTempValue < minValue) {
			minValue = minTempValue;
		}
	}
	return ADC_COEFFICIENT * minValue;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double max(CircularBufferADC_Result *v) {
	double maxValue = 0.0;
	double maxTempValue = 0.0;
	double rmsValue = rms(v);
	if (!checkData(&rmsValue)) {
		maxValue = 0;
		return maxValue;
	}
	for (int i = 0; i < v->size; i++) {
		maxTempValue = ConvertU16_from_ADCToINT(v->Values[i]);
		if (maxTempValue > maxValue) {
			maxValue = maxTempValue;
		}
	}
	return ADC_COEFFICIENT * maxValue;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double avg(CircularBufferADC_Result *v) {
	double avg = 0.0;
	for (int i = 0; i < v->size; i++) {
		avg += ConvertU16_from_ADCToINT(v->Values[i]);
	}
	return ADC_COEFFICIENT * (avg / v->size);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//goertzel

float doGoertzelAlgorithm(CircularBufferADC_Result *v) {

	return ADC_COEFFICIENT * goertzel(SIZE_BUF_ADC, 50, 1024, v->Values);

}

