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
extern bool endOfADCInterrupt;
#if !SOFTWARE_SPI
void SPI2_Init(void) {
	/* No low frequency clock source selected */

	/* Enable GPIO clock */
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO ->P[2].DOUT |= (1 << 2);
	/*  PC2  Push-pull */GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE2_MASK) | GPIO_P_MODEL_MODE2_PUSHPULL;
	/* PC3 is configured to Input */GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE3_MASK) | GPIO_P_MODEL_MODE3_INPUT;
	/*  PC4 Push-pull */GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE4_MASK) | GPIO_P_MODEL_MODE4_PUSHPULL;
	/* PC5 */GPIO ->P[2].DOUT |= (1 << 5);
	/* PC5  Push-pull */GPIO ->P[2].MODEL = (GPIO ->P[2].MODEL
			& ~_GPIO_P_MODEL_MODE5_MASK) | GPIO_P_MODEL_MODE5_PUSHPULL;

	/* Enable clock for USART2 */
	CMU_ClockEnable(cmuClock_USART2, true);
	/* Custom initialization for USART2 */
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

	//Setting up pointer and indexes
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
	TIMER0forADC_Setup();
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
	/* Clear flag for TIMER0 overflow interrupt */
	TIMER_IntClear(TIMER0, TIMER_IF_OF);
	*RxFrame = ReadFrameFromSPI_SW();
	endOfADCInterrupt = true;
	/*
	 static int i ,x;
	 i++;
	 if (i >= 1000) {
	 x++;
	 GLCD_GoTo(1, 0);
	 GLCD_WriteChar((char)x);

	 i = 0;
	 }
	 */
}

#endif

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

static bool checkData(double *data) {
	if (((int)*data) < 1.000000) {
		*data = 0;
		return false;
	}
 return true;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void ConvertDOUBLEtoLCD(double digit, char* StringOutput) {
	const double COEFFICIENT = 0.00122*184 ;

	if (checkData(&digit)) {
		digit = digit * COEFFICIENT ;
	}
	else digit=0;
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
char* ParseDataToSendThroughBTM(char* data, char typeOfMessage) {
	for (uint8_t i = 0; i < 6; i++) {

		data[i + 1] = data[i];
	}
	data[6] = 'x'; // end delimiter
	switch (typeOfMessage) {
	case 'r':
	case 'a':
	case 'm':
	case 'n':
		data[0] = typeOfMessage;
		break;
	default:
		break;
	}
	data[0] = 'r';
	return data;
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
				cb->Values[cb->size - i - 1] = cb->Values[cb->size - i - 2];
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
		TemporaryVariable = ConvertU16_from_ADCToINT(v->Values[i]);
		sum += (TemporaryVariable * TemporaryVariable);
	}
	return sqrt(sum / v->size);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double min(CircularBufferADC_Result *v) {
	double minValue = 0.0;
	double minTempValue = 0.0;
	for (int i = 0; i < v->size; i++) {
		minTempValue = ConvertU16_from_ADCToINT(v->Values[i]);
		if (minTempValue < minValue) {
			minValue = minTempValue;
		}
	}
	return minValue;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double max(CircularBufferADC_Result *v) {
	double maxValue = 0.0;
	double maxTempValue = 0.0;
	for (int i = 0; i < v->size; i++) {
		maxTempValue = ConvertU16_from_ADCToINT(v->Values[i]);
		if (maxTempValue > maxValue) {
			maxValue = maxTempValue;
		}
	}
	return maxValue;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double avg(CircularBufferADC_Result *v) {
	double avg = 0.0;
	for (int i = 0; i < v->size; i++) {
		avg += ConvertU16_from_ADCToINT(v->Values[i]);
	}
	return (avg / v->size);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//goertzel

float doGoertzelAlgorithm(CircularBufferADC_Result *v) {
	float magnitudeSquared = 0;
	float real, imag;
#if 0
	for (int i = 0; i < v->size; i++) {
		ProcessSample(ConvertU16_from_ADCToINT(v->Values[i]));
	}
	/* optimized algorithm

	 magnitudeSquared = GetMagnitudeSquared(); // magnitude squared

	 */
	/*basic algorithm*/
	GetRealImag(&real, &imag);
	magnitudeSquared = real*real + imag*imag;
	return sqrt(magnitudeSquared); //relative magntude
#endif

	return goertzel_mag(100, 50, 1000, v->Values);

}

