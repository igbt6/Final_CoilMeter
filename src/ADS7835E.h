/*
 * ADS7835E.h
 *
 *  Created on: May 11, 2013
 *      Author: lukasz
 */

#ifndef ADS7835E_H_
#define ADS7835E_H_

#include "em_chip.h"
#include <malloc.h>
#include <stdbool.h>
/* Defines */

#define SIZE_BUF_ADC  64 //ilosc probek
#define NUMBER_OF_VALUES_FOR_AVG 5 //ilosc do obliczenia sredniej

/* Circular buffer object */
typedef struct {
#define TYPE_OF_ADC_RESULT  uint16_t
	int size; /* maximum number of elements           */
	int start; /* index of oldest element              */
	int end; /* index at which to write new element  */
	TYPE_OF_ADC_RESULT *Values; /* vector of elements     */
	bool Buf_isFull;
} CircularBufferADC_Result;

/////extern CircularBufferADC_Result ADC_RESULT;   // note , you MUST init this struct first by { ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); }  function  in main.c file

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef volatile struct {
	double rms;
	double min;
	double max;
	double avg;
	uint32_t batLevel;
	double rmsAVG[NUMBER_OF_VALUES_FOR_AVG];
	double avgAVG[NUMBER_OF_VALUES_FOR_AVG];
	double minAVG[NUMBER_OF_VALUES_FOR_AVG];
	double maxAVG[NUMBER_OF_VALUES_FOR_AVG];
} Results;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
	uint8_t batVoltageConstant;
	uint16_t batVoltageFraction;
	char batVolatgeString[5];
} BatVoltage;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline int ResultADC_Buf_IsFull(CircularBufferADC_Result *cb) {
	return (cb->end + 1) % cb->size == cb->start;
}

inline int ResultADC_Buf_IsEmpty(CircularBufferADC_Result *cb) {
	return cb->end == cb->start;
}

void SPI_setup(void); // hardware SPI
void SPI2_Init(void);
void SPI2_setupRXInt(uint16_t* receiveBuffer, int bytesToReceive);
void SPI2_setupRXInt_SW(uint16_t* receiveBuffer); // software SPI
void SPI2_disableRXInt_SW(void);
void USART2_sendBuffer(uint16_t* txBuffer, int bytesToSend);

void ConvertU16ToINTtoLCD(uint16_t digit, char* StringOutput);
void ConvertDOUBLEtoLCD(double digit, char* StringOutput, bool factorEnable);
int ConvertU16_from_ADCToINT(uint16_t digit);
char* ParseDataToSendThroughBTM(char* data, char typeOfMessage,uint8_t numOfHarmFFT);
//void ResultADC_Buf_Read(CircularBufferADC_Result *cb);
void ResultADC_Buf_Write(CircularBufferADC_Result *cb, TYPE_OF_ADC_RESULT x);
void ResultADC_Buf_Init(CircularBufferADC_Result *cb, int size);
void ResultADC_Buf_Free(CircularBufferADC_Result *cb);
double rms(CircularBufferADC_Result *v);
double min(CircularBufferADC_Result *v);
double max(CircularBufferADC_Result *v);
double avg(CircularBufferADC_Result *v);
float doGoertzelAlgorithm(CircularBufferADC_Result *v);

#endif /* ADS7835E_H_ */

