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
#define HFRCO_FREQUENCY         14000000
#define SPI_PERCLK_FREQUENCY    HFRCO_FREQUENCY
#define SPI_BAUDRATE            10000000

#define SIZE_BUF_ADC  100



/* Circular buffer object */
typedef struct {
#define TYPE_OF_ADC_RESULT  uint16_t
    int         size;   /* maximum number of elements           */
    int         start;  /* index of oldest element              */
    int         end;    /* index at which to write new element  */
    TYPE_OF_ADC_RESULT   *Values;  /* vector of elements     */
    bool Buf_isFull;
} CircularBufferADC_Result;




inline int ResultADC_Buf_IsFull(CircularBufferADC_Result *cb) {
    return (cb->end + 1) % cb->size == cb->start; }

inline int ResultADC_Buf_IsEmpty(CircularBufferADC_Result *cb) {
    return cb->end == cb->start; }



void SPI_setup(void); // hardware SPI
void SPI2_setupRXInt(uint16_t* receiveBuffer, int bytesToReceive);
void USART2_sendBuffer(uint16_t* txBuffer, int bytesToSend);


void ConvertU16ToINTtoLCD(uint16_t digit, char* StringOutput);
void ConvertDOUBLEtoLCD(double digit, char* StringOutput);
int ConvertU16_from_ADCToINT(uint16_t digit);
//void ResultADC_Buf_Read(CircularBufferADC_Result *cb);
void ResultADC_Buf_Write(CircularBufferADC_Result *cb,TYPE_OF_ADC_RESULT x);
void ResultADC_Buf_Init(CircularBufferADC_Result *cb, int size);
void ResultADC_Buf_Free(CircularBufferADC_Result *cb);
double rms(CircularBufferADC_Result *v);


#endif /* ADS7835E_H_ */



