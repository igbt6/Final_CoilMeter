/*
 * BTM222.h
 *
 *  Created on: May 29, 2013
 *      Author: lukasz
 */

#ifndef BTM222_H_
#define BTM222_H_

#include <stdbool.h>
#include "efm32.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

//PINOUT
// bluetooth
// PB9	TX	26                  U1_TX2
// PB10 RX	27                  U1_RX2
// antena 37

/* Declare a circular buffer structure to use for Rx and Tx queues */

#define BTM222_ERR			0
#define BTM222_OK			1

#define BUFFERSIZE          256

#define CIRCULAR_BUFFER_INIT_DEFAULT                   \
{ .data[BUFFERSIZE]={0},                               \
  .rdI=0,                                              \
  .wrI=0,                                              \
  .pendingBytes=0,                                     \
  .overflow=false                                      \
}



volatile typedef struct {
uint8_t data[BUFFERSIZE];  // data buffer
uint32_t rdI;               // read index
uint32_t wrI;               // write index
uint32_t pendingBytes;      // count of how many bytes are not yet handled
bool overflow;          // buffer overflow indicator
} circularBuffer;          // rxBuf, txBuf = { {0}, 0, 0, 0, false };

extern circularBuffer rxBuf;
extern circularBuffer txBuf;

void uart1Setup(void);
void cmuSetup(void);
void uart1SendData(uint8_t * dataPtr, uint32_t dataLen);
uint32_t uart1ReadData(uint8_t * dataPtr, uint32_t dataLen);
void uart1SendChar(uint8_t charPtr);
uint8_t uart1ReadChar(void);

          ///

void BTM222_Init();
uint8_t BTM222_SetName(char * Name);
uint8_t BTM222_EnterATMode();
void BTM222_GetName(char * Name);
void BTM222_GetPin(char * Name);
void BTM222_SendData(char * buffer);
uint32_t BTM222_ReadData(char * buffer);
#endif /* BTM222_H_ */
