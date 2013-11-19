/*
 * BTM222.c
 *
 *  Created on: Oct 2, 2013
 *      Author: lukasz
 */
#include "BTM222.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include <string.h>

static USART_TypeDef *uart1 = UART1; //
static USART_InitAsync_TypeDef uart1Init = USART_INITASYNC_DEFAULT;

circularBuffer rxBuf,
txBuf; // externowe

void uart1Setup(void) {
	cmuSetup();
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(gpioPortB, 9, gpioModePushPull, 1); // out , Pushpull
	GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
	uart1Init.enable = usartDisable;
	uart1Init.refFreq = 0;
	uart1Init.baudrate = 19200;      // Baud Rate - default value  for my BTM222
	uart1Init.oversampling = usartOVS16;
	uart1Init.parity = usartNoParity;  // Parity mode
	uart1Init.stopbits = usartStopbits1; //  Number of stop bits. Range is 0 to 2
	uart1Init.mvdis = false;          //  Disable majority voting
	uart1Init.prsRxEnable = false; //  Enable USART Rx via Peripheral Reflex System
	uart1Init.prsRxCh = usartPrsRxCh0;  //  Select PRS channel if enabled
	USART_InitAsync(uart1, &uart1Init);

	USART_IntClear(uart1, _UART_IF_MASK);
	USART_IntEnable(uart1, UART_IF_RXDATAV);
	NVIC_ClearPendingIRQ(UART1_RX_IRQn);
	NVIC_ClearPendingIRQ(UART1_TX_IRQn);
	NVIC_EnableIRQ(UART1_RX_IRQn);
	NVIC_EnableIRQ(UART1_TX_IRQn);

//	txBuf= CIRCULAR_BUFFER_INIT_DEFAULT ; //TODO
// rxBuf= CIRCULAR_BUFFER_INIT_DEFAULT ;

	uart1->ROUTE = UART_ROUTE_LOCATION_LOC2 | UART_ROUTE_TXPEN
			| UART_ROUTE_RXPEN;  // USART1_LOC2
	USART_Enable(uart1, usartEnable);

}

/******************************************************************************
 * @brief  uartReadChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will hang until a character is received.
 *
 *****************************************************************************/
uint8_t uart1ReadChar(void) {
	uint8_t ch;

	/* Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data */

	if (rxBuf.pendingBytes < 1) {
		while (rxBuf.pendingBytes < 1)
			;
	}

	/* Copy data from buffer */
	ch = rxBuf.data[rxBuf.rdI];
	rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

	/* Decrement pending byte counter */
	rxBuf.pendingBytes--;

	return ch;
}

/******************************************************************************
 * @brief  uartPutChar function
 *
 *****************************************************************************/
void uart1SendChar(uint8_t ch) {
	/* Check if Tx queue has room for new data */
	if ((txBuf.pendingBytes + 1) > BUFFERSIZE) {
		/* Wait until there is room in queue */
		while ((txBuf.pendingBytes + 1) > BUFFERSIZE)
			;
	}

	/* Copy ch into txBuffer */
	txBuf.data[txBuf.wrI] = ch;
	txBuf.wrI = (txBuf.wrI + 1) % BUFFERSIZE;

	/* Increment pending byte counter */
	txBuf.pendingBytes++;

	/* Enable interrupt on USART TX Buffer*/
	USART_IntEnable(uart1, UART_IF_TXBL);
}

/******************************************************************************
 * @brief  uartPutData function
 *
 *****************************************************************************/
void uart1SendData(uint8_t * dataPtr, uint32_t dataLen) {
	uint32_t i = 0;
/*
	// Check if buffer is large enough for data
	if (dataLen > BUFFERSIZE) {
		// Buffer can never fit the requested amount of data
		return;
	}

	// Check if buffer has room for new data
	if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) {
		//Wait until room
		while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
			;
	}
*/
	/* Fill dataPtr[0:dataLen-1] into txBuffer */
//TODO i<dataLen
	/*
	while (i<dataLen) {
		txBuf.data[i] = *(dataPtr + i);
		txBuf.wrI = (txBuf.wrI + 1) % BUFFERSIZE;
		i++;
	}

	// Increment pending byte counter
  txBuf.pendingBytes += dataLen;

	// Enable interrupt on USART TX Buffer
	 */

 USART_IntEnable(uart1, UART_IF_TXBL);
}

/******************************************************************************
 * @brief  uartGetData function
 *
 *****************************************************************************/
uint32_t uart1ReadData(uint8_t * dataPtr, uint32_t dataLen) {
	uint32_t i = 0;

	/* Wait until the requested number of bytes are available */
	if (rxBuf.pendingBytes < dataLen) {
		while (rxBuf.pendingBytes < dataLen)
			;
	}

	if (dataLen == 0) {
		dataLen = rxBuf.pendingBytes;
	}

	/* Copy data from Rx buffer to dataPtr */
	while (i < dataLen) {
		*(dataPtr + i) = rxBuf.data[rxBuf.rdI];
		rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;
		i++;
	}

	/* Decrement pending byte counter */
	rxBuf.pendingBytes -= dataLen;

	return i;
}

/***************************************************************************//**
 * @brief Set up Clock Management Unit
 ******************************************************************************/
void cmuSetup(void) {
	/* Start HFXO and wait until it is stable */
	/* CMU_OscillatorEnable( cmuOsc_HFXO, true, true); */

	/* Select HFXO as clock source for HFCLK */
	/* CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO ); */

	/* Disable HFRCO */
	/* CMU_OscillatorEnable( cmuOsc_HFRCO, false, false ); */

	/* Enable clock for HF peripherals */
	CMU_ClockEnable(cmuClock_HFPER, true);

	/* Enable clock for USART module */
	CMU_ClockEnable(cmuClock_UART1, true);
}

/**************************************************************************//**
 @brief UART1 RX IRQ Handler
 Set up the interrupt prior to use
 Note that this function handles overflows in a very simple way.
 *****************************************************************************/
void UART1_RX_IRQHandler(void) {
	/* Check for RX data valid interrupt */
	if (uart1->STATUS & UART_STATUS_RXDATAV) {
		/* Copy data into RX Buffer */
		uint8_t rxData = USART_Rx(uart1);
		rxBuf.data[rxBuf.wrI] = rxData;
		rxBuf.wrI = (rxBuf.wrI + 1) % BUFFERSIZE;
		rxBuf.pendingBytes++;
		/* Flag Rx overflow */
		if (rxBuf.pendingBytes > BUFFERSIZE) {
			rxBuf.overflow = true;
		}
		/* Clear RXDATAV interrupt */
		USART_IntClear(UART1, UART_IF_RXDATAV);
	}
}
//

/**************************************************************************//**
 * @brief UART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *****************************************************************************/
void UART1_TX_IRQHandler(void) {
	/* Clear interrupt flags by reading them. */
	USART_IntGet(UART1 );

	/* Check TX buffer level status */
	if (uart1->STATUS & UART_STATUS_TXBL) {
		/*
		if (txBuf.pendingBytes > 0) {
			// Transmit pending character
			USART_Tx(uart1, txBuf.data[txBuf.rdI]);
			txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
			txBuf.pendingBytes--;
		}
	   */
		USART_Tx(uart1, 'x');
		USART_Tx(uart1, 'A');
		USART_Tx(uart1, 'B');
		USART_Tx(uart1, 'C');
		USART_Tx(uart1, '\r');

		/* Disable Tx interrupt if no more bytes in queue */
		//if (txBuf.pendingBytes == 0) {
			USART_IntDisable(uart1, UART_IF_TXBL);
		//}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////
//BLUETOOTH
///////////////////////////////////////////////////////////////////////////////////////////////

void BTM222_Init(){
	 uart1Setup();
}


///////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t BTM222_EnterATMode()
{
	char buf[32]={0};
char  ATmodeEnab[]={"+++\r"};
char ATDisabEcho[]={"ATE0\r"};
	// Send +++ for AT mode enabling
	BTM222_SendData(ATmodeEnab);
	//uart1SendData(ATmodeEnab, strlen((char*)ATmodeEnab));
	//Disable echo, it speeds up communication, because, you don't have to wait for echo
	//uart1SendData(ATDisabEcho, strlen((char*)ATDisabEcho));
	BTM222_SendData(ATDisabEcho);
	BTM222_ReadData(buf);

	//return (char*)buf;
if (strcmp(buf, "OK")==0) return BTM222_OK; else return BTM222_ERR;
}

////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t  BTM222_SetName(char * Name)
{
	char buf[32];

	sprintf(buf, "ATN=%s\r", Name);
	BTM222_SendData(buf);
	BTM222_ReadData(buf);
	if (strcmp(buf, "OK")==0) return BTM222_OK; else return BTM222_ERR;
//	return (char*)buf;
}


////////////////////////////////////////////////////////////////////////////////////////////////
void BTM222_SendData(char * buffer)
{
	uart1SendData((uint8_t*)buffer, (uint32_t)strlen(buffer));
}


void BTM222_ReadData(char* buffer){

	uart1ReadData((uint8_t*)buffer, rxBuf.pendingBytes);

}





