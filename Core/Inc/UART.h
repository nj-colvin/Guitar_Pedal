/*
 * UART.h
 *
 *  Created on: 3 May 2022
 *      Author: nathan
 */

#include "stm32f767xx.h"

#ifndef INC_UART_H_
#define INC_UART_H_

void UART_Setup(void);
void UART_DMA_Setup(void);
void UART_Transmit(unsigned char * TxBuffer, unsigned int length);
char UART_Buffer_Is_Free(void);

#endif /* INC_UART_H_ */
