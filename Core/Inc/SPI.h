/*
 * SPI.h
 *
 *  Created on: 3 May 2022
 *      Author: nathan
 */

#include "stm32f767xx.h"

#ifndef INC_SPI_H_
#define INC_SPI_H_

void SPI_Setup(void);
void SPI_DMA_Setup(void);
void TIMER2_Setup(void);
void TIMER3_Setup(void);

char SPI_In_Progress(void);
void SPI_Stream(unsigned char *TxBuffer);
void SPI_Communicate(volatile unsigned char *TxBuffer, volatile unsigned char *RxBuffer, int length);

#endif /* INC_SPI_H_ */
