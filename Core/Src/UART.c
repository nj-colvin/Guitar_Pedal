/*
 * UART.c
 *
 *  Created on: 3 May 2022
 *      Author: nathan
 */

#include "UART.h"

void UART_Setup(void){
	// UART  Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// Data length

	//set BAUD Rate
	//f_CK(APB1) / Baud = 54MHz / 115200
	USART3->BRR = 469;

	// enable UART
	USART3->CR1 |= USART_CR1_UE;

	//             DMA transmit   | DMA receive
	USART3->CR3 |= USART_CR3_DMAT;// | USART_CR3_DMAR;

	// enable transmission and receive
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;

	UART_DMA_Setup();
}

void UART_DMA_Setup(void){
	// DMA Clock Enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

	// USART3-Tx - Stream 4 - Channel 7

	DMA1_Stream4->PAR = (long)&(USART3->TDR);
//	DMA1_Stream4->M0AR = (long)&(dataTx);
	DMA1_Stream4->NDTR = 0;
	//                 channel 7                 | mmry inc      | mmry - prphrl  | enable
	DMA1_Stream4->CR = (7 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;// | DMA_SxCR_EN;


}


void UART_Transmit(unsigned char * TxBuffer, unsigned int length){
	// wait for DAM to finish
	while(DMA1_Stream4->CR & DMA_SxCR_EN);
	//             clear flags
	DMA1->HIFCR |= DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4;

	DMA1_Stream4->M0AR = (long)&(TxBuffer[0]); // memory address
	DMA1_Stream4->NDTR = length;

	//                    Enable DMA
	DMA1_Stream4->CR |= DMA_SxCR_EN;
}


// check if the DMA is currently transferring from the UART buffer
char UART_Buffer_Is_Free(void){
	if(DMA1_Stream4->CR & DMA_SxCR_EN){
		return 0;
	}
	return 1;
}
