/*
 * SPI.c
 *
 *  Created on: 3 May 2022
 *      Author: nathan
 */


#include "SPI.h"

void SPI_Setup(void){
	// SPI Clock Enable
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	//           software SS | NSS high    | prescale /8 = 13.5Mhz | master mode  | read on falling edge
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI | (2 << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | SPI_CR1_CPHA;

	//           align FIFO 8b | 8 bit data size       | Rx DMA
	SPI1->CR2 |= SPI_CR2_FRXTH | (7 << SPI_CR2_DS_Pos) | SPI_CR2_RXDMAEN;

	SPI_DMA_Setup();

	//           Tx DMA
	SPI1->CR2 |= SPI_CR2_TXDMAEN;

	TIMER3_Setup();
	TIMER2_Setup();

	//           enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}


void SPI_DMA_Setup(void){
	// DMA Clock Enable
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;

	// SPI1 Rx - Stream 0 - Channel 3
	DMA2_Stream0->PAR = (long)&(SPI1->DR);
//	DMA2_Stream0->M0AR = (long)&(dataRx);
	//                  channel 3                 | very high   | mmry inc      | transfer complete interrupt
	DMA2_Stream0->CR |= (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	DMA2_Stream0->NDTR = 0;

	// SPI1 Tx - Stream 3 - Channel 3
	DMA2_Stream3->PAR = (long)&(SPI1->DR);
//	DMA2_Stream3->M0AR = (long)&(dataTx);
	DMA2_Stream3->NDTR = 0;
	//                 channel 3                 | very high   |  mmry inc      | mmry - prphrl  | enable
	DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL | DMA_SxCR_MINC | DMA_SxCR_DIR_0;// | DMA_SxCR_EN;

//
}

// Sync Pulse Generator
void TIMER2_Setup(void){
	// Timer2 Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//            trigger mode            | TIM3 as TRGI
	TIM2->SMCR |= (6 << TIM_SMCR_SMS_Pos) | (2 << TIM_SMCR_TS_Pos);

	//           one pulse mode
	TIM2->CR1 |= TIM_CR1_OPM;

	TIM2->PSC = 1-1;
	TIM2->ARR = 20-1;
	TIM2->CCR1 = 10-1;

	//              PWM Mode 2               | preload
//	TIM2->CCMR1 |= (7 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

	//              Force high            | preload
	TIM2->CCMR1 |= (5 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

	TIM2->CCER |= TIM_CCER_CC1E;

	//           enable counter
	TIM2->CR1 |= TIM_CR1_CEN;
}

// Input capture on SPI clock for timing sync pulse
void TIMER3_Setup(void){
	// Timer3 Clock Enable
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//            external clock mode     | clock trigger = TI2FP2
	TIM3->SMCR |= (7 << TIM_SMCR_SMS_Pos) | (6 << TIM_SMCR_TS_Pos);

	//             TI2 on CH2
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0;

	//            falling edge trigger
	TIM3->CCER |= TIM_CCER_CC2P;

	//            update as output trigger (TRGO)
	TIM3->CR2 |= (2 << TIM_CR2_MMS_Pos);

	// 34 clock counts
	TIM3->ARR = 24-1;

//	TIM3->DIER |= TIM_DIER_UIE;

	//           enable
	TIM3->CR1 |= TIM_CR1_CEN;

	//TRGI = external clock, TRGO = Update;
}


char SPI_In_Progress(void){
	if(DMA2_Stream0->CR & DMA_SxCR_EN){
		return 1;
	}
	return 0;
}

void SPI_Stream(unsigned char *TxBuffer){
	//Tx
	DMA2_Stream3->M0AR = (long)TxBuffer;
	DMA2_Stream3->NDTR = 3;
	//                 channel 3                 | mmry inc      | circular mode | mmry - prphrl  | enable
	DMA2_Stream3->CR = (3 << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_DIR_0 | DMA_SxCR_EN;
}

void SPI_Communicate(volatile unsigned char *TxBuffer, volatile unsigned char *RxBuffer, int length){

	// wait for DMA to finish
	while((DMA2_Stream3->CR & DMA_SxCR_EN) || (DMA2_Stream0->CR & DMA_SxCR_EN));
	//             clear flags
	DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CFEIF3 | DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CFEIF0;

	//Rx
	DMA2_Stream0->M0AR = (long)RxBuffer;
	DMA2_Stream0->NDTR = length;
	//                 enable
	DMA2_Stream0->CR |= DMA_SxCR_EN;

	// pull slave select low
	GPIOA->BSRR |= (GPIO_BSRR_BR4);

	//Tx
	DMA2_Stream3->M0AR = (long)TxBuffer;
	DMA2_Stream3->NDTR = length;

	//                 enable
	DMA2_Stream3->CR |= DMA_SxCR_EN;

}
