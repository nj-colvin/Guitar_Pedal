/*
 * peripherals.h
 *
 *  Created on: 3 May 2022
 *      Author: nathan
 */

#include "stm32f767xx.h"

#ifndef SRC_PERIPHERALS_H_
#define SRC_PERIPHERALS_H_

void GPIO_Setup(void);
void TIMER4_Setup(void);
void TIMER5_Setup(void);
void TIMER6_Setup(void);
void TIMER12_Setup(void);

void EXTI_Setup(void);

void RNG_Setup(void);

void ADC_Setup(long adcDataAdress, long adcDataAdress2);
void ADC_DMA_Setup(long adcDataAdress, long adcDataAdress2);

#endif /* SRC_PERIPHERALS_H_ */
