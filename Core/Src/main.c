/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
//#include "arm_math.h"
#include "dsp/complex_math_functions.h"
#include "dsp/fast_math_functions.h"
#include "dsp/transform_functions.h"

#include "peripherals.h"
#include "SPI.h"
#include "UART.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMELE_BUFFER_SIZE 100000
#define FFT_SAMPLES 1024 // must be 2^n for integer n
#define FFT_TO_HOP_RATIO 2
#define HOP_SIZE FFT_SAMPLES / FFT_TO_HOP_RATIO
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
volatile unsigned short dataIn;
volatile unsigned short dataOut;
volatile unsigned char dataTx[8], dataRx[8];

// general sample buffer
volatile unsigned short sampleBuffer[SAMELE_BUFFER_SIZE];
volatile unsigned long sampleBufferIndex;
volatile unsigned char dataReadyFlag = 0;

// FFT
float FFTBuffer[FFT_SAMPLES], FFTOutBuffer[FFT_SAMPLES];
float FFTWindow[FFT_SAMPLES];
volatile unsigned int startFFTFlag, FFTBufferSegment;

// looping and sampling
unsigned char isRecording, looperOverflow = 0;
unsigned long endIndex = 20000;
unsigned int textureSampleOffset, textureSamplesRemaining;

// effect control
volatile unsigned int effectNumber = 0;
unsigned short control[2], control2[2];
unsigned short controlShadow[2];

unsigned short averageValue;

// phaser poles
long a, b, c, d;
int a0 = 190;
int b0 = 175;
int c0 = 160;
int d0 = 147;

// UART buffer
unsigned char buffer[24];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */

void Configure_MCP3561R(unsigned char *buffer, char readback);

void Clean(void);
void Delay(void);
void Flange(void);
void Phaser_Stage(int stageNumber, int a);
void Phaser(void);
void Reverb(void);
void Sin_Delay(void);
void Reverse_Delay(void);
void Bit_Reduction(void);
void Random_Sample_Delay(void);
void Clouds_Delay(void);
void Looper(void);
void FFT_Test(arm_rfft_fast_instance_f32 * S);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned int lookUp[256] = {
		0x8000,0x8324,0x8647,0x896a,0x8c8b,0x8fab,0x92c7,0x95e1,
		0x98f8,0x9c0b,0x9f19,0xa223,0xa527,0xa826,0xab1f,0xae10,
		0xb0fb,0xb3de,0xb6b9,0xb98c,0xbc56,0xbf17,0xc1cd,0xc47a,
		0xc71c,0xc9b3,0xcc3f,0xcebf,0xd133,0xd39a,0xd5f5,0xd842,
		0xda82,0xdcb3,0xded7,0xe0eb,0xe2f1,0xe4e8,0xe6cf,0xe8a6,
		0xea6d,0xec23,0xedc9,0xef5e,0xf0e2,0xf254,0xf3b5,0xf504,
		0xf641,0xf76b,0xf884,0xf989,0xfa7c,0xfb5c,0xfc29,0xfce3,
		0xfd89,0xfe1d,0xfe9c,0xff09,0xff61,0xffa6,0xffd8,0xfff5,
		0xffff,0xfff5,0xffd8,0xffa6,0xff61,0xff09,0xfe9c,0xfe1d,
		0xfd89,0xfce3,0xfc29,0xfb5c,0xfa7c,0xf989,0xf884,0xf76b,
		0xf641,0xf504,0xf3b5,0xf254,0xf0e2,0xef5e,0xedc9,0xec23,
		0xea6d,0xe8a6,0xe6cf,0xe4e8,0xe2f1,0xe0eb,0xded7,0xdcb3,
		0xda82,0xd842,0xd5f5,0xd39a,0xd133,0xcebf,0xcc3f,0xc9b3,
		0xc71c,0xc47a,0xc1cd,0xbf17,0xbc56,0xb98c,0xb6b9,0xb3de,
		0xb0fb,0xae10,0xab1f,0xa826,0xa527,0xa223,0x9f19,0x9c0b,
		0x98f8,0x95e1,0x92c7,0x8fab,0x8c8b,0x896a,0x8647,0x8324,
		0x8000,0x7cdb,0x79b8,0x7695,0x7374,0x7054,0x6d38,0x6a1e,
		0x6707,0x63f4,0x60e6,0x5ddc,0x5ad8,0x57d9,0x54e0,0x51ef,
		0x4f04,0x4c21,0x4946,0x4673,0x43a9,0x40e8,0x3e32,0x3b85,
		0x38e3,0x364c,0x33c0,0x3140,0x2ecc,0x2c65,0x2a0a,0x27bd,
		0x257d,0x234c,0x2128,0x1f14,0x1d0e,0x1b17,0x1930,0x1759,
		0x1592,0x13dc,0x1236,0x10a1,0xf1d,0xdab,0xc4a,0xafb,
		0x9be,0x894,0x77b,0x676,0x583,0x4a3,0x3d6,0x31c,
		0x276,0x1e2,0x163,0xf6,0x9e,0x59,0x27,0xa,
		0x0,0xa,0x27,0x59,0x9e,0xf6,0x163,0x1e2,
		0x276,0x31c,0x3d6,0x4a3,0x583,0x676,0x77b,0x894,
		0x9be,0xafb,0xc4a,0xdab,0xf1d,0x10a1,0x1236,0x13dc,
		0x1592,0x1759,0x1930,0x1b17,0x1d0e,0x1f14,0x2128,0x234c,
		0x257d,0x27bd,0x2a0a,0x2c65,0x2ecc,0x3140,0x33c0,0x364c,
		0x38e3,0x3b85,0x3e32,0x40e8,0x43a9,0x4673,0x4946,0x4c21,
		0x4f04,0x51ef,0x54e0,0x57d9,0x5ad8,0x5ddc,0x60e6,0x63f4,
		0x6707,0x6a1e,0x6d38,0x7054,0x7374,0x7695,0x79b8,0x7cdb};

unsigned int lookupIndex = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_ETH_Init();
//  MX_USART3_UART_Init();
//  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  GPIO_Setup();
  TIMER4_Setup();
  TIMER5_Setup();
  TIMER12_Setup();
  SPI_Setup();
  UART_Setup();
  RNG_Setup();
  ADC_Setup((long)control, (long)control2);
//  EXTI_Setup();
  TIMER6_Setup();

  sprintf((char*)buffer, "Start\r\n");
  UART_Transmit(buffer, 7);

  while(!UART_Buffer_Is_Free());

  Configure_MCP3561R(buffer, 1);

  // read from adc

  dataTx[0] = 0x41; //01 0000 01 - command byte: static read at adcdata

  SPI_Communicate(dataTx, dataRx, 4);

  HAL_Delay(100);

  while(!UART_Buffer_Is_Free());

  sprintf((char*)buffer, "%2x %2x %2x %2x\r\n", dataRx[0], dataRx[1], dataRx[2], dataRx[3]);
  UART_Transmit(buffer, 13);

  // resync sync pulses (may get out of sync with ADC communication)
//  TIM3->CNT = 0;
  dataTx[0] = 0;
  //enable sync pulse output pulse
  TIM2->CCMR1 |= (7 << TIM_CCMR1_OC1M_Pos);

  TIM6->CNT=0;
  NVIC_SetPriority(TIM6_DAC_IRQn, 0);
  NVIC_EnableIRQ(TIM6_DAC_IRQn);
  NVIC_EnableIRQ(DMA2_Stream0_IRQn);

//  unsigned int Ucounter;

  effectNumber = 0;

  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, FFT_SAMPLES);

  for (int i = 0; i < FFT_SAMPLES; i++){
	  FFTWindow[i] = 0.5f *(1 -  arm_cos_f32((float)i * 2.0f * PI / (float)(FFT_SAMPLES - 1))); // Hann window function
  }

  // find average value
  averageValue = dataIn;

  sprintf((char*)buffer, "%5i\r\n", averageValue);
  UART_Transmit(buffer, 7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
//	  if (TIM6->SR & TIM_SR_UIF){
	  if (dataReadyFlag){

		  // clear flag
		  dataReadyFlag = 0;

		  effectNumber = 11;//((GPIOC->IDR >> 8) & 0x7) + 1;

		  // Effects Here

		  switch(effectNumber){
		  	  case 0:
		  		  Clean();
		  		  break;
		  	  case 1:
		  		  Delay();
		  		  break;
		  	  case 2:
		  		  Flange();
				  break;
		  	  case 3:
				  Phaser();
				  break;
			  case 4:
				  Reverb();
				  break;
			  case 5:
				  Reverse_Delay();
				  break;
			  case 6:
				  Bit_Reduction();
				  break;
			  case 7:
				  Clouds_Delay();
				  break;
			  case 8:
				  Looper();
				  break;
			  case 9:
				  Sin_Delay();
				  break;
			  case 10:
				  Random_Sample_Delay();
				  break;
			  case 11:
				  FFT_Test(&S);
				  break;

		  }
		  // End Effects
	  }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.Speed = ETH_SPEED_100M;
  heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Clean(void){
	dataOut = dataIn;// + 0x7FFF;
}

void Delay(void){
	dataOut = sampleBuffer[sampleBufferIndex];

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= (control[0] >> 2)){
	  sampleBufferIndex = 0;
	}
}

void Flange(void){
	dataOut = sampleBuffer[(sampleBufferIndex + (lookUp[TIM12->CNT]>>9)) % 1000];

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 1000){
	  sampleBufferIndex = 0;
	}
}

void Phaser_Stage(int stageNumber, int a){
	int inputOffset = (stageNumber - 1) * 2;
	int outputOffset = stageNumber * 2;

	sampleBuffer[sampleBufferIndex + outputOffset] = (a * (sampleBuffer[!sampleBufferIndex + outputOffset] - sampleBuffer[sampleBufferIndex + inputOffset]) + \
		  (sampleBuffer[!sampleBufferIndex + inputOffset] << 8)) >> 8;

	// constrain magintude
	if (sampleBuffer[sampleBufferIndex + outputOffset] > 0xFFFF){
	  sampleBuffer[sampleBufferIndex + outputOffset] = 0xFFFF;
	}
	else if (sampleBuffer[sampleBufferIndex + outputOffset] < 0){
	  sampleBuffer[sampleBufferIndex + outputOffset] = 0;
	}
}

void Phaser(void){
	  sampleBuffer[sampleBufferIndex] = dataIn;

	  a = a0 + (lookUp[TIM12->CNT] >> 10);
	  b = b0 + (lookUp[TIM12->CNT] >> 10);
	  c = c0 + (lookUp[TIM12->CNT] >> 10);
	  d = d0 + (lookUp[TIM12->CNT] >> 10);

	  Phaser_Stage(1, a);
	  Phaser_Stage(2, b);
	  Phaser_Stage(3, c);
	  Phaser_Stage(4, d);

	  dataOut = sampleBuffer[sampleBufferIndex+8];

	  TIM12->PSC = (control[0] >> 3) > 500 ? (control[0] >> 3) : 500;

	  sampleBufferIndex++;
	  if (sampleBufferIndex >= 2){
		  sampleBufferIndex = 0;
	  }
}

void Reverb(void){
	dataOut = ((sampleBuffer[sampleBufferIndex] + \
		  sampleBuffer[(sampleBufferIndex + 2000) % 10000] + \
		  sampleBuffer[(sampleBufferIndex + 4000) % 10000] + \
		  sampleBuffer[(sampleBufferIndex + 7500) % 10000]) >> 2);

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 10000){
	  sampleBufferIndex = 0;
	}
}

void Sin_Delay(void){
	dataOut = sampleBuffer[(sampleBufferIndex + (lookUp[TIM12->CNT]>>6)) % 10000];
//	dataOut = sampleBuffer[(sampleBufferIndex + (lookUp[TIM12->CNT]>>6)) % control[0]];

	sampleBuffer[sampleBufferIndex] = dataIn;

	TIM12->PSC = (control[0] >> 2) > 1000 ? (control[0] >> 2) : 1000;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 10000){
//	if (sampleBufferIndex >= control[0]){
	  sampleBufferIndex = 0;
	}
}

void Reverse_Delay(void){
	dataOut = sampleBuffer[controlShadow[0] - sampleBufferIndex];

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;

	if (sampleBufferIndex >= controlShadow[0]){
		sampleBufferIndex = 0;
		controlShadow[0] = control[0];
	}
}


void Bit_Reduction(void){
	dataOut = dataIn & (((1 << 16) - 1) - ((1 << (control[0] >> 12)) - 1));
}


void Random_Sample_Delay(void){
	if (textureSamplesRemaining == 0){
		textureSamplesRemaining = 10000;

		// get random number
		if (RNG->SR == 0x01){
			textureSampleOffset = (RNG->DR >> 18);
		}
		else{

		}

	}
	else{
		textureSamplesRemaining--;
	}

	dataOut = sampleBuffer[(sampleBufferIndex - textureSampleOffset + 40000) % 40000];

	sampleBuffer[sampleBufferIndex] = dataIn;

	sampleBufferIndex++;
	if (sampleBufferIndex >= 40000){
	  sampleBufferIndex = 0;
	}
}


void Clouds_Delay(void){
	// check if switch pressed, read into buffer
	if (!(GPIOG->IDR & GPIO_IDR_ID2)){
		if (!isRecording){
			sampleBufferIndex = 0;
			isRecording = 1;
		}
		// index has returned to 0
		else if (sampleBufferIndex == 0){
			looperOverflow = 1;
		}
		// if currently playing write to buffer
		sampleBuffer[sampleBufferIndex] = dataIn;
		dataOut = sampleBuffer[sampleBufferIndex];

	}
	else{
		if (isRecording){
			if (looperOverflow){
				endIndex = SAMELE_BUFFER_SIZE;
			}
			else{
				endIndex = sampleBufferIndex > 10000 ? sampleBufferIndex : 10000;
			}
			isRecording = 0;
			looperOverflow = 0;
		}
		if (textureSamplesRemaining == 0){
			textureSamplesRemaining = 20000;
			// get random number
			if (RNG->SR == 0x01){
				textureSampleOffset = (RNG->DR >> 16);
			}
			else{
			}
		}
		else {
			textureSamplesRemaining--;
		}
		// read out of buffer
		dataOut = sampleBuffer[(sampleBufferIndex - textureSampleOffset + endIndex) % endIndex];
	}

	sampleBufferIndex++;
	// if recording limit to max buffer size, if looping limit to loop length
	if (sampleBufferIndex > (isRecording ? SAMELE_BUFFER_SIZE : endIndex)){
	  sampleBufferIndex = 0;
	}
}

void Looper(void){
	// check if switch pressed, read into buffer
	if (!(GPIOG->IDR & GPIO_IDR_ID2)){
		if (!isRecording){
			sampleBufferIndex = 0;
			isRecording = 1;
		}
		// index has returned to 0
		else if (sampleBufferIndex == 0){
			looperOverflow = 1;
		}
		// if currently playing write to buffer
		sampleBuffer[sampleBufferIndex] = dataIn;

	}
	else{
		if (isRecording){
			if (looperOverflow){
				endIndex = SAMELE_BUFFER_SIZE;
			}
			else{
				endIndex = sampleBufferIndex > 10000 ? sampleBufferIndex : 10000;
			}
			isRecording = 0;
			looperOverflow = 0;
		}
	}
	// read out of buffer
	dataOut = sampleBuffer[sampleBufferIndex];
	sampleBufferIndex++;

	// if recording limit to max buffer size, if looping limit to loop length
	if (sampleBufferIndex > (isRecording ? SAMELE_BUFFER_SIZE : endIndex)){ //200000){
	  sampleBufferIndex = 0;
	}
}


// input samples are stored in sample buffer indices 0 to 2 * FFT_SAMPLES
// to allow half the buffer to be filled while the other half is processed
// output is stored in 2 * FFT_SAMPLES to 4*FFT_SAMPLES


void __attribute__((optimize("Ofast"))) FFT_Test(arm_rfft_fast_instance_f32 * S){
	if (startFFTFlag){



		static float previousPhase[FFT_SAMPLES >> 1];
		static float previousShiftedPhase[FFT_SAMPLES >> 1];
		startFFTFlag = 0;

		// 0 if currently in second half of buffer
		FFTBufferSegment = ((sampleBufferIndex >> 9) + 2) & 3; // Need to change bit shift amount with FFT size

//		GPIOC->BSRR |= (GPIO_BSRR_BS6);

		// copy, cast and window
		for (int i = 0; i < FFT_SAMPLES; i++){
			// segment 3 needs to wrap past end of the buffer
			FFTBuffer[i] = ((float)(sampleBuffer[(FFTBufferSegment * (FFT_SAMPLES >> 1) + i) & (2 * FFT_SAMPLES - 1)]) - (float)averageValue) * FFTWindow[i] + (float)averageValue;
		}

		// FFT
		arm_rfft_fast_f32(S, FFTBuffer, FFTOutBuffer, 0);

		float magnitude[FFT_SAMPLES >> 1];
		float phase[FFT_SAMPLES >> 1];

		// phase and magnitude
		arm_cmplx_mag_f32(FFTOutBuffer, magnitude, FFT_SAMPLES>>1);
		for (int i = 0; i < (FFT_SAMPLES >> 1); i++){
//			arm_atan2_f32(FFTOutBuffer[2 * i + 1], FFTOutBuffer[2 * i], &phase[i]);
			phase[i] = atan2f(FFTOutBuffer[2 * i + 1], FFTOutBuffer[2 * i]);
		}

		// Processing
		float fractionalBins[FFT_SAMPLES >> 1];

		for (int i = 0; i < (FFT_SAMPLES >> 1); i++){
			float phaseDifference = phase[i] - previousPhase[i] - 2.0f * PI * (float)i / (float)FFT_TO_HOP_RATIO; //HOP_SIZE/FFT_SAMPLES;
			// constraint to -PI<p<PI

			///
		    if (phaseDifference >= 0)
		    	phaseDifference = fmodf(phaseDifference + PI, 2.0f * PI) - PI;
		    else
		    	phaseDifference = fmodf(phaseDifference - PI, -2.0f * PI) + PI;
		    ///

			// calculate exact frequency
			fractionalBins[i] = phaseDifference * (float)FFT_SAMPLES / (2.0f * PI * (float)HOP_SIZE) + (float)i;

//			if(UART_Buffer_Is_Free()){
//				sprintf((char*)buffer, "%.3i %5.2f %5.1f\n\r", i, phaseDifference, fractionalBins[i]);
//				UART_Transmit(buffer, 17);
//			}

			previousPhase[i] = phase[i]; // save phase
		}

		volatile float shiftedMagnitude[FFT_SAMPLES >> 1];
		float shiftedFractionalBins[FFT_SAMPLES >> 1];

		for (int i = 0; i < (FFT_SAMPLES >> 1); i++){
			shiftedMagnitude[i] = 0.0f;
			shiftedFractionalBins[i] = 0.0f;
		}

		for (int i = 0; i < (FFT_SAMPLES >> 1); i++){
			int newBin = (int)(4.0f * (float)i + 0.5f);// * 4.0f + 0.5f);
			// only use if the frequency is below nyquist frequency
			if (newBin < (FFT_SAMPLES >> 1)){
				shiftedMagnitude[newBin] = magnitude[i];
				shiftedFractionalBins[newBin] = fractionalBins[i] * 4.0f;// * 4.0f;
//				shiftedMagnitude[i] = magnitude[i];
//				shiftedFractionalBins[i] = fractionalBins[i];
			}
		}

		// phase from frequency
		for (int i = 0; i < (FFT_SAMPLES >> 1); i++){
			float phaseDifference = (shiftedFractionalBins[i] - (float)i) * 2.0f * PI / (float)FFT_TO_HOP_RATIO;// * HOP_SIZE/FFT_SAMPLES;

			phase[i] = previousShiftedPhase[i] + phaseDifference + 2.0f * PI * (float)i / (float)FFT_TO_HOP_RATIO;;// * HOP_SIZE / (float)FFT_SAMPLES;//

			// constraint to -PI<p<PI
			///
		    if (phase[i] >= 0)
		    	phase[i] = fmodf(phase[i] + PI, 2.0f * PI) - PI;
		    else
		    	phase[i] = fmodf(phase[i] - PI, -2.0f * PI) + PI;
		    ///

			previousShiftedPhase[i] = phase[i]; // save phase
		}

		// real and complex parts
		for (int i = 0; i < (FFT_SAMPLES >> 1); i++){
			FFTOutBuffer[2 * i] = shiftedMagnitude[i] * arm_cos_f32(phase[i]); // real
			FFTOutBuffer[2 * i + 1] = shiftedMagnitude[i] * arm_sin_f32(phase[i]); // imaginary
		}

		// IFFT
//		FFTOutBuffer[1] = 0.0f;
		arm_rfft_fast_f32(S, FFTOutBuffer, FFTBuffer, 1);

		// copy back, cast and re-window
		for (int i = 0; i < FFT_SAMPLES; i++){

			int FFTOut = sampleBuffer[2 * FFT_SAMPLES + ((FFTBufferSegment * (FFT_SAMPLES >> 1) + i) & (2 * FFT_SAMPLES - 1))] + ((int)FFTBuffer[i] - averageValue) * FFTWindow[i];

			if (FFTOut > 0xFFFF){
			  FFTOut = 0xFFFF;
			}
			else if (FFTOut < 0){
				FFTOut = 0;
			}
			sampleBuffer[2 * FFT_SAMPLES + ((FFTBufferSegment * (FFT_SAMPLES >> 1) + i) & (2 * FFT_SAMPLES - 1))] = (unsigned short)FFTOut;// - 32768.0f) * FFTWindow[i] + 32768.0f;
		}

//		GPIOC->BSRR |= (GPIO_BSRR_BR6);
		if (startFFTFlag){
			  if(UART_Buffer_Is_Free()){
				  sprintf((char*)buffer, "Slow\r\n");
				  UART_Transmit(buffer, 6);
			  }
		}


	}
}

void TIM6_DAC_IRQHandler(void){
//	dataOut = lookUp[TIM12->CNT] >> 1;
//	dataOut = dataIn;
  	dataTx[1] = dataOut >> 8;
  	dataTx[2] = dataOut;

	// analog read/write
	SPI_Communicate(dataTx, dataRx, 3);

	//clear flag
	TIM6->SR = 0;//&= ~TIM_SR_UIF;
}

void DMA2_Stream0_IRQHandler(void){
//	TIM3->CNT = 0;
	dataIn = ((dataRx[0] << 8) + dataRx[1]) << 1;

//	 FFT
	if (effectNumber == 11){
		sampleBuffer[sampleBufferIndex] = dataIn;
		dataOut = sampleBuffer[sampleBufferIndex + 2 * FFT_SAMPLES] + averageValue;

		sampleBuffer[sampleBufferIndex + 2 * FFT_SAMPLES] = 0; // clear sample

		// set flag if sample index = FFT_SAMPLES * n / 2, n = 0,1,2,3
		if (!(sampleBufferIndex & (FFT_SAMPLES/2 - 1))){
			startFFTFlag = 1;
		}
		sampleBufferIndex++;
		if (sampleBufferIndex >= (2 * FFT_SAMPLES)){
		  sampleBufferIndex = 0;
		}
	}

	dataReadyFlag = 1;

	//clear flag
	DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
}

/*
// handles data for FFT
void DMA2_Stream0_IRQHandler(void){

	sampleBuffer[sampleBufferIndex] = dataIn;

//	FFTBuffer[sampleBufferIndex % FFT_SAMPLES] = (float)sampleBuffer[sampleBufferIndex];
//	FFTBuffer[sampleBufferIndex % FFT_SAMPLES] = (float)sampleBuffer[(FFTBufferSegment * FFT_SAMPLES + (sampleBufferIndex % FFT_SAMPLES))];


//	dataOut = (unsigned short)FFTBuffer[sampleBufferIndex % FFT_SAMPLES];

	dataOut = sampleBuffer[sampleBufferIndex];//sampleBuffer[2 * FFT_SAMPLES + sampleBufferIndex];
//	sampleBuffer[2 * FFT_SAMPLES + sampleBufferIndex] = 0;

//	dataOut = sampleBuffer[(sampleBufferIndex & (FFT_SAMPLES - 1))];

	// set flag if sample index = FFT_SAMPLES * n / 2, n = 0,1,2,3
//	if (!(sampleBufferIndex & (FFT_SAMPLES / 2 - 1))){

	if (!(sampleBufferIndex & (FFT_SAMPLES - 1))){
//		FFTBufferSegment = (FFTBufferSegment + 1) & 1;
		startFFTFlag = 1;
	}
//	if (sampleBufferIndex == FFT_SAMPLES){
//		startFFTFlag = 1;
//	}

//	while(!UART_Buffer_Is_Free());
//	sprintf((char*)buffer, "%4li\r\n", sampleBufferIndex);
//	UART_Transmit(buffer, 6);

	sampleBufferIndex++;
	if (sampleBufferIndex >= (2 * FFT_SAMPLES)){
	  sampleBufferIndex = 0;
	}

	//clear flag
	DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
}*/

void EXTI9_5_IRQHandler(void){
	effectNumber = (GPIOC->IDR >> 8) & 0x0F;

	//clear flag
	EXTI->PR |= EXTI_PR_PR8;
}

void Configure_MCP3561R(unsigned char *buffer, char readback){
	  // configure ADC registers

	  dataTx[0] = 0x46; //01 0001 10 - command byte: write starting at config0
//	  dataTx[1] = 0xE3; // 1110 0011 - config 0: internal reference, start adc conversions
	  dataTx[1] = 0x63; // 0110 0011 - config 0: external reference, start adc conversions
	  dataTx[2] = 0x00; // 0000 0000 - config 1: 16bit adc
	  dataTx[3] = 0x89; // 1000 1001 - config 2: gain = 1; no auto zero
	  dataTx[4] = 0xC0; // 1100 0000 - config 3: continuous conversion, 24bit adc register, no calibration
	  dataTx[5] = 0x77; // 0111 0111 - IRQ
	  dataTx[6] = 0x08; // 0000 1000 - Multiplexer: V+ = CH0, V- = GND


	  SPI_Communicate(dataTx, dataRx, 7);

	  HAL_Delay(100);

	  GPIOA->BSRR |= (GPIO_BSRR_BS4);

	  // read back adc oncfig

	  if (readback){
		  dataTx[0] = 0x47; //01 0001 11 - command byte: read starting at config0

		  SPI_Communicate(dataTx, dataRx, 7);

		  HAL_Delay(100);

		  // Slave select high
		  GPIOA->BSRR |= (GPIO_BSRR_BS4);

		  for (int i = 0; i < 7; i++){
			  while(!UART_Buffer_Is_Free());

			  sprintf((char*)buffer, "%3x\r\n", dataRx[i]);
			  UART_Transmit(buffer, 5);
		  }
	  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
