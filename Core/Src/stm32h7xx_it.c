/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim17;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
	volatile uint16_t len;
	uint8_t string[50];
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)==SET)
  {
		HAL_GPIO_TogglePin(SIGNAL_GPIO_Port,SIGNAL_Pin);
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
		len = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
		HAL_UART_DMAStop(&huart3);
		USART5_RX_Buffer[50-len]=0;
		sprintf(string,"l:%d  s:%s\r\n",len,USART5_RX_Buffer);
		HAL_UART_Transmit(&huart1,string,strlen(string),0xFFFF);
  }
	HAL_UART_Receive_DMA(&huart3,USART5_RX_Buffer,50);
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
	volatile uint16_t len;
	uint8_t string[50];
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
	if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE)==SET)
  {
		HAL_GPIO_TogglePin(SIGNAL_GPIO_Port,SIGNAL_Pin);
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
		len = __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
		HAL_UART_DMAStop(&huart4);
		USART5_RX_Buffer[50-len]=0;
		sprintf(string,"l:%d  s:%s\r\n",len,USART5_RX_Buffer);
		HAL_UART_Transmit(&huart1,string,strlen(string),0xFFFF);
  }
	HAL_UART_Receive_DMA(&huart4,USART5_RX_Buffer,50);
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  static uint8_t n=0,string[100];
  static uint16_t RemoteChannel[5];
	int16_t CH[16];
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
	if(__HAL_UART_GET_FLAG(&huart5,UART_FLAG_IDLE)==SET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    if(USART5_RX_Buffer[0]==0x0F)
    {
      CH[ 0] = ((int16_t)USART5_RX_Buffer[ 1] >> 0 | ((int16_t)USART5_RX_Buffer[ 2] << 8 )) & 0x07FF;
      CH[ 1] = ((int16_t)USART5_RX_Buffer[ 2] >> 3 | ((int16_t)USART5_RX_Buffer[ 3] << 5 )) & 0x07FF;
      CH[ 2] = ((int16_t)USART5_RX_Buffer[ 3] >> 6 | ((int16_t)USART5_RX_Buffer[ 4] << 2 )  | (int16_t)USART5_RX_Buffer[ 5] << 10 ) & 0x07FF;
      CH[ 3] = ((int16_t)USART5_RX_Buffer[ 5] >> 1 | ((int16_t)USART5_RX_Buffer[ 6] << 7 )) & 0x07FF;
      CH[ 4] = ((int16_t)USART5_RX_Buffer[ 6] >> 4 | ((int16_t)USART5_RX_Buffer[ 7] << 4 )) & 0x07FF;
      CH[ 5] = ((int16_t)USART5_RX_Buffer[ 7] >> 7 | ((int16_t)USART5_RX_Buffer[ 8] << 1 )  | (int16_t)USART5_RX_Buffer[ 9] <<  9 ) & 0x07FF;
      CH[ 6] = ((int16_t)USART5_RX_Buffer[ 9] >> 2 | ((int16_t)USART5_RX_Buffer[10] << 6 )) & 0x07FF;
      CH[ 7] = ((int16_t)USART5_RX_Buffer[10] >> 5 | ((int16_t)USART5_RX_Buffer[11] << 3 )) & 0x07FF;

      CH[ 8] = ((int16_t)USART5_RX_Buffer[12] << 0 | ((int16_t)USART5_RX_Buffer[13] << 8 )) & 0x07FF;
      CH[ 9] = ((int16_t)USART5_RX_Buffer[13] >> 3 | ((int16_t)USART5_RX_Buffer[14] << 5 )) & 0x07FF;
      CH[10] = ((int16_t)USART5_RX_Buffer[15] >> 6 | ((int16_t)USART5_RX_Buffer[15] << 2 )  | (int16_t)USART5_RX_Buffer[16] << 10 ) & 0x07FF;
      CH[11] = ((int16_t)USART5_RX_Buffer[16] >> 1 | ((int16_t)USART5_RX_Buffer[17] << 7 )) & 0x07FF;
      CH[12] = ((int16_t)USART5_RX_Buffer[17] >> 4 | ((int16_t)USART5_RX_Buffer[18] << 4 )) & 0x07FF;
      CH[13] = ((int16_t)USART5_RX_Buffer[18] >> 7 | ((int16_t)USART5_RX_Buffer[19] << 1 )  | (int16_t)USART5_RX_Buffer[20] <<  9 ) & 0x07FF;
      CH[14] = ((int16_t)USART5_RX_Buffer[20] >> 2 | ((int16_t)USART5_RX_Buffer[21] << 6 )) & 0x07FF;
      CH[15] = ((int16_t)USART5_RX_Buffer[21] >> 5 | ((int16_t)USART5_RX_Buffer[22] << 3 )) & 0x07FF;
    }
		for(n=0;n<5;n++)
    {
      RemoteChannel[n] = (CH[n]-352)*0.744f+1000;
    }
		sprintf(string,"%d  %d  %d  %d  %d\r\n",RemoteChannel[0],RemoteChannel[1],RemoteChannel[2],RemoteChannel[3],RemoteChannel[4]);
//		sprintf(string,"%u  %u  %u  %u  %u\r\n",CH[0],CH[1],CH[2],CH[3],CH[4]);
		f_printf(&SDFile,string);
		f_sync(&SDFile);
		HAL_UART_Transmit(&huart1,string,strlen(string),0xFFFF);
  }
	HAL_UART_Receive_DMA(&huart5,USART5_RX_Buffer,50);
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */
  /* USER CODE END TIM17_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
