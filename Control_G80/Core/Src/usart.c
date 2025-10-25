/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
#include <stdio.h>
#include <string.h>
uint8_t Flag = 0x00;
uint8_t message[128];
static char RxFlag[] = "RxOK\r\n";

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (uartHandle->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */

    /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspInit 1 */

    /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{

  if (uartHandle->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */

    /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
    /* USER CODE BEGIN USART1_MspDeInit 1 */

    /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// void USART1_IRQHandler(void)
//{
//     HAL_UART_IRQHandler(&huart1);
// }

int16_t SpeedTarget = 0x0000;
static uint8_t FlagBit = 0x00;
/// @brief 解算数据
static void ParsData(void)
{

  {
    float tem = 000.0f;
    if (FlagBit == 2)
    {
      SpeedTarget = (message[1] - 0x30) * 10 + (message[3] - 0x30) * 1;
      SpeedTarget = SpeedTarget / 1000.00f * 2000;
    }
    else if (FlagBit == 3)
    {
      SpeedTarget = (message[1] - 0x30) * 100 + (message[2] - 0x30) * 10 + (message[4] - 0x30) * 1;
      SpeedTarget = SpeedTarget / 1000.00f * 2000;
    }
    else
    {
      SpeedTarget = 2000;
    }
    SpeedTarget = SpeedTarget > 2000 ? 2000 : SpeedTarget;
    SpeedTarget = SpeedTarget < 800 ? 800 : SpeedTarget;
  }
}

/// @brief 设置PWM占空比
void Set_PWM()
{
  if (Flag)
  {
    uint32_t tempdata = SpeedTarget;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, SpeedTarget);
    Flag = 0x00;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    if (message[0] == 0x3A)
    {
      if (message[1] == 's')
      {
        SpeedTarget = 0;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)RxFlag, sizeof(RxFlag));
      }
      else if (message[1] == 'f')
      {
        SpeedTarget = 2000;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)RxFlag, sizeof(RxFlag));
      }
      else if ((message[2] == 0x2E) || (message[3] == 0x2E) || (message[4] == 0x2E))
      {
        if (message[2] == 0x2E)
        {
          FlagBit = 2;
        }
        else if (message[3] == 0x2E)
        {
          FlagBit = 3;
        }
        else
        {
          FlagBit = 4;
        }
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)RxFlag, sizeof(RxFlag));

        ParsData();
      }

      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, message, Size);
      __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
      memset(message, 0xFF, Size);
      Flag = 0x01;
    }
    Set_PWM();
  }
}
/* USER CODE END 1 */
