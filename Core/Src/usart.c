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

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

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

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_USART1_ENABLE();

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* Enable USART1 IRQ (used by HAL transmit/receive IT) */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
#include <string.h>
#include <stdlib.h>

static volatile int32_t g_target_speed = 0;

/* DMA RX buffer and parser state */
#define UART_RX_BUF_SIZE 128
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
static uint32_t uart_rx_last_pos = 0;
static char uart_line[48];
static uint32_t uart_line_idx = 0;

/* Start DMA circular reception into uart_rx_buf */
void UART1_Start_DMA(void)
{
  uart_rx_last_pos = 0;
  uart_line_idx = 0;
  memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
  HAL_UART_Receive_DMA(&huart1, uart_rx_buf, UART_RX_BUF_SIZE);
}

/* Return last parsed target speed */
int32_t Get_TargetSpeed(void)
{
  return g_target_speed;
}

/* Safe non-blocking send (skip if busy) */
void UART1_SendString(const char *s)
{
  if (s == NULL) return;
  if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY) {
    HAL_UART_Transmit_IT(&huart1, (uint8_t *)s, (uint16_t)strlen(s));
  } else {
    /* fallback: blocking transmit with timeout */
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
  }
}

/* Call periodically to parse new bytes placed into uart_rx_buf by DMA */
void UART1_Process(void)
{
  DMA_HandleTypeDef *hdma = huart1.hdmarx;
  if (hdma == NULL) return; /* DMA not linked */

  uint32_t pos = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(hdma);
  while (uart_rx_last_pos != pos) {
    uint8_t c = uart_rx_buf[uart_rx_last_pos++];
    if (uart_rx_last_pos >= UART_RX_BUF_SIZE) uart_rx_last_pos = 0;

    if (c == '\r' || c == '\n') {
      if (uart_line_idx > 0) {
        uart_line[uart_line_idx] = '\0';
        g_target_speed = atoi(uart_line);
        uart_line_idx = 0;
      }
    } else {
      if (uart_line_idx + 1 < sizeof(uart_line)) {
        uart_line[uart_line_idx++] = (char)c;
      } else {
        /* overflow, reset line */
        uart_line_idx = 0;
      }
    }
  }
}

/* USER CODE END 1 */
