/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../BSP/slave_to_plc.h"
#include "../../BSP/master_to_motordriver.h"
#include "../../BSP/master_to_load.h"
#include "string.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define UART1_RX_BUF_SIZE	128
#define UART2_RX_BUF_SIZE	64
#define UART3_RX_BUF_SIZE	16

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern  uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];
extern volatile uint8_t uart1_rx_len;

extern  uint8_t uart2_rx_buf[UART2_RX_BUF_SIZE];
extern volatile uint8_t uart2_rx_len;

extern  uint8_t uart3_rx_buf[UART3_RX_BUF_SIZE];
extern volatile uint8_t uart3_rx_len;

extern uint8_t UART1_RX_INT_FLAG;
extern uint8_t UART2_RX_INT_FLAG;
extern uint8_t UART3_RX_INT_FLAG;

extern uint8_t MONITORING_MOTOR_FLAG;
extern uint8_t MONITORING_LOAD_FLAG;

extern osSemaphoreId_t frameSendSemaphoreHandle;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
