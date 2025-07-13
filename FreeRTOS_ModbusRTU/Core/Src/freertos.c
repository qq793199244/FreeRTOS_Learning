/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern ModbusRtuSlaveStruct slaveStr;
extern MotorDriverStruct	 motorDriverStr;
extern LoadStruct 		loadStr;

extern uint8_t REC_CTRL_MOTOR_FLAG;
extern uint8_t REC_CTRL_LOAD_FLAG;
extern uint8_t REC_CTRL_ARRAY_IDX;


const uint8_t send_to_motor[4][8] = {
		{0x01, 0x03, 0x71, 0x10, 0x00, 0x0A, 0xDF, 0x34},
		{0x02, 0x03, 0x71, 0x10, 0x00, 0x0A, 0xDF, 0x07},
		{0x03, 0x03, 0x71, 0x10, 0x00, 0x0A, 0xDE, 0xD6},
		{0x04, 0x03, 0x71, 0x10, 0x00, 0x0A, 0xDF, 0x61}
};
const uint8_t send_to_load[4][8] = {
		{0x01, 0x03, 0x00, 0x37, 0x00, 0x02, 0x75, 0xC5},
		{0x02, 0x03, 0x00, 0x37, 0x00, 0x02, 0x75, 0xF6},
		{0x03, 0x03, 0x00, 0x37, 0x00, 0x02, 0x74, 0x27},
		{0x04, 0x03, 0x00, 0x37, 0x00, 0x02, 0x75, 0x90}
};


/* USER CODE END Variables */
/* Definitions for RecvData_Task */
osThreadId_t RecvData_TaskHandle;
const osThreadAttr_t RecvData_Task_attributes = {
  .name = "RecvData_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ProcData_Task */
osThreadId_t ProcData_TaskHandle;
const osThreadAttr_t ProcData_Task_attributes = {
  .name = "ProcData_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MoniMotor_Task */
osThreadId_t MoniMotor_TaskHandle;
const osThreadAttr_t MoniMotor_Task_attributes = {
  .name = "MoniMotor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Ctrl_Load_Task */
osThreadId_t Ctrl_Load_TaskHandle;
const osThreadAttr_t Ctrl_Load_Task_attributes = {
  .name = "Ctrl_Load_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Main_Ctrl_Task */
osThreadId_t Main_Ctrl_TaskHandle;
const osThreadAttr_t Main_Ctrl_Task_attributes = {
  .name = "Main_Ctrl_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for frameSendSemaphore */
osSemaphoreId_t frameSendSemaphoreHandle;
const osSemaphoreAttr_t frameSendSemaphore_attributes = {
  .name = "frameSendSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_RecvData(void *argument);
void Start_Process_Data(void *argument);
void Start_MonitorMotor_Data(void *argument);
void Start_Control_Load(void *argument);
void Start_Main_Control(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of frameSendSemaphore */
  frameSendSemaphoreHandle = osSemaphoreNew(1, 1, &frameSendSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RecvData_Task */
  RecvData_TaskHandle = osThreadNew(Start_RecvData, NULL, &RecvData_Task_attributes);

  /* creation of ProcData_Task */
  ProcData_TaskHandle = osThreadNew(Start_Process_Data, NULL, &ProcData_Task_attributes);

  /* creation of MoniMotor_Task */
  MoniMotor_TaskHandle = osThreadNew(Start_MonitorMotor_Data, NULL, &MoniMotor_Task_attributes);

  /* creation of Ctrl_Load_Task */
  Ctrl_Load_TaskHandle = osThreadNew(Start_Control_Load, NULL, &Ctrl_Load_Task_attributes);

  /* creation of Main_Ctrl_Task */
  Main_Ctrl_TaskHandle = osThreadNew(Start_Main_Control, NULL, &Main_Ctrl_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_RecvData */
/**
  * @brief  Function implementing the RecvData_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_RecvData */
void Start_RecvData(void *argument)
{
  /* USER CODE BEGIN Start_RecvData */
  /* Infinite loop */
  for(;;)
  {
	  // UART1
	  if (1 == UART1_RX_INT_FLAG) {
		  if (uart1_rx_len > 0 && uart1_rx_len <= UART1_RX_BUF_SIZE) {
			  memcpy(slaveStr.RecvBuff, uart1_rx_buf, uart1_rx_len);
			  slaveStr.recCnt = uart1_rx_len;
			  slaveStr.recFrameFlag = 1;
		  }
		  else {
			  uart1_rx_len = 0;
		  }
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buf, UART1_RX_BUF_SIZE);
		  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

		  UART1_RX_INT_FLAG = 0;
	  }

	  // UART2
	  if (1 == UART2_RX_INT_FLAG) {
		  if (uart2_rx_len > 0 && uart2_rx_len <= UART2_RX_BUF_SIZE) {
			  memcpy(motorDriverStr.RecvBuff, uart2_rx_buf, uart2_rx_len);
			  motorDriverStr.recCnt = uart2_rx_len;
			  motorDriverStr.recFrameFlag = 1;
		  }
		  else {
			  uart2_rx_len = 0;
		  }
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_buf, UART2_RX_BUF_SIZE);
		  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

		  UART2_RX_INT_FLAG = 0;
	  }

	  // UART3
	  if (1 == UART3_RX_INT_FLAG) {
		  if (uart3_rx_len > 0 && uart3_rx_len <= UART3_RX_BUF_SIZE) {
			  memcpy(loadStr.RecvBuff, uart3_rx_buf, uart3_rx_len);
			  loadStr.recCnt = uart3_rx_len;
			  loadStr.recFrameFlag = 1;
		  }
		  else {
			  uart3_rx_len = 0;
		  }
		  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_buf, UART3_RX_BUF_SIZE);
		  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

		  UART3_RX_INT_FLAG = 0;
	  }


    osDelay(20);
  }
  /* USER CODE END Start_RecvData */
}

/* USER CODE BEGIN Header_Start_Process_Data */
/**
* @brief Function implementing the ProcData_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Process_Data */
void Start_Process_Data(void *argument)
{
  /* USER CODE BEGIN Start_Process_Data */
  /* Infinite loop */

  for(;;)
  {
	  if (slaveStr.recFrameFlag) {
		  ProcessData_RecvFromPLC();
	  }

	  if (motorDriverStr.recFrameFlag) {
		  ProcessData_RecvFromMotorDriver();
	  }

	  if (loadStr.recFrameFlag) {
		  ProcessData_RecvFromLoad();
	  }

    osDelay(20);
  }
  /* USER CODE END Start_Process_Data */
}

/* USER CODE BEGIN Header_Start_MonitorMotor_Data */
/**
* @brief Function implementing the MoniMotor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MonitorMotor_Data */
void Start_MonitorMotor_Data(void *argument)
{
  /* USER CODE BEGIN Start_MonitorMotor_Data */
  /* Infinite loop */
  for(;;)
  {
//	  osStatus_t status = osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever);
	  switch (MONITORING_MOTOR_FLAG) {
	  	  case 1:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					// 发送查询
					HAL_UART_Transmit_DMA(&huart2, send_to_motor[0], 8);
					osDelay(10);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}
				MONITORING_MOTOR_FLAG = 0;
	  		  break;

	  	  case 2:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					// 发送查询
					HAL_UART_Transmit_DMA(&huart2, send_to_motor[1], 8);
					osDelay(10);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}
				MONITORING_MOTOR_FLAG = 0;
			  break;

	  	  case 3:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					// 发送查询
					HAL_UART_Transmit_DMA(&huart2, send_to_motor[2], 8);
					osDelay(10);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}
				MONITORING_MOTOR_FLAG = 0;
			  break;

	  	  case 4:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					// 发送查询
					HAL_UART_Transmit_DMA(&huart2, send_to_motor[3], 8);
					osDelay(10);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}
				MONITORING_MOTOR_FLAG = 0;
			  break;
	  }
    osDelay(20);
  }
  /* USER CODE END Start_MonitorMotor_Data */
}

/* USER CODE BEGIN Header_Start_Control_Load */
/**
* @brief Function implementing the Ctrl_Load_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Control_Load */
void Start_Control_Load(void *argument)
{
  /* USER CODE BEGIN Start_Control_Load */
  /* Infinite loop */
  for(;;)
  {
	  switch (MONITORING_LOAD_FLAG) {
		  case 1:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					HAL_UART_Transmit_DMA(&huart3, send_to_load[0], 8);
					osDelay(40);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误�?
				}

				if (1 == slaveStr.ReadRegsGroup[0]) {
					if (slaveStr.ReadRegsGroup[6] > 2600) {
						uint8_t send_data[8] = {0x01, 0x06, 0x00, 0x36, 0x00, 0x28,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
					if (slaveStr.ReadRegsGroup[6] < 1700) {
						uint8_t send_data[8] = {0x01, 0x06, 0x00, 0x36, 0x01, 0x5E,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
				}

				MONITORING_LOAD_FLAG = 0;
			  break;

		  case 2:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					HAL_UART_Transmit_DMA(&huart3, send_to_load[1], 8);
					osDelay(25);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误�?
				}

				if (1 == slaveStr.ReadRegsGroup[10]) {
					if (slaveStr.ReadRegsGroup[16] > 2600) {
						uint8_t send_data[8] = {0x02, 0x06, 0x00, 0x36, 0x00, 0x28,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
					if (slaveStr.ReadRegsGroup[16] < 1700) {
						uint8_t send_data[8] = {0x02, 0x06, 0x00, 0x36, 0x01, 0x5E,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
				}

				MONITORING_LOAD_FLAG = 0;
			  break;

		  case 3:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					HAL_UART_Transmit_DMA(&huart3, send_to_load[2], 8);
					osDelay(25);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误）
				}
				if (1 == slaveStr.ReadRegsGroup[20]) {
					if (slaveStr.ReadRegsGroup[26] > 2600) {
						uint8_t send_data[8] = {0x03, 0x06, 0x00, 0x36, 0x00, 0x28,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
					if (slaveStr.ReadRegsGroup[26] < 1700) {
						uint8_t send_data[8] = {0x03, 0x06, 0x00, 0x36, 0x01, 0x5E,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
				}

				MONITORING_LOAD_FLAG = 0;
			  break;

		  case 4:
				if (osSemaphoreAcquire(frameSendSemaphoreHandle, osWaitForever) == osOK) {
					HAL_UART_Transmit_DMA(&huart3, send_to_load[3], 8);
					osDelay(25);
					osSemaphoreRelease(frameSendSemaphoreHandle);
				}
				else {
						// 处理信号量获取失败的情况（如记录错误�?
				}
				if (1 == slaveStr.ReadRegsGroup[30]) {
					if (slaveStr.ReadRegsGroup[36] > 2600) {
						uint8_t send_data[8] = {0x03, 0x06, 0x00, 0x36, 0x00, 0x28,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
					if (slaveStr.ReadRegsGroup[36] < 1700) {
						uint8_t send_data[8] = {0x03, 0x06, 0x00, 0x36, 0x01, 0x5E,};
						UART_SendDataWithSemaphore(&huart3, send_data, 8, osWaitForever, 5);
					}
				}

				MONITORING_LOAD_FLAG = 0;
			  break;
	  }

	  osDelay(20);
  }
  /* USER CODE END Start_Control_Load */
}

/* USER CODE BEGIN Header_Start_Main_Control */
/**
* @brief Function implementing the Main_Ctrl_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Main_Control */
void Start_Main_Control(void *argument)
{
  /* USER CODE BEGIN Start_Main_Control */
  /* Infinite loop */
  for(;;)
  {
	  if (REC_CTRL_MOTOR_FLAG) {
		  ProcessData_TransToMotorDriver();
		  REC_CTRL_MOTOR_FLAG = 0;
	  }

	  if (REC_CTRL_LOAD_FLAG) {
		  ProcessData_TransToLoad();
		  REC_CTRL_LOAD_FLAG = 0;
	  }

    osDelay(20);
  }
  /* USER CODE END Start_Main_Control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

