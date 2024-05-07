/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t uart_rx_data[1];
uint8_t spi_rx_data[1];
uint8_t uart_to_spi[100];
uint8_t spi_to_uart[100];
uint8_t uart_to_spi_lenght=0;
uint8_t spi_to_uart_lenght=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_comm */
osThreadId_t UART_commHandle;
const osThreadAttr_t UART_comm_attributes = {
  .name = "UART_comm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SPI_comm */
osThreadId_t SPI_commHandle;
const osThreadAttr_t SPI_comm_attributes = {
  .name = "SPI_comm",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uart_RX_ */
osEventFlagsId_t uart_RX_Handle;
const osEventFlagsAttr_t uart_RX__attributes = {
  .name = "uart_RX_"
};
/* Definitions for uart_TX_ */
osEventFlagsId_t uart_TX_Handle;
const osEventFlagsAttr_t uart_TX__attributes = {
  .name = "uart_TX_"
};
/* Definitions for spi_RX_ */
osEventFlagsId_t spi_RX_Handle;
const osEventFlagsAttr_t spi_RX__attributes = {
  .name = "spi_RX_"
};
/* Definitions for spi_TX_ */
osEventFlagsId_t spi_TX_Handle;
const osEventFlagsAttr_t spi_TX__attributes = {
  .name = "spi_TX_"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	osEventFlagsSet(uart_RX_Handle, 0x01);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	osEventFlagsSet(uart_TX_Handle, 0x01);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *huart)
{
	osEventFlagsSet(spi_RX_Handle, 0x01);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *huart)
{
	osEventFlagsSet(spi_TX_Handle, 0x01);
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void UART_communication(void *argument);
void SPI_communication(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_comm */
  UART_commHandle = osThreadNew(UART_communication, NULL, &UART_comm_attributes);

  /* creation of SPI_comm */
  SPI_commHandle = osThreadNew(SPI_communication, NULL, &SPI_comm_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of uart_RX_ */
  uart_RX_Handle = osEventFlagsNew(&uart_RX__attributes);

  /* creation of uart_TX_ */
  uart_TX_Handle = osEventFlagsNew(&uart_TX__attributes);

  /* creation of spi_RX_ */
  spi_RX_Handle = osEventFlagsNew(&spi_RX__attributes);

  /* creation of spi_TX_ */
  spi_TX_Handle = osEventFlagsNew(&spi_TX__attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UART_communication */
/**
* @brief Function implementing the UART_comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_communication */
void UART_communication(void *argument)
{
  /* USER CODE BEGIN UART_communication */
	HAL_SPI_Receive_DMA(&hspi1,spi_rx_data,2);
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(spi_RX_Handle, 0x01, osFlagsWaitAny, osWaitForever);


	while(spi_rx_data[0]!= 0U){
		spi_to_uart[uart_to_spi_lenght] = uart_rx_data[0];
		spi_to_uart[uart_to_spi_lenght+1] = uart_rx_data[1];
		HAL_SPI_Receive(&hspi1,spi_rx_data,2,50);
		spi_to_uart_lenght+=2;
	}

	if (spi_to_uart_lenght != 0) {
		HAL_UART_Transmit_DMA(&huart2,spi_to_uart,spi_to_uart_lenght);
		spi_to_uart_lenght = 0;
	}


	HAL_SPI_Receive_DMA(&hspi1,spi_rx_data,1);
  }
  /* USER CODE END UART_communication */
}

/* USER CODE BEGIN Header_SPI_communication */
/**
* @brief Function implementing the SPI_comm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SPI_communication */
void SPI_communication(void *argument)
{
  /* USER CODE BEGIN SPI_communication */
  HAL_UART_Receive_DMA(&huart2,uart_to_spi,1);
  /* Infinite loop */
  for(;;)
  {
    osEventFlagsWait(uart_RX_Handle, 0x01, osFlagsWaitAny, osWaitForever);

    while(uart_rx_data[0]!= '\0'){
		uart_to_spi[uart_to_spi_lenght] = uart_rx_data[0];
		HAL_UART_Receive(&huart2,uart_rx_data,1,50);
		uart_to_spi_lenght++;
	}

    if (uart_to_spi_lenght != 0) {
    	HAL_SPI_Transmit_DMA(&hspi1,uart_to_spi,uart_to_spi_lenght);
    	uart_to_spi_lenght = 0;
    }


    HAL_UART_Receive_DMA(&huart2,uart_rx_data,1);
  }
  /* USER CODE END SPI_communication */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

