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
#include "usart.h"
#include "gpio.h"
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

/* USER CODE END Variables */

/* Definitions for tasks */
const osThreadAttr_t toggleGreenTask_attributes = {
  .name = "ToggleGreenTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t toggleRedTask_attributes = {
  .name = "ToggleRedTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t uartCommandTask_attributes = {
  .name = "UARTCommandTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ToggleGreenTask(void *argument);
void ToggleRedTask(void *argument);
void UARTCommandTask(void *argument);
/* USER CODE END FunctionPrototypes */

/* USER CODE BEGIN Application */

extern UART_HandleTypeDef huart3;  // Usando USART3

void ToggleGreenTask(void *argument) {
  for (;;) {
    // Azul ON (PA3)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    // Parpadea verde (PC0) 20Hz durante 4 segundos (160 toggles * 25ms)
    for (int i = 0; i < 160; ++i) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
      osDelay(25);
    }

    // Verde OFF
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    // Azul OFF
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    // Esperar 2 segundos para completar ciclo de 6s
    osDelay(2000);
  }
}

void ToggleRedTask(void *argument) {
  for (;;) {
    // Amarillo ON (PD11)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);

    // Parpadea rojo (PC3) 20Hz durante 2 segundos (80 toggles * 25ms)
    for (int i = 0; i < 80; ++i) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
      osDelay(25);
    }

    // Rojo OFF
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    // Amarillo OFF
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);

    // Esperar 0 segundos extra ya que el parpadeo dura 2s, y la tarea es periódica a 2s
  }
}

void UARTCommandTask(void *argument) {
  char rxChar;
  char buffer[32];
  uint8_t idx = 0;

  for (;;) {
    HAL_UART_Receive(&huart3, (uint8_t*)&rxChar, 1, HAL_MAX_DELAY);

    if (rxChar == '\n') {
      buffer[idx] = '\0';
      idx = 0;

      if (strcmp(buffer, "GREEN ON") == 0)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
      else if (strcmp(buffer, "GREEN OFF") == 0)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
      else if (strcmp(buffer, "BLUE ON") == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
      else if (strcmp(buffer, "BLUE OFF") == 0)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
      else if (strcmp(buffer, "RED ON") == 0)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
      else if (strcmp(buffer, "RED OFF") == 0)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      else if (strcmp(buffer, "YELLOW ON") == 0)
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
      else if (strcmp(buffer, "YELLOW OFF") == 0)
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
    } else {
      if (idx < sizeof(buffer) - 1)
        buffer[idx++] = rxChar;
    }
  }
}

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN RTOS_THREADS */
  osThreadNew(ToggleGreenTask, NULL, &toggleGreenTask_attributes);
  osThreadNew(ToggleRedTask, NULL, &toggleRedTask_attributes);
  osThreadNew(UARTCommandTask, NULL, &uartCommandTask_attributes);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE END Application */
