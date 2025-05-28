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

/* Definitions for UARTCommandTask */

osThreadId_t UARTCommandTaskHandle;

const osThreadAttr_t UARTCommandTask_attributes = {

  .name = "UARTCommandTask",

  .stack_size = 128 * 4,

  .priority = (osPriority_t) osPriorityBelowNormal,

};

/* Definitions for ToggleRedTask */

osThreadId_t ToggleRedTaskHandle;

const osThreadAttr_t ToggleRedTask_attributes = {

  .name = "ToggleRedTask",

  .stack_size = 128 * 4,

  .priority = (osPriority_t) osPriorityNormal,

};

/* Definitions for ToggleGreenTask */

osThreadId_t ToggleGreenTaskHandle;

const osThreadAttr_t ToggleGreenTask_attributes = {

  .name = "ToggleGreenTask",

  .stack_size = 128 * 4,

  .priority = (osPriority_t) osPriorityNormal,

};



/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN FunctionPrototypes */

void ToggleGreenTask(void *argument);

void ToggleRedTask(void *argument);

void UARTCommandTask(void *argument);

/* USER CODE END FunctionPrototypes */



void StartTask03(void *argument);

void StartDefaultTask(void *argument);

void StartTask02(void *argument);



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

  /* creation of UARTCommandTask */

  UARTCommandTaskHandle = osThreadNew(StartTask03, NULL, &UARTCommandTask_attributes);



  /* creation of ToggleRedTask */

  ToggleRedTaskHandle = osThreadNew(StartDefaultTask, NULL, &ToggleRedTask_attributes);



  /* creation of ToggleGreenTask */

  ToggleGreenTaskHandle = osThreadNew(StartTask02, NULL, &ToggleGreenTask_attributes);



  /* USER CODE BEGIN RTOS_THREADS */

  osThreadNew(ToggleGreenTask, NULL, &ToggleGreenTask_attributes);

  osThreadNew(ToggleRedTask, NULL, &ToggleRedTask_attributes);

  osThreadNew(UARTCommandTask, NULL, &UARTCommandTask_attributes);

  /* USER CODE END RTOS_THREADS */



  /* USER CODE BEGIN RTOS_EVENTS */

  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */



}



/* USER CODE BEGIN Header_StartTask03 */

/**

  * @brief  Function implementing the UARTCommandTask thread.

  * @param  argument: Not used

  * @retval None

  */

/* USER CODE END Header_StartTask03 */

void StartTask03(void *argument)

{

  /* USER CODE BEGIN StartTask03 */

  /* Infinite loop */

  for(;;)

  {

    osDelay(1);

  }

  /* USER CODE END StartTask03 */

}



/* USER CODE BEGIN Header_StartDefaultTask */

/**

* @brief Function implementing the ToggleRedTask thread.

* @param argument: Not used

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



/* USER CODE BEGIN Header_StartTask02 */

/**

* @brief Function implementing the ToggleGreenTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartTask02 */

void StartTask02(void *argument)

{

  /* USER CODE BEGIN StartTask02 */

  /* Infinite loop */

  for(;;)

  {

    osDelay(1);

  }

  /* USER CODE END StartTask02 */

}



/* Private application code --------------------------------------------------*/

/* USER CODE BEGIN Application */



extern UART_HandleTypeDef huart3;  // Usando USART3



void ToggleGreenTask(void *argument) {

  for (;;) {

    // Azul ON (PC0)

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);



    // Parpadea verde (PA3) 20Hz durante 4 segundos (160 toggles * 25ms)

    for (int i = 0; i < 160; ++i) {

      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

      osDelay(25);

    }



    // Verde OFF

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

    // Azul OFF

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);



    // Esperar 2 segundos para completar ciclo de 6s

    osDelay(2000);

  }

}



void ToggleRedTask(void *argument) {

  for (;;) {

    // Amarillo ON (PF3)

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);



    // Parpadea rojo (PC3) 20Hz durante 2 segundos (80 toggles * 25ms)

    for (int i = 0; i < 80; ++i) {

      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);

      osDelay(25);

    }



    // Rojo OFF

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

    // Amarillo OFF

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);



    // Esperar 0 segundos extra ya que el parpadeo dura 2s, y la tarea es periódica a 2s

    osDelay(4000);

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

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

      else if (strcmp(buffer, "GREEN OFF") == 0)

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

      else if (strcmp(buffer, "BLUE ON") == 0)

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

      else if (strcmp(buffer, "BLUE OFF") == 0)

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

      else if (strcmp(buffer, "RED ON") == 0)

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

      else if (strcmp(buffer, "RED OFF") == 0)

        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

      else if (strcmp(buffer, "YELLOW ON") == 0)

        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);

      else if (strcmp(buffer, "YELLOW OFF") == 0)

        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);

    } else {

      if (idx < sizeof(buffer) - 1)

        buffer[idx++] = rxChar;

    }

  }

}









/* USER CODE END Application */

