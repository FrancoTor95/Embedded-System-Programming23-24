/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPIO_BIT_0 			0 		// PC0
#define GPIO_BIT_1 			1 		// PC1
#define GPIO_BIT_2 			2 		// PC2
#define GPIO_EVENT_DETECTOR 3 		// PC3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t my_counter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void display_counter(volatile uint8_t *counter);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI3_IRQHandler() {
	NVIC_ClearPendingIRQ(EXTI3_IRQn);
	my_counter = (my_counter + 1) % (8);
	EXTI->PR |= (0x01 << GPIO_EVENT_DETECTOR);
}
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
  my_counter = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  RCC->AHB1ENR 		|=  (1 	<< RCC_AHB1ENR_GPIOCEN_Pos);
  RCC->APB2ENR 		|=  (0x01 << RCC_APB2ENR_SYSCFGEN_Pos);

  // PC BIT_0 Set Up
  GPIOC->MODER 		|=  (0x01 	<< 2*GPIO_BIT_0);
  GPIOC->OTYPER 	&= ~(0x03 	<< GPIO_BIT_0);
  GPIOC->OSPEEDR 	&= ~(0x03 	<< 2*GPIO_BIT_0);
  GPIOC->PUPDR 		&= ~(0x03 	<< 2*GPIO_BIT_0);

  // PC BIT_1 Set Up
  GPIOC->MODER 		|=  (0x01 	<< 2*GPIO_BIT_1);
  GPIOC->OTYPER 	&= ~(0x03 	<< GPIO_BIT_1);
  GPIOC->OSPEEDR 	&= ~(0x03 	<< 2*GPIO_BIT_1);
  GPIOC->PUPDR 		&= ~(0x03 	<< 2*GPIO_BIT_1);

  // PC BIT_2 Set Up
  GPIOC->MODER 		|=  (0x01 	<< 2*GPIO_BIT_2);
  GPIOC->OTYPER 	&= ~(0x03 	<< GPIO_BIT_2);
  GPIOC->OSPEEDR 	&= ~(0x03 	<< 2*GPIO_BIT_2);
  GPIOC->PUPDR 		&= ~(0x03 	<< 2*GPIO_BIT_2);


  // PC Counter Bit Event Detector
  GPIOC->MODER 		&= ~(0x03 	<< 2*GPIO_EVENT_DETECTOR);
  GPIOC->PUPDR 		|= ~(0x01 	<< 2*GPIO_EVENT_DETECTOR);

  /* SYStem ConFiGuration (Multiplexer) */
  SYSCFG->EXTICR[0] |= (SYSCFG_EXTICR1_EXTI3_PC);

  /* EXTernal Interrupt configuration */
  EXTI->IMR  |= (EXTI_IMR_IM3);
  EXTI->FTSR |= (0x01 << EXTI_FTSR_TR3_Pos);

  /* NVIC Configuration */
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_SetPriority(EXTI3_IRQn, 0);

  /* Initialization Pattern */
  GPIOC->ODR |= (0x01 << GPIO_BIT_0);
  HAL_Delay(1000);
  GPIOC->ODR |= (0x01 << GPIO_BIT_1);
  HAL_Delay(1000);
  GPIOC->ODR |= (0x01 << GPIO_BIT_2);
  HAL_Delay(1000);
  GPIOC->ODR &= ~(0x01 << GPIO_BIT_2);
  HAL_Delay(1000);
  GPIOC->ODR &= ~(0x01 << GPIO_BIT_1);
  HAL_Delay(1000);
  GPIOC->ODR &= ~(0x01 << GPIO_BIT_0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  // Example for polling mode
//	  if(!((GPIOC->IDR >> GPIO_IDR_ID3_Pos) & 0x01)) {
//		    my_counter = (my_counter + 1) % (8);
//			display_counter(&my_counter);
//			HAL_Delay(400);
//	  }
	  display_counter(&my_counter); // function at line 291
	  HAL_Delay(400);

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void display_counter(volatile uint8_t *counter) {

	GPIOC->ODR &= ~(0x07);
	GPIOC->ODR |= (*counter); // PC0 PC1 and PC2 are contiguous

	return;

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
