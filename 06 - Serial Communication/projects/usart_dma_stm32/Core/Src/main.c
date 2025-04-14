/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char usart_buffer[6] = {0, 0, 0, 0, '\n', '\r'};
uint8_t data_available = 0;

void send_str(char *buff, uint8_t len) {
	for(int i = 0; i < len; i++) {
		while(!((USART2->SR >> USART_SR_TXE_Pos) & 0x01));
		USART2->DR = buff[i];
	}
}

void DMA1_Stream5_IRQHandler() {
	if(DMA1->HISR >> DMA_HISR_TCIF5_Pos & 0b1) {
		NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
		data_available = 1;
		DMA1->HIFCR |= (0b1 << DMA_HIFCR_CTCIF5_Pos); 		// Clear status bit writing 1
	}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* GPIO Configuration (PA2 TX and PA3 RX) */
  RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos);

  // PA2 USART2 TX
  GPIOA->MODER 	|= (0x02 << 4); 		// Alternate function
  GPIOA->AFR[0] |= (0x07 << 8);			// Alternate function n7 (USART2 TX)
  GPIOA->OTYPER &= ~(0x03 << 4); 		// Push-Pull (Best suited for single direction line)
  GPIOA->PUPDR  |= (0x01 << 4); 		// USAR should have external Pull-up for higher data rates

  // PA3 USART2 RX
  GPIOA->MODER 	|= (0x02 << 6); 		// Alternate function
  GPIOA->AFR[0] |= (0x07 << 12);		// Alternate function n7 (USART2 TX)
  GPIOA->PUPDR  |= (0x01 << 6); 		// USAR should have external Pull-up for higher data rates


  /* USART Configuration */
  RCC->APB1ENR |= (0x01 << RCC_APB1ENR_USART2EN_Pos); 	// Provide clock

  USART2->CR1 |= (0x01 << USART_CR1_UE_Pos); 			// Enable USART (UE)
  USART2->CR1 |= (0x01 << USART_CR1_M_Pos); 			// Define word length (M)
  USART2->CR2 |= (0x02 << USART_CR2_STOP_Pos); 			// Define number of stop bits (STOP)
  USART2->CR1 |= (0x1 << USART_CR1_PCE_Pos); 			// Enable parity check (PCE)
  USART2->CR1 &= ~(0x01 << USART_CR1_PS_Pos);			// Even parity (PS)

  //273.4372
  USART2->BRR |= (0x111 << USART_BRR_DIV_Mantissa_Pos); // Define Mantissa
  USART2->BRR  |= (0x07 << 0); 							// Define Fractional part
  USART2->CR1 |= (0x01 << USART_CR1_TE_Pos); 			// Send one idle frame at the beginning (TE)
  USART2->CR1 |= (0x01 << USART_CR1_RE_Pos);			// Enable Reception (RE)

  USART2->CR3 |= (0x01 << USART_CR3_DMAR_Pos); 			// enable DMA on RXNE activation

  /* DMA Setup */
  RCC->AHB1ENR |= (0x01 << RCC_AHB1ENR_DMA1EN_Pos);

  DMA1_Stream5->CR |= (0b100 << DMA_SxCR_CHSEL_Pos);	// Stream 5 Channel 4 UART2 connection (101)

  DMA1_Stream5->PAR = (uint32_t)&USART2->DR;			// Peripheral address
  DMA1_Stream5->M0AR = (uint32_t)usart_buffer;			// Memory address
  DMA1_Stream5->CR &= ~(0b11 << DMA_SxCR_DIR_Pos); 		// Direction Peripheral to memory (00)

  DMA1_Stream5->CR &= ~(0b11 << DMA_SxCR_MSIZE_Pos); 	// 8 bit of memory for DMA transaction (00)
  DMA1_Stream5->CR &= ~(0b11 << DMA_SxCR_PSIZE_Pos); 	// 8 bit of peripheral for DMA transaction (00)

  DMA1_Stream5->CR |= (0b01 << DMA_SxCR_MINC_Pos);		// Increment memory address to fill buffer (1)
  DMA1_Stream5->CR &= ~(0b01 << DMA_SxCR_PINC_Pos);		// Do not increment peripheral address (0)

  DMA1_Stream5->CR |= (0b01 << DMA_SxCR_CIRC_Pos); 		// Enable circular mode (1)
  DMA1_Stream5->CR |= (0b01 << DMA_SxCR_TCIE_Pos); 		// Enable interrupt on transfer complete (1)

  DMA1_Stream5->FCR &= ~(0b1 << DMA_SxFCR_DMDIS_Pos); 	// Enable direct mode, no FIFO (0)

  DMA1_Stream5->NDTR = 4; 								// Number of transaction for data transmission
  DMA1_Stream5->CR |= (0b1 << DMA_SxCR_EN_Pos); 		// Enable DMA stream

  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  NVIC_SetPriority(DMA1_Stream5_IRQn, 0);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(data_available) {
		send_str(usart_buffer, 6);
		data_available = 0;
	}

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
