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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SPIConfig(void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/
  RCC->APB2ENR |= (1 << 12);  // Enable SPI1 CLock

  SPI1->CR1 |= (1 << 0) | (1 << 1);   // CPOL=1, CPHA=1


  SPI1->CR1 |= (7 << 3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz

  SPI1->CR1 &= ~(1 << 7);  // LSBFIRST = 0, MSB first

  SPI1->CR1 |= (1 << 8) | (1 << 9);  // SSM=1, SSi=1 -> Software Slave Management

  SPI1->CR1 &= ~(1 << 10);  // RXONLY = 0, full-duplex

  SPI1->CR1 &= ~(1 << 11);  // DFF=0, 8 bit data

  SPI1->CR2 = 0;

  SPI1->CR1 |= (1 << 2);  // Master Mode
}

void SPI_Transmit(uint8_t *data, int size)
{

	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/

	int i=0;
	while (i<size)
	{
	   while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPI1->DR = data[i];  // load the data into the Data Register
	   i++;
	}


	/*During discontinuous communications, there is a 2 APB clock period delay between the
	write operation to the SPI_DR register and BSY bit setting. As a consequence it is
	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
	data.
	*/
	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPI1->DR;
	temp = SPI1->SR;

}

void SPI_Receive(uint8_t *data, int size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/
	GPIOB->ODR &= ~(1 << 6);
	while (size)
	{
		while (((SPI1->SR) & (1 << 7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;  // send dummy data
		while (!((SPI1->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	    *data++ = (SPI1->DR);
		size--;
	}
	GPIOB->ODR |= (1 << 6);
}

void GPIOConfig(void)
{
	RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA Clock
	RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB Clock

	GPIOA->MODER |= (2 << 10) | (2 << 12) | (2 << 14) | (1 << 18);  // Alternate functions for PA5, PA6, PA7

	GPIOA->OSPEEDR |= (3 << 10) | (3 << 12) | (3 << 14) | (3 << 18);  // HIGH Speed for PA5, PA6, PA7

	GPIOA->AFR[0] |= (5 << 20) | (5 << 24) | (5 << 28);   // AF5(SPI1) for PA5, PA6, PA7

	GPIOB->MODER 	|= (1 << 12);  	//Output for PB6
	GPIOB->OSPEEDR 	|= (3 << 12);  	// HIGH Speed for PB6
	GPIOB->ODR 		|= (1 << 6);
}

void SPI_Enable(void)
{
	SPI1->CR1 |= (1 << 6);   // SPE=1, Peripheral enabled
}

void SPI_Disable(void)
{
	SPI1->CR1 &= ~(1 << 6);   // SPE=0, Peripheral Disabled
}

#define RED 0b00000010
#define GRE 0b00001000
#define BLU 0b00100000
#define ORA 0b00001010
#define VIO 0b00100010
#define LBL 0b00101000
#define WHI 0b00101010
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  GPIOConfig();
  SPIConfig();
  SPI_Enable();
  uint8_t data = BLU;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	GPIOB->ODR &= ~(1 << 6);
	  SPI_Transmit(&data, 1);
	GPIOB->ODR |= (1 << 6);
	  HAL_Delay(1000);

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
