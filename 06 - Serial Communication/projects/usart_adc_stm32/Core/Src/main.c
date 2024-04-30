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
#include <stdio.h>
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
#include <math.h>
#define ADC_FS 3.3f

volatile uint16_t voltage = 0;
volatile float voltage_f = 0;
void ADC_IRQHandler() {
	if(((ADC1->SR >> ADC_SR_EOC_Pos) & 0x01)) {
		voltage = (uint16_t)ADC1->DR;
		voltage_f = voltage*(ADC_FS/pow(2, 12));
		ADC1->CR2 |= (0x1 << ADC_CR2_SWSTART_Pos);
	}
}

volatile char buff_tx[20];
volatile uint8_t rx_len, tx_len, cmd_received;
uint8_t tx_len_max;
void USART2_IRQHandler() {
	NVIC_ClearPendingIRQ(USART2_IRQn);
	if(tx_len < tx_len_max && (USART2->SR >> USART_SR_TXE_Pos) & 0x01) {
		USART2->DR = buff_tx[++tx_len];
	}
	else if((USART2->SR >> USART_SR_TC_Pos) & 0x01) {
		tx_len = 0;
		USART2->CR1 &= ~(0x01 << USART_CR1_TXEIE_Pos); 		// Enable Interrupt on USART2 transmission
	}
}

void send_str_it(volatile char *buff, uint8_t len) {
	USART2->DR = buff[0];
	tx_len_max = len - 1;
	USART2->CR1 |= (0x01 << USART_CR1_TXEIE_Pos); 		// Enable Interrupt on USART2 transmission
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

  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_SetPriority(USART2_IRQn, 0);

  /* GPIO Configuration */
    // Provide clock to PORTA (PA_0 and PA_1 are associated to CH0 and CH1)
    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOAEN_Pos); 	// Clock to GPIOA

    // Set both GPIOs in Analog mode
    GPIOA->MODER |= (3 << GPIO_MODER_MODE0_Pos);		// Set PA0 in Analog mode

    /* ADC Configuration */
    // Provide clock to ADC1
    RCC->APB2ENR |= (1 << RCC_APB2ENR_ADC1EN_Pos); 	// Clock to ADC1


    ADC1->CR2 	|= (0x01 << ADC_CR2_ADON_Pos);			// (ADON) Power On ADC1
    ADC1->CR2 	&= ~(0x01 << ADC_CR2_CONT_Pos);			// (CONT) Single Conversion Mode
    ADC1->SQR3 	&= ~(0x1F << 0);						// Clear before Selection of Channel 0
    ADC1->SQR3 	|= (0x00 << 0);							// Selection of Channel 0
    ADC1->CR1 	&= ~(0x1 << ADC_CR1_SCAN_Pos); 			// (SCAN) Disable Scan Mode
    ADC1->CR1 	&= ~(0x1 << ADC_CR1_DISCEN_Pos); 		// (DISCEN) Disable Discontinuous Mode
    ADC1->CR1 	&= ~(0x07 << ADC_CR1_DISCNUM_Pos);		// (DISCNUM) Clear Number of Discontinuous Channels
    ADC1->CR2 	&= ~(0x1 << ADC_CR2_ALIGN_Pos); 		// (ALIGN) Align right
    ADC1->SMPR2 	&= ~(0x0F << ADC_SMPR2_SMP0_Pos);		// Set 3 Cycles per Samples
    ADC1->CR1 	&= ~(0x03 << ADC_CR1_RES_Pos);			// 12 Bit resolution
    ADC1->CR2 	|= (0x01 << ADC_CR2_EOCS_Pos); 			// (EOCS) Notify when each conversion of a sequence is complete
    ADC1 ->CR1 	|= (0x01 << ADC_CR1_EOCIE_Pos); 		// (EOCIE) Generate an interrupt every time EOC is set

    NVIC_EnableIRQ(ADC_IRQn); 		// Check file stm32f446xx.h for the name definition
    NVIC_SetPriority(ADC_IRQn, 0); 	// Priority
    ADC1->CR2 |= (0x1 << ADC_CR2_SWSTART_Pos);


  buff_tx[0] = 'L';
  buff_tx[1] = 'E';
  buff_tx[2] = 'T';
  buff_tx[3] = 'S';
  buff_tx[4] = ' ';
  buff_tx[5] = 'S';
  buff_tx[6] = 'T';
  buff_tx[7] = 'A';
  buff_tx[8] = 'R';
  buff_tx[9] = 'T';
  buff_tx[10] = '\r';
  buff_tx[11] = '\n';


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  send_str_it(buff_tx, 12);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	sprintf(buff_tx, "%1.3f\r\n", voltage_f);
	send_str_it(buff_tx, 7);
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

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
