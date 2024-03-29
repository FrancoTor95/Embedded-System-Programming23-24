/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* USER CODE BEGIN PFP */
void configure_gpio();
void configure_gpio_address();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 *
 * 	1 ----------> Enable GPIO PORT WITH RCC Register
 *  2 ----------> SETUP THE GPIO MODALITY (INPUT-OUTPUT) WITH MODER REGISTER
 *  3 ----------> IF OUTPUT SET UP THE OUTPUT TYPE (PUSH-PULL OPEN-DRAIN)
 *  5 ----------> IF OUTPUT SET UP THE OUTPUT SPEED WITH OSPEEDR
 *  4 ----------> SETUP THE PULL UP PULL DOWN RESISTORS
 *  5 ----------> OUTPUT VALUE WITH ODR AND READ VALUE WITH IDR
 *
 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
//  configure_gpio(); 			// uncomment for MACRO GPIO configuration
  configure_gpio_address(); 	// uncomment for address GPIO configuration

  // Output high
  GPIOB->ODR |= (0x1 << 10);

  /* Infinite loop */
  while (1){
	  if(!(GPIOA->IDR >> 10 & 0x1)){ 		// read GPIO A10 input value
		  GPIOB->ODR ^= (0x1 << 10); 		// toggle GPIO B10 value
		  HAL_Delay(400); 					// Simple deboucing
	  }
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void configure_gpio()
{
	/*Configure GPIOA10 as input and GPIOB10 as output */
	// Enable clock on GPIOA/GPIOB
	RCC->AHB1ENR = RCC->AHB1ENR | 0x3;

	/** GPIOA Configuration **/
	GPIOA->MODER  &= ~(0x03 << 20); // Clear GPIOA10
	GPIOA->MODER |= (0x00 << 20);   // Set Input
	// Pull-Up Mode su pin di input
	GPIOA->PUPDR |= (0x01 << 20);

	/** GPIOB Configuration **/
	GPIOB->MODER  &= ~(0x03 << 20); // Clear GPIOB10
	GPIOB->MODER |= (0x01 << 20);  // Set Output
	// Push-Pull output & Pull-up
	GPIOB->OTYPER &= ~(0x1 << 10);
	GPIOB->PUPDR &= ~(0x03 << 20); // Clear GPIOB10
	GPIOB->PUPDR |= (0x01 << 20);

}

void configure_gpio_address()
{
	/*Configure GPIOA10 as input and GPIOB10 as output */
	// Enable clock on GPIOA/GPIOB
	uint32_t rcc_base = (0x40000000UL + 0x00020000UL + 0x00003800);
	uint32_t *rcc_AHB1EN_p = (uint32_t *)(rcc_base + 0x00000030UL);
	(*rcc_AHB1EN_p) = (*rcc_AHB1EN_p) | 0x1;
	(*rcc_AHB1EN_p) = (*rcc_AHB1EN_p) | 0x2;

	/** GPIOA Configuration **/
	uint32_t gpioA_base = (0x40000000UL + 0x00020000UL + 0x00000000);
	uint32_t *gpioA_MODER_p = (uint32_t *)(gpioA_base + 0x00000000UL);
	(*gpioA_MODER_p) = (*gpioA_MODER_p) & ~(0x11 << 20);
	(*gpioA_MODER_p) = (*gpioA_MODER_p) | (0x00 << 20);
	uint32_t *gpioA_PUPDR_p = (uint32_t *)(gpioA_base +  0x0000000CUL);
	(*gpioA_PUPDR_p) = (*gpioA_PUPDR_p) | (0x01 << 20);

	/** GPIOB Configuration **/
	uint32_t gpioB_base = (0x40000000UL + 0x00020000UL + 0x00000400UL);
	uint32_t *gpioB_MODER_p = (uint32_t *)(gpioB_base + 0x00000000UL);
	(*gpioB_MODER_p) = (*gpioB_MODER_p) & ~(0x11 << 20);
	(*gpioB_MODER_p) = (*gpioB_MODER_p) | (0x01 << 20);
	uint32_t *gpioB_PUPDR_p = (uint32_t *)(gpioB_base +  0x0CUL);
	(*gpioB_PUPDR_p) = (*gpioB_PUPDR_p) | (0x01 << 20);
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

