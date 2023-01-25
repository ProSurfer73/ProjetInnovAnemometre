/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* Commandes de l'afficheur 7segs---------------------------------------------*/
const uint8_t CMD_CLEAR = 0x76; // commande pour effacer l'afficheur 7seg
const uint8_t CMD_DECIMAL = 0x77; // commande pour afichher les décimales (les virgules)
const uint8_t CMD_DECIMAL_ZERO = 0x08; // commande pour afficher aucun  après la virgule
const uint8_t CMD_DECIMAL_ONE = 0x04; // commande pour afficher un chiffre après la virgule
const uint8_t CMD_DECIMAL_TWO = 0x02; // commande pour afficher deux chiffres après la virgule
const uint8_t CMD_DECIMAL_THREE = 0x00; // commande pour afficher trois chiffres après la virgule
const uint8_t CMD_BRIGHTNESS = 0x7A; // commande pour régler la luminosité de l'afficheur 7seg
const uint8_t CMD_MOVE_CURSOR = 0x79; // commande pour déplacer le cursor de l'afficheur 7seg
const uint8_t CMD_CURSOR1 = 0x00; // commande pour placer le cursor sur le digit 1 de l'afficheur 7seg
const uint8_t CMD_CURSOR2 = 0x01; // commande pour placer le cursor sur le digit 2 de l'afficheur 7seg
const uint8_t CMD_CURSOR3 = 0x02; // commande pour placer le cursor sur le digit 3 de l'afficheur 7seg
const uint8_t CMD_CURSOR4 = 0x03; // commande pour placer le cursor sur le digit 4 de l'afficheur 7seg
const uint8_t CMD_DIGIT1 = 0x7B; 
const uint8_t CMD_DIGIT2 = 0x7C;
const uint8_t CMD_DIGIT3 = 0x7D;
const uint8_t CMD_DIGIT4 = 0x7E;
const uint8_t CMD_BAUDRATE = 0x7F;
const uint8_t CMD_FACTORY_RESET = 0x81;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
void ClearDisplay(void);
void SetDigit(uint8_t iValue, int iDigitNb);
void SetDecimal(int iDecimalNb);
void SetDisplay(unsigned number);
void SetDisplayFloat(float decimalNumber);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	ClearDisplay();
	
	SetDisplayFloat(000.0);
	SetDecimal(1);
	//SetDecimal(0);
	int i=0;
	volatile uint16_t raw;
	volatile float voltage;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    raw = HAL_ADC_GetValue(&hadc1);
		voltage = ((float)raw * 3.3)/(float)4095;
		HAL_Delay(1);
		SetDisplayFloat(voltage);
		
//		SetDisplay(i);
//		i++;
//		if(i==9999){
//			i=0;
//		}
//		HAL_Delay(1000);
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
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
  /**Initializes the CPU, AHB and APB busses clocks 
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @brief  cette fonction permet de netoyer l'afficheur 7seg
  * @retval None
  */
void ClearDisplay(void)
{
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_CLEAR, 1, 100);
}

/**
  * @brief  cette fonction permet d'afficher un chiffre sur un digit
	* @param  iValue: valeur à afficher 
	* @param  iDigitNb: numéro du digit
  * @retval None
  */
void SetDigit(uint8_t iValue, int iDigitNb)
{
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_MOVE_CURSOR, 1, 100);
	switch(iDigitNb)
	{
		case 1 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_CURSOR1, 1, 100);
			break;
		case 2 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_CURSOR2, 1, 100);
			break;
		case 3 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_CURSOR3, 1, 100);
			break;
		case 4 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_CURSOR4, 1, 100);
			break;
	}
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&iValue, 1, 100);
}

/**
  * @brief  cette fonction permet d'afficher les virgule e fonction du nombre de décimales souhaitée
	* @param  iDecimalNb: nombre de décimales
  * @retval None
  */
void SetDecimal(int iDecimalNb)
{
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_DECIMAL, 1, 100);
	switch(iDecimalNb)
	{
		case 0 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_DECIMAL_ZERO, 1, 100);
			break;
		case 1 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_DECIMAL_ONE, 1, 100);
			break;
		case 2 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_DECIMAL_TWO, 1, 100);
			break;
		case 3 : HAL_SPI_Transmit(&hspi1, (uint8_t *)&CMD_DECIMAL_THREE, 1, 100);
			break;
	}
	

}

/**
  * @brief  cette fonction permet d'afficher un float (avec une décimale) sur l'afficheur
	* @param  fdecimalNumber: float valeur à afficher
  * @retval None
  */
void SetDisplayFloat(float fdecimalNumber)
{
		SetDisplay(fdecimalNumber*10);
}


/**
  * @brief  cette fonction permet d'afficher un entier sur l'afficheur
	* @param  number: valeur à afficher
  * @retval None
  */
void SetDisplay(unsigned number)
{
		unsigned i=4;
	
		while(i > 0)
		{
				SetDigit(number%10, i--);
				number /= 10;
		}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
