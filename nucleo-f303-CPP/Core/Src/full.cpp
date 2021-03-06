
#include "main.h"
#include <math.h>


UART_HandleTypeDef huart2;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

// Kahden muuttujan vaihtaminen ristiin; väärin, oikein ja Oikein

// Naiivi swap. Argumentit välitetään arvoina.
// Toimii väärin, mieti miksi
void naiveSwap (int16_t lahde, int16_t kohde) {
	int16_t temp;

	temp = kohde;
	kohde = lahde;
	lahde = temp;
}

// C-tyylinen swap. Argumentit välitetään osoittimina (pointer).
// Toimii oikein annetulle datatyypille ( tässä int16_t ).
void swap16( int16_t *lahde, int16_t *kohde) {
	int16_t temp;

	temp = *kohde;
	*kohde = *lahde;
	*lahde = temp;
}

// C++-tyylinen swap. Argumentit välitetään viitteinä (reference).
// Toimii oikein annetulle datatyypille ( tässä int16_t ).
void swap16R( int16_t& lahde, int16_t& kohde) {
	int16_t temp;

	temp = kohde;
	kohde = lahde;
	lahde = temp;
}

// C++-tyylinen swap-templaatti. Argumentit välitetään viitteinä (reference).
// Toimii oikein kaikille tyypeille joille on määritelty operaattori '='.
template <class T> void swap( T& lahde, T& kohde ) {
	T temp;
	temp = kohde;
	kohde = lahde;
	lahde = temp;
}

// C++ -tyylinen esimerkkistruktuuri
// C++ -kielessä jokainen struktuuri on samalla objektiluokka
// tarkoittaa että jäseninä voi olla sekä dataa että funktioita (metodeja)
// Struktuuri eroaa "aidosta" luokasta lähinnä vain näkyvyyssääntöjen osalta
// Kaikki struktuurin data ja funktiot ovat julkisia
struct threeSpaceQuaternion {
	float A;
	float x;
	float y;
	float z;
	float get2Norm() {
		return A * sqrt(x*x + y*y + z*z);
	};
	threeSpaceQuaternion operator+(const threeSpaceQuaternion& Q) {
		threeSpaceQuaternion sumQ;	// tilapäinen struktuuri pinossa...
		sumQ.A = this->A + Q.A;
		sumQ.x = this->x + Q.x;
		sumQ.y = this->y + Q.y;
		sumQ.z = this->z + Q.z;
		return sumQ;				// ...kopioidaan kohdemuuttujaan palatessa
	};
};


int main(void) {
	uint8_t A8, B8;
	int16_t A16, B16;
	float Af, Bf;
	threeSpaceQuaternion qA, qB, qC;
	float lengthA, lengthB, lengthC;

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	// Annetaan muuttujille alkuarvot
	A8 = 0x0f; B8 = 0xf0;
	A16 = 10000; B16 = 8000;
	Af = 100.5; Bf = 50.7;
	qA = {
		.A = 1.0,
		.x = 1.0,
		.y = 1.0,
		.z = 0.0
	};
	qB = {
		.A = 5.0,
		.x = -1.5,
		.y = 3.0,
		.z = -2
	};


	while (1) {
		naiveSwap( A16, B16 );
		swap16( &A16, &B16 );
		swap16R( A16, B16 );
		swap(A8, B8);
		swap(A16, B16);
		swap(Af, Bf);

		lengthA = qA.get2Norm();
		lengthB = qB.get2Norm();
		swap(qA, qB);
		lengthA = qA.get2Norm();
		lengthB = qB.get2Norm();
		qC = qA + qB;			// C++ kääntäjä osaa sijoituksen kopioimalla
								// me ollaan kerrottu miten yhteenlasku tehdään
		lengthC = qC.get2Norm();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
