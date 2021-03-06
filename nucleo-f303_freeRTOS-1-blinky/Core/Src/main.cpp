/*
 * Yksinkertainen ledinvätkytys FreeRTOSin alaisuudessa.
 * leditaski vilkuttaa lediä tasaiseen tahtiin
 * Käyttäjän napin (sininen) lukutaski pysäyttää leditaskin kun nappula on painettuna
 * ja vapauttaa sen ajoon kun nappula on ylhäällä.
 */

#include "main.h"

#include <FreeRTOS.h>
#include <task.h>

// vvv--- nämä kopioitu alkuperäisestä main.c tiedostosta
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
// ^^^--- tähän asti

//Taskien kahvat. Taskiin voidaan tarvittaessa viitata käyttäen sen kahvaa tunnisteena.
//
TaskHandle_t hLED, hUI;

// Nucleon ledin ohjaustaski
// ainoa tehtävä on vätkyttää lediä määrätahtiin joka määrätään päällä/pois -viiveillä
// HUOM! Viivettä EI generoida HALin HAL_Delay() funktiolla koska se on busy loop
// FreeRTOS-funktio vTaskDelay() luovuttaa ajovuoron aktiivisille taskeille, ja LEDTask on
// odottavien taskien listalla kunnes viiveajastin laukeaa ja se palaa ajokelpoisten taskien listaan
// Odottavat taskit eivät syö prosessorin kapasiteettia!
void LEDTask( void *pvParameters ) {
	for ( ;; ) {
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		vTaskDelay( 100/portTICK_PERIOD_MS );
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		vTaskDelay( 500/portTICK_PERIOD_MS );
	}
}

// Nucleon käyttäjän napin lukutaski
// Tunnistetaan napin tilamuutokset vertaamalla nykyistä ja edellistä tilaa
// Jos nappi meni alas niin pysäytetään leditaski. Kun nappi nousee ylös niin vapautetaan leditaski ajoon.
void buttonTask( void *pvParameters ) {
	GPIO_PinState btnStatus, prevBtnStatus;
	TaskHandle_t hOtherTask;

	hOtherTask = *(TaskHandle_t *)pvParameters;		// laitetaan leditaskin kahva talteen
	btnStatus = prevBtnStatus = GPIO_PIN_SET;
	for ( ;; ) {
		btnStatus = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
		if ( btnStatus == GPIO_PIN_RESET && prevBtnStatus ==  GPIO_PIN_SET ) {
			vTaskSuspend( hOtherTask ); // https://www.freertos.org/a00130.html
		}
		else if ( btnStatus == GPIO_PIN_SET && prevBtnStatus ==  GPIO_PIN_RESET ) {
			vTaskResume( hOtherTask );	// https://www.freertos.org/a00131.html
		}
		prevBtnStatus = btnStatus;
		vTaskDelay( 50/portTICK_PERIOD_MS );	// huilataan 50 ms ettei tehdä tästä busy looppia
		// https://www.freertos.org/a00127.html
	}
}

int main(void) {

	// vvv--- alkuperäiset alustukset kopioitu main.c tiedostosta
	HAL_Init();

	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	// ^^^---

	// FreeRTOS-taskien luonti. Annetaan kernelin varata tarvittavat muistilohkot kekomuistista (heap)
	// Näiden funktiokutsujen jälkeen FreeRTOSin kerneliin on perustettu taskeille niiden tarvitsemat tietorakenteet,
	// varattu taskien pinomuistit ja tehty kaikki muutkin valmistelut jotta suoritus voidaan siirtää kernelin alaisuuteen
	// xTaskCreate referenssi, kts: https://www.freertos.org/a00125.html
	xTaskCreate( LEDTask, "LED1", 256, NULL, tskIDLE_PRIORITY+1, &hLED );		// Luodaan ledin vätkytystaski
	xTaskCreate( buttonTask, "BTN", 256, &hLED, tskIDLE_PRIORITY+1, &hUI );		// Luodaan napin lukutaski ja annetaan sille parametrina leditaskin kahva

	// Käynnistetään FreeRTOS-kernelin skeduleri joka ohjaa taskien suoritusta
	// Tästä funktiosta ei koskaan palata joten on samantekevää mitä ohjelmassa lukee tämän funktion jälkeen
	// https://www.freertos.org/a00132.html
	vTaskStartScheduler();

	while (1)
	{
	}
}

// vvv--- tästä eteenpäin kopioitu aluperäisestä main.c tiedostosta

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


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
