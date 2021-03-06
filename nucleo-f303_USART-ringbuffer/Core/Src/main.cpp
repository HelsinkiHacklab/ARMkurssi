/* USART ringbuffer demo */

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "usart.h"
#include "string.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);

// FreeRTOS:in muistiallokaattorit, jos satuttaisi tarviimaan
void * operator new( size_t size )
{
    return pvPortMalloc( size );
}

void * operator new[]( size_t size )
{
    return pvPortMalloc(size);
}

void operator delete( void * ptr )
{
    vPortFree ( ptr );
}

void operator delete[]( void * ptr )
{
    vPortFree ( ptr );
}

Usart::usart *U2 = U2->getInstance();	// haetaan luokan ainoan instanssin osoite. osoitin SINGLETON USART2-objektiin

// FreeRTOS:in kellokeskeytykseen koukutettu apuproseduuri jota kutsutaan
// jokaisella kellokeskeytyksellä (millisekunnin välein)
// Proseduuri päivittää usart-luokan rengaspuskurin kirjoitusindeksin
// DMA-ohjaimen rautarekisteristä luokan jäsenmuuttujaan.
void vApplicationTickHook( void ) {
	U2->updatewrDMAIndex();
}

// U2 = U2->getInstance();	// haetaan luokan ainoan instanssin osoite

// serial line task
#define SER_STACKSIZE	256
static StackType_t serStack[SER_STACKSIZE];
static StaticTask_t serTCB;
static TaskHandle_t hSER;

// Yksinkertainen taski joka 'keskustelee' käyttäjän kanssa demonstroiden
// usart-luokan ja rengaspuskurin käyttöä.
void serialTask( void *pvParameters ) {
uint8_t greeting[] = "Hello, World! serialTask here!\r\n";
uint8_t select[] = "Hit '1' for line input (max 80 chars) or '2' for block input (max 5 chars)\r\n";
uint8_t prompt1[] = "Waiting for text line!\r\n";
uint8_t prompt2[] = "Waiting for text block!\r\n";
uint8_t unknown[] = "That did not compute :( \r\n";
uint8_t ch;
uint8_t inputLine[80];
uint16_t len;


	U2->begin( &huart2, &hdma_usart2_rx, &hdma_usart2_tx);
	U2->putLine(greeting);
	vTaskDelay(100);
	for ( ;; ) {
		U2->putLine(select);
		U2->clear();
		ch = U2->getChar();
		if ( ch == '1' ) {
			U2->putLine(prompt1);
			len = 0;
			U2->getLine( inputLine, 80, 20000, &len);
			if ( len ) {
				U2->putLine( inputLine );
				vTaskDelay(100);
				U2->putLine((uint8_t *)"\r\n");
			}
			else U2->putLine( (uint8_t *)"No text within timeout!\r\n");
		}
		else if ( ch == '2' ){
			U2->putLine(prompt2);
			len = 0;
			U2->getBlock( inputLine, 5, 20000, &len);
			if ( len ) {
				inputLine[len] = '\0';
				U2->putLine( inputLine );
				vTaskDelay(100);
				U2->putLine((uint8_t *)"\r\n");
			}
			else U2->putLine( (uint8_t *)"No text within timeout!\r\n");
		}
		else U2->putLine( unknown );
		vTaskDelay(500);
	}
}

int main(void) {

	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();

	// usart-luokka käyttää FreeRTOS:in Timer-palvelua. Palvelu on kernelin sisäinen taski joka kilpailee ajoajasta
	// käyttäjän taskien kanssa. SEN OLETUSPRIORITEETTI ON tskIDLE_PRIORITY+2. Pitää olla erittäin varovainen
	// jos käyttäjän taskeilla on korkeampi prioriteetti (isompi numeroarvo) koska Timer-palvelu ei ehkä saa koskaan ajovuoroa.
	// FreeRTOS:in konfiguraatiossa Timerin prioriteettia voi nostaa tarvittaessa.
	hSER = xTaskCreateStatic( serialTask, "SER", SER_STACKSIZE, NULL, tskIDLE_PRIORITY+1, serStack, &serTCB );

	vTaskStartScheduler();


	while (1){

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
	while ( 1 );
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
