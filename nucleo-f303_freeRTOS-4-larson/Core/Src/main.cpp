 /*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * LARSON-SKANNERI HYÖDYNTÄEN FREERTOS-KÄYTTÖJÄRJESTELMÄÄ
  *
  *
  *
  ******************************************************************************
  */

#include "main.h"
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

#define NUMCHANNELS			8		// näin montaa lediä ohjataan
#define TASKSTACKDEPTH		128		// ohjaustaskille varattavan pinon koko (uint32_t kokoisina muuttujina!)
#define MAXQUEUEDMESSAGES	2		// korkeintaan näin monta viestiä mahtuu jonoon kerrallaan
#define MAXMODULATION		1000	// timerin counter periodia vastaava lukuarvo jolla saavutetaan 100% pwm-modulaatio

// tässä tyyppimääritellään ledien ohjaustaskien saamat alustusparametrit yhteen struktuuriin.
// Struktuurissa esitellään viestijonot, sekä ledin pulssinleveysmodulaatiota varten tarvittava
// ledikohtainen timer ja pwm-kanava
typedef struct {
	QueueHandle_t inQ;			// taskin oma jono josta luetaan komennot
	QueueHandle_t upQ;			// ylävirran puoleisen naapuritaskin jono
	QueueHandle_t dnQ;			// alavirran puoleisen naapuritaskin jono
	TIM_HandleTypeDef *phtim;	// ledin pwm-timer
	uint8_t pwmchannel;			// ledin pwm-kanava
} pwmTaskParams_t;

// Taskien ymmärtämät ja toisilleen välittämät viestit
// msg_dn: käynnistä oma pwm-sekvenssi ja välitä jatkokäsky alavirtaan
// msg_up: käynnistä oma pwm-sekvenssi ja välitä jatkokäsky ylävirtaan
enum msg { msg_dn, msg_up };

//======================================================
// Ledin ohjaustaski
// Taski lukee omasta viestijonostaan yksinkertaisen yhden tavun mittaisen käskyn ja
// käynnistää pwm-sekvenssin. Kun sekvenssi on saavuttanut maksimiarvonsa, viesti välitetään eteenpäin
// ja aletaan ajamaan pwm-modulaatiota alaspäin kohti nollaa.
// suunta vaihtuu kun taski havaitsee, että jatkosuunnassa olevan taskin kahva on NULL (eli siinä suunnassa ei enää ole ketään)
// silloin jatkokäsky lähetetään siihen suuntaan mistä se saatiinkin.
void pwmTask( void *pvParameters ) {
#define PWMSTEP		25	// paljonko stepataan pwm-pulssisuhdetta kerralla (maksimi on alustettu olemaan 1000)
#define LOOPDELAY	5	// askellusviive ms

	pwmTaskParams_t *par;

	bool waitForMessage;
	msg message;

	uint32_t pwm;
	volatile uint32_t *ccr;		// osoitin timerin (muistimapattuun) counter compare-rekisteriin

	// puretaan kutsuargumenttina saadut parametrit
	par = (pwmTaskParams_t *)pvParameters;
	switch ( par->pwmchannel ) {
		case 1: {
			ccr = &par->phtim->Instance->CCR1;
			break;
		}
		case 2: {
			ccr = &par->phtim->Instance->CCR2;
			break;
		}
		case 3: {
			ccr = &par->phtim->Instance->CCR3;
			break;
		}
		case 4: {
			ccr = &par->phtim->Instance->CCR4;
			break;
		}
	}

	// jos ollaan alimman ledin ohjaustaski niin ekalla kerralla aloitetaan
	// odottamatta saapuvaa viestiä (koska kukaan ei ole lähettämässä sitä)
	if ( par->dnQ == NULL ) {
		message = msg_up;
		waitForMessage = false;
	}
	else waitForMessage = true;

	for ( ;; ) {
		// viestin odottelu
		if (waitForMessage) xQueueReceive(par->inQ, &message, portMAX_DELAY);
		waitForMessage = true;

		// ryhdytään ohjaamaan lediä

		// startataan pwm ja ajetaan pulssisuhde 0->100%
		HAL_TIM_PWM_Start( par->phtim, (par->pwmchannel-1)<<2 );
		for (pwm = PWMSTEP; pwm < MAXMODULATION; pwm += PWMSTEP ) {
			*ccr = pwm;
			vTaskDelay( LOOPDELAY / portTICK_PERIOD_MS);
		}

		// kun ledi palaa täysillä, viestitetään käynnistyskäsky seuraavan ledin ohjaustaskille
		if ( message == msg_up ) {
			if ( par->upQ != NULL ) xQueueSend( par->upQ, &message, portMAX_DELAY );
			else {
				message = msg_dn;
				xQueueSend( par->dnQ, &message, portMAX_DELAY );
			}
		}
		else {
			if ( par->dnQ != NULL ) xQueueSend( par->dnQ, &message, portMAX_DELAY );
			else {
				message = msg_up;
				xQueueSend( par->upQ, &message, portMAX_DELAY );
			}
		}

		// ajetaan pulssisuhde 100->0% ja pysäytetään pwm
		for ( ; pwm > 0; pwm -= PWMSTEP ) {
			*ccr = pwm;
			vTaskDelay( LOOPDELAY / portTICK_PERIOD_MS);
		}
		HAL_TIM_PWM_Stop( par->phtim, (par->pwmchannel-1)<<2 );
	}
}
//======================================================

// Taskeille välitettävät parametristruktuurit
static pwmTaskParams_t params[NUMCHANNELS];

// Taskien kontekstit - staattisesti allokoituna
static StackType_t stack[NUMCHANNELS][TASKSTACKDEPTH];		// Taskien pinot
static StaticTask_t tcb[NUMCHANNELS];						// kernelin Task Control Blockit

int main(void) {
uint8_t channel;
uint8_t taskName[] = "PWMx";

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	memset(stack, 0xff, sizeof(stack));
	memset(tcb, 0xff, sizeof(tcb));

	// luodaan jokaiselle taskille oma viestijono
	for ( channel = 0; channel < NUMCHANNELS; channel++ ) {
		params[channel].inQ = xQueueCreate( MAXQUEUEDMESSAGES, sizeof(msg) );
	}
	// linkitetään viestijonot ja pwm-kanavat taskien alustusparametri-struktuureihin
	// Ensin timerin 2 pwm-ohjain
	for ( channel = 0; channel < NUMCHANNELS/2; channel++ ) {
		params[channel].upQ = params[channel+1].inQ;
		if ( channel == 0 ) params[channel].dnQ = NULL;
		else params[channel].dnQ = params[channel-1].inQ;
		params[channel].phtim = &htim2;
		params[channel].pwmchannel = channel +1;
	}
	// ja sitten timerin 3 pwm-ohjain
	for ( channel = NUMCHANNELS/2; channel < NUMCHANNELS; channel++ ) {
		params[channel].dnQ = params[channel-1].inQ;
		if ( channel == NUMCHANNELS -1 ) params[channel].upQ = NULL;
		else params[channel].upQ = params[channel+1].inQ;
		params[channel].phtim = &htim3;
		params[channel].pwmchannel = channel - NUMCHANNELS/2 + 1;
	}
	// Lopuksi luodaan taskit, kukin saa omat alustusparametrinsa ja staattisesti varatut resurssit
	// HUOMAA: kaikilla taskeilla on yhteinen koodi. Taskin identiteetin määrää sen Task Control Block, EI sen koodi.
	// Huomaa myös: Nyt luodaan 8 kpl taskeja ja voisi hyvin käydä, ettei heap riittäisikään tälle määrälle
	// Kun taskien TCB ja heap on staattisesti allokoitu tuolla ylempänä niin linkkausvaiheessa jo tiedetään onnistuuko
	for ( channel = 0; channel < NUMCHANNELS; channel++ ) {
		taskName[3] = channel + '1';
		xTaskCreateStatic( pwmTask, (const char *)taskName, TASKSTACKDEPTH, &params[channel], tskIDLE_PRIORITY+1, &stack[channel][0], &tcb[channel] );
	}

	// ja menoksi
	vTaskStartScheduler();

	while (1) {
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
