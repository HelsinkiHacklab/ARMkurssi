/*
 * ADCTask.cpp
 *
 *  Created on: Apr 8, 2020
 *      Author: martti
 */

#include <freeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <portmacro.h>

#include "main.h"
#include "ADCTask.h"

static 	SemaphoreHandle_t adcSema;
BaseType_t hiPrioTaskWoken;
static uint16_t adBuffer[ADCBUFFERLEN+8];

// Muunnos valmis
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if ( HAL_OK != HAL_ADC_Stop_DMA( hadc ) ) {
		Error_Handler();
	}
	xSemaphoreGiveFromISR( adcSema, &hiPrioTaskWoken );
	if ( hiPrioTaskWoken ) taskYIELD();
}

// A/D-muunnostaski
void ADCTask( void *pvParameters ) {
	ADC_HandleTypeDef *hadc;
	DMA_HandleTypeDef *hdma_adc;
	QueueHandle_t qh;
	uint32_t avg;
	ADCTaskParams p;
	hadc = ((ADCTaskParams *)pvParameters)->phadc;
	hdma_adc = ((ADCTaskParams *)pvParameters)->phdma_adc;
	qh = ((ADCTaskParams *)pvParameters)->qh;

	// Luodaan synkronointisemafori
	adcSema = xSemaphoreCreateBinary();

	// Kalibroidaan A/D -muunnin
	if ( HAL_OK != HAL_ADCEx_Calibration_Start( hadc, ADC_SINGLE_ENDED ) ) {
		Error_Handler();
	}

	// Käynnistetään 1. muunnoskierros
	if ( HAL_OK != HAL_ADC_Start_DMA(hadc, (uint16_t *)adBuffer, ADCBUFFERLEN ) ) {
		Error_Handler();
	}

	while ( 1 ) {

		// odotetaan että muunnoskierros tulee valmiiksi
		xSemaphoreTake( adcSema, portMAX_DELAY );
		// lasketaan näytepuskurin keskiarvo...
		avg = 0;
		for ( uint8_t cnt = 0; cnt < ADCBUFFERLEN; cnt++ ) {
			avg += adBuffer[cnt];
		}
		avg /= ADCBUFFERLEN;

		// ... ja lähetetään se UI-taskille näytettäväksi
		xQueueSendToBack( qh, &avg, portMAX_DELAY );

		// odotellaan hetki ja käynnistetään seuraava kierros
		vTaskDelay(100);
		if ( HAL_OK != HAL_ADC_Start_DMA(hadc, (uint16_t *)adBuffer, ADCBUFFERLEN ) ) {
			Error_Handler();
		}
	}
}
