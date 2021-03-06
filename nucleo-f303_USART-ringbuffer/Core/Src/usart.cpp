/*
 * usart.cpp
 * USART RING BUFFER CLASS
 *  Created on: May 24, 2020
 *      Author: Kremmen
 */

#include "usart.h"
#include "string.h"

namespace Usart {

#define DEFAULT_RX_TIMEOUT 1000

#define STATBUFSIZE 8
uint8_t staticBuffer[STATBUFSIZE];
uint8_t statIndex = 0;

__weak void usartTimerCallback( TimerHandle_t xTimer ) {
	xTimerStop( xTimer, 0 );
}

usart *usart::instance = 0;


usart* usart::getInstance() {
    if (!instance)
    instance = new usart;
    return instance;

}

usart::usart() {
	serMode = lineMode;
	rdDMABufferIndex = 0;
	wrDMABufferIndex = 0;
	rdSerBufferIndex = 0;
	wrSerBufferIndex = 0;
	serBuffer = NULL;
	serBufferSize = 0;
	rxResult = 0;
}

usart::~usart() {
	// TODO Auto-generated destructor stub
}

// Alustusmetodi joka valmistelee usart-objektin käyttöä varten
void usart::begin( UART_HandleTypeDef *hUart, DMA_HandleTypeDef *hrxDma, DMA_HandleTypeDef *htxDma ) {
	hUART = hUart;
	hrxDMA = hrxDma;
	htxDMA = htxDma;
	// vastaanottometodeilla on timeout joka on toteutettu FreeRTOS:in timer-palvelulla
	// tässä luodaan metodien käyttämä timeri
	hRxTimer = xTimerCreate( "RxT", pdMS_TO_TICKS( DEFAULT_RX_TIMEOUT ), pdFALSE, NULL, usartTimerCallback );
	if ( hRxTimer ) {
		// enabloidaan USARTin vastaanotto ja DMA-siirto
		SET_BIT(hUART->Instance->CR3, USART_CR3_DMAR);
		HAL_DMA_Start( hrxDMA, (uint32_t)(&(hUART->Instance->RDR)), (uint32_t)&dmaBuffer, BUFSIZE );
		enabled = true;
	}
	else Error_Handler();
}

void usart::enable() {
    SET_BIT(hUART->Instance->CR3, USART_CR3_DMAR);
	HAL_DMA_Start( hrxDMA, (uint32_t)(hUART->Instance->RDR), (uint32_t)&dmaBuffer, BUFSIZE );
	enabled = true;
}

void usart::disable() {
    CLEAR_BIT(hUART->Instance->CR3, USART_CR3_DMAR);
	HAL_DMA_Abort( hrxDMA );
	enabled = false;
}

void usart::clear() {
	rdDMABufferIndex = BUFSIZE - hrxDMA->Instance->CNDTR;
}

// USARTin lähetysmetodit (ei käytä rengaspuskurointia, paitsi putChar ihan piruuttaan)
HAL_StatusTypeDef usart::putChar( uint8_t c ) {
	uint8_t i;
	i = statIndex;
	staticBuffer[statIndex++] = c;
	if ( statIndex >= STATBUFSIZE ) statIndex = 0;
	return putBlock( &staticBuffer[i], 1);
}

HAL_StatusTypeDef usart::putLine(uint8_t *txData) {
		return putBlock( txData, strlen( (char *)txData ) );
}

// Tällä metodilla oikeasti lähtee kaikki tavara ulos
HAL_StatusTypeDef usart::putBlock( uint8_t *blockData, uint16_t blockLength ) {
	if ( htxDMA->Instance->CNDTR == 0 ) {
		hUART->gState = HAL_UART_STATE_READY;
		return HAL_UART_Transmit_DMA( hUART, blockData, blockLength );
	}
	return HAL_ERROR;
}

// palauttaa luettavissa olevien merkkien määrän.
uint16_t usart::available() {
	if ( wrDMABufferIndex >= rdDMABufferIndex ) return wrDMABufferIndex - rdDMABufferIndex;
	else return ( BUFSIZE - ( rdDMABufferIndex - wrDMABufferIndex ) );
}

// Metodi palauttaa seuraavan vastaanotetun merkin ilman timeoutia (eli odottaa vaikka ikuisesti)
uint8_t usart::getChar() {
	uint8_t ch;
	while ( wrDMABufferIndex == rdDMABufferIndex );
	ch = dmaBuffer[rdDMABufferIndex++];
	if ( rdDMABufferIndex >= BUFSIZE ) rdDMABufferIndex = 0;
	return ch;
}

// Metodi lukee USARTin rengaspuskurista tekstirivin joka päättyy joko '\r' tai '\n' -merkkiin.
// Jos rivi on pidempi kuin varattu puskuri, teksti katkaistaan väkisin ja terminoidaan '\0'
// Käyttäjän täytyy selvittää että rivi on vielä vajaa.
// Argumenttiin *rxLen palautetaan luetun rivin todellinen pituus (jos timeout ennen kuin rivin lopetus)
bool usart::getLine(uint8_t *lineBuffer, uint16_t lineSize, uint32_t timeout, uint16_t *rxLen ) {
	bool st = true;
	serBuffer = lineBuffer;
	serBufferSize = lineSize;
	wrSerBufferIndex = 0;
	serMode = lineMode;		// halutaan lukea riveittäin. (rivi tai sen osa päättyy '\0')
	if ( xTimerChangePeriod( hRxTimer, pdMS_TO_TICKS( timeout ), 1 ) == pdFAIL ) Error_Handler();
	if ( xTimerStart( hRxTimer, 100 ) == pdFAIL ) Error_Handler();
	while ( !spoolDMABuffer() ) {
		if ( !xTimerIsTimerActive( hRxTimer ) ) {
			st = false;
			serBuffer[wrSerBufferIndex] = '\0';
			break;
		}
		taskYIELD();
	}
	*rxLen = wrSerBufferIndex;
	return st;
}

// Metodi lukee USARTin rengaspuskurista määrämittaisen lohkon (esim protokollakehyksen)
// Lohkoa ei terminoida millään vaan se palautetaan juuri kuten luettiin linjalta.
// Argumenttiin *rxLen palautetaan luetun lohkon todellinen pituus (jos timeout ennen kuin lohko täysi)
bool usart::getBlock(uint8_t *blockBuffer, uint16_t blockSize, uint32_t timeout, uint16_t *rxLen ) {
	bool st = true;
	serBuffer = blockBuffer;
	serBufferSize = blockSize;
	wrSerBufferIndex = 0;
	serMode = blockMode;
	xTimerChangePeriod( hRxTimer, timeout / portTICK_PERIOD_MS, 100);
	xTimerStart( hRxTimer, 100 );
	while ( !spoolDMABuffer() ) {
		if ( !xTimerIsTimerActive( hRxTimer ) ) {
			st = false;
			break;
		}
		taskYIELD();
	}
	*rxLen = wrSerBufferIndex;
	return st;
}

// Spooleri siirtää merkkejä DMA-vastaanottopuskurista käyttäjän lukumetodin osoittamaan puskuriin
bool usart::spoolDMABuffer() {
/*
	while ( rdDMABufferIndex != wrDMABufferIndex ) {
		if ( serMode == lineMode ) {
			if ( dmaBuffer[rdDMABufferIndex] == '\r' || dmaBuffer[rdDMABufferIndex] == '\n') {  // rivin loppumerkki?
				rdDMABufferIndex++;
				if ( rdDMABufferIndex >= BUFSIZE ) rdDMABufferIndex = 0;
			}
			else break;
		}
	}
*/
	while ( rdDMABufferIndex != wrDMABufferIndex ) {
		if ( serMode == lineMode ) {
			if ( dmaBuffer[rdDMABufferIndex] == '\r' || dmaBuffer[rdDMABufferIndex] == '\n') {  // rivin loppumerkki?
				serBuffer[wrSerBufferIndex++] = '\0';
				rdDMABufferIndex++;
				return true;
			}
		}
		serBuffer[wrSerBufferIndex++] = dmaBuffer[rdDMABufferIndex++];
		if ( rdDMABufferIndex >= BUFSIZE ) rdDMABufferIndex = 0;
		if ( serMode == lineMode ) {
			if ( wrSerBufferIndex >= serBufferSize-1 ) {
				serBuffer[wrSerBufferIndex] = '\0';
				return true;
			}
		}
		else {
			if ( wrSerBufferIndex >= serBufferSize ) {
				return true;
			}
		}
	}
	return false;
}


} /* namespace Usart */


