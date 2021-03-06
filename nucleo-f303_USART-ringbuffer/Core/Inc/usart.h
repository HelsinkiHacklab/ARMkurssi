/*
 * usart.h
 * USART RING BUFFER CLASS
 *  Created on: May 24, 2020
 *      Author: Kremmen
 */

#ifndef SRC_USART_H_
#define SRC_USART_H_

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "main.h"

namespace Usart {

#define BUFSIZE 256
#define NUMRXBUFS 8
#define RXBUFSIZE 40
enum serialMode {charMode, lineMode, blockMode};

// USARTin ohjausluokka (USART2:lle). Luokka toteuttaa SINGLETON-design patternin
// jossa luokalla voi olla korkeintaan 1 instanssi. Tunnusmerkkejä:
// - Luokan konstruktori on private, joten sitä ei voi kutsua luokan ulkopuolelta lainkaan.
// - Luokalla on staattinen metodi getInstance (t. vast.) joka tarvittaessa luo ja palauttaa
//   osoittimen luokan ainoaan instanssiin.
class usart {
private:
	usart();
public:
	static usart *getInstance();
	~usart();
	void begin( UART_HandleTypeDef *hUart, DMA_HandleTypeDef *hrxDma, DMA_HandleTypeDef *htxDma );
	uint8_t getChar();
	bool getLine( uint8_t *lineBuffer, uint16_t lineSize, uint32_t timeout, uint16_t *rxLen );
	bool getBlock( uint8_t *blockBuffer, uint16_t blockSize, uint32_t timeout, uint16_t *rxLen );
	void clear();
	HAL_StatusTypeDef putChar( uint8_t c );
	HAL_StatusTypeDef putLine( uint8_t *lineData );
	HAL_StatusTypeDef putBlock( uint8_t *blockData, uint16_t blockLength );
	uint16_t available();
	bool isEnabled() { return enabled; }
	void updatewrDMAIndex() { wrDMABufferIndex =  BUFSIZE - hrxDMA->Instance->CNDTR; };
private:
	static usart *instance;
	uint8_t rxResult;
	bool spoolDMABuffer();
	UART_HandleTypeDef *hUART;
	DMA_HandleTypeDef *hrxDMA;
	DMA_HandleTypeDef *htxDMA;
	TimerHandle_t hRxTimer;
	void enable();
	void disable();
	serialMode serMode;
	bool enabled;
	uint8_t dmaBuffer[BUFSIZE];	// USARTin datapuskuri(rengas) DMA-ohjain kirjoitaa tänne
	uint16_t rdDMABufferIndex;	// DMA-rengaspuskurin lukuosoitin (luokka lukee)
	uint16_t wrDMABufferIndex;	// DMA-rengaspuskurin kirjoitusosoitin (DMA-ohjain kirjoittaa)
	// serial buffer
	uint8_t *serBuffer;
	uint16_t serBufferSize;
	uint16_t rdSerBufferIndex;
	uint16_t wrSerBufferIndex;
};


} /* namespace Usart */

#endif /* SRC_USART_H_ */
