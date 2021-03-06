/*
 * spi.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: Kremmen
 *
 * SPI-virtualisointiluokan esittely
 *
 *
 */


#ifndef SRC_SPI_H_
#define SRC_SPI_H_

#include<stm32f3xx_hal.h>	// STM32 HAL-perusmäärittelyt
#include <vector>			// C++ Standard Template Libraryn (STL) tietorakenne "vektori"

namespace spidev {			// Tämä luokkahierarkia on suljettu nimiavaruuteen "spidev"

class SPIDevice;

const uint8_t SPIBUFSIZE = 16;

// SPI-väylän virtualisointiluokka
class spi {
public:
	spi(SPI_HandleTypeDef *hspi);
	virtual ~spi();
	SPIDevice *addDevice( SPIDevice *device );
	void transfer();
private:
	SPI_HandleTypeDef *hSPI;
	std::vector<SPIDevice *> Bus;
	uint8_t numBytes;
	uint8_t inBuffer[SPIBUFSIZE];
	uint8_t outBuffer[SPIBUFSIZE];
};

// SPI-väylään kytketyn laitteen virtualisointiluokka
class SPIDevice {
public:
	SPIDevice(GPIO_TypeDef *SS_port, uint16_t SS_pin);
	~SPIDevice() {};
	void set(uint8_t D) { devData = D; };
	uint8_t get() { return devData; };
	virtual void load(uint8_t **pData) {};
	virtual void dump(uint8_t **pData) {};
protected:
	GPIO_TypeDef * xferPort;
	uint16_t xferPin;
	uint8_t devData;
private:
	GPIO_TypeDef * ssPort;
	uint16_t ssPin;
};


// luokka joka virtualisoi 74hc165 -piirin
class SPI165: public SPIDevice {
public:
	SPI165(GPIO_TypeDef * SS_port, uint16_t SS_pin, GPIO_TypeDef * LOAD_port, uint16_t LOAD_pin);
	~SPI165() {};
	void load(uint8_t **pData);
	void dump(uint8_t **pData);

};

// luokka joka virtualisoi 74hc595 -piirin
class SPI595: public SPIDevice {
public:
	SPI595(GPIO_TypeDef * SS_port, uint16_t SS_pin, GPIO_TypeDef * DUMP_port, uint16_t DUMP_pin);
	~SPI595() {};
	void load(uint8_t **pData);
	void dump(uint8_t **pData);

};

} /* namespace spidev */

#endif /* SRC_SPI_H_ */
