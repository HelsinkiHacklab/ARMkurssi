/*
 * spi.cpp
 *
 *  Created on: Apr 24, 2020
 *      Author: Kremmen
 *
 * SPI-virtualisointiluokan määrittely
 *
 *
 */

#include "spi.h"

namespace spidev {

// SPI-väylän virtualisointiluokka "spi"
// Luokka käyttää HAL-kerrosta sulautetun oheislaitteen ohjaukseen

// konstruktori. Argumenttina HAL-tietorakenteen osoite
spi::spi(SPI_HandleTypeDef *hspi) {
	hSPI = hspi;
}

// Destruktori.
spi::~spi() {
	SPIDevice *pD;
	// spi-objekti ylläpitää vektoria Bus. STL-luokkakirjaston objektit osaavat hoitaa itsensä.
	// HUOM: jos vektorin elementit ovat osoittimia, niiden osoittamia objekteja ei tuhota.
	// Eli meidän pitää kutsua niiden destruktoria eksplisiittisesti:
	for (std::vector<SPIDevice *>::iterator it = Bus.begin() ; it != Bus.end(); ++it) {
		pD = *it;
		delete pD;
	}
}

// Rekisteröidään väylään kytketty piiri ohjaukseen
SPIDevice *spi::addDevice( SPIDevice *device ) {
	Bus.push_back(device);
	return device;
}

// Siirretään väylän ohjaamien piirien data
// Ennen SPI-siirtoa ladataan tulopiirien siirtorekisterit, sekä
// kopioidaan lähtöpiirieluokkien sisäinen data lähetyspuskuriin.
// SPI-siirron jälkeen kopioidaan tulopuskurin data tulopiiriluokkien sisäiseksi
// ja dumpataan siirretty lähtödata rekisterien pinneihin.
void spi::transfer() {
	SPIDevice *pD;
	uint8_t *bufPtr;
	// Iteroidaan piirilistra läpi ja ladataan data siirota varten
	bufPtr = outBuffer;
	numBytes = 0;
	for (std::vector<SPIDevice *>::iterator it = Bus.begin() ; it != Bus.end(); ++it) {
		pD = *it;
		pD->load(&bufPtr);
		numBytes++;
	}
	//SPI-siirto
	HAL_SPI_TransmitReceive(hSPI, outBuffer, inBuffer, numBytes/2, 0xffffffff);
	// Iteroidaan piirilista läpi ja dumpataan siirretty data
	bufPtr = inBuffer;
	for (std::vector<SPIDevice *>::iterator it = Bus.begin() ; it != Bus.end(); ++it) {
		pD = *it;
		pD->dump(&bufPtr);
	}
}

// SPI-piirien kantaluokka
SPIDevice::SPIDevice(GPIO_TypeDef *SS_port, uint16_t SS_pin) {
	if ( SS_port != 0 ) {
		ssPort = SS_port;
		ssPin = SS_pin;
		HAL_GPIO_WritePin(ssPort, ssPin, GPIO_PIN_SET);
	}
}

// 74HC165 -virtualisointiluokka
SPI165::SPI165(GPIO_TypeDef * SS_port, uint16_t SS_pin, GPIO_TypeDef * Xport, uint16_t Xpin): SPIDevice(SS_port, SS_pin) {
	xferPort = Xport;
	xferPin = Xpin;
}


void SPI165::load(uint8_t **pData) {
	HAL_GPIO_WritePin(xferPort, xferPin, GPIO_PIN_RESET);	// rinnakkaislataus
	HAL_Delay(1);
	HAL_GPIO_WritePin(xferPort, xferPin, GPIO_PIN_SET);		// siirtomoodi
}

void SPI165::dump(uint8_t **pData) {
	devData = **pData;
	*pData++;
}

SPI595::SPI595(GPIO_TypeDef * SS_port, uint16_t SS_pin, GPIO_TypeDef * Xport, uint16_t Xpin): SPIDevice(SS_port, SS_pin) {
	xferPort = Xport;
	xferPin = Xpin;
}

// 74HC595 -virtualisointiluokka
void SPI595::load(uint8_t **pData) {
	**pData = devData;
	*pData++;
}

void SPI595::dump(uint8_t **pData) {
	HAL_GPIO_WritePin(xferPort, xferPin, GPIO_PIN_SET);		// lähtö disable, lähtörekisterin lataus (RCLK nouseva reuna)
	HAL_Delay(1);
	HAL_GPIO_WritePin(xferPort, xferPin, GPIO_PIN_RESET);	// Lähtö enable

}


} /* namespace spidev */
