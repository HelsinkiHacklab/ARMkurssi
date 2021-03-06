/*
 * eeprom.cpp
 *
 *  Created on: 20.4.2020
 *      Author: martti
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "eeprom.h"

const uint8_t BLOCKSIZE=32;	// kerralla kirjoitettavan muistiblokin maksimikoko

eeprom::eeprom(uint8_t i2cAddress, GPIO_TypeDef* wePort, uint16_t wePin) {
	i2caddr = i2cAddress;
	weport = wePort;
	wepin = wePin;
	HAL_GPIO_WritePin(weport, wepin, GPIO_PIN_SET); //kirjoitussuojaus päälle oletuksena
}

eeprom::~eeprom() {
	// TODO Auto-generated destructor stub
}

// lukee eeprommista annettuun puskuriin halutun tavumäärän
// Palauttaa TRUE jos onnistu, muuten false.
// Mahdollinen virhe saadaan kutsumalla eeprom::getError()
bool eeprom::read(uint16_t memAddr, uint8_t *bufAddr, uint16_t bufSize) {
	uint16_t memaddr;
	// muisti on Big Endian joten vaihdetaan osoitteen tavut ristiin
	memaddr = __builtin_bswap16(memAddr);

	// Lähetetään osoite muistille jotta sen sisäinen osoiterekisteri asettuu halutuksi
	// Rekisteri on autoinkrementoiva, joten se kasvaa sitä mukaa kuin tavuja luetaan
	HAL_I2C_Master_Transmit(&hi2c1, i2caddr, (uint8_t *)&memaddr, sizeof(memaddr), 0xffffffff );

	Error = HAL_I2C_Master_Receive(&hi2c1, i2caddr, bufAddr, bufSize, 0xffffffff );
	if (Error != HAL_OK) return false;
	else return true;

}

bool eeprom::write(uint16_t memAddr, uint8_t *bufAddr, uint16_t bufSize) {
	uint16_t addr, memaddr;
	uint8_t wrBuffer[BLOCKSIZE+2];
	uint16_t wrBlockCount, numWrBlocks;
	uint8_t firstBlockSize, lastBlockSize;

	addr = memAddr;
	HAL_GPIO_WritePin(weport, wepin, GPIO_PIN_RESET); //kirjoitussuojaus pois

	// kirjoitetaan muisti, MUTTA: EEPROM:in sisäinen puskuri on max 32 tavua ja vielä niin, että
	// yksi kirjoituskerta ei voi ylittää 32-tavuisen lohkon rajaa muistin alusta asti laskettuna.
	// Eli koko muisti on jaettu peräkkäisiin 32 tavun lohkoihin ja kertakirjoitus voi olla max 1 lohko
	// eikä rajaa saa ylittää
	firstBlockSize = BLOCKSIZE - (memAddr % BLOCKSIZE);	// lasketaan mahdollisen ekan vajaan puskurin koko
	if (firstBlockSize >= bufSize) {
		lastBlockSize = 0;
		numWrBlocks = 0;
	}
	else {
		lastBlockSize = (memAddr+bufSize) % BLOCKSIZE;		// ja mahdollisen vikan vajaan puskurin koko
		if ( firstBlockSize + lastBlockSize == bufSize ) numWrBlocks = 0;
		else numWrBlocks = (bufSize-firstBlockSize-lastBlockSize) / BLOCKSIZE;
	}
	// kirjoitetaan ensimmäinen (vajaa) blokki
	if ( firstBlockSize ) {
		// muisti on Big Endian joten vaihdetaan osoitteen tavut ristiin
		memaddr = __builtin_bswap16(addr);
		memcpy(wrBuffer, &memaddr, sizeof(memaddr));
		memcpy(&wrBuffer[2],bufAddr,firstBlockSize);
		Error = HAL_I2C_Master_Transmit(&hi2c1, i2caddr, wrBuffer, sizeof(memaddr)+firstBlockSize, 0xffffffff );
		if ( Error ) return false;
		HAL_Delay(50);
		addr += firstBlockSize;
		bufAddr += firstBlockSize;
	}
	// kirjoitetaan täydet blokit
	for ( wrBlockCount = 0; wrBlockCount < numWrBlocks; wrBlockCount++ ) {
		memaddr =  __builtin_bswap16(addr);
		memcpy(wrBuffer, &memaddr, sizeof(memaddr));
		memcpy(&wrBuffer[2],bufAddr,firstBlockSize);
		Error = HAL_I2C_Master_Transmit(&hi2c1, i2caddr, wrBuffer, sizeof(memaddr)+BLOCKSIZE, 0xffffffff );
		if ( Error ) return false;
		HAL_Delay(50);
		addr += BLOCKSIZE;
		bufAddr += BLOCKSIZE;
	}
	// kirjoitetaan viimeinen (vajaa) blokki
	if ( lastBlockSize ) {
		memaddr =  __builtin_bswap16(addr);
		memcpy(wrBuffer, &memaddr, sizeof(memaddr));
		memcpy(&wrBuffer[2],bufAddr,lastBlockSize);
		Error = HAL_I2C_Master_Transmit(&hi2c1, i2caddr, wrBuffer, sizeof(memaddr)+lastBlockSize, 0xffffffff );
		if ( Error ) return false;
		HAL_Delay(50);
	}
	HAL_GPIO_WritePin(weport, wepin, GPIO_PIN_SET); //kirjoitussuojaus päälle
	return true;
}

#define ROW_LENGTH 16
#define PRINT_ROWS 16
void eeprom::print( uint16_t memAddr ) {
	uint8_t rdBuffer[16];
	memAddr /= ROW_LENGTH;
	memAddr *= ROW_LENGTH;
	for( uint8_t cnt = 0; cnt < PRINT_ROWS; cnt++ ) {
		read(memAddr, rdBuffer, ROW_LENGTH);
		printf("%04x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x  ", memAddr, rdBuffer[0], \
			rdBuffer[1], rdBuffer[2], rdBuffer[3], rdBuffer[4], rdBuffer[5], rdBuffer[6], rdBuffer[7], rdBuffer[8], \
			rdBuffer[9], rdBuffer[10], rdBuffer[11], rdBuffer[12], rdBuffer[13], rdBuffer[14], rdBuffer[15]);
		for ( uint8_t cnt2 = 0; cnt2 < ROW_LENGTH; cnt2++ ) {
			if ( rdBuffer[cnt2] >= 0x20 && rdBuffer[cnt2] < 0x80 ) printf("%c", rdBuffer[cnt2]);
			else printf(".");
		}
		printf("\r\n");
		memAddr += ROW_LENGTH;
	}
}


