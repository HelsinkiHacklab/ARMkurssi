/*
 * eeprom.h
 *
 *  Created on: 20.4.2020
 *      Author: martti
 */

#ifndef SRC_EEPROM_H_
#define SRC_EEPROM_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f3xx_hal.h"

typedef uint8_t eepromError;
extern I2C_HandleTypeDef hi2c1;

class eeprom {
public:
	eeprom(uint8_t i2cAddress, GPIO_TypeDef* wePort, uint16_t wePin);
	virtual ~eeprom();
	HAL_StatusTypeDef getError();
	bool read(uint16_t memAddr, uint8_t *bufAddr, uint16_t bufSize);
	bool write(uint16_t memAddr, uint8_t *bufAddr, uint16_t bufSize);
	void print(uint16_t memAddr);
private:
	GPIO_TypeDef* weport;
	uint16_t wepin;
	HAL_StatusTypeDef Error;
	uint8_t i2caddr;
};

#endif /* SRC_EEPROM_H_ */
