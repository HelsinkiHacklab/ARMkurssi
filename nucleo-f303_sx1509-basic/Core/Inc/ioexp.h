/*
 * ioexp.h
 *
 *  Created on: May 13, 2020
 *      Author: martti
 */

#ifndef SRC_IOEXP_H_
#define SRC_IOEXP_H_

#include <stdint.h>
#include <vector>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

namespace sx1509 {


#include "ioexp-defs.h"

enum initMode_t { inin, inout, outin, outout };
enum ioexpStat_t { expOK=0, expERROR };

class ioexp {
public:
	ioexp(I2C_HandleTypeDef *hI2C, uint8_t i2cAddr, GPIO_TypeDef *reset_Port, uint16_t reset_Pin, GPIO_TypeDef *int_Port, uint16_t int_Pin, initMode_t mode);
	virtual ~ioexp();
	void configPushPullOutput(uint16_t portMask);
	void configOpenDrainOutput(uint16_t portMask);
	void configInput(uint16_t portMask);
	void configInputPullUp(uint16_t portMask);
	void configInputPullDn(uint16_t portMask);
	void configLEDDriver(uint16_t portMask);
	void setPort(uint16_t portMask);
	void writePortA(uint8_t portVal);
	void writePortB(uint8_t portVal);
	void clearPort(uint16_t portMask);
	uint16_t getPort();
	void setBit(uint8_t bitIndex);
	void clearBit(uint8_t bitIndex);
	bool getBit(uint8_t bitIndex);
	uint16_t getEXTIPin() { return intPin; };
	static ioexpStat_t stat()  { return lastStatus; };
	static TaskHandle_t getHandler() { return hHandler; };
	static std::vector<ioexp *> expanders;
	uint16_t getPortData() { return portData; };
private:
	static ioexpStat_t lastStatus;
	static TaskHandle_t hHandler;
	I2C_HandleTypeDef *hi2c;
	void writeRegister(uint8_t reg, uint8_t regData);
	uint8_t readRegister(uint8_t reg);
	uint8_t i2caddr;
	GPIO_TypeDef *resetPort;
	uint16_t resetPin;
	GPIO_TypeDef *intPort;
	uint16_t intPin;
	uint16_t portData;
};

} /* namespace sx1509 */

#endif /* SRC_IOEXP_H_ */
