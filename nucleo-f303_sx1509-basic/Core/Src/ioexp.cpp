/*
 * ioexp.cpp
 *
 *  Created on: May 13, 2020
 *      Author: martti
 */
#include <assert.h>
#include "ioexp.h"

namespace sx1509 {

#define SX1509_I2C_RETRY_CNT	2
#define SX1509_I2C_TIMEOUT		2

StaticSemaphore_t EXTISyncBuffer;
SemaphoreHandle_t hEXTISync;

// exti handler
void extiHandler( void *pvParameters);
#define EXTI_STACKSIZE	128
static StackType_t extiStack[EXTI_STACKSIZE];
static StaticTask_t extiTCB;
static TaskHandle_t hEXTI;


void Delay( uint32_t ms ) { vTaskDelay(ms / portTICK_PERIOD_MS); }

ioexpStat_t ioexp::lastStatus;
TaskHandle_t ioexp::hHandler;
std::vector<ioexp *> ioexp::expanders;

ioexp::ioexp(I2C_HandleTypeDef *hI2C, uint8_t i2cAddr, GPIO_TypeDef *reset_Port, uint16_t reset_Pin, GPIO_TypeDef *int_Port, uint16_t int_Pin, initMode_t mode) {
	HAL_StatusTypeDef stat;
	hi2c = hI2C;
	i2caddr = i2cAddr<<1;
	resetPort = reset_Port;
	resetPin = reset_Pin;
	intPort = int_Port;
	intPin = int_Pin;
	portData = 0;

	// resetoidaan piiri
	HAL_GPIO_WritePin(resetPort, resetPin, GPIO_PIN_RESET);
	Delay(1);
	HAL_GPIO_WritePin(resetPort, resetPin, GPIO_PIN_SET);
	Delay(10);

	// tsekataan että se näkyy i2c-väylässä
	stat = HAL_I2C_IsDeviceReady(hi2c, i2caddr, SX1509_I2C_RETRY_CNT, SX1509_I2C_TIMEOUT );
	if ( stat != HAL_I2C_ERROR_NONE ) {
		ioexp::lastStatus = expERROR;
	}
	else {
		ioexp::lastStatus = expOK;
		// asetetaan portit aloitusmodeen
		writeRegister( RegDataA, 0x00 );
		writeRegister( RegDataB, 0x00 );
		switch ( mode ) {
			case inin: {
				writeRegister( RegDirA, 0xff );
				writeRegister( RegDirB, 0xff );
				break;
			}
			case inout: {
				writeRegister( RegDirA, 0xff );
				writeRegister( RegDirB, 0x00 );
				break;

			}
			case outin: {
				writeRegister( RegDirA, 0x00 );
				writeRegister( RegDirB, 0xff );
				break;

			}
			case outout: {
				writeRegister( RegDirA, 0x00 );
				writeRegister( RegDirB, 0x00 );
				break;

			}
			default: {
				writeRegister( RegDirA, 0xff );
				writeRegister( RegDirB, 0xff );

			}
		}

		if ( expanders.empty() ) {
			hEXTISync = xSemaphoreCreateBinaryStatic( &EXTISyncBuffer );
			hEXTI = xTaskCreateStatic( extiHandler, "EXTI", EXTI_STACKSIZE, NULL, tskIDLE_PRIORITY+3, extiStack, &extiTCB );
			hHandler = hEXTI;
		}
		expanders.push_back(this);
	}
}

ioexp::~ioexp() {
	// TODO Auto-generated destructor stub
}

void ioexp::configPushPullOutput(uint16_t portMask) {
	uint16_t tmp;
	uint8_t R;
	tmp = ( readRegister( RegDirB ) << 8 ) | readRegister( RegDirA );
	R = ~portMask & tmp;
	writeRegister( RegDirA, R );
	R = (~portMask & tmp) >> 8;
	writeRegister( RegDirB, R );
	tmp = ( readRegister( RegOpenDrainB ) << 8 ) | readRegister( RegOpenDrainA );
	R = ~portMask & tmp;
	writeRegister( RegOpenDrainA, R );
	R = (~portMask & tmp) >> 8;
	writeRegister( RegOpenDrainB,  R);
}

void ioexp::configOpenDrainOutput(uint16_t portMask) {
	uint16_t tmp;
	tmp = ( readRegister( RegDirB ) << 8 ) | readRegister( RegDirA );
	writeRegister( RegDirA, (~portMask & tmp) | 0xff00 );
	writeRegister( RegDirB, ((~portMask & tmp) >> 8) | 0xff00 );
	tmp = ( readRegister( RegOpenDrainB ) << 8 ) | readRegister( RegOpenDrainA );
	writeRegister( RegOpenDrainA, (portMask | tmp) & 0x00ff );
	writeRegister( RegOpenDrainB, ((portMask | tmp) >> 8) & 0x00ff );

}

void ioexp::configInput(uint16_t portMask) {

}

void ioexp::configInputPullUp(uint16_t portMask) {

}

void ioexp::configInputPullDn(uint16_t portMask) {

}


void ioexp::configLEDDriver(uint16_t portMask) {
// Disable input buffer (RegInputDisable)
	writeRegister( RegInputDisableA, portMask & 0x00ff );
	writeRegister( RegInputDisableB, (portMask>>8) & 0x00ff );
// Disable pull-up (RegPullUp)
	writeRegister( RegPullUpA, ~(portMask & 0x00ff) );
	writeRegister( RegPullUpB, ~((portMask>>8) & 0x00ff) );
// Enable open drain (RegOpenDrain)
	writeRegister( RegOpenDrainA, portMask & 0x00ff );
	writeRegister( RegOpenDrainB, (portMask>>8) & 0x00ff );
// Set direction to output (RegDir) – by default RegData is set high => LED OFF
	writeRegister( RegDirA, portMask & 0x00ff );
	writeRegister( RegDirB, (portMask>>8) & 0x00ff );
// Enable oscillator (RegClock)
// Configure LED driver clock and mode if relevant (RegMisc)
// Enable LED driver operation (RegLEDDriverEnable)
	writeRegister( RegLEDDriverEnableA, portMask & 0x00ff );
	writeRegister( RegLEDDriverEnableB, (portMask>>8) & 0x00ff );
// Configure LED driver parameters (RegTOn, RegIOn, RegOff, RegTRise, RegTFall)
// Set RegData bit low => LED driver started
}

// Asetetaan I/O-pinnit joiden maskibitti on '1'. Muut pinnit pysyvät ennallaan.
void ioexp::setPort(uint16_t portMask) {
	writeRegister( RegDataA, ( ( portMask & 0x00ff ) | readRegister( RegDataA ) ) );
	writeRegister( RegDataB, ( ( ( portMask>>8 ) & 0x00ff ) | readRegister( RegDataB ) ) );
}
void ioexp::writePortA(uint8_t portVal) {
	writeRegister( RegDataA, portVal);
}
void ioexp::writePortB(uint8_t portVal) {
	writeRegister( RegDataB, portVal);
}

// Nollataan I/O-pinnit joiden maskibitti on '1'. Muut pinnit pysyvät ennallaan.
void ioexp::clearPort(uint16_t portMask) {
	writeRegister( RegDataA, ( ~( portMask & 0x00ff ) & readRegister( RegDataA ) ) );
	writeRegister( RegDataB, ( ~( ( portMask >> 8 ) & 0x00ff ) & readRegister( RegDataB ) ) );

}

// palauttaa I/O-pinnien tilan
uint16_t ioexp::getPort() {
	portData = ( readRegister( RegDataB ) << 8 ) | readRegister( RegDataA );
	return portData;
}

// asettaa indeksin osoittaman bitin
void ioexp::setBit(uint8_t bitIndex) {
	assert ( bitIndex < 16 );
	if (bitIndex < 8) writeRegister( RegDataA, 1<<bitIndex | readRegister( RegDataA ) );
	else writeRegister( RegDataB, 1<<bitIndex / 2 | readRegister( RegDataB ) );
}


void ioexp::clearBit(uint8_t bitIndex) {
	assert ( bitIndex < 16 );
	if (bitIndex < 8) writeRegister( RegDataA, ~( 1 << bitIndex ) & readRegister( RegDataA ) );
	else writeRegister( RegDataB, ~( 1 << bitIndex / 2 ) & readRegister( RegDataB ) );

}


bool ioexp::getBit(uint8_t bitIndex) {
	assert ( bitIndex < 16 );
	if (bitIndex < 8) return ( readRegister( RegDataA ) & ( 1 << bitIndex ) ) > 0 ;
	else return ( readRegister( RegDataB ) & ( 1 << ( bitIndex / 2 ) ) ) > 0 ;
}

void ioexp::writeRegister(uint8_t reg, uint8_t regData) {
	HAL_StatusTypeDef st;
	st = HAL_I2C_Mem_Write(hi2c, i2caddr, reg, sizeof(uint8_t), &regData, sizeof(regData), 0xffffffff );
}

uint8_t ioexp::readRegister(uint8_t reg) {
	uint8_t regData;
	HAL_I2C_Mem_Read(hi2c, i2caddr, reg, sizeof(uint8_t), &regData, sizeof(regData), 0xffffffff );
	return regData;
}



// Exti-keskeytyksen käsittely taskitasolla.
// Tätä ei tehdä keskeytyshandlerissa koska i2c-väylän käsittely,
// ja kestää muutenkin turhan kauan ajettavaksi keskeytyskontekstissa
void extiHandler( void *pvParameters) {
	ioexp *pIOE;
	for ( ;; ) {
		if ( xSemaphoreTake( hEXTISync, portMAX_DELAY ) == pdTRUE ) {
			for ( std::vector<ioexp *>::iterator it = ioexp::expanders.begin() ; it != ioexp::expanders.end(); ++it) {
				pIOE = *it;
				if ( pIOE->getEXTIPin() == GPIO_PIN_1 ) pIOE->getPort();
			}
		}
	}
}

// Exti-keskeytyksen callback joka välittää tiedon exti-käsittelijätaskille
// samalla vapauttaen keskeytyskäsittelijän jottei sen konteksti kestä
// kauempaa kuin on välttämätöntä.
// tämä funktio korvaa samannimisen __weak__ attribuutilla varustetun oletuskäsittelijän
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	BaseType_t xHigherPriorityTaskWoken;
	if ( ioexp::getHandler() != NULL ) {
		if ( GPIO_Pin == GPIO_PIN_1 ) {
			xSemaphoreGiveFromISR( hEXTISync, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}
	}
}

} /* namespace sx1509 */
