/*
 * lcd.cpp
 *
 *  Created on: Apr 2, 2020
 *      Author: Kremmen
 */

#include <FreeRTOS.h>
#include <lcd.h>
#include <task.h>

#include <stdio.h>
#include <stm32f3xx_hal_gpio.h>

namespace lcd {

#ifdef USING_FREERTOS
void Delay( uint32_t ms ) { vTaskDelay(ms / portTICK_PERIOD_MS); }
#else
void Delay(uint32_t ms { HAL_Delay( ms ); }
#endif

//----------------------------
const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
const uint8_t ROW_20[] = {0x00, 0x40, 0x14, 0x54};

LCD::LCD( Lcd_PortType port[], Lcd_PinType pin[], Lcd_PortType rs_Port, Lcd_PinType rs_Pin, Lcd_PortType rw_Port, Lcd_PinType rw_Pin, Lcd_PortType en_Port, Lcd_PinType en_Pin, Lcd_ModeTypeDef mode){
	mode = mode;
	rw_pin = rw_Pin;
	rw_port = rw_Port;
	en_pin = en_Pin;
	en_port = en_Port;
	rs_pin = rs_Pin;
	rs_port = rs_Port;
	data_pin = pin;
	data_port = port;
	HAL_GPIO_WritePin(en_port, en_pin, LCD_ENABLE_OFF);
	HAL_GPIO_WritePin(rw_port, rw_pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(rs_port, rs_pin, LCD_DATA_REG);
}

LCD::~LCD() {
	// TODO Auto-generated destructor stub
}


void LCD::write_init(uint8_t data)
{
	HAL_GPIO_WritePin(rs_port, rs_pin, LCD_COMMAND_REG);			// Select control register
	HAL_GPIO_WritePin(rw_port, rw_pin, GPIO_PIN_RESET);			// Write

		write(data >> 4, LCD_NIB);
}


void LCD::begin() {
	Delay(50);
	if(mode == LCD_4_BIT_MODE) {
		write_command(0x33);
		Delay(5);
		write_command(0x32);
		Delay(5);
		write_command(FUNCTION_SET | OPT_N);
		write_command(CLEAR_DISPLAY);						// Clear screen
		write_command(DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
		write_command(ENTRY_MODE_SET | OPT_INC);			// Increment cursor
	}
	else write_command(FUNCTION_SET | OPT_DL | OPT_N);
	Delay(2);
}

bool LCD::isBusy() {
//	return (read_status() & 0x80) > 0;
	Delay(50);
	return false;
}

void LCD::write_int(uint8_t len, int32_t number) {
	char buffer[11];
	sprintf(buffer, "%*d", len, number);
	write_string(buffer);
}

void LCD::write_float(float number, uint8_t decimals) {
	char buffer[80];
	sprintf(buffer, "%1.*f", decimals, number);

	write_string(buffer);
}


void LCD::write_string(char *string) {
	for(uint8_t i = 0; i < strlen(string); i++)
	{
		write_data(string[i]);
	}
}


void LCD::move_cursor(uint8_t row, uint8_t col) {
	#ifdef LCD20xN
	write_command(SET_DDRAM_ADDR + ROW_20[row] + col);
	#endif

	#ifdef LCD16xN
	write_command(SET_DDRAM_ADDR + ROW_16[row] + col);
	#endif
}


void LCD::clear() {
	write_command(CLEAR_DISPLAY);
	vTaskDelay(2);
}



void LCD::write_command( uint8_t command ) {
	HAL_GPIO_WritePin(rs_port, rs_pin, LCD_COMMAND_REG);		// Write to command register

	if(mode == LCD_4_BIT_MODE)
	{
		write((command >> 4), LCD_NIB);
		write(command & 0x0F, LCD_NIB);
	}
	else
	{
		write(command, LCD_BYTE);
	}

}


void LCD::write_data(uint8_t data) {
	HAL_GPIO_WritePin(rs_port, rs_pin, LCD_DATA_REG);			// Write to data register

	if(mode == LCD_4_BIT_MODE)
	{
		write(data >> 4, LCD_NIB);
		write(data & 0x0F, LCD_NIB);
	}
	else
	{
		write(data, LCD_BYTE);
	}

}

void LCD::write(uint8_t data, uint8_t len)
{
	HAL_GPIO_WritePin(rw_port, rw_pin, LCD_WRITE);
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(data_port[i], data_pin[i], (GPIO_PinState)((data >> i) & 0x01));
	}

	Delay(1);
	HAL_GPIO_WritePin(en_port, en_pin, LCD_ENABLE_ON);
	Delay(1);
	HAL_GPIO_WritePin(en_port, en_pin, LCD_ENABLE_OFF);
}

uint8_t LCD::read_status() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t retVal = 0;

	GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(rs_port, rs_pin, LCD_COMMAND_REG);			// Read from status register
	if(mode == LCD_4_BIT_MODE) {
		retVal = read(LCD_NIB)<<4;
		retVal += read(LCD_NIB) & 0x0f;
	}
	else {
		retVal = read(LCD_BYTE);
	}
	GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	return retVal;
}

uint8_t LCD::read_data() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t retVal = 0;

	GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	HAL_GPIO_WritePin(rs_port, rs_pin, LCD_DATA_REG);			// Read from data register
	if(mode == LCD_4_BIT_MODE) {
		retVal = read(LCD_NIB)<<4;
		retVal += read(LCD_NIB) & 0x0f;
	}
	else {
		retVal = read(LCD_BYTE);
	}
	GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	return retVal;
}

uint8_t LCD::read(uint8_t len) {
	uint8_t retVal = 0;
	HAL_GPIO_WritePin(rw_port, rw_pin, LCD_READ);
	Delay(1);
	HAL_GPIO_WritePin(en_port, en_pin, LCD_ENABLE_ON);
	Delay(1);
	for(uint8_t i = 0; i < len; i++)
	{
		retVal |= (HAL_GPIO_ReadPin(data_port[i], data_pin[i]) & 0x01) << i;
	}
	Delay(1);
	HAL_GPIO_WritePin(en_port, en_pin, LCD_ENABLE_OFF);
	Delay(1);
	HAL_GPIO_WritePin(rw_port, rw_pin, LCD_WRITE);
	return retVal;
}

} //namespace lcd

