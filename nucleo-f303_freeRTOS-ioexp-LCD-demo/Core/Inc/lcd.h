/*
 * lcd.h
 *
 *  Created on: Apr 2, 2020
 *      Author: Kremmen
 */

#ifndef HD44780_H_
#define HD44780_H_

//-------------------
#include "stm32f3xx_hal.h"
#include "string.h"
#include "main.h"
#include "ioexp.h"

namespace lcd {
using namespace sx1509;

#define LCD20xN 		// For 20xN LCDs
//#define LCD16xN			// For 16xN LCDs

// For row start addresses
extern const uint8_t ROW_16[];
extern const uint8_t ROW_20[];

/************************************** Command register **************************************/
#define LCD_BOOT 0x30

#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x00
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_CONTROL 0x00
#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display linES
#define OPT_F 0x04						// Set alternate font

#define SET_DDRAM_ADDR 0x80				// Set DDRAM address


/************************************** Helper macros **************************************/
#define DELAY(X) HAL_Delay(X)


/************************************** LCD defines **************************************/
#define LCD_NIB 4
#define LCD_BYTE 8
#define LCD_DATA_REG GPIO_PIN_SET
#define LCD_COMMAND_REG GPIO_PIN_RESET
#define LCD_READ GPIO_PIN_SET
#define LCD_WRITE GPIO_PIN_RESET
#define LCD_ENABLE_ON GPIO_PIN_SET
#define LCD_ENABLE_OFF GPIO_PIN_RESET


/************************************** LCD typedefs **************************************/
#define Lcd_PortType GPIO_TypeDef*
#define Lcd_PinType uint16_t

// i2c-i/o-expanderin portin offsetit
#define EXP_RS 9
#define EXP_RW 10
#define EXP_EN 11

typedef enum {
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE,
	LCD_EXP1509_MODE	// I2C-väylälaajentimen avulla 8-bittinen moodi
} Lcd_ModeTypeDef;

enum vType {u8, i8, u16, i16, u32, i32, f, d, s};
union vUnion {
	uint8_t U8;
	int8_t	I8;
	uint16_t U16;
	int16_t I16;
	uint32_t U32;
	int32_t I32;
	float F;
	double D;
	char *S;
};

class LCDMessage {
public:
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, uint8_t val) {x=X; y=Y; T=u8; Len=len; v.U8=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, int8_t val) {x=X; y=Y; T=i8; Len=len; v.I8=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, uint16_t val) {x=X; y=Y; T=u16; Len=len; v.U16=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, int16_t val) {x=X; y=Y; T=i16; Len=len; v.I16=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, uint32_t val) {x=X; y=Y; T=u32; Len=len; v.U32=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, int32_t val) {x=X; y=Y; T=i32; Len=len; v.I32=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, float val) {x=X; y=Y; T=f; Len=len; v.F=val; };
	LCDMessage(uint8_t X, uint8_t Y, uint8_t len, double val) {x=X; y=Y; T=d; Len=len; v.D=val; };
	LCDMessage(uint8_t X, uint8_t Y, char *val) {x=X; y=Y; T=s; v.S=val; };
//	~LCDMessage() { if ( T == s ) delete(v.S); };
	uint8_t x,y;
	uint8_t Len;
	vType T;
	vUnion v;
};

//-------------------
class LCD {
public:
	LCD( Lcd_PortType port[], Lcd_PinType pin[], Lcd_PortType rs_Port, Lcd_PinType rs_Pin, Lcd_PortType rw_Port, Lcd_PinType rw_Pin, Lcd_PortType en_Port, Lcd_PinType en_Pin, Lcd_ModeTypeDef mode);
	virtual ~LCD();
	LCD( ioexp* Exp );
	void begin();
	void write_int(uint8_t len, int32_t number);
	void write_float( float number, uint8_t decimals );
	void write_string(char *string);
	void move_cursor(uint8_t row, uint8_t col);
	void clear();
private:
	void write_init(uint8_t data);
	bool isBusy();
	void write_command(uint8_t command);
	void write_data(uint8_t data);
	void write(uint8_t data, uint8_t len);
	uint8_t read_status();
	uint8_t read_data();
	uint8_t read(uint8_t len);
	Lcd_PortType *data_port;
	Lcd_PinType *data_pin;
	Lcd_PortType rs_port;
	Lcd_PinType rs_pin;
	Lcd_PortType rw_port;
	Lcd_PinType rw_pin;
	Lcd_PortType en_port;
	Lcd_PinType en_pin;
	Lcd_ModeTypeDef mode;
	ioexp *ioExp;
};

} // namespace LCD

#endif /* HD44780_H_ */
