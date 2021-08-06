/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 *  optimised and edited by Maksim Jeskevic, 2021.08.06
 */

#ifndef LCD_H_
#define LCD_H_

#include "main.h"

#include "delay.h"

// #define LCD20xN 		// For 20xN LCDs
#define LCD16xN			// For 16xN LCDs

//#define USE_SPRINTF	// use sprintf to convert int to string

// For row start addresses
extern const uint8_t ROW_16[];
extern const uint8_t ROW_20[];

/************************************** Command register **************************************/
#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display lines
#define OPT_F 0x04						// Set alternate font
#define SETCGRAM_ADDR 0x040
#define SET_DDRAM_ADDR 0x80				// Set DDRAM address


/************************************** Helper macros **************************************/
#define DELAY_US(X) delay_us(X)

#define mini_delay 42
#define maxi_delay 1500
/************************************** LCD defines **************************************/
#define LCD_NIB 4
#define LCD_BYTE 8
#define LCD_DATA_REG 1
#define LCD_COMMAND_REG 0


/************************************** LCD typedefs **************************************/
#define LCD_PortType GPIO_TypeDef*
#define LCD_PinType uint16_t

typedef enum {
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE
} LCD_ModeTypeDef;

typedef enum {
	LCD_DISABLE = 0,
	LCD_ENABLE = !LCD_DISABLE
} LCD_ONOFF;

typedef enum {
	CURSOR_DISABLE = 0,
	CURSOR_ENABLE = !CURSOR_DISABLE
} LCD_CURSOR;

typedef enum {
	NO_BLINK = 0,
	BLINK = !NO_BLINK
} LCD_BLINK;


typedef struct {
	LCD_PortType * data_port;
	LCD_PinType * data_pin;

	LCD_PortType rs_port;
	LCD_PinType rs_pin;

	LCD_PortType en_port;
	LCD_PinType en_pin;

	LCD_ModeTypeDef mode;

} LCD_HandleTypeDef;


/************************************** Public functions **************************************/
LCD_HandleTypeDef lcd_create(
		LCD_PortType port[], LCD_PinType pin[],
		LCD_PortType rs_port, LCD_PinType rs_pin,
		LCD_PortType en_port, LCD_PinType en_pin, LCD_ModeTypeDef mode);
//void lcd_init(LCD_HandleTypeDef * lcd); - not actually needed, use create
void lcd_string(LCD_HandleTypeDef * lcd, uint8_t * string); // uint8_t instead of char to comply with USB
void lcd_set_xy(LCD_HandleTypeDef * lcd, uint8_t x, uint8_t y);
void lcd_mode(LCD_HandleTypeDef * lcd, LCD_ONOFF state, LCD_CURSOR cursor, LCD_BLINK blink);
void lcd_define_char(LCD_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]);
void lcd_define_chars(LCD_HandleTypeDef * lcd, uint8_t bitmap[]); // load all 8 chars
void lcd_clear(LCD_HandleTypeDef * lcd);
void lcd_write_command(LCD_HandleTypeDef * lcd, uint8_t command);
void lcd_write_data(LCD_HandleTypeDef * lcd, uint8_t data);
#ifdef USE_SPRINTF
void lcd_int(LCD_HandleTypeDef * lcd, int number);
#endif

#endif /* LCD_H_ */
