/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 *  optimised and edited by Maksim Jeskevic, 2021.08.06
 */

#include "lcd.h"

#include "stm32f1xx_hal.h"
#ifdef USE_SPRINTF
#include "string.h"
#include "stdio.h"
#endif

const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
const uint8_t ROW_20[] = {0x00, 0x40, 0x14, 0x54};
/************************************** Static declarations **************************************/

static void lcd_write(LCD_HandleTypeDef * lcd, uint8_t data, uint8_t len);
static void lcd_init(LCD_HandleTypeDef * lcd);

bool lcd_8line_mode = true;
/************************************** Function definitions **************************************/

/**
 * Create new LCD_HandleTypeDef and initialize the LCD
 */
LCD_HandleTypeDef lcd_create(
		LCD_PortType port[], LCD_PinType pin[],
		LCD_PortType rs_port, LCD_PinType rs_pin,
		LCD_PortType en_port, LCD_PinType en_pin, LCD_ModeTypeDef mode)
{
	LCD_HandleTypeDef lcd;

	lcd.mode = mode;

	lcd.en_pin = en_pin;
	lcd.en_port = en_port;

	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;

	lcd.data_pin = pin;
	lcd.data_port = port;

	delay_us(20000); // Display needs about 10ms to start, so you should provide delay

	lcd_init(&lcd);

	return lcd;
}

/**
 * Initialize 16x2-lcd without cursor
 */
void lcd_init(LCD_HandleTypeDef * lcd)
{
	if(lcd->mode == LCD_4_BIT_MODE)
	{
			lcd_write_command(lcd, 0x33);
			lcd_write_command(lcd, 0x32);
			lcd_write_command(lcd, FUNCTION_SET | OPT_N);				// 4-bit mode
			lcd_8line_mode = false;
	}
	else
	{
		lcd_write_command(lcd, FUNCTION_SET | OPT_DL | OPT_N);
		lcd_8line_mode = true;
	}


	lcd_clear(lcd);											// Clear screen
	lcd_mode(lcd, LCD_ENABLE, CURSOR_DISABLE, NO_BLINK);
	lcd_write_command(lcd, ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}

/**
 * Set display & cursor mode
 */
void lcd_mode(LCD_HandleTypeDef * lcd, LCD_ONOFF state, LCD_CURSOR cursor, LCD_BLINK blink)
{
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL |
								 (state ?OPT_D:0) |
								 (cursor?OPT_C:0) |
								 (blink ?OPT_B:0));
}

#ifdef USE_SPRINTF
/**
 * Write a number on the current position
 */
void Lcd_int(Lcd_HandleTypeDef * lcd, int number)
{
	char buffer[11];
	sprintf(buffer, "%d", number);

	Lcd_string(lcd, buffer);
}
#endif

/**
 * Write a string on the current position
 */
void lcd_out(LCD_HandleTypeDef * lcd, uint8_t * arr, uint8_t length)
{
	for (int i = 0; i < length; i++)
	{
		lcd_write_data(lcd, arr[i]);
	}
}

/**
 * Write a string on the current position
 */
void lcd_string(LCD_HandleTypeDef * lcd, char * string)
{
	while (*string)
	{
		lcd_write_data(lcd, (uint8_t)*(string++));
	}
}

/**
 * Set the cursor position
 */
void lcd_set_xy(LCD_HandleTypeDef * lcd, uint8_t x, uint8_t y)
{
	#ifdef LCD20xN
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_20[y] + x);
	#endif

	#ifdef LCD16xN
	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_16[y] + x);
	#endif
}

/**
 * Clear the screen
 */
void lcd_clear(LCD_HandleTypeDef * lcd) {
	lcd_write_command(lcd, CLEAR_DISPLAY);
	delay_us(maxi_delay);
}

void lcd_define_char(LCD_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]){
	lcd_write_command(lcd, SETCGRAM_ADDR + (code << 3));
	for(uint8_t i=0;i<8;++i){
		lcd_write_data(lcd, bitmap[i]);
	}
	lcd_write_command(lcd, SET_DDRAM_ADDR);
}

// load all 8 chars
void lcd_define_chars(LCD_HandleTypeDef * lcd, uint8_t bitmap[]){
	lcd_write_command(lcd, SETCGRAM_ADDR);
	for(uint8_t i=0;i<64;++i){
		lcd_write_data(lcd, bitmap[i]);
	}
	lcd_write_command(lcd, SET_DDRAM_ADDR);
}


/************************************** Static function definition **************************************/

/**
 * Write a byte to the command register
 */
void lcd_write_command(LCD_HandleTypeDef * lcd, uint8_t command)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG);		// Write to command register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write(lcd, (command >> 4), LCD_NIB);
		if (lcd_8line_mode) delay_us(mini_delay);
		lcd_write(lcd, command & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(lcd, command, LCD_BYTE);
	}
	delay_us(mini_delay);
}

/**
 * Write a byte to the data register
 */
void lcd_write_data(LCD_HandleTypeDef * lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG);			// Write to data register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		lcd_write(lcd, data >> 4, LCD_NIB);
		if (lcd_8line_mode) delay_us(mini_delay);
		lcd_write(lcd, data & 0x0F, LCD_NIB);
	}
	else
	{
		lcd_write(lcd, data, LCD_BYTE);
	}
	delay_us(mini_delay);
}

/**
 * Set len bits on the bus and toggle the enable line
 */
void lcd_write(LCD_HandleTypeDef * lcd, uint8_t data, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}

	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 1);
	DELAY_US(1);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, 0); 		// Data receive on falling edge
	DELAY_US(1);
}
