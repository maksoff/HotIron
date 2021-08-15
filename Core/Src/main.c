/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "delay.h"
#include "lcd.h"
#include "string.h"
#include "spi_rxonly.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HOT_TEMP (40) // 40 grad
#define MAX_TEMP (270)
#define STEP_TEMP (5)


#define DEBUG_A1 HAL_GPIO_WritePin(debug_a_GPIO_Port, debug_a_Pin, 1)
#define DEBUG_B1 HAL_GPIO_WritePin(debug_b_GPIO_Port, debug_b_Pin, 1)
#define DEBUG_A0 HAL_GPIO_WritePin(debug_a_GPIO_Port, debug_a_Pin, 0)
#define DEBUG_B0 HAL_GPIO_WritePin(debug_b_GPIO_Port, debug_b_Pin, 0)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

LCD_HandleTypeDef lcd;

// ports & pins should be globally visible
LCD_PortType ports[] = {	hd_4_GPIO_Port,
  	  	  					hd_5_GPIO_Port,
							hd_6_GPIO_Port,
							hd_7_GPIO_Port};
LCD_PinType pins[] = {		hd_4_Pin,
	  	    				hd_5_Pin,
							hd_6_Pin,
							hd_7_Pin};

enum {
	errOK = 0,
	errTEMP_SP = 1<<0,
	errTEMP_PV = 1<<1,
	errHEATER = 1<<2,
	errI_LIMIT = 1<<3,
	errMAX6675_invalid = 1<<4,
	errTIMEOUT = 1<<5,
	errFATAL = 1 << 6
};

uint8_t global_error = errOK;
uint8_t ticktack = 0; // using for display blink


#define encoder_value (TIM3->CNT)
struct sBUTTON {
	bool pressed;
	bool long_press;
} button = {.pressed = false, .long_press = false};


struct sMAX6675 {
	uint16_t temperature; // precise to 0.25 grad
	bool data_valid;
	uint8_t ascii[7];  // converted to ASCII
} MAX6675 = {.data_valid = false, .temperature = 0};

uint16_t pwm_value = 0;
uint16_t temperature_SP = 0;
struct sPID {
	int32_t P;
	int32_t I;
	int32_t D;
} PID;
bool tick = false;


typedef struct {
	int32_t temp;
	uint32_t time;
} sSTEPS;

const sSTEPS steps_default[] = {
		{.temp = 50, .time=30},
		{.temp = 45, .time=25},
//		{.temp = 130, .time=90},
//		{.temp = 210, .time=15},
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void int2string(uint32_t digit, uint8_t * buf, uint8_t len)
{
	for (int i = len - 1; i >= 0; i--)
	{
		if (digit || (i == (len-1)))
		{
			buf[i] = digit % 10 + '0';
			digit /= 10;
		}
		else
			buf[i] = ' ';
	}
}

/**
 * converts int to time string
 * @param time - time in msec
 * @param buf - provide buffer for 5 positions
 */
char * int2time(uint32_t time, uint8_t * buf)
{
	buf[4] = '\0';
	if (time >= 600)
	{
		// display time in minutes
		buf[3] = 'm';
		time /= 60;
		if (time >= 999)
		{
			global_error |= errTIMEOUT;
			time = 999;
		}
		int2string(time, buf, 3);
	} else
	{
		buf[1] = ':';
		buf[3] = time % 10 + '0';
		time /= 10;
		buf[2] = time % 6 + '0';
		time /= 6;
		buf[0] = time + '0';
	}
	return (char *)buf;
}

// custom characters
enum {
	  ccENTER,
	  ccDOWN,
	  ccUPEQ,
	  ccUP,
	  ccHOT,
	  ccHOTmirror,
	  cc3dots,
	  ccDOT
};

// useful standart characters
enum {
	scAR = 0x7E, // arrow right
	scGRAD = 223, // ° symbol
	scDOT = 0xA5, // big dot in the middle
	scSIGMA = 0xF6, // sigma
};

void init_lcd(void)
{
	  lcd = lcd_create(ports, pins,
						hd_RS_GPIO_Port, hd_RS_Pin,
						hd_E_GPIO_Port, hd_E_Pin,
						LCD_4_BIT_MODE);

	  /* load symbols */

	  uint8_t symbols [] = {
	  	  	  				0x0, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4, 0x0,   // ENTER
							0x0, 0x0, 0x0, 0x0, 0x11, 0xa, 0x4, 0x0,	// DOWN SMALL
							0x4, 0xe, 0x1f, 0x0, 0x1f, 0x0, 0x0, 0x0,	// UP EQUAL
							0x4, 0xe, 0x1f, 0x0, 0x0, 0x0, 0x0, 0x0,   // UP
							0x9, 0x12, 0x9, 0x12, 0x0, 0x1f, 0x1f, 0x0, // HOT
							0x12, 0x9, 0x12, 0x9, 0x0, 0x1f, 0x1f, 0x0, // HOT mirror
							0x0, 0x4, 0x0, 0x4, 0x0, 0x4, 0x0, 0x0,    // 3 dots
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0,		// 1 dot
//						    0x4, 0xe, 0x1f, 0x0, 0x1f, 0xe, 0x4, 0x0,  // UP DOWN FULL
//							0x0, 0x0, 0x0, 0x0, 0x1f, 0xe, 0x4, 0x0,   // DOWN FULL
//							0xa, 0x1f, 0x1f, 0x1f, 0xe, 0x4, 0x0, 0x0, // heart big
//			  	  	  	  	0x0, 0xe, 0x11, 0x15, 0x11, 0xe, 0x0, 0x0, // OFF
//			  	  	  	  	0x0, 0x4, 0x15, 0x15, 0x11, 0xe, 0x0, 0x0, // ON
//							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x15, 0x0,   // ellips
//							0x0, 0x0, 0xa, 0x1f, 0xe, 0x4, 0x0, 0x0, // heart
	  };
	  lcd_define_chars(&lcd, symbols);
	  lcd_set_xy(&lcd, 0, 0);
	  lcd_string(&lcd, "Maksim Jeskevic ");
	  lcd_set_xy(&lcd, 0, 1);
	  lcd_string(&lcd, "         2021 08");
	  HAL_Delay(1500);
	  lcd_clear(&lcd);
}

void do_button(void)
{
	const uint32_t time_for_long_press = 700;
	static uint32_t last_time = 0;
	static bool last_button = false;
	static uint32_t but_time = 0;
	if (HAL_GetTick() - last_time < 20)
		return;
	button.pressed = !HAL_GPIO_ReadPin(enc_s_GPIO_Port, enc_s_Pin);
	if (button.pressed)
	{
		if (!last_button)
			but_time = HAL_GetTick();
		if (HAL_GetTick() - but_time > time_for_long_press)
			button.long_press = true;
	}
	else
		button.long_press = false;
	last_button = button.pressed;

	last_time = HAL_GetTick();
}

void do_blink(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 500)
		return;
	last_time = HAL_GetTick();
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/**
 * read temperature from max6675
 * and check for validity
 */
void get_max6675(void)
{
	uint16_t data;
	HAL_SPI_ReceiveOnly(&hspi1, (uint8_t*)(&data), 1, 100);

	MAX6675.data_valid = !(data & 0b110);
	MAX6675.temperature = data >> 3;
	if (!(MAX6675.data_valid))
		MAX6675.temperature = 0xfff;
}

void ascii_max6675(void)
{
	if (MAX6675.data_valid)
		{
			uint32_t digit = 25*(MAX6675.temperature&0b11);
			digit += (MAX6675.temperature>>2)*1000;
			int8_t i = 6;
			while (digit)
			{
				MAX6675.ascii[i--] = '0' + digit%10;
				digit /= 10;
			}
			while (i >= 0)
			{
				if (i > 2)
					MAX6675.ascii[i--] = '0';
				else
					MAX6675.ascii[i--] = ' ';
			}
			MAX6675.ascii[4] = '.';
		}
		else
		{
			for (int i = 0; i < sizeof(MAX6675); i ++)
				MAX6675.ascii[i] = 'x';
		}
}

void do_usb(void)
{
	if (!tick)
		return;
	tick = false; // sync with ticks

	uint8_t buf[200];
	uint16_t n = snprintf((char*)buf, 200,
			"Tick: %lu; PV: %u.%02u; SP: %u; PWM: %u; P: %li; I: %li; D: %li\r",
						HAL_GetTick()/1000,
						MAX6675.temperature>>2,
						((MAX6675.temperature)&0b11)*25,
						temperature_SP,
						pwm_value,
						PID.P,
						PID.I,
						PID.D);
	CDC_Transmit_FS(buf, n);
}

typedef enum {
	uiSTART,
	uiLONG_PRESS,
	uiMENUenter,
	uiMENU,
	uiHEATPLATEenter,
	uiHEATPLATE,
	uiSETTINGSenter,
	uiSETTINGS,
	uiREFLOW,
	uiMALFUNCTION,
} eUISTATE;
eUISTATE ui_state = uiSTART;

/**
 * Function for the main interface, also for error states and co.
 */
void do_interface(void)
{
#define MAX_STEPS (MAX_STEPS)
	static sSTEPS steps[9]; // steps for reflow
	static uint8_t max_steps = 2; // how many steps, max. 10

	static bool first_time = true;
	if (first_time)
	{
		max_steps = sizeof(steps_default)/sizeof(steps_default[0]);
		for (int i = 0; i < max_steps; i ++)
		{
			steps[i].temp = steps_default[i].temp;
			steps[i].time = steps_default[i].time;
		}
		first_time = false;
	}

	/**
	 * clears left part of the display
	 */
	void lcd_mini_clear(LCD_HandleTypeDef * lcd)
	{
		lcd_mode(lcd, ENABLE, CURSOR_DISABLE, NO_BLINK);
		lcd_set_xy(lcd, 0, 0);
		lcd_string(lcd, "            ");
		lcd_set_xy(lcd, 0, 1);
		lcd_string(lcd, "             ");
	}

	/**
	 * helper function to change temperature with encoder
	 * @param t - current value
	 * @param diff - delta from encoder
	 * @return - value to set
	 */
	uint32_t change_temperature(uint32_t t, int32_t diff)
	{
		int32_t temp = (int32_t)t; // to avoid zero-cross
		temp = (temp / STEP_TEMP) * STEP_TEMP; // round for more beauty
		temp += (diff>>1)*STEP_TEMP;
		if (temp < 0)
			temp = 0;
		if (temp > MAX_TEMP)
			temp = MAX_TEMP;
		return (uint32_t)temp;
	}

	/**
	 * helper function to change time with encoder
	 * @param t - current value
	 * @param diff - delta from encoder
	 * @return - value to set
	 */
	uint32_t change_time(uint32_t t, int32_t diff)
	{
		int32_t temp = (int32_t)t; // to avoid zero-cross
		int32_t step = 5; // how much to change the time in seconds
		if (t < 60)
			step = 5;
		else if (t < 120)
			step = 10;
		else if (t < 300)
			step = 20;
		else if (t < 600)
			step = 30;
		else if (t < 1200)
			step = 60;
		else if (t < 3600)
			step = 300;
		else if (t < 7200)
			step = 600;
		else
			step = 1800;

		temp = (temp / step) * step; // round
		temp += (diff>>1)*step;
		if (temp < 5)
			temp = 5;
		if (temp > 900*60)
			temp = 900*60;
		return (uint32_t)temp;
	}

	/**
	 * "heatplate mode" - just constant heating
	 */
	void heatplate(bool reset)
	{
		static uint16_t last_encoder = 0;
		static volatile int16_t diff = 0;
		static const uint16_t upper_limit=(MAX_TEMP/STEP_TEMP*2);

		static uint32_t last_time = 0;

		if (reset)
		{
			last_encoder = encoder_value;
			diff = 0;
			last_time = HAL_GetTick();
			return;
		}

		diff-=(int16_t)(encoder_value - last_encoder);
		last_encoder = encoder_value;

		if (diff < 0)
			diff = 0;
		if (diff > upper_limit)
			diff = upper_limit; // between 0*2 and MAX_TEMP*2

		if (button.pressed)
			diff = 0;

		// show time
		uint8_t tbuf[6];
		tbuf[0] = '+';
		int2time((HAL_GetTick() - last_time)/1000, tbuf+1);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, (char*)tbuf);

		// show temperature
		uint8_t buf[3];
		temperature_SP = STEP_TEMP*(diff>>1);
		int2string(temperature_SP, buf, sizeof(buf));
		lcd_set_xy(&lcd, 7, 0);
		lcd_out(&lcd, buf, sizeof(buf));
		lcd_write_data(&lcd, scGRAD); // grad
		lcd_write_data(&lcd, scAR); // arrow right
		lcd_set_xy(&lcd, 9, 0);
		lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);

	}

	/**
	 * profile settings
	 * provides interfece to edit/add/remove steps
	 */
	bool do_profile_settings(bool reset)
	{
		static int8_t pos = 0; // signed, to prevent negative overflow
		static uint8_t profile_state = 0;
		static uint32_t last_time = 0;
		static uint16_t last_encoder = 0;
		static volatile int16_t diff = 0;
		static bool last_button = false;

		uint8_t time_buf[5];

		if (reset)
		{
			last_encoder = encoder_value;
			diff = 0;
			pos = 0;
			profile_state = 0;
			last_time = HAL_GetTick();
			return false;
		}
		if (HAL_GetTick() - last_time < 1000)
		{
			return false; // delay to show intro text
		}

		void show_step_menu(void)
		{
			// show time
			int2time(steps[pos].time, time_buf);
			lcd_set_xy(&lcd, 5, 1);
			lcd_string(&lcd, (char*)time_buf);

			// show temperature
			uint8_t buf[3];
			int2string(steps[pos].temp, buf, sizeof(buf));
			lcd_set_xy(&lcd, 0, 1);
			lcd_out(&lcd, buf, sizeof(buf));
			lcd_write_data(&lcd, scGRAD); // grad
			lcd_write_data(&lcd, ' ');

			// write number of step
			lcd_set_xy(&lcd, 0, 0);
			lcd_write_data(&lcd, '#');
			lcd_write_data(&lcd, pos+'1');
			lcd_write_data(&lcd, '/');
			lcd_write_data(&lcd, max_steps + '0');
			lcd_string(&lcd, " step   ");
		}

		diff-=(int16_t)(encoder_value - last_encoder);
		last_encoder = encoder_value;

		switch (profile_state)
		{
		case 0: // init
			pos = 0;
			profile_state = 1;
			break;
		case 1: // select profile
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
					profile_state = 10;
			}
			else
			{
				if (diff > 1)
				{
					pos++;
					diff = 0;
				}
				else if (diff < -1)
				{
					pos--;
					diff = 0;
				}
			}
			if ((pos < 0) || (pos >= max_steps))
			{
				profile_state = 90;
				break;
			}

			show_step_menu();

			lcd_set_xy(&lcd, 9, 1);
			lcd_string(&lcd, "  ");
			lcd_write_data(&lcd, cc3dots);
			lcd_write_data(&lcd, ' ');
			lcd_set_xy(&lcd, 11, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);

			break;
		case 10: // confirm current profile
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
					profile_state = 1; // stop editing
			}
			else
			{
				if (diff > 1)
				{
					profile_state = 21;
					diff = 0;
				}
				else if (diff < -1)
				{
					profile_state = 12;
					diff = 0;
				}
			}

			show_step_menu();

			lcd_set_xy(&lcd, 9, 1);
			lcd_string(&lcd, " +x");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 12, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
			break;
		case 11: // add new step
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
				{
					if (max_steps >= 9)
						profile_state = 13;
					else
					{
						// adding position
						for (int i = 8; i > pos; i--)
						{
							steps[i].time = steps[i-1].time;
							steps[i].temp = steps[i-1].temp;
						}
						pos++;
						steps[pos].time = 60;
						steps[pos].temp = 100;
						max_steps++;
						profile_state = 10; // wait for confirmation
					}
				}
			}
			else
			{
				if (diff > 1)
				{
					profile_state = 12;
					diff = 0;
				}
				else if (diff < -1)
				{
					profile_state = 23;
					diff = 0;
				}
			}

			show_step_menu();

			lcd_set_xy(&lcd, 9, 1);
			lcd_string(&lcd, " +x");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 10, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);

			break;
		case 12: // delete current step
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
				{
					if (max_steps <= 1)
						profile_state = 14;
					else
					{
						for (int i = pos; i < 8; i++)
						{
							steps[i].time = steps[i+1].time;
							steps[i].temp = steps[i+1].temp;
						}
						max_steps--;
						if (pos >= max_steps) 	// if it was last position
							pos = max_steps-1; 	// move to the previous one
						profile_state = 10; 	// wait for confirmation
					}
				}
			}
			else
			{
				if (diff > 1)
				{
					profile_state = 10;
					diff = 0;
				}
				else if (diff < -1)
				{
					profile_state = 11;
					diff = 0;
				}
			}

			show_step_menu();

			lcd_set_xy(&lcd, 9, 1);
			lcd_string(&lcd, " +x");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 11, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);

			break;
		case 13: // can't add steps anymore
			if (last_button && (!button.pressed)) // wait for confirmation
				profile_state = 10;
			lcd_set_xy(&lcd, 0, 0);
			lcd_string(&lcd, "Not possible");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "max 9 steps ");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 12, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
			break;
		case 14: // can't delete steps anymore
			if (last_button && (!button.pressed)) // wait for confirmation
				profile_state = 10;
			lcd_set_xy(&lcd, 0, 0);
			lcd_string(&lcd, "Not possible");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "min 1 step  ");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 12, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
			break;
		case 21: // wait temperature edit
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
					profile_state = 22; // start edit
			}
			else
			{
				if (diff > 1)
				{
					profile_state = 23;
					diff = 0;
				}
				else if (diff < -1)
				{
					profile_state = 10;
					diff = 0;
				}
			}

			show_step_menu();

			lcd_set_xy(&lcd, 9, 1);
			lcd_string(&lcd, " +x");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 2, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);

			break;
		case 22: // edit temperature
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
					profile_state = 21; // stop editing
			}
			else
			{
				if (diff > 1)
				{
					diff = 0;
				}
				else if (diff < -1)
				{
					diff = 0;
				}
			}

			show_step_menu();


			break;
		case 23: // wait time edit
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
					profile_state = 24; // start edit
			}
			else
			{
				if (diff > 1)
				{
					profile_state = 11;
					diff = 0;
				}
				else if (diff < -1)
				{
					profile_state = 21;
					diff = 0;
				}
			}

			show_step_menu();

			lcd_set_xy(&lcd, 0, 1);

			lcd_set_xy(&lcd, 9, 1);
			lcd_string(&lcd, " +x");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 8, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);

			break;
		case 24: // edit time
			if (diff>>1 == 0)
			{
				if (last_button && (!button.pressed))
					profile_state = 23; // stop editing
			}
			else
			{
				if (diff > 1)
				{
					diff = 0;
				}
				else if (diff < -1)
				{
					diff = 0;
				}
			}

			show_step_menu();


			break;
		case 90: // last menu
			lcd_set_xy(&lcd, 0, 0);
			lcd_write_data(&lcd, ' ');
			lcd_write_data(&lcd, scSIGMA);
			lcd_write_data(&lcd, ' ');
			lcd_write_data(&lcd, max_steps + '0');
			lcd_string(&lcd, " steps  ");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "start prof. ");
			lcd_write_data(&lcd, ccENTER);
			lcd_set_xy(&lcd, 12, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
			if (diff == 0)
			{
				if (last_button && (!button.pressed))
				{
					lcd_mini_clear(&lcd);
					return true;
				}
			}
			else
			{
				if (diff > 1)
				{
					pos = 0;
					diff = 0;
				}
				else if (diff < -1)
				{
					pos = max_steps - 1;
					diff = 0;
				}
				profile_state = 1;
			}
			break;
		default:
			global_error |= errFATAL;
			break;
		}

		last_button = button.pressed;
		return false;
	}

	/**
	 * do heat/wait in steps
	 */
	void do_reflow(bool reset)
	{
		static uint32_t last_time = 0;
		volatile static uint8_t pos = 0;

		if (reset)
		{
			last_time = HAL_GetTick();
			pos = 0;
			return;
		}

		uint8_t time_buf[6];

		int32_t dt = ((int32_t)(temperature_SP<<2)) -
					 ((int32_t)MAX6675.temperature);

		if (pos >= (2*max_steps))
		{
			// TODO peep-peep and mute on press
			lcd_mode(&lcd, ENABLE, CURSOR_DISABLE, NO_BLINK);
			temperature_SP = 0;
			lcd_set_xy(&lcd, 0, 0);
			lcd_string(&lcd, "Cooldown    ");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "       +");
			lcd_string(&lcd, int2time((HAL_GetTick() - last_time)/1000, time_buf));
			lcd_write_data(&lcd, ' ');
			return;
		}

		static uint32_t check_time = 0;
		if (pos%2 == 0)
		{
			// we are going to temperature
			temperature_SP = steps[pos>>1].temp;
			time_buf[0] = '+';
			int2time((HAL_GetTick() - last_time)/1000, time_buf+1);
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "goto");
			if ((dt > -(4<<2)) && (dt < (4<<2)))
			{
				if (HAL_GetTick() - check_time > 5000) // we should be at least 5 sec in range
				{
					last_time = HAL_GetTick();
					pos++;
					temperature_SP = steps[pos>>1].temp;
				}
			}
			else
			{
				check_time = HAL_GetTick();
			}
		}
		else
		{
			// we are waiting for timeout
			temperature_SP = steps[pos>>1].temp;
			if (HAL_GetTick() - last_time >= steps[pos>>1].time*1000)
			{
				last_time = HAL_GetTick();
				check_time = HAL_GetTick();
				pos++;
				if (pos < (2*max_steps))
					temperature_SP = steps[pos>>1].temp;
				else
					temperature_SP = 0;
			}
			else
			{
				time_buf[0] = '-';
				// how long to wait? add some time to avoid negative time
				int2time(steps[pos>>1].time - (HAL_GetTick() - last_time + 1000)/1000, time_buf+1);
				lcd_set_xy(&lcd, 0, 1);
				lcd_string(&lcd, "hold");
			}
		}


		// write number of step
		lcd_set_xy(&lcd, 0, 0);
		lcd_write_data(&lcd, '#');
		lcd_write_data(&lcd, (pos>>1)+'1');
		lcd_write_data(&lcd, '/');
		lcd_write_data(&lcd, max_steps + '0');

		// show time
		lcd_set_xy(&lcd, 7, 1);
		lcd_string(&lcd, (char*)time_buf);

		// write temperature
		uint8_t buf[3];
		int2string(temperature_SP, buf, sizeof(buf));
		lcd_set_xy(&lcd, 7, 0);
		lcd_out(&lcd, buf, sizeof(buf));
		lcd_write_data(&lcd, scGRAD); // grad
		lcd_write_data(&lcd, scAR); // arrow right
		lcd_set_xy(&lcd, 9, 0);

		// *** let's allow some in-process variable tuning
		static uint8_t rf_ui_state = 0;
		static uint8_t last_pos = 0;
		if (last_pos != pos)
		{
			// position changed, we should reset menu
			rf_ui_state = 0;
		}
		last_pos = pos;
		static bool last_button = false;
		static uint16_t last_encoder = 0;
		static volatile int16_t diff = 0;

		diff-=(int16_t)(encoder_value - last_encoder);

		switch (rf_ui_state)
		{
		case 0: // reset all to default view
			diff = 0;
			rf_ui_state = 1;
			break;
		case 1: // wait for temperature edit
			lcd_set_xy(&lcd, 9, 0);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
			if (((encoder_value & 0b10) != (last_encoder & 0b10)) && (pos&0b1))
				rf_ui_state = 3;
			if ((last_button) && (!button.pressed))
				rf_ui_state = 2;
			break;
		case 2: // edit temperature
			if (diff)
				steps[pos>>1].temp = change_temperature(steps[pos>>1].temp, diff);
			lcd_set_xy(&lcd, 9, 0);
			lcd_mode(&lcd, ENABLE, CURSOR_DISABLE, BLINK);
			if ((last_button) && (!button.pressed))
				rf_ui_state = 1;
			break;
		case 3: // wait for time edit
			lcd_set_xy(&lcd, 11, 1);
			lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
			if ((encoder_value & 0b10) != (last_encoder & 0b10))
				rf_ui_state = 1;
			if ((last_button) && (!button.pressed))
				rf_ui_state = 4;
			break;
		case 4: // edit time
			if (diff)
			{
				uint32_t tmp = (HAL_GetTick() - last_time)/1000;
				steps[pos>>1].time = 1 + tmp + change_time(steps[pos>>1].time - tmp + 2, diff);
			}
			lcd_set_xy(&lcd, 11, 1);
			lcd_mode(&lcd, ENABLE, CURSOR_DISABLE, BLINK);
			if ((last_button) && (!button.pressed))
				rf_ui_state = 3;
			break;
		default:
			rf_ui_state = 0;
			break;
		}
		diff = 0;
		last_encoder = encoder_value;
		last_button = button.pressed;

	}
	// ****** END DO_REFLOW *************** //


	/***********************
	 *  END HELP FUNCTIONS *
	 ***********************/

	/**
	 * this happens every 100 ms
	 */
	static uint32_t last_time = 0;
	static bool last_button = false;
	if (HAL_GetTick() - last_time < 100)
		return;
	last_time = HAL_GetTick();

	/*** Right always visible section ***/

	int32_t dT=((int32_t)temperature_SP)-((int32_t)(MAX6675.temperature>>2));

	lcd_set_xy(&lcd, 12, 0);
	if (MAX6675.data_valid)
	{
		lcd_out(&lcd, MAX6675.ascii+1, 3);
		lcd_write_data(&lcd, scGRAD);
	}
	else
	{
		lcd_string(&lcd, "___");
		temperature_SP = 0;
		global_error = errMAX6675_invalid;
		ui_state = uiMALFUNCTION;
	}

	lcd_set_xy(&lcd, 13, 1);
	// first symbol
	if (button.pressed)
		lcd_write_data(&lcd, scDOT);
	else
		lcd_write_data(&lcd, ' ');
	// second symbol
	if (!MAX6675.data_valid)
		lcd_write_data(&lcd, ' ');
	else
	{
		if ((temperature_SP == 0) && (MAX6675.temperature < (HOT_TEMP<<2)))
			lcd_write_data(&lcd, '-');
		else if (((pwm_value + 9)/10)*3 > ticktack)
		{
			if ((STEP_TEMP > dT) && (dT > -STEP_TEMP))
				lcd_write_data(&lcd, ccUPEQ);
			else
				lcd_write_data(&lcd, ccUP);
		}
		else if (dT >= STEP_TEMP)
			lcd_write_data(&lcd, '^');
		else if ((STEP_TEMP > dT) && (dT > -STEP_TEMP))
		{
			lcd_write_data(&lcd, '=');
		}
		else if (dT <= -STEP_TEMP)
			lcd_write_data(&lcd, ccDOWN);
		else
			lcd_write_data(&lcd, '?'); // should never happen

	}
	// last symbol
	if ((MAX6675.temperature > (HOT_TEMP<<2)) || (!MAX6675.data_valid))
	{
		if (ticktack < 5)
			lcd_write_data(&lcd, ccHOT);
		else
			lcd_write_data(&lcd, ccHOTmirror);
	}
	else
	{
		if (ticktack < 5)
			lcd_write_data(&lcd, ccDOT);
		else
			lcd_write_data(&lcd, ' ');
	}
	if (++ticktack > 9)
		ticktack = 0;

	/* Check errors --------------------------------------------*/

	if (MAX6675.temperature > ((MAX_TEMP + 2*STEP_TEMP)<<2))
	{
		temperature_SP = 0;
		global_error |= errTEMP_PV;
	}

	if (temperature_SP > MAX_TEMP)
	{
		temperature_SP = 0;
		global_error |= errTEMP_SP;
	}

	//check if heater works
	static int32_t last_dT = 0;
	static uint32_t last_SP = 0xffff; // not realistic value, to be immediately replaced
	static uint32_t time_dT = 0;
	static bool check_heater = false;
	static uint32_t heater_timeout = 20;

	if (temperature_SP != last_SP) // user changed T
	{
		if (dT >= 20) // T diff is more than 20 grad
		{
			check_heater = true;
			last_dT = dT; // save value
			time_dT = HAL_GetTick(); // start timer
			heater_timeout = 120;
			if (dT >= 50)
				heater_timeout = 20; // if diff 50 grad, react faster
		}
		else
			check_heater = false;
	}
	else if (check_heater)
	{
		if (last_dT - dT > 5) // temperature changed more than 5 grad, ok!
			check_heater = false;
		else if (HAL_GetTick() - time_dT > heater_timeout*1024) // timeout, go in error
		{
			check_heater = false;
			global_error |= errHEATER;
		}
	}
	last_SP = temperature_SP;

	/************************************/

	uint8_t enc_val = (encoder_value>>1)&0b1;


	if (button.long_press)
	{
		ui_state = uiLONG_PRESS;
		temperature_SP = 0;
		global_error = errOK;
	}

	if (global_error)
		ui_state = uiMALFUNCTION;

	switch(ui_state)
	{
	case uiSTART:
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, "Long press  ");
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, "T=0 & reset ");
		lcd_write_data(&lcd, ccENTER);
		lcd_set_xy(&lcd, 12, 1);
		lcd_mode(&lcd, ENABLE, (ticktack < 5), NO_BLINK);
		if (!button.pressed && last_button)
		{
			lcd_mode(&lcd, ENABLE, CURSOR_DISABLE, NO_BLINK);
			ui_state = uiMENUenter;
		}
		break;
	case uiLONG_PRESS:
		lcd_mode(&lcd, ENABLE, CURSOR_DISABLE, NO_BLINK);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, "T set to 0  ");
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, "err. cleared ");
		if (!button.pressed)
			ui_state = uiMENUenter;
		break;
	case uiMENUenter:
		lcd_mini_clear(&lcd);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, " Profile");
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, " Heatplate");
		lcd_set_xy(&lcd, 0, (encoder_value>>1)&0b1);
		lcd_write_data(&lcd, scAR);
		ui_state = uiMENU;
		break;
	case uiMENU:
		lcd_set_xy(&lcd, 0, enc_val);
		lcd_write_data(&lcd, scAR);
		lcd_set_xy(&lcd, 0, 1 - enc_val);
		lcd_write_data(&lcd, ' ');
		if (!button.pressed && last_button)
		{
			ui_state = enc_val?uiHEATPLATEenter:uiSETTINGSenter;
		}
		break;
	case uiHEATPLATEenter:
		lcd_mini_clear(&lcd);
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, "Heatplate   ");
		heatplate(true);
		ui_state = uiHEATPLATE;
		break;
	case uiHEATPLATE:
		heatplate(false);
		break;
	case uiSETTINGSenter:
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, " Profile    ");
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, " settings    ");
		ui_state = uiSETTINGS;
		do_profile_settings(true);
		break;
	case uiSETTINGS:
		if (do_profile_settings(false))
		{
			lcd_mini_clear(&lcd);
			ui_state = uiREFLOW;
			do_reflow(true);
		}
		break;
	case uiREFLOW:
		do_reflow(false);
		break;
	case uiMALFUNCTION: // just check errors
		temperature_SP = 0;
		lcd_mode(&lcd, ENABLE, CURSOR_DISABLE, NO_BLINK);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, " * Error! * ");
		lcd_set_xy(&lcd, 0, 1);
		if (global_error & errMAX6675_invalid)
			lcd_string(&lcd, "Temp. Sensor");
		else if (global_error & errTEMP_SP)
			lcd_string(&lcd, "SP too high ");
		else if (global_error & errTEMP_PV)
			lcd_string(&lcd, "T too high  ");
		else if (global_error & errHEATER)
			lcd_string(&lcd, "heater power");
		else if (global_error & errI_LIMIT)
			lcd_string(&lcd, "heater limit");
		else if (global_error & errTIMEOUT)
			lcd_string(&lcd, "timeout     ");
		else
			lcd_string(&lcd, "fatal error ");

		break;
	default:
		global_error = errFATAL;
		ui_state = uiMALFUNCTION;
		break;
	}
	last_button = button.pressed;
}


/**
 * PID controller
 * @param PV - process variable, with 1/4 grad resolution
 * @param SP - set point, with 1/4 grad resolution
 */
uint8_t pid(uint16_t PV, uint16_t SP)
{
	// initialize coefficients and limits
	static const int32_t P=1*32768;
	static const int32_t I=0.00153*32768;
	static const int32_t D=10*32768;
	static const int32_t limit_top=25*4/0.00153; // max 25% of PWM

	int32_t deltaT(uint16_t PV)
	{
#define size 4
		static int32_t arr[size];
		static bool first_time = true;
		if (first_time)
		{
			first_time = false;
			for (int i = 0; i < size; i++)
				arr[i] = PV;
		}
		int32_t temp = arr[0];
		for (int i = 1; i < size; i++)
			arr[i-1] = arr[i];
		arr[size-1] = PV;
		return (temp - PV)/size;
	}

	static int32_t integral = 0;
	static int32_t last_PV = -1;
	if (last_PV < 0)
		last_PV = PV; // first time, init this thing to avoid jump

	int32_t error = SP-PV;
	int32_t p = error * P;
	if (error > 0)
	{
		integral += error;
		if (error < 4*4) // almost here, but we need some boost
			integral += error*8;
	}
	else
	{
		integral += error/4; // cool down is slower
		if (error > -4*4)
			integral += error*16; // almost here, we need boost!
	}
	if (integral > limit_top)
	{
		global_error |= errI_LIMIT;
		temperature_SP = 0;
		integral = limit_top;
	}
	if (integral < 0)
		integral = 0;
	int32_t i = integral * I;
	/* signal is noisy, but slow, I use additional filter for D */
	//int32_t d = (last_PV - PV)*D;
	int32_t d = deltaT(PV)*D;
	if (d > 0)
		d = 0;
	last_PV = PV;
	int32_t out = (p+i+d)/4/32768;
	if (out > 100)
		out = 100;
	if (out < 0)
		out = 0;

	PID.P = p/4/32768;
	PID.I = i/4/32768;
	PID.D = d/4/32768;

	return out;
}

/**
 * here happens two interrupts, at 998ms (for pid update)
 * and also at ~500ms (for second temperature update
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
	get_max6675();
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		if (!(MAX6675.data_valid))
			temperature_SP = 0;
		pwm_value = pid(MAX6675.temperature, temperature_SP<<2);
		uint16_t val = 10*pwm_value;
		TIM2->CCR1 = val;
		tick = true;
	}
	ascii_max6675();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, 1); // enable USB
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PWM for output
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // ENCODER
  TIM2->CCR2 = 998; // timer for PID interrupt + temperature update
  TIM2->CCR3 = 499; // timer for temperature update
  TIM2->DIER |= TIM_DIER_CC2IE|TIM_DIER_CC3IE; // interrupt enable
  HAL_TIM_Base_Start_IT(&htim2); // Enable Interrupts
  delay_init(&htim1); // inits the library for us delay
  init_lcd(); // init lcd and load special symbols

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  do_button(); // update button status
	  do_blink(); // led heartbeat
	  do_usb();  // output debug information
	  do_interface(); // here happens the magic
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = HAL_RCC_GetSysClockFreq()/1000000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = HAL_RCC_GetSysClockFreq()/1000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, hd_7_Pin|hd_6_Pin|hd_RS_Pin|hd_E_Pin
                          |hd_4_Pin|hd_5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, debug_a_Pin|debug_b_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : hd_7_Pin hd_6_Pin hd_RS_Pin hd_E_Pin
                           hd_4_Pin hd_5_Pin debug_a_Pin debug_b_Pin */
  GPIO_InitStruct.Pin = hd_7_Pin|hd_6_Pin|hd_RS_Pin|hd_E_Pin
                          |hd_4_Pin|hd_5_Pin|debug_a_Pin|debug_b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_EN_Pin */
  GPIO_InitStruct.Pin = USB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : enc_s_Pin */
  GPIO_InitStruct.Pin = enc_s_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(enc_s_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
