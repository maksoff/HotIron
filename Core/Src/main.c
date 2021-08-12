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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HOT_TEMP (40) // 40 grad
#define MAX_TEMP (275)
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
	OK = 0,
	TEMP_SP = 1<<0,
	TEMP_PV = 1<<1,
	HEATER = 1<<2,
	MAX6675_invalid = 1<<3,
	FATAL = 1 << 7
};

uint8_t error = OK;

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

const sSTEPS steps_arr[] = {
		{.temp = 150, .time=100},
		{.temp = 210, .time=30},
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

#define encoder_value (TIM3->CNT)
struct sBUTTON {
	bool pressed;
	bool long_press;
} button = {.pressed = false, .long_press = false};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

enum {
	  sENTER,
	  sUP,
	  sDOWN,
	  sUPDOWN,
	  sHOT,
	  sHOTmirror,
	  s3dots,
	  sDOT
};

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

void init_lcd(void)
{
	  lcd = lcd_create(ports, pins,
						hd_RS_GPIO_Port, hd_RS_Pin,
						hd_E_GPIO_Port, hd_E_Pin,
						LCD_4_BIT_MODE);

	  /* load symbols */

	  uint8_t symbols [] = {
	  	  	  				0x1, 0x1, 0x5, 0x9, 0x1f, 0x8, 0x4, 0x0,   // ENTER
							0x4, 0xe, 0x1f, 0x0, 0x0, 0x0, 0x0, 0x0,   // UP
							0x0, 0x0, 0x0, 0x0, 0x1f, 0xe, 0x4, 0x0,   // DOWN
						    0x4, 0xe, 0x1f, 0x0, 0x1f, 0xe, 0x4, 0x0,  // UP DOWN
							0x9, 0x12, 0x9, 0x12, 0x0, 0x1f, 0x1f, 0x0, // HOT
							0x12, 0x9, 0x12, 0x9, 0x0, 0x1f, 0x1f, 0x0, // HOT mirror
							0x0, 0x4, 0x0, 0x4, 0x0, 0x4, 0x0, 0x0,    // 3 dots
							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x0,		// 1 dot
//							0xa, 0x1f, 0x1f, 0x1f, 0xe, 0x4, 0x0, 0x0, // heart big
//			  	  	  	  	0x0, 0xe, 0x11, 0x15, 0x11, 0xe, 0x0, 0x0, // OFF
//			  	  	  	  	0x0, 0x4, 0x15, 0x15, 0x11, 0xe, 0x0, 0x0, // ON
//							0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x15, 0x0,   // ellips
//							0x0, 0x0, 0xa, 0x1f, 0xe, 0x4, 0x0, 0x0, // heart
	  };
	  lcd_define_chars(&lcd, symbols);
	  lcd_set_xy(&lcd, 0, 0);
	  lcd_string(&lcd, "Maksim Jeskevic");
	  lcd_set_xy(&lcd, 0, 1);
	  lcd_string(&lcd, "         2021 08");
	  HAL_Delay(1500);
	  lcd_clear(&lcd);
}

void do_button(void)
{
	const uint32_t time_for_long_press = 1000;
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
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	last_time = HAL_GetTick();
}

void get_max6675(void)
{
	uint16_t data;
	HAL_SPI_Receive(&hspi1, (uint8_t*)(&data), 1, 100);

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
			"Tick: %lu, PV: %u.%02u; SP: %u; PWM: %u; P: %li; I: %li; D: %li\r",
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
	START,
	MENU_0,
	MENU,
	HEATPLATE_0,
	HEATPLATE,
	SETTINGS_0,
	SETTINGS,
	REFLOW,
	MALFUNCTION,
} eUISTATE;
eUISTATE ui_state = START;

/**
 * Function for the main interface, also for error states and co.
 */
void do_interface(void)
{
	static sSTEPS steps [10]; // steps for reflow

	/**
	 * clears left part of the display
	 */
	void lcd_mini_clear(LCD_HandleTypeDef * lcd)
	{
		lcd_mode(lcd, ENABLE, CURSOR_DISABLE, NO_BLINK);
		lcd_set_xy(lcd, 0, 0);
		lcd_string(lcd, "            ");
		lcd_set_xy(lcd, 0, 1);
		lcd_string(lcd, "            ");
	}

	/**
	 * "heatplate mode" - just constant heating
	 */
	void heatplate(bool reset)
	{
		static uint16_t last_encoder = 0;
		static volatile int16_t diff = 0;
#define upper_limit (MAX_TEMP/STEP_TEMP*2)
		if (reset)
		{
			last_encoder = encoder_value;
			diff = 0;
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

		uint8_t buf[3];
		temperature_SP = STEP_TEMP*(diff>>1);
		int2string(temperature_SP, buf, sizeof(buf));
		lcd_set_xy(&lcd, 7, 0);
		lcd_out(&lcd, buf, sizeof(buf));
		lcd_write_data(&lcd, 223); // grad
		lcd_write_data(&lcd, 0x7E); // arrow right
		lcd_set_xy(&lcd, 9, 0);
		lcd_mode(&lcd, ENABLE, CURSOR_ENABLE, NO_BLINK);

	}

	/**
	 * do heat/wait in steps
	 */
	void do_reflow(bool reset)
	{
		static uint32_t last_time = 0;
		volatile static uint8_t pos = 0;
		if (reset)
			pos = 0;

		int32_t dt = ((int32_t)(temperature_SP<<2)) -
					 ((int32_t)MAX6675.temperature);

		if (pos >= (2*sizeof(steps_arr)/sizeof(steps_arr[0])))
		{
			temperature_SP = 0;
			lcd_set_xy(&lcd, 0, 0);
			lcd_string(&lcd, "Cooldown    ");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "+00:10      ");
			return;
		}
		if (pos%2 == 0)
		{
			// we are going to temperature
			temperature_SP = steps_arr[pos>>1].temp;
			if ((dt > -(4<<2)) && (dt < (4<<2)))
			{
				last_time = HAL_GetTick();
				pos++;
				temperature_SP = steps_arr[pos>>1].temp;
			}
			lcd_set_xy(&lcd, 0, 0);
			lcd_string(&lcd, "#");
			lcd_write_data(&lcd, (pos>>1)+'1');
			lcd_string(&lcd, ": up      ");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "+00:00      ");
		}
		else
		{
			// we are waiting for timeout
			temperature_SP = steps_arr[pos>>1].temp;
			if (HAL_GetTick() - last_time > steps_arr[pos>>1].time*1000)
			{
				pos++;
				if (pos < (2*sizeof(steps_arr)/sizeof(steps_arr[0])))
					temperature_SP = steps_arr[pos>>1].temp;
				else
					temperature_SP = 0;
			}
			lcd_set_xy(&lcd, 0, 0);
			lcd_string(&lcd, "#");
			lcd_write_data(&lcd, (pos>>1)+'1');
			lcd_string(&lcd, ": hold    ");
			lcd_set_xy(&lcd, 0, 1);
			lcd_string(&lcd, "-00:00      ");
		}
	}

	/**
	 * this happens every 100 ms
	 */
	static uint32_t last_time = 0;
	static uint8_t ticktack = 0; // using for blink "hot"
	static bool last_button = false;
	if (HAL_GetTick() - last_time < 100)
		return;
	last_time = HAL_GetTick();

	/*** Right always visible section ***/

	lcd_set_xy(&lcd, 12, 0);
	if (MAX6675.data_valid)
	{
		lcd_out(&lcd, MAX6675.ascii+1, 3);
		lcd_write_data(&lcd, 223);
	}
	else
	{
		lcd_string(&lcd, "___");
		temperature_SP = 0;
		error = MAX6675_invalid;
		ui_state = MALFUNCTION;
	}

	lcd_set_xy(&lcd, 12, 1);
	// first symbol
	if (button.long_press)
		lcd_write_data(&lcd, 0xDB);
	else if (button.pressed)
		lcd_write_data(&lcd, 0xA5);
	else
		lcd_write_data(&lcd, ' ');
	// second symbol
	lcd_write_data(&lcd, ' ');
	// third symbol
	lcd_write_data(&lcd, ' ');
	if ((MAX6675.temperature > (HOT_TEMP<<2)) || (!MAX6675.data_valid))
	{
		if (ticktack < 5)
			lcd_write_data(&lcd, sHOT);
		else
			lcd_write_data(&lcd, sHOTmirror);
	}
	else
	{
		lcd_write_data(&lcd, ' ');
	}
	if (++ticktack > 9)
		ticktack = 0;

	/* Check errors --------------------------------------------*/

	if (MAX6675.temperature > ((MAX_TEMP + STEP_TEMP)<<2))
	{
		temperature_SP = 0;
		error = TEMP_PV;
		ui_state = MALFUNCTION;
	}

	if (temperature_SP > MAX_TEMP)
	{
		temperature_SP = 0;
		error = TEMP_SP;
		ui_state = MALFUNCTION;
	}


	/************************************/

	uint8_t enc_val = (encoder_value>>1)&0b1;


	if (button.long_press)
	{
		ui_state = START;
		temperature_SP = 0;
	}

	switch(ui_state)
	{
	case START:
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, "Long press  ");
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, "to stop     ");
		lcd_set_xy(&lcd, 11, 1);
		lcd_write_data(&lcd, sENTER);
		lcd_set_xy(&lcd, 11, 1);
		lcd_mode(&lcd, ENABLE, CURSOR_ENABLE, NO_BLINK);
		if (!button.pressed && last_button)
			ui_state = MENU_0;
		last_button = button.pressed;
		break;
	case MENU_0:
		lcd_mini_clear(&lcd);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, " Profile");
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, " Heatplate");
		lcd_set_xy(&lcd, 0, (encoder_value>>1)&0b1);
		lcd_write_data(&lcd, 0x7E);
		ui_state = MENU;
		break;
	case MENU:
		lcd_set_xy(&lcd, 0, enc_val);
		lcd_write_data(&lcd, 0x7E);
		lcd_set_xy(&lcd, 0, 1 - enc_val);
		lcd_write_data(&lcd, ' ');
		if (!button.pressed && last_button)
		{
			ui_state = enc_val?HEATPLATE_0:SETTINGS_0;
		}
		last_button = button.pressed;
		break;
	case HEATPLATE_0:
		lcd_mini_clear(&lcd);
		lcd_set_xy(&lcd, 0, 1);
		lcd_string(&lcd, "Heatplate");
		heatplate(true);
		ui_state = HEATPLATE;
		break;
	case HEATPLATE:
		heatplate(false);
		break;
	case SETTINGS_0:
		lcd_mini_clear(&lcd);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, "SETTINGS");
		ui_state = SETTINGS;
		break;
	case SETTINGS:
		lcd_mini_clear(&lcd);
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, "REFLOW");
		ui_state = REFLOW;
		do_reflow(true);
		break;
	case REFLOW:
		do_reflow(false);
		break;
	case MALFUNCTION:
		temperature_SP = 0;
		lcd_set_xy(&lcd, 0, 0);
		lcd_string(&lcd, " * Error! * ");
		lcd_set_xy(&lcd, 0, 1);
		if (error & MAX6675_invalid)
			lcd_string(&lcd, "Temp. Sensor");
		else if (error & TEMP_SP)
			lcd_string(&lcd, "SP too high ");
		else if (error & TEMP_PV)
			lcd_string(&lcd, "T too high  ");
		else if (error & HEATER)
			lcd_string(&lcd, "heater power");
		else
			lcd_string(&lcd, "fatal error ");

		break;
	default:
		error = FATAL;
		ui_state = MALFUNCTION;
		break;
	}
}


/**
 * PID controller
 * @param PV - process variable, with 1/4 grad resolution
 * @param SP - set point, with 1/4 grad resolution
 */
uint8_t pid(uint16_t PV, uint16_t SP)
{
	static const int32_t P=1*32768;
	static const int32_t I=0.00153*32768;
	static const int32_t D=10*32768;
	static const int32_t limit_top=100*4*32768;

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
		integral = limit_top;
	if (integral < 0)
		integral = 0;
	int32_t i = integral * I;
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
  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  TIM2->CCR2 = 998; // timer for PID interrupt + temperature update
  TIM2->CCR3 = 499; // timer for temperature update
  TIM2->DIER |= TIM_DIER_CC2IE|TIM_DIER_CC3IE; // interrupt enable
  HAL_TIM_Base_Start_IT(&htim2);
  delay_init(&htim1); // inits the library for us delay
  init_lcd();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


#if 0
  /*********** TIME measurement macros ****************/
  volatile uint32_t last_ttime, ttemp, max_time = 0;

#define USE_DWT

#ifdef USE_DWT
#define STARTT do {	last_ttime = DWT->CYCCNT; } while (0);

#define STOPP do {\
	ttemp = DWT->CYCCNT - last_ttime;\
	if (ttemp > max_time)\
		max_time = ttemp;\
	} while (0);

#else

#define STARTT do {	last_ttime = HAL_GetTick(); } while (0);

#define STOPP do {\
	ttemp = HAL_GetTick() - last_ttime;\
	if (ttemp > max_time)\
		max_time = ttemp;\
	} while (0);
#endif

  /*********** TIME measurement macros END*************/
#endif


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
