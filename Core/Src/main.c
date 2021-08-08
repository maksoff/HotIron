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
//#include "hd44780_driver.h"
#include "delay.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

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


struct sMAX6675 {
	uint16_t temperature; // precise to 0.25 grad
	bool data_valid;
	uint8_t ascii[7];  // converted to ASCII
} MAX6675 = {.data_valid = false, .temperature = 0};

uint16_t pwm_value = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void process_encoder();
uint16_t encoder_value = 0;
struct sBUTTON {
	bool pressed;
	bool long_press;
} button = {.pressed = false, .long_press = false};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
							0xa, 0x1f, 0x1f, 0x1f, 0xe, 0x4, 0x0, 0x0, // heart big
	//		  	  	  	  	0x0, 0xe, 0x11, 0x15, 0x11, 0xe, 0x0, 0x0, // OFF
	//		  	  	  	  	0x0, 0x4, 0x15, 0x15, 0x11, 0xe, 0x0, 0x0, // ON
	//						0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x15, 0x0,   // ellips
	//						0x0, 0x0, 0xa, 0x1f, 0xe, 0x4, 0x0, 0x0, // heart
	  };
	  lcd_define_chars(&lcd, symbols);
	  lcd_set_xy(&lcd, 0, 0);
	  lcd_string(&lcd, "made with \7 by");
	  lcd_set_xy(&lcd, 0, 1);
	  lcd_string(&lcd, "Maksim Jeskevic");
	  HAL_Delay(3000);
	  lcd_clear(&lcd);

	  /* print all chars
	  for (uint8_t i = 0; i < 8; i++)
	  {
		  lcd_set_xy(&lcd, 0, 0);
		  for (uint8_t j = i*32; j < i*32+16; j++)
			  lcd_write_data(&lcd, j);
		  lcd_set_xy(&lcd, 0, 1);
		  for (uint16_t j = i*32+16; j < i*32+32; j++)
			  lcd_write_data(&lcd, (uint8_t)j);
		  HAL_Delay(2000);
	  }
	  */
}

void do_encoder(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 1)
		return;
	process_encoder();
	last_time = HAL_GetTick();
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

void do_max6675(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 500)
		return;
	last_time = HAL_GetTick();

	uint16_t data;
	HAL_SPI_Receive(&hspi1, (uint8_t*)(&data), 1, 100);
	MAX6675.data_valid = !(data & 0b110);
	MAX6675.temperature = data >> 3;

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

void do_pwm(void)
{
	static uint16_t last_encoder = 0;
	static volatile int16_t diff = 0;

	// Check button
	static uint8_t state = 0;
	switch (state) {
	case 0:
		if (pwm_value == 0)
		{
			if (button.long_press)
			{
				diff = 100 << 1; // full power
				state = 1;
			}
		}
		else if (button.pressed)
		{
			diff = 0; // zero power
			state = 1;
		}
		break;
	case 1: // wait button release
		if (!button.pressed)
			state = 0;
		break;
	default:
		break;
	}

	if (MAX6675.temperature > (300<<2)) // hardcoded protect
		diff = 0;
	else
	{
		diff+=(int16_t)(encoder_value - last_encoder);
		if (diff < 0)
			diff = 0;
		if (diff > (100<<1))
			diff = 100<<1;
	}
	last_encoder = encoder_value;
	pwm_value = diff>>1; // in %
	TIM2->CCR1 = pwm_value*10;
}

void do_usb(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 250)
		return;
	last_time = HAL_GetTick();

	uint8_t buf[13];
	int i = 0;
	for (i = 0; i < sizeof(MAX6675.ascii); i++)
		buf[i] = MAX6675.ascii[i];
	buf[i++] = ' ';

	uint16_t temp = pwm_value;
	for (int j = 0; j < 3; j++)
	{
		if ((!temp) && j)
		{
			buf[i+2-j] =' ';
		}
		else
		{
			buf[i+2-j] = temp % 10 + '0';
			temp /= 10;
		}
	}

	buf[11] = '\r';
	buf[12] = '\n';

	CDC_Transmit_FS(buf, sizeof(buf));
}

void do_lcd(void)
{
	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time < 100)
		return;
	last_time = HAL_GetTick();

	// output temperature
	lcd_set_xy(&lcd, 0, 1);
	lcd_out(&lcd, MAX6675.ascii, sizeof(MAX6675.ascii));
	lcd_write_data(&lcd, 223);

	// output temperature
	uint8_t buf[4];
	uint16_t temp = pwm_value;
	for (int i = 0; i < 3; i++)
	{
		if ((!temp) && i)
		{
			buf[2-i] =' ';
		}
		else
		{
			buf[2-i] = temp % 10 + '0';
			temp /= 10;
		}
	}
	buf[3] = '%';
	lcd_set_xy(&lcd, 0, 0);
	lcd_out(&lcd, buf, 4);
	lcd_mode(&lcd, LCD_ENABLE, CURSOR_ENABLE, NO_BLINK);
	lcd_set_xy(&lcd, 2, 0);
}


/**
 * PID controller
 * @param PV - process variable, with 1/4 grad resolution
 * @param SP - set point, with 1/4 grad resolution
 */
uint8_t pid(uint16_t PV, uint16_t SP)
{
	const uint32_t P=1*32768;
	const uint32_t I=0.0015*32768;
	const uint32_t D=10*32768;
	const uint32_t limit_top=100*4*32768;

	static int32_t integral = 0;
	static int32_t last_PV = -1;
	if (last_PV < PV)
		last_PV = PV; // first time, init this thing to avoid jump

	int32_t error = SP-PV;
	int32_t p = error * P;
	integral += error;
	if (integral > limit_top)
		integral = limit_top;
	if (integral < 0)
		integral = 0;
	int32_t i = integral * I;
	int32_t d = (last_PV - PV)*D;
	last_PV = PV;
	int32_t out = (p+i+d)/4/32768;
	if (out > 100)
		out = 100;
	if (out < 0)
		out = 0;
	return out;
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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  delay_init(&htim1);
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
	  do_encoder();
	  do_button();
	  do_blink();
	  do_max6675();
	  do_pwm();
	  do_usb();
	  do_lcd();
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : hd_7_Pin hd_6_Pin hd_RS_Pin hd_E_Pin
                           hd_4_Pin hd_5_Pin */
  GPIO_InitStruct.Pin = hd_7_Pin|hd_6_Pin|hd_RS_Pin|hd_E_Pin
                          |hd_4_Pin|hd_5_Pin;
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

  /*Configure GPIO pins : enc_s_Pin enc_a_Pin enc_b_Pin */
  GPIO_InitStruct.Pin = enc_s_Pin|enc_a_Pin|enc_b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void process_encoder(void) {
	static uint8_t old;
	uint8_t new;
	new = (HAL_GPIO_ReadPin(enc_a_GPIO_Port, enc_a_Pin) ? 0b10 : 0
		 + HAL_GPIO_ReadPin(enc_b_GPIO_Port, enc_b_Pin) ? 0b01 : 0);
	switch (old) {
		case 2: {
			if (new == 3)
				encoder_value++;
			if (new == 0)
				encoder_value--;
			break;
		}

		case 0: {
			if (new == 2)
				encoder_value++;
			if (new == 1)
				encoder_value--;
			break;
		}
		case 1: {
			if (new == 0)
				encoder_value++;
			if (new == 3)
				encoder_value--;
			break;
		}
		case 3: {
			if (new == 1)
				encoder_value++;
			if (new == 2)
				encoder_value--;
			break;
		}
	}
	old = new;
}

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
