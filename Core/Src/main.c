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

/* USER CODE BEGIN PV */

LCD_HandleTypeDef lcd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void process_encoder();
uint16_t encoder_value = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, 1);
  delay_init(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_time = HAL_GetTick();
  uint16_t data;

  LCD_PortType ports[] = {	hd_4_GPIO_Port,
  	  	  					hd_5_GPIO_Port,
							hd_6_GPIO_Port,
							hd_7_GPIO_Port};
  LCD_PinType pins[] = {	hd_4_Pin,
	  	    				hd_5_Pin,
							hd_6_Pin,
							hd_7_Pin};
  lcd = lcd_create(ports, pins,
					hd_RS_GPIO_Port, hd_RS_Pin,
					hd_E_GPIO_Port, hd_E_Pin,
					LCD_4_BIT_MODE);

  /* load symbols */

  uint8_t symbols [] = {0x0, 0xe, 0x11, 0x15, 0x11, 0xe, 0x0, 0x0, // OFF
		  	  	  	  	0x0, 0x4, 0x15, 0x15, 0x11, 0xe, 0x0, 0x0, // ON
						0x4, 0xe, 0x1f, 0x0, 0x0, 0x0, 0x0, 0x0,   // UP
						0x0, 0x0, 0x0, 0x0, 0x1f, 0xe, 0x4, 0x0,   // DOWN
					    0x4, 0xe, 0x1f, 0x0, 0x1f, 0xe, 0x4, 0x0,  // UP DOWN
						0x9, 0x12, 0x9, 0x12, 0x0, 0x1f, 0x1f, 0x0, // HOT
						0x12, 0x9, 0x12, 0x9, 0x0, 0x1f, 0x1f, 0x0, // HOT 2
						0x0, 0x0, 0xa, 0x1f, 0xe, 0x4, 0x0, 0x0 // heart
  };
  lcd_define_chars(&lcd, symbols);
  lcd_set_xy(&lcd, 0, 0);
  lcd_string(&lcd, "Just testing");
  while (HAL_GetTick() - last_time < 1000);

  volatile uint32_t last_ttime, ttemp, max_time = 0;

#define STARTT do {	last_ttime = HAL_GetTick(); } while (0);

#define STOPP do {\
	ttemp = HAL_GetTick() - last_ttime;\
	if (ttemp > max_time)\
		max_time = ttemp;\
	} while (0);

  while (1)
  {
	  if (HAL_GetTick() - last_time > 2)
	  {
		  process_encoder();
	  }
	  if (HAL_GetTick() - last_time > 500)
	  {
		  STARTT;
		  last_time = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  HAL_SPI_Receive(&hspi1, (uint8_t*)(&data), 1, 100);

#define SIGNIFICANT 4
		  uint8_t buf[SIGNIFICANT + 3+2];
		  buf[SIGNIFICANT + 3] = '\0';
		  if (data & 0b110)
		  {
			  // MAX 6675 not okay (wrong ID or TH not connected
			  for (int i = 0; i < sizeof(buf); i++)
				  buf[i] = 'x';
		  }
		  else
		  {
			  //MAX 6675 okay, prepare data
			  data >>= 3;
			  uint32_t digit = 25*(data&0b11);
			  digit += (data>>2)*1000;
			  int8_t i = SIGNIFICANT + 2;
			  while (digit)
			  {
				  buf[i--] = '0' + digit%10;
				  digit /= 10;
			  }
			  while (i > -1)
			  {
				  buf[i--] = '0';
			  }
			  buf[SIGNIFICANT] = '.';
		  }
		  lcd_set_xy(&lcd, 0, 1);
		  lcd_string(&lcd, buf);
		  lcd_write_data(&lcd, 223);
		  buf[SIGNIFICANT + 3] = '\r';
		  buf[SIGNIFICANT + 3+1] = '\n';
		  CDC_Transmit_FS(buf, sizeof(buf));
		  for (int i = 0; i < sizeof(buf); i++)
		  {
			  buf[i] = 0;
		  }

		  uint16_t temp = (encoder_value>>1)&0xff;
		  for (int i = 0; i < 5; i++)
		  {
			  buf[4-i] = temp % 10 + '0';
			  temp /= 10;
		  }
		  buf[5] = (encoder_value>>1)&0xff;
		  lcd_string(&lcd, buf);
		  lcd_set_xy(&lcd, 15, 0);
		  lcd_write_data(&lcd, (encoder_value>>1)&0xff);
		  lcd_mode(&lcd, LCD_ENABLE, CURSOR_ENABLE, NO_BLINK);
		  lcd_set_xy(&lcd, 15, 1);
		  STOPP;
	  }

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
void process_encoder(void)
{
	static uint8_t old;
	uint8_t new;
	new = (HAL_GPIO_ReadPin(enc_a_GPIO_Port, enc_a_Pin)?0b10:0 +
		   HAL_GPIO_ReadPin(enc_b_GPIO_Port, enc_b_Pin)?0b01:0);
	switch(old)
		{
		case 2:
			{
			if(new == 3) encoder_value++;
			if(new == 0) encoder_value--;
			break;
			}

		case 0:
			{
			if(new == 2) encoder_value++;
			if(new == 1) encoder_value--;
			break;
			}
		case 1:
			{
			if(new == 0) encoder_value++;
			if(new == 3) encoder_value--;
			break;
			}
		case 3:
			{
			if(new == 1) encoder_value++;
			if(new == 2) encoder_value--;
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
