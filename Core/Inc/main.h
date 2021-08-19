/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define HOT_LEDS_Pin GPIO_PIN_14
#define HOT_LEDS_GPIO_Port GPIOC
#define SSR_control_Pin GPIO_PIN_0
#define SSR_control_GPIO_Port GPIOA
#define hd_7_Pin GPIO_PIN_10
#define hd_7_GPIO_Port GPIOB
#define hd_6_Pin GPIO_PIN_11
#define hd_6_GPIO_Port GPIOB
#define hd_RS_Pin GPIO_PIN_12
#define hd_RS_GPIO_Port GPIOB
#define hd_E_Pin GPIO_PIN_13
#define hd_E_GPIO_Port GPIOB
#define hd_4_Pin GPIO_PIN_14
#define hd_4_GPIO_Port GPIOB
#define hd_5_Pin GPIO_PIN_15
#define hd_5_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_15
#define USB_EN_GPIO_Port GPIOA
#define enc_s_Pin GPIO_PIN_3
#define enc_s_GPIO_Port GPIOB
#define enc_a_Pin GPIO_PIN_4
#define enc_a_GPIO_Port GPIOB
#define enc_b_Pin GPIO_PIN_5
#define enc_b_GPIO_Port GPIOB
#define Loudspeaker_Pin GPIO_PIN_7
#define Loudspeaker_GPIO_Port GPIOB
#define debug_a_Pin GPIO_PIN_8
#define debug_a_GPIO_Port GPIOB
#define debug_b_Pin GPIO_PIN_9
#define debug_b_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
