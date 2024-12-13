/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USB_CDC_RxHandler(uint8_t*, uint32_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_4_Pin GPIO_PIN_13
#define LED_4_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_1
#define LED_2_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_3
#define LED_1_GPIO_Port GPIOA
#define INT_X_Pin GPIO_PIN_12
#define INT_X_GPIO_Port GPIOB
#define INT_X_EXTI_IRQn EXTI15_10_IRQn
#define INT_Y_Pin GPIO_PIN_13
#define INT_Y_GPIO_Port GPIOB
#define INT_Y_EXTI_IRQn EXTI15_10_IRQn
#define INT_Z_Pin GPIO_PIN_14
#define INT_Z_GPIO_Port GPIOB
#define INT_Z_EXTI_IRQn EXTI15_10_IRQn
#define BTN_1_Pin GPIO_PIN_15
#define BTN_1_GPIO_Port GPIOB
#define BTN_2_Pin GPIO_PIN_8
#define BTN_2_GPIO_Port GPIOA
#define BTN_3_Pin GPIO_PIN_9
#define BTN_3_GPIO_Port GPIOA
#define BTN_4_Pin GPIO_PIN_10
#define BTN_4_GPIO_Port GPIOA
#define BTN_5_Pin GPIO_PIN_6
#define BTN_5_GPIO_Port GPIOB
#define BTN_6_Pin GPIO_PIN_7
#define BTN_6_GPIO_Port GPIOB
#define BTN_7_Pin GPIO_PIN_8
#define BTN_7_GPIO_Port GPIOB
#define BTN_8_Pin GPIO_PIN_9
#define BTN_8_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
