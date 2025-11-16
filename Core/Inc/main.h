/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_nucleo.h"
#include <stdio.h>

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern I2C_HandleTypeDef hi2c2;

extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

extern volatile uint8_t tick;

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
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Eana3_Pin GPIO_PIN_0
#define Eana3_GPIO_Port GPIOC
#define Eana4_Pin GPIO_PIN_1
#define Eana4_GPIO_Port GPIOC
#define Eana1_Pin GPIO_PIN_0
#define Eana1_GPIO_Port GPIOA
#define Servo1_PWM_Pin GPIO_PIN_1
#define Servo1_PWM_GPIO_Port GPIOA
#define Mot2_PWM_Pin GPIO_PIN_4
#define Mot2_PWM_GPIO_Port GPIOA
#define Mot1_PWM_Pin GPIO_PIN_6
#define Mot1_PWM_GPIO_Port GPIOA
#define Mot3_PWM_Pin GPIO_PIN_0
#define Mot3_PWM_GPIO_Port GPIOB
#define Servo3_PWM_Pin GPIO_PIN_10
#define Servo3_PWM_GPIO_Port GPIOB
#define Servo4_PWM_Pin GPIO_PIN_11
#define Servo4_PWM_GPIO_Port GPIOB
#define Eana2_Pin GPIO_PIN_14
#define Eana2_GPIO_Port GPIOB
#define Codeur2_A_Pin GPIO_PIN_6
#define Codeur2_A_GPIO_Port GPIOC
#define Codeur2_B_Pin GPIO_PIN_7
#define Codeur2_B_GPIO_Port GPIOC
#define Codeur1_A_Pin GPIO_PIN_11
#define Codeur1_A_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Servo2_PWM_Pin GPIO_PIN_15
#define Servo2_PWM_GPIO_Port GPIOA
#define LED_RGB_Pin GPIO_PIN_12
#define LED_RGB_GPIO_Port GPIOC
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define Codeur1_B_Pin GPIO_PIN_7
#define Codeur1_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
