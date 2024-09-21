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
#include "stm32f0xx_hal.h"

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
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CUSTOM_FREQ_Pin GPIO_PIN_0
#define CUSTOM_FREQ_GPIO_Port GPIOA
#define _500_HZ_Pin GPIO_PIN_1
#define _500_HZ_GPIO_Port GPIOA
#define _1000_HZ_Pin GPIO_PIN_2
#define _1000_HZ_GPIO_Port GPIOA
#define _1500_HZ_Pin GPIO_PIN_3
#define _1500_HZ_GPIO_Port GPIOA
#define _2000_HZ_Pin GPIO_PIN_4
#define _2000_HZ_GPIO_Port GPIOA
#define _2500_HZ_Pin GPIO_PIN_5
#define _2500_HZ_GPIO_Port GPIOA
#define _5000_HZ_Pin GPIO_PIN_6
#define _5000_HZ_GPIO_Port GPIOA
#define DAC_Pin GPIO_PIN_11
#define DAC_GPIO_Port GPIOA
#define INP_SIG_LED_Pin GPIO_PIN_7
#define INP_SIG_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
void freq_map() ;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
