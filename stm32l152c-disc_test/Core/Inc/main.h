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
#include "stm32l1xx_hal.h"

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
#define IDD_CNT_EN_Pin GPIO_PIN_13
#define IDD_CNT_EN_GPIO_Port GPIOC
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define SEG14_Pin GPIO_PIN_0
#define SEG14_GPIO_Port GPIOC
#define SEG15_Pin GPIO_PIN_1
#define SEG15_GPIO_Port GPIOC
#define SEG16_Pin GPIO_PIN_2
#define SEG16_GPIO_Port GPIOC
#define SEG17_Pin GPIO_PIN_3
#define SEG17_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define SEG0_Pin GPIO_PIN_1
#define SEG0_GPIO_Port GPIOA
#define SEG1_Pin GPIO_PIN_2
#define SEG1_GPIO_Port GPIOA
#define SEG2_Pin GPIO_PIN_3
#define SEG2_GPIO_Port GPIOA
#define IDD_Measurement_Pin GPIO_PIN_4
#define IDD_Measurement_GPIO_Port GPIOA
#define GRP2_Sampling_Pin GPIO_PIN_6
#define GRP2_Sampling_GPIO_Port GPIOA
#define GRP2_Ground_Pin GPIO_PIN_7
#define GRP2_Ground_GPIO_Port GPIOA
#define GRP9_Sampling_Pin GPIO_PIN_4
#define GRP9_Sampling_GPIO_Port GPIOC
#define GRP9_Ground_Pin GPIO_PIN_5
#define GRP9_Ground_GPIO_Port GPIOC
#define GRP3_Sampling_Pin GPIO_PIN_0
#define GRP3_Sampling_GPIO_Port GPIOB
#define GRP3_Ground_Pin GPIO_PIN_1
#define GRP3_Ground_GPIO_Port GPIOB
#define SEG6_Pin GPIO_PIN_10
#define SEG6_GPIO_Port GPIOB
#define SEG7_Pin GPIO_PIN_11
#define SEG7_GPIO_Port GPIOB
#define SEG8_Pin GPIO_PIN_12
#define SEG8_GPIO_Port GPIOB
#define SEG9_Pin GPIO_PIN_13
#define SEG9_GPIO_Port GPIOB
#define SEG10_Pin GPIO_PIN_14
#define SEG10_GPIO_Port GPIOB
#define SEG11_Pin GPIO_PIN_15
#define SEG11_GPIO_Port GPIOB
#define SEG18_Pin GPIO_PIN_6
#define SEG18_GPIO_Port GPIOC
#define SEG19_Pin GPIO_PIN_7
#define SEG19_GPIO_Port GPIOC
#define SEG20_Pin GPIO_PIN_8
#define SEG20_GPIO_Port GPIOC
#define SEG21_Pin GPIO_PIN_9
#define SEG21_GPIO_Port GPIOC
#define COM0_Pin GPIO_PIN_8
#define COM0_GPIO_Port GPIOA
#define COM1_Pin GPIO_PIN_9
#define COM1_GPIO_Port GPIOA
#define COM2_Pin GPIO_PIN_10
#define COM2_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SEG12_Pin GPIO_PIN_15
#define SEG12_GPIO_Port GPIOA
#define SEG22_Pin GPIO_PIN_10
#define SEG22_GPIO_Port GPIOC
#define SEG23_Pin GPIO_PIN_11
#define SEG23_GPIO_Port GPIOC
#define SEG4_Pin GPIO_PIN_4
#define SEG4_GPIO_Port GPIOB
#define SEG5_Pin GPIO_PIN_5
#define SEG5_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_6
#define LD4_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_7
#define LD3_GPIO_Port GPIOB
#define SEG13_Pin GPIO_PIN_8
#define SEG13_GPIO_Port GPIOB
#define COM3_Pin GPIO_PIN_9
#define COM3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
