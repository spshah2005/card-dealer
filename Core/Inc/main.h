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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DISPLAY_MISO_Pin GPIO_PIN_2
#define DISPLAY_MISO_GPIO_Port GPIOC
#define LOADCELL_SCK_Pin GPIO_PIN_0
#define LOADCELL_SCK_GPIO_Port GPIOA
#define LOADCELL_DATA_Pin GPIO_PIN_1
#define LOADCELL_DATA_GPIO_Port GPIOA
#define SERVO_PWM_Pin GPIO_PIN_2
#define SERVO_PWM_GPIO_Port GPIOA
#define SPEAKER_DAC_Pin GPIO_PIN_5
#define SPEAKER_DAC_GPIO_Port GPIOA
#define PS2_MISO_Pin GPIO_PIN_14
#define PS2_MISO_GPIO_Port GPIOF
#define PS2_SS_Pin GPIO_PIN_15
#define PS2_SS_GPIO_Port GPIOF
#define PS2_SCK_Pin GPIO_PIN_11
#define PS2_SCK_GPIO_Port GPIOE
#define PS2_MOSI_Pin GPIO_PIN_13
#define PS2_MOSI_GPIO_Port GPIOE
#define DISPLAY_SCK_Pin GPIO_PIN_10
#define DISPLAY_SCK_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_7
#define RST_GPIO_Port GPIOC
#define SD_DET_Pin GPIO_PIN_9
#define SD_DET_GPIO_Port GPIOC
#define D_C_Pin GPIO_PIN_9
#define D_C_GPIO_Port GPIOA
#define DISPLAY__Pin GPIO_PIN_0
#define DISPLAY__GPIO_Port GPIOD
#define DISPLAY_MOSI_Pin GPIO_PIN_4
#define DISPLAY_MOSI_GPIO_Port GPIOD
#define RFID_SCK_Pin GPIO_PIN_9
#define RFID_SCK_GPIO_Port GPIOG
#define RFID_MISO_Pin GPIO_PIN_10
#define RFID_MISO_GPIO_Port GPIOG
#define RFID_MOSI_Pin GPIO_PIN_11
#define RFID_MOSI_GPIO_Port GPIOG
#define RFID_NSS_Pin GPIO_PIN_12
#define RFID_NSS_GPIO_Port GPIOG
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
