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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CLK_TriState_Pin GPIO_PIN_2
#define CLK_TriState_GPIO_Port GPIOA
#define CS_N_Pin GPIO_PIN_5
#define CS_N_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_6
#define SCLK_GPIO_Port GPIOA
#define Din_Pin GPIO_PIN_7
#define Din_GPIO_Port GPIOA
#define SDA_dp_Pin GPIO_PIN_10
#define SDA_dp_GPIO_Port GPIOB
#define Interrupt_Pin GPIO_PIN_14
#define Interrupt_GPIO_Port GPIOB
#define Start_Pin GPIO_PIN_7
#define Start_GPIO_Port GPIOC
#define Trig_LOOK_AT_Pin GPIO_PIN_9
#define Trig_LOOK_AT_GPIO_Port GPIOA
#define Enable_Pin GPIO_PIN_10
#define Enable_GPIO_Port GPIOA
#define SCL_dp_Pin GPIO_PIN_5
#define SCL_dp_GPIO_Port GPIOB
#define Dout_Pin GPIO_PIN_6
#define Dout_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
