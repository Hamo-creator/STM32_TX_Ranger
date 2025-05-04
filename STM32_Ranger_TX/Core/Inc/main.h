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
#include "stm32f4xx_hal.h"

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
#define UART_LED_Pin GPIO_PIN_13
#define UART_LED_GPIO_Port GPIOC
#define BatVolWarning_Pin GPIO_PIN_14
#define BatVolWarning_GPIO_Port GPIOC
#define BatVolDanger_Pin GPIO_PIN_15
#define BatVolDanger_GPIO_Port GPIOC
#define SWITCH_ARM_Pin GPIO_PIN_0
#define SWITCH_ARM_GPIO_Port GPIOC
#define SWITCH_AUX2_Pin GPIO_PIN_1
#define SWITCH_AUX2_GPIO_Port GPIOC
#define SWITCH_AUX3_Pin GPIO_PIN_2
#define SWITCH_AUX3_GPIO_Port GPIOC
#define SWITCH_AUX4_Pin GPIO_PIN_3
#define SWITCH_AUX4_GPIO_Port GPIOC
#define PinRudder_Pin GPIO_PIN_0
#define PinRudder_GPIO_Port GPIOA
#define PinThrottle_Pin GPIO_PIN_1
#define PinThrottle_GPIO_Port GPIOA
#define PinAileron_Pin GPIO_PIN_2
#define PinAileron_GPIO_Port GPIOA
#define PinElevator_Pin GPIO_PIN_3
#define PinElevator_GPIO_Port GPIOA
#define VOLTAGE_READ_PIN_Pin GPIO_PIN_4
#define VOLTAGE_READ_PIN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
