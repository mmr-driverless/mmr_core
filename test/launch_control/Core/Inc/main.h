/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.

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
#include "stm32f3xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define GEARUP_SWITCH_Pin GPIO_PIN_1
#define GEARUP_SWITCH_GPIO_Port GPIOC
#define GEARN_SWITCH_Pin GPIO_PIN_2
#define GEARN_SWITCH_GPIO_Port GPIOC
#define GEARDOWN_SWITCH_Pin GPIO_PIN_3
#define GEARDOWN_SWITCH_GPIO_Port GPIOC
#define APPS1_IN_Pin GPIO_PIN_0
#define APPS1_IN_GPIO_Port GPIOA
#define APPS2_IN_Pin GPIO_PIN_1
#define APPS2_IN_GPIO_Port GPIOA
#define APPS1_OUT_Pin GPIO_PIN_4
#define APPS1_OUT_GPIO_Port GPIOA
#define WATCHDOG_Pin GPIO_PIN_6
#define WATCHDOG_GPIO_Port GPIOA
#define LSW_LED1_Pin GPIO_PIN_4
#define LSW_LED1_GPIO_Port GPIOC
#define LSW_LEDEBS_Pin GPIO_PIN_5
#define LSW_LEDEBS_GPIO_Port GPIOC
#define LSW_ASSI_BLUE_Pin GPIO_PIN_0
#define LSW_ASSI_BLUE_GPIO_Port GPIOB
#define LSW_ASSI_YELLOW_Pin GPIO_PIN_1
#define LSW_ASSI_YELLOW_GPIO_Port GPIOB
#define ASMS_SUPPLY_Pin GPIO_PIN_12
#define ASMS_SUPPLY_GPIO_Port GPIOB
#define EBS_CONTROL1_Pin GPIO_PIN_15
#define EBS_CONTROL1_GPIO_Port GPIOB
#define EBS_CONTROL2_Pin GPIO_PIN_6
#define EBS_CONTROL2_GPIO_Port GPIOC
#define BUTTON1_Pin GPIO_PIN_8
#define BUTTON1_GPIO_Port GPIOC
#define SDC_IS_READY_Pin GPIO_PIN_9
#define SDC_IS_READY_GPIO_Port GPIOC
#define AS_SDC_CLOSE_Pin GPIO_PIN_8
#define AS_SDC_CLOSE_GPIO_Port GPIOA
#define AS_DRIVING_MODE_Pin GPIO_PIN_9
#define AS_DRIVING_MODE_GPIO_Port GPIOA
#define CTR_LED1_Pin GPIO_PIN_3
#define CTR_LED1_GPIO_Port GPIOB
#define CTR_LED2_Pin GPIO_PIN_4
#define CTR_LED2_GPIO_Port GPIOB
#define CTR_LED3_Pin GPIO_PIN_5
#define CTR_LED3_GPIO_Port GPIOB
#define SIG__Pin GPIO_PIN_9
#define SIG__GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
