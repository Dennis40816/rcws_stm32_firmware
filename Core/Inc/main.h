/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_X_Pin GPIO_PIN_0
#define PWM_X_GPIO_Port GPIOA
#define PWM_Y_Pin GPIO_PIN_1
#define PWM_Y_GPIO_Port GPIOA
#define PWM_Z_Pin GPIO_PIN_2
#define PWM_Z_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOA
#define KX_INT1_Pin GPIO_PIN_4
#define KX_INT1_GPIO_Port GPIOC
#define KX_INT1_EXTI_IRQn EXTI4_IRQn
#define KX_INT2_Pin GPIO_PIN_5
#define KX_INT2_GPIO_Port GPIOC
#define KX_INT2_EXTI_IRQn EXTI9_5_IRQn
#define MPU_INT_Pin GPIO_PIN_6
#define MPU_INT_GPIO_Port GPIOC
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define PWM_Z_OPT_Pin GPIO_PIN_8
#define PWM_Z_OPT_GPIO_Port GPIOC
#define PWM_Y_OPT_Pin GPIO_PIN_9
#define PWM_Y_OPT_GPIO_Port GPIOC
#define PWM_X_OPT_Pin GPIO_PIN_8
#define PWM_X_OPT_GPIO_Port GPIOA
#define RASP_INT_Pin GPIO_PIN_2
#define RASP_INT_GPIO_Port GPIOD
#define RASP_INT_EXTI_IRQn EXTI2_IRQn
#define RASP_GPIO1_Pin GPIO_PIN_4
#define RASP_GPIO1_GPIO_Port GPIOB
#define TCA_NRST_Pin GPIO_PIN_5
#define TCA_NRST_GPIO_Port GPIOB
#define RASP_GPIO2_Pin GPIO_PIN_8
#define RASP_GPIO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
