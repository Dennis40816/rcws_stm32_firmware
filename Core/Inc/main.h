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
#include "lra/lra_main.h"
#include "lra/lra_usb.h"
#include "lra/lra_LED.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// TCA9546 related
extern const uint8_t tca9546_default_addr;
extern const uint16_t tca9546_default_timeout_ms;

// DRV2605L related
extern const uint8_t drv2605l_default_addr;
extern const uint16_t drv2605l_default_timeout_ms;

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
#define MPU_INT_Pin GPIO_PIN_6
#define MPU_INT_GPIO_Port GPIOC
#define MPU_INT_EXTI_IRQn EXTI9_5_IRQn
#define PWM_Z_OPT_Pin GPIO_PIN_8
#define PWM_Z_OPT_GPIO_Port GPIOC
#define PWM_Y_OPT_Pin GPIO_PIN_9
#define PWM_Y_OPT_GPIO_Port GPIOC
#define PWM_X_OPT_Pin GPIO_PIN_8
#define PWM_X_OPT_GPIO_Port GPIOA
#define KX_INT2_Pin GPIO_PIN_2
#define KX_INT2_GPIO_Port GPIOD
#define KX_INT2_EXTI_IRQn EXTI2_IRQn
#define KX_INT1_Pin GPIO_PIN_4
#define KX_INT1_GPIO_Port GPIOB
#define TCA_NRST_Pin GPIO_PIN_5
#define TCA_NRST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
