/*
 * lra_pwm.h
 *
 *  Created on: Mar 10, 2023
 *      Author: Dennis Liu
 *
 *  Warnings: This library assumes that APB1 or APB2 timer clocks are same as
 *  HCLK (SysCoreClock).
 */

#ifndef INC_LRA_LRA_PWM_H_
#define INC_LRA_LRA_PWM_H_

/* includes */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

#include "lra/lra_user_config.h"

/* exported variables */

/* macros */

#define LRA_DRV_PWM_FREQ_COEFF (128)

/* enums */

typedef enum {
  TIM_CH1 = TIM_CHANNEL_1,
  TIM_CH2 = TIM_CHANNEL_2,
  TIM_CH3 = TIM_CHANNEL_3,
  TIM_CH4 = TIM_CHANNEL_4,
} LRA_TIM_CH;

/* structs */

typedef struct {
  TIM_HandleTypeDef* htim;
  LRA_TIM_CH ch;
} LRA_PWM_t;

/* public functions */

HAL_StatusTypeDef Lra_PWM_Init(LRA_PWM_t* handle,
                               uint32_t freq_hz,
                               uint16_t duty);
HAL_StatusTypeDef Lra_PWM_Enable(LRA_PWM_t* handle);
HAL_StatusTypeDef Lra_PWM_Disable(LRA_PWM_t* handle);
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Freq(LRA_PWM_t* handle, uint32_t freq_hz);
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Duty(LRA_PWM_t* handle,
                                           uint16_t duty_permil);

#endif /* INC_LRA_LRA_PWM_H_ */
