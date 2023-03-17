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
} LRA_PWM;

/* public functions */

HAL_StatusTypeDef Lra_PWM_Init(LRA_PWM* handle, uint32_t freq_hz);

HAL_StatusTypeDef Lra_PWM_Enable(LRA_PWM* handle);

HAL_StatusTypeDef Lra_PWM_Disable(LRA_PWM* handle);

/**
 * @brief Dynamicaly set PWM frequency (internal function).
 *
 * @version 1.0
 * @note
 * V1.0: This function try not to change the TIMx->PSC register. Therefore, only
 * TIMx->ARR register will be changed. Therefore, the maximum PWM frequency
 * should be less than HCLK(we assume APB1 and APB2 timer clock is same as HCLK)
 * / TIMx->PSC.
 * @note ----------------------------------------------------------------
 * @note If hclk / (psc + 1) / freq_hz == 0, this function will return HAL_ERROR
 * and do nothing.
 *
 * @param hpwm
 * @param freq_hz desired frequency in Hz
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Freq(LRA_PWM* handle, uint32_t freq_hz);

/**
 * @brief
 *
 * @warning This function can only calculate 16 bit compare register (<=65536)
 * @param handle
 * @param duty_permil Duty cycle in permils. e.g. 896 ‰ = 896 = 89.6%. Therefore
 * duty_permil should be less than 1000.
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Duty(LRA_PWM* handle,
                                           uint16_t duty_permil);

#endif /* INC_LRA_LRA_PWM_H_ */
