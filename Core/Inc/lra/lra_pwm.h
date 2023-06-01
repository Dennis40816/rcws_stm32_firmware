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

/* enums */

typedef enum {
  TIM_CH1 = TIM_CHANNEL_1,
  TIM_CH2 = TIM_CHANNEL_2,
  TIM_CH3 = TIM_CHANNEL_3,
  TIM_CH4 = TIM_CHANNEL_4,
} LRA_TIM_Ch_t;

/* structs */

typedef struct {
  TIM_HandleTypeDef* htim;
  LRA_TIM_Ch_t ch;
} LRA_PWM_t;

/* alias to PwmInfo & RcwsPwmInfo for raspberry side */
typedef struct PwmInfo {
  float amp;
  float freq;
} LRA_PWM_Info_t;

typedef struct RcwsPwmInfo {
  LRA_PWM_Info_t x;
  LRA_PWM_Info_t y;
  LRA_PWM_Info_t z;
} LRA_RCWS_PWM_Info_t;

/* public functions */

HAL_StatusTypeDef Lra_PWM_Init(LRA_PWM_t* handle,
                               uint32_t freq_hz,
                               uint16_t duty);
HAL_StatusTypeDef Lra_PWM_Enable(LRA_PWM_t* handle);
HAL_StatusTypeDef Lra_PWM_Disable(LRA_PWM_t* handle);
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Freq(LRA_PWM_t* handle, uint32_t freq_hz);
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Duty(LRA_PWM_t* handle,
                                           uint16_t duty_permil);
HAL_StatusTypeDef LRA_Parse_RCWS_PWM_Info(const uint8_t* const pdata,
                                          LRA_RCWS_PWM_Info_t* info);
HAL_StatusTypeDef LRA_RCWS_PWM_Info_Range_Check(
    const LRA_RCWS_PWM_Info_t* info);

#endif /* INC_LRA_LRA_PWM_H_ */
