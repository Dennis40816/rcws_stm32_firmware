/*
 * lra_pwm.c
 *
 *  Created on: Mar 10, 2023
 *      Author: Dennis Liu
 */

#include "lra/lra_pwm.h"
#include "lra/lra_util.h"

/* public functions */

HAL_StatusTypeDef Lra_PWM_Init(LRA_PWM_t* handle, uint32_t freq_hz, uint16_t duty) {
  if (handle == NULL || handle->htim == NULL)
    return HAL_ERROR;

  uint8_t ret = 0;

  // Disable PWM channel first
  ret = Lra_PWM_Disable(handle);
  if (ret != HAL_OK)
    return ret;

  ret = Lra_PWM_Dynamic_Set_Freq(handle, freq_hz);
  if (ret != HAL_OK)
    return ret;

  // set duty cycle to 50% (accroding to DRV2605L datasheet, in open pwm control
  // mode 50 % duty cycle represent 0 V  output)
  return Lra_PWM_Dynamic_Set_Duty(handle, duty);
}

HAL_StatusTypeDef Lra_PWM_Enable(LRA_PWM_t* handle) {
  if (handle == NULL || handle->htim == NULL)
    return HAL_ERROR;

  return HAL_TIM_PWM_Start(handle->htim, handle->ch);
}

HAL_StatusTypeDef Lra_PWM_Disable(LRA_PWM_t* handle) {
  if (handle == NULL || handle->htim == NULL)
    return HAL_ERROR;

  return HAL_TIM_PWM_Stop(handle->htim, handle->ch);
}

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
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Freq(LRA_PWM_t* handle, uint32_t freq_hz) {
  if (handle == NULL || handle->htim == NULL || freq_hz == 0)
    return HAL_ERROR;

  volatile uint32_t hclk = HAL_RCC_GetHCLKFreq();
  volatile uint32_t psc = handle->htim->Instance->PSC;

  /* psc is an uint16_t, so using an uint32_t to store psc should never cause
   * (psc + 1) == 0. But a check still takes place to make sure a div0 event
   * will never happen
   */
  if ((psc + 1) == 0 || freq_hz > (hclk / (psc + 1)))
    return HAL_ERROR;

  // calculate desired autoreload value

  // TODO: avoid float calulation
  uint16_t new_ARR = ((float)hclk) / (psc + 1) / freq_hz;

  __HAL_TIM_SET_AUTORELOAD(handle->htim, new_ARR);

  return HAL_OK;
}

/**
 * @brief
 *
 * @warning This function can only calculate 16 bit compare register (<=65536)
 * @param handle
 * @param duty_permil Duty cycle in permils. e.g. 896 ‰ = 896 = 89.6%. Therefore
 * duty_permil should be less than 1000.
 * @return HAL_StatusTypeDef
 * 
 */
/* XXX, need to be optimized */
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Duty(LRA_PWM_t* handle,
                                           uint16_t duty_permil) {
  /* check parameters are valid */
  if (handle == NULL || handle->htim == NULL || duty_permil > 1000)
    return HAL_ERROR;

  /* get ARR min for requested accuracy, if duty_permil can be devided by 10,
   * than ARR_min is 100. We calculate this part for we don't want to calculate
   * in float precision. Therefore, the next step - calculate new CCR value
   * should be optimized later -> should not use any float or double.
   */
  uint8_t div10_remain = duty_permil % 10;
  uint32_t ARR_min = (!div10_remain) ? 99 : 999;

  /* get ARR value  */
  volatile uint32_t ARR_cache = __HAL_TIM_GET_AUTORELOAD(handle->htim);

  // TODO: make ARR_maximum become to 2^32 - 1 (for TIM2 and TIM8)
  if (ARR_cache < ARR_min || ARR_cache > 65535)
    return HAL_ERROR;

    /* TODO: calculate new CCR value */
    // uint32_t new_CCR = (div10_remain)
    //                        ? (ARR_cache + 1) / 1000.0 * duty_permil
    //                        : (ARR_cache + 1) / 100.0 * duty_permil / 10;
  uint16_t new_CCR = (ARR_cache + 1) / 1000.0 * duty_permil;

    /* update to hardware register CCR */
  __HAL_TIM_SET_COMPARE(handle->htim, handle->ch, new_CCR);

  return HAL_OK;
}
