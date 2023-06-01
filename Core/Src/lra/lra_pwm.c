/*
 * lra_pwm.c
 *
 *  Created on: Mar 10, 2023
 *      Author: Dennis Liu
 */

#include "lra/lra_pwm.h"
#include "lra/lra_util.h"

/* static vars */

static const size_t axis_count = 3;
static const size_t parameters_per_axis = 2;
static const size_t byte_count_per_parameter = sizeof(float);
static const size_t byte_count_delimiter = 1;
static const size_t byte_count_per_axis =
    parameters_per_axis * (byte_count_per_parameter + byte_count_delimiter);
static const size_t offset_freq =
    byte_count_per_parameter + byte_count_delimiter;

static const float pwm_min_amp = 0.0f;     // Define your min amp value
static const float pwm_max_amp = 1000.0f;  // Define your max amp value
static const float pwm_min_amp = 1.0f;     // Define your min freq value
static const float pwm_max_freq = 10.0f;   // Define your max freq value

/* public functions */

HAL_StatusTypeDef Lra_PWM_Init(LRA_PWM_t* handle,
                               uint32_t freq_hz,
                               uint16_t duty) {
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
HAL_StatusTypeDef Lra_PWM_Dynamic_Set_Freq(LRA_PWM_t* handle,
                                           uint32_t freq_hz) {
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
 * @brief Dynamicaly set PWM duty cycle
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

/**
 * @brief Parse binary data into RcwsPwmInfo structure.
 *
 * @details
 * This function takes an array of bytes as input and parses the data into an
 * RcwsPwmInfo structure. The input data should be organized as follows:
 *   - For each axis (X, Y, Z), there are two parameters, amplitude (amp) and
 * frequency (freq).
 *   - Each parameter is represented as a 4-byte floating point number (total of
 * 8 bytes per axis).
 *   - Parameters are separated by a comma (',') and axes are separated by a
 * semicolon (';').
 *
 * Therefore, the expected length of the input data is 30 bytes + "\r\n".
 *
 * @param data A pointer to an array of bytes representing the parameters for
 * each axis.
 * @param info A struct pointer holds LRA_RCWS_PWM_Info_t. Will be modified if
 * parsing successed.
 * @return RcwsPwmInfo structure filled with the parsed parameters.
 */
HAL_StatusTypeDef LRA_Parse_RCWS_PWM_Info(const uint8_t* const pdata,
                                          LRA_RCWS_PWM_Info_t* info) {
  /* save check \r\n exist */
  if ((pdata + axis_count * byte_count_per_axis) != '\r' &&
      (pdata + axis_count * byte_count_per_axis + 1) != '\n')
    return HAL_ERROR;

  /* parse */
  LRA_PWM_Info_t* pwm_info_arr[axis_count] = {&(info->x), &(info->y),
                                              &(info->z)};

  /* Assuming little-endian system, and float is 4 bytes */
  for (size_t i = 0; i < axis_count; ++i) {
    const uint8_t* amp_bytes = pdata + i * byte_count_per_axis;
    const uint8_t* freq_bytes = amp_bytes + offset_freq;
    LRA_PWM_Info_t* pwm_info = pwm_info_arr[i];

    /* Assuming that the platform uses IEEE 754 floating-point representation */
    memcpy(&(pwm_info->amp), amp_bytes, sizeof(float));
    memcpy(&(pwm_info->freq), freq_bytes, sizeof(float));
  }

  return HAL_OK;
}

/**
 * @brief Checks if all parameters in the LRA_RCWS_PWM_Info_t structure are
 * within their predefined limits.
 *
 * This function iterates over each axis in the LRA_RCWS_PWM_Info_t structure,
 * checking if the amplitude and frequency parameters are within their
 * respective minimum and maximum limits. The limits are defined by MIN_AMP,
 * MAX_AMP, MIN_FREQ, and MAX_FREQ constants.
 *
 * @param info The LRA_RCWS_PWM_Info_t structure that contains the parameters to
 * be checked.
 * @return Returns 'HAL_OK' if all parameters are within their respective
 * limits, 'HAL_ERROR' otherwise.
 */
HAL_StatusTypeDef LRA_RCWS_PWM_Info_Range_Check(
    const LRA_RCWS_PWM_Info_t* info) {
  LRA_PWM_Info_t* pwm_info_arr[axis_count] = {&(info->x), &(info->y),
                                              &(info->z)};

  for (size_t i = 0; i < axis_count; ++i) {
    LRA_PWM_Info_t* pwm_info = pwm_info_arr[i];
    if (pwm_info->amp <= pwm_min_amp || pwm_info->amp >= pwm_max_amp) {
      return HAL_ERROR;
    }
    if (pwm_info->freq <= pwm_min_amp || pwm_info->freq >= pwm_max_freq) {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}
