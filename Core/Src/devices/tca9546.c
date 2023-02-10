/*
 * tca9546a.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

/* includes */

#include "devices/tca9546.h"

/* exported variables */

// when A1 ~ A3 are all low -> default address is 0x70 (0b111 0000)
const uint8_t tca9546_default_addr = 0x70;
const uint16_t tca9546_default_timeout_ms = STM32_I2C_MIN_TIMEOUT_MS;

/* public functions */

/**
 * @brief 唯一修改 TCA channel 的 function
 *
 * @param pTca tca 實例的指標
 * @param ch 目標 channel，應介在 0 ~ 4 之間，0 表示沒有 channel 被選擇 (default
 * state)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_Modify_CH(TCA9546_t* const pTca, const uint8_t ch) {
  if (pTca == NULL)
    return HAL_ERROR;

  if (pTca->hi2c == NULL)
    return HAL_ERROR;

  // range check
  if (ch > TCA9546_CH_NUM)
    return HAL_ERROR;

  uint8_t ch2hex = (ch == 0) ? 0x0 : 0x01 << (ch - 1);

  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
      pTca->hi2c, (pTca->dev_addr << 1) | I2C_W, &ch2hex, 1, pTca->timeout_ms);

  if (ret == HAL_OK) {
    pTca->ch = ch;
  }

  return ret;
}

/**
 * @brief 從硬件讀取並更新 pTca 的通道，當成功讀取返回 ch，失敗 (HAL_ERROR or
 * HAL_BUSY) 則返回 -1
 *
 * @details ch 是指 struct TCA9546_t 中的 ch，而不是 TCA9546 手冊中的寄存器通道
 * (詳情請見 tca9546.h - tca9546 channel mapping)
 * @param pTca
 * @return int8_t
 */
int8_t TCA_Get_CH(TCA9546_t* const pTca) {
  if (pTca == NULL)
    return HAL_ERROR;

  if (pTca->hi2c == NULL)
    return HAL_ERROR;

  uint8_t ch_hex = 0xff;

  HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(
      pTca->hi2c, (pTca->dev_addr << 1) | I2C_R, &ch_hex, 1, pTca->timeout_ms);

  if (ret == HAL_OK) {
    uint8_t ch = 0xff;

    if (ch_hex == 0x0)
      ch = 0;

    for (int n = 0; n < TCA9546_CH_NUM; n++) {
      if (1 << n == ch_hex) {
        ch = n + 1;
        break;
      }
    }

    // unexpected error, indicates ch == 0xff
    if (ch > TCA9546_CH_NUM)
      return HAL_ERROR;

    // update channel
    pTca->ch = ch;
    return ch;
  }

  // HAL_ERROR or HAL_BUSY
  return -1;
}

/**
 * @brief Try to reset TCA9546 by pull low and then high at NRST pin. Note that
 * this function return HAL_ERROR if no NRST pin.
 *
 * @param pTca
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_Reset(TCA9546_t* const pTca) {
  if (pTca == NULL)
    return HAL_ERROR;

  if (pTca->hi2c == NULL)
    return HAL_ERROR;

  if (pTca->reset_port == NULL)
    return HAL_ERROR;

  // Try to pull low reset pin and then pull high again.
  // May cause bug at delay
  HAL_GPIO_WritePin(pTca->reset_port, pTca->reset_pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(pTca->reset_port, pTca->reset_pin, GPIO_PIN_SET);
  HAL_Delay(1);

  // read ch and update to pTca, ret should always return 0
  int8_t ret = TCA_Get_CH(pTca);

  // read not ok
  if (ret == -1)
    return HAL_ERROR;

  return HAL_OK;
}
