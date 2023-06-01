/**
 * @file lra_it.c
 * @author DennisLiu16
 * @brief
 * @date 2023-05-25
 *
 * @copyright Copyright (c) 2023
 *
 */

/* includes */

#include <lra/lra_it.h>

#include <lra/lra_LED.h>
#include <lra/lra_spi_devices.h>
#include <main.h>

/* extern variables */

uint8_t acc_should_send_flag = LRA_FLAG_UNSET;

/* extern functions pointers */

HAL_StatusTypeDef (*const LRA_IT_ADXL355_DRDY_Callback_P)(
    ADXL355_t* const,
    LRA_DualBuf_t* const) = ADXL355_Read_NewestData_DualBuf;

/* public functions */

/**
 * @brief Make LED_Pin toggle ever
 *
 * @details This function depends on extern var: led_state
 *
 * @param it_times
 */
void LRA_IT_LED_Flash_Every(const uint32_t it_times) {
  static uint32_t counter = 0;

  switch (led_state) {
    case LRA_LED_FLASH:
      if (counter++ >= it_times) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        counter = 0;
      }
      break;

    case LRA_LED_DOWN:
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      counter = 0;
      break;
  }
}