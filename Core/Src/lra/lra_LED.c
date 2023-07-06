/*
 * lra_LED.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */

#include "lra/lra_LED.h"
#include "main.h"

/* public variables */

LRA_LEDStates_t led_state = LRA_LED_DOWN;

/* public functions */

void LRA_LED_State_Change(LRA_LEDStates_t next_state) {
  led_state = next_state;
}

/**
 * @brief LED flash n time, blocked function
 *
 * @param n
 * @param delay
 */
void LRA_LED_Flash_N(uint8_t n, uint32_t delay) {
  for (uint8_t i = 0; i < n; i++) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(delay);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(delay);
  }
}