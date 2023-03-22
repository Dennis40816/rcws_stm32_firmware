/*
 * lra_LED.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LED_H_
#define INC_LRA_LED_H_

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

/* exported enums */ 

typedef enum {
	LRA_LED_DOWN,
	LRA_LED_UP,
	LRA_LED_FLASH
}LRA_LEDStates_t;

/* extern variables */ 

extern LRA_LEDStates_t led_state;

/* public functions */

void LRA_LED_Flash_N(uint8_t n, uint32_t delay);

#endif /* INC_LRA_LED_H_ */
