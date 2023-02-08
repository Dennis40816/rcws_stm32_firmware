/*
 * lra_LED.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LED_H_
#define INC_LRA_LED_H_

// exported enums
enum ledStates {
	e_led_toggle,
	e_led_getMsg,
};

// extern variables
extern enum ledStates led_state;

#endif /* INC_LRA_LED_H_ */
