/*
 * lra_main.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_MAIN_H_
#define INC_LRA_LRA_MAIN_H_

/* predefined macros */

// comment next line if you don't need to debug
#define LRA_DEBUG

// comment next line to suspend system info send to Rasp
#define LRA_SYSTEM_INFO

/* enums */

// UEV for update event
typedef enum {
  LRA_PWM_FREQ_UEV_BIT = 0,
  LRA_PWM_DUTY_USV_BIT = 1,
} LRA_PWM_UPDATE_FLAG;

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

#include <lra/lra_error.h>
#include <lra/lra_i2c_devices.h>
#include <lra/lra_usb.h>
#include <lra/lra_LED.h>
#include <lra/lra_pwm.h>

/* macros */

#define LRA_DEFAULT_PWM_FREQ (128 * 175)

// half of 1000 (1000‰)
#define LRA_DEFAULT_PWM_DUTY (500)

/* public functions */

void LRA_Main_EnterPoint(void);
HAL_StatusTypeDef LRA_Main_System_Init(void);

#endif /* INC_LRA_LRA_MAIN_H_ */
