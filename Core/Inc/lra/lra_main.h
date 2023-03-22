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
} LRA_PWM_Update_Bit_t;

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

// for main.c (not lra_main.c)
#include "lra/lra_LED.h"

/* macros */

/* public functions */

void LRA_Main_EnterPoint(void);
HAL_StatusTypeDef LRA_Main_System_Init(void);

#endif /* INC_LRA_LRA_MAIN_H_ */