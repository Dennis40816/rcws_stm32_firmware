/*
 * lra_main.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_MAIN_H_
#define INC_LRA_LRA_MAIN_H_

/* enums */

// UEV for update event
typedef enum {
  LRA_PWM_FREQ_UEV_BIT = 0,
  LRA_PWM_DUTY_UEV_BIT = 1,
} LRA_PWM_Update_Bit_t;

typedef enum {
  LRA_DEVICE_STM32,
  LRA_DEVICE_MPU6500,
  LRA_DEVICE_ADXL355,
  LRA_DEVICE_DRV2605L_X,
  LRA_DEVICE_DRV2605L_Y,
  LRA_DEVICE_DRV2605L_Z,
  LRA_DEVICE_INVALID
} LRA_Device_Index_t;

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

// for main.c (not lra_main.c)
#include "lra/lra_LED.h"

/* macros */

/* typedef */

typedef void (*f_ptr)();

/* public functions */

void LRA_Main_EnterPoint(void);
HAL_StatusTypeDef LRA_Main_System_Init(void);

/* inline public functions */

uint8_t LRA_Device_Is_Valid(uint8_t device_index);

#endif /* INC_LRA_LRA_MAIN_H_ */
