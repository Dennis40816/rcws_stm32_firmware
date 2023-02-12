/*
 * lra_main.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_MAIN_H_
#define INC_LRA_LRA_MAIN_H_

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

#include "lra/lra_LED.h"
#include "lra/lra_error.h"
#include "lra/lra_i2c_devices.h"
#include "lra/lra_usb.h"

/* public functions */

void LRA_Main_EnterPoint(void);
HAL_StatusTypeDef LRA_Main_System_Init(void);

#endif /* INC_LRA_LRA_MAIN_H_ */
