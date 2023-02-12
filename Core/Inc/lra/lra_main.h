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

/* includes */

#include <stm32f4xx_hal.h>

#include <lra/lra_error.h>
#include <lra/lra_i2c_devices.h>
#include <lra/lra_usb.h>
#include <lra/lra_LED.h>

/* public functions */

void LRA_Main_EnterPoint(void);
HAL_StatusTypeDef LRA_Main_System_Init(void);

#endif /* INC_LRA_LRA_MAIN_H_ */
