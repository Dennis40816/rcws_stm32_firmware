/*
 * lra_error.h
 *
 *  Created on: Feb 11, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_ERROR_H_
#define INC_LRA_LRA_ERROR_H_

/* includes */

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

/* enums */
typedef enum {
  LRA_ERR_FAILURE = -1,
} LRA_ERR_t;

typedef enum {
  LRA_INIT_ERR_I2C_DEVS = 1,
  LRA_INIT_ERR_PWM = 2,
  LRA_INIT_ERR_MPU6500 = 4,
  LRA_INIT_ERR_ADXL355 = 8,
  LRA_INIT_ERR_USB = 16
} LRA_Init_Error_Bias_t;

#endif /* INC_LRA_LRA_ERROR_H_ */
