/*
 * lra_timer.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Dennis Liu
 */

#ifndef INC_LRA_LRA_TIMER_H_
#define INC_LRA_LRA_TIMER_H_

/* includes */

#include "lra/lra_user_config.h"

/* extern variables */

/* public functions */

float LRA_Get_Time();
unsigned long long LRA_Get_Time_Ms();
void LRA_Ms_Increment();

#endif /* INC_LRA_LRA_TIMER_H_ */
