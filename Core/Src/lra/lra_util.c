/*
 * lra_util.c
 *
 *  Created on: Mar 16, 2023
 *      Author: Dennis Liu
 */

/* includes */

#include "stdbool.h"
#include "lra/lra_util.h"

/* public functios */

bool Lra_Check_SysClk(uint32_t hz) {
  return HAL_RCC_GetHCLKFreq() == hz;
}



