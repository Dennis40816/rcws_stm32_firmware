/*
 * lra_util.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Dennis Liu
 */

#ifndef INC_LRA_LRA_UTIL_H_
#define INC_LRA_LRA_UTIL_H_

/* includes */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"

#include "lra/lra_user_config.h"

/* structs */

/**
 * @brief A struct for double buffer
 *
 * @param cur_buf current activate buffer index, 0 or 1.
 * @param buf_full show buffer can't do next operation, 1 for full.
 * @param dbuf the buffer porinter array to store two buffers address
 * @param buf_size the maximum size of the buffer
 * @param buf_index where the buffer start to write, also equal to current
 * buffer len before writing
 *
 */
typedef struct {
  volatile uint8_t cur_buf;
  volatile uint8_t buf_full[2];
  uint8_t* dbuf[2];
  uint16_t buf_size[2];
  volatile uint16_t buf_index[2];
} LRA_DualBuf_t;

/* public */

int LRA_Util_MakeStrNoNull(uint8_t* dest, const char* format, ...);
#endif /* INC_LRA_LRA_UTIL_H_ */
