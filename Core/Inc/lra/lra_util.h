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

/* enums */

typedef enum { LRA_FLAG_UNSET, LRA_FLAG_SET } LRA_Flag_t;

/* structs */

typedef struct {
} LRA_DualBuf_Info_t;

/**
 * @brief A struct for double buffering
 *
 * @param cur_w_buf current activated buffer index, 0 or 1.
 * @param buf_full show buffer can't do next operation, 1 for full.
 * @param dbuf the buffer porinter array to store two buffers address
 * @param buf_size the maximum size of the buffer
 * @param buf_index where the buffer start to write, also equal to current
 * buffer len before writing
 *
 */
typedef struct {
  volatile uint8_t cur_w_buf;
  uint16_t buf_size;
  volatile uint8_t buf_full[2];
  uint8_t* dbuf[2];
  volatile uint16_t buf_index[2];
} LRA_DualBuf_t;

/* public functions */

int LRA_Util_MakeStrNoNull(uint8_t* dest, const char* format, ...);
uint8_t Check_Addr_Overlap(uint32_t addr_begin,
                           uint32_t addr_end,
                           uint32_t f_addr_begin,
                           uint32_t f_addr_end);
HAL_StatusTypeDef LRA_Read_From_DualBuf(LRA_DualBuf_t* const dualBuf,
                                        uint8_t* const pdata);
HAL_StatusTypeDef LRA_Write_To_DualBuf(LRA_DualBuf_t* const dBuf,
                                       const uint8_t* const pdata,
                                       const uint16_t len);
#endif /* INC_LRA_LRA_UTIL_H_ */
