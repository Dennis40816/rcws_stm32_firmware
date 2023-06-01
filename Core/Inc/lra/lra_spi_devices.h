/*
 * lra_spi_devices.h
 *
 *  Created on: Mar 22, 2023
 *      Author: Dennis Liu
 */

#ifndef INC_LRA_SPI_DEVICES_H_
#define INC_LRA_SPI_DEVICES_H_

/* includes */

#include "lra/lra_user_config.h"
#include "lra/lra_util.h"

#include "devices/adxl355.h"

/* external variables */

extern uint8_t lra_acc_buf1[LRA_ACC_BUFFER_SIZE];
extern uint8_t lra_acc_buf2[LRA_ACC_BUFFER_SIZE];
extern LRA_DualBuf_t lra_acc_dbuf;

/* public structs */

typedef struct {
  volatile uint8_t cur_w_buf;
  uint16_t buf_size;
  volatile uint8_t buf_full[2];
  ADXL355_RawDataSet_t* dbuf[2];
  volatile uint16_t buf_index[2];
} LRA_AccDualBuf_t;

/* public functions */

// ADXL355 related
HAL_StatusTypeDef ADXL355_Read_NewestData_DualBuf(ADXL355_t* const pAdxl,
                                                  LRA_DualBuf_t* const dbuf);

#endif /* INC_LRA_SPI_DEVICES_H_ */
