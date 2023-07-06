/*
 * lra_spi_devices.h
 *
 *  Created on: Mar 22, 2023
 *      Author: Dennis Liu
 */

#ifndef INC_LRA_LRA_SPI_DEVICES_H_
#define INC_LRA_LRA_SPI_DEVICES_H_

/* includes */

#include "lra/lra_user_config.h"

#include "lra/lra_dbuf.h"
#include "lra/lra_util.h"

#include "devices/adxl355.h"
#include "stdbool.h"

/* public structs */

/**
 * @brief
 *
 * @test align verification: https://godbolt.org/z/bs775aePb
 * @result decide to use ADXL355_DataSet_t instead ADXL355_RawDataSet_t
 * (consider speed)
 *
 */
typedef struct {
  uint32_t w_index;
  uint32_t r_index;
  bool is_full;
  uint32_t size;
  ADXL355_DataSet_t data[LRA_ACC_BUFFER_SIZE];
} LRA_AccRingBuffer_t;

/* external variables */
extern LRA_AccRingBuffer_t lra_acc_rb;

/* public functions */

// ADXL355 related
HAL_StatusTypeDef ADXL355_Append_To_AccRingBuf(ADXL355_t* const pAdxl,
                                               LRA_AccRingBuffer_t* const rbuf);

HAL_StatusTypeDef LRA_Read_One_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                               ADXL355_DataSet_t* dest);

uint32_t LRA_Read_Available_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                            void* dest);

uint32_t LRA_Read_N_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                    void* dest,
                                    uint32_t n);

uint32_t LRA_Copy_N_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                    void* dest,
                                    uint32_t n);

uint32_t LRA_Xfer_NAcc_Rbuf2Dbuf(LRA_AccRingBuffer_t* const rbuf,
                                 LRA_Acc_Dbuf_t* const dbuf,
                                 uint32_t n);

uint32_t LRA_Maintain_AccRingBuf_States(LRA_AccRingBuffer_t* rbuf,
                                        char action,
                                        uint32_t step);

#endif /* INC_LRA_SPI_DEVICES_H_ */
