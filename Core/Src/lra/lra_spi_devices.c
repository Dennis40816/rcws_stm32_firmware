/*
 * lra_spi_devices.c
 *
 *  Created on: Mar 22, 2023
 *      Author: Dennis Liu
 */

/* includes */

#include "lra/lra_spi_devices.h"

#include "lra/lra_dbuf.h"
#include "lra/lra_timer.h"
#include "string.h"

// debug
#include "lra/lra_it.h"

/* private functions definition */

static uint32_t LRA_AccRingBuf_GetReadMaxStep(LRA_AccRingBuffer_t* rbuf);

/* extern variables definitions */
LRA_AccRingBuffer_t lra_acc_rb = {.w_index = 0,
                                  .r_index = 0,
                                  .is_full = false,
                                  .size = LRA_ACC_BUFFER_SIZE};

/* private functions */

static inline ADXL355_DataSet_t* _Get_RingBuf_W_Pointer(
    LRA_AccRingBuffer_t* rbuf) {
  return &(rbuf->data[rbuf->w_index]);
}

/* public functions */

/**
 * @brief Optimized ADXL355 SPI IO for ring buffer.
 *
 * @param pAdxl
 * @param rbuf
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Append_To_AccRingBuf(
    ADXL355_t* const pAdxl,
    LRA_AccRingBuffer_t* const rbuf) {
  if (pAdxl == NULL || rbuf == NULL || rbuf->data == NULL ||
      pAdxl->hspi == NULL)
    return HAL_ERROR;

  static float last_time = 0;

  // 10 for 9 data bytes + 1 dummy byte
  static const uint8_t tx_tmp[10] = {(ADXL355_XDATA3 << 1 | ADXL355_SPI_R)};
  uint8_t rx_tmp[10] = {0};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_tmp, rx_tmp,
                                                  10, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret != HAL_OK)
    return ret;

  // get current time
  float current_time = LRA_Get_Time();

  last_time = current_time;

  // get pointer
  ADXL355_DataSet_t* dest = _Get_RingBuf_W_Pointer(rbuf);

  // parse raw acc data, rx_tmp + 1 for skipping dummy byte
  ret = ADXL355_ParseDataSet(pAdxl, rx_tmp + 1, dest);

  if (ret != HAL_OK)
    return ret;

  dest->t = current_time;

  // move, is_full maintain
  LRA_Maintain_AccRingBuf_States(rbuf, 'w', 1);

  return HAL_OK;
}

/* Read functions facing 1-D array */

/**
 * @brief
 *
 * @param rbuf
 * @param raw_data
 * @return HAL_ERROR represents rbuf is empty
 */
HAL_StatusTypeDef LRA_Read_One_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                               ADXL355_DataSet_t* dest) {
  if (rbuf == NULL || rbuf->data == NULL || rbuf->w_index == rbuf->r_index)
    return HAL_ERROR;

  // get r_index
  uint32_t r_index = rbuf->r_index;

  memcpy(dest, &rbuf->data[r_index], sizeof(ADXL355_DataSet_t));

  // move and is_full maintain
  return LRA_Maintain_AccRingBuf_States(rbuf, 'r', 1);
}

uint32_t LRA_Read_Available_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                            void* dest) {
  uint32_t step = LRA_AccRingBuf_GetReadMaxStep(rbuf);
  return LRA_Read_N_From_AccRingBuf(rbuf, dest, step);
}

uint32_t LRA_Read_N_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                    void* dest,
                                    uint32_t n) {
  uint32_t step = LRA_Copy_N_From_AccRingBuf(rbuf, dest, n);

  // move and is_full maintain
  return LRA_Maintain_AccRingBuf_States(rbuf, 'r', step);
}

/**
 *
 * @brief Copy n rawdata starting from r_index. Note that this function won't
 * change r_index.
 *
 * @param dbuf
 * @param raw_data
 * @return HAL_StatusTypeDef
 */
uint32_t LRA_Copy_N_From_AccRingBuf(LRA_AccRingBuffer_t* const rbuf,
                                    void* dest,
                                    uint32_t n) {
  // check n is valid
  uint32_t step = min(LRA_AccRingBuf_GetReadMaxStep(rbuf), n);

  if (step == 0)
    return step;

  // TODO: Use local variable to avoid data racing

  // if read will go through end of the buffer
  if ((rbuf->r_index > rbuf->w_index) && step >= (rbuf->size - rbuf->r_index)) {
    uint32_t first_step = rbuf->size - rbuf->r_index;
    uint32_t second_step = step - first_step;
    memcpy(dest, &rbuf->data[rbuf->r_index],
           first_step * sizeof(ADXL355_DataSet_t));
    memcpy(dest + first_step * sizeof(ADXL355_DataSet_t), &rbuf->data[0],
           second_step * sizeof(ADXL355_DataSet_t));
  } else {
    memcpy(dest, &rbuf->data[rbuf->r_index], step * sizeof(ADXL355_DataSet_t));
  }

  return step;
}

/* Read functions facing double buffers (for USB) */

/**
 * @brief Transfer N LRA_AccRingBuffer_t from ring buffer to double buffer
 *
 * @param rbuf
 * @param dbuf
 * @param n
 * @return uint32_t The actual number of data we transfered
 */
uint32_t LRA_Xfer_NAcc_Rbuf2Dbuf(LRA_AccRingBuffer_t* const rbuf,
                                 LRA_Acc_Dbuf_t* const dbuf,
                                 uint32_t n) {
  if (n == 0 || rbuf == NULL || dbuf == NULL)
    return 0;

  // make sure n <= available data in ringbuf
  uint32_t max_step = LRA_AccRingBuf_GetReadMaxStep(rbuf);

  uint32_t step = min(max_step, n);

  //  if (step == 0)
  //    return 0;

  uint32_t rb_critical_step = rbuf->size - rbuf->r_index;
  uint32_t db_critical_step = dbuf->size - dbuf->count[dbuf->current_buffer];

  // decide transfer size, make sure transfer size can always be completed
  // before state maintain process
  uint32_t transfer_size = step;
  if (transfer_size > rb_critical_step || transfer_size > db_critical_step) {
    transfer_size = min(rb_critical_step, db_critical_step);
  }

  // normal case (no cross happens)
  memcpy(&(dbuf->data[dbuf->current_buffer][dbuf->count[dbuf->current_buffer]]),
         &(rbuf->data[rbuf->r_index]),
         transfer_size * sizeof(ADXL355_DataSet_t));

  // update buffers' states
  LRA_Maintain_AccRingBuf_States(rbuf, 'r', transfer_size);
  LRA_Maintain_Dbuf_States(dbuf, transfer_size);

  // recursive part
  //  if (transfer_size < step) {
  //    LRA_Xfer_NAcc_Rbuf2Dbuf(rbuf, dbuf, step - transfer_size);
  //  }

  return transfer_size;
}

/**
 * @brief The function maintains the states of ring buffer. Call this function
 * after every io to ring buffer.
 *
 * @param rbuf
 * @param action
 * @param step
 * @return uint32_t actually process steps, compare to input args can
 * know everything is ok or not.
 *
 * @test https://godbolt.org/z/ofvxGEz6s
 */
uint32_t LRA_Maintain_AccRingBuf_States(LRA_AccRingBuffer_t* rbuf,
                                        char action,
                                        uint32_t step) {
  if (step == 0)
    return 0;

  switch (action) {
    case 'r': {
      // flag:
      bool is_empty = (!rbuf->is_full) && (rbuf->r_index == rbuf->w_index);

      if (is_empty)
        return 0;

      uint32_t max_step = LRA_AccRingBuf_GetReadMaxStep(rbuf);

      // w_index == r_index
      if (rbuf->is_full) {
        // is_full 只要經過讀就一定不是滿的
        rbuf->is_full = false;
      }

      step = min(max_step, step);

      rbuf->r_index = (rbuf->r_index + step) % rbuf->size;
      break;
    }

    case 'w':
      // is full 還寫，必定還是 full

      if (rbuf->w_index > rbuf->r_index) {
        if (step > (rbuf->size - (rbuf->w_index - rbuf->r_index)))
          rbuf->is_full = true;
      } else if (rbuf->w_index < rbuf->r_index) {
        if (step > (rbuf->r_index - rbuf->w_index))
          rbuf->is_full = true;
      } else {
        // is_full or empty 不變
      }

      rbuf->w_index = (rbuf->w_index + step) % rbuf->size;

      if (rbuf->is_full)
        rbuf->r_index = rbuf->w_index;

      break;

    default:
      break;
  }

  return step;
}

static uint32_t LRA_AccRingBuf_GetReadMaxStep(LRA_AccRingBuffer_t* rbuf) {
  if (rbuf->r_index == rbuf->w_index) {
    if (rbuf->is_full)
      return rbuf->size;
    return 0;
  }

  if (rbuf->w_index > rbuf->r_index)
    return rbuf->w_index - rbuf->r_index;

  return rbuf->size - (rbuf->r_index - rbuf->w_index);
}
