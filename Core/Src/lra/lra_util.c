/*
 * lra_util.c
 *
 *  Created on: Mar 16, 2023
 *      Author: Dennis Liu
 */

/* includes */

#include "lra/lra_util.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* public functions */

int LRA_Util_MakeStrNoNull(uint8_t* dest, const char* format, ...) {
  va_list args;
  va_start(args, format);

  char buffer[256];
  int length = vsnprintf(buffer, sizeof(buffer), format, args);

  va_end(args);

  memcpy(dest, buffer, length);
  return length;
}

uint8_t Check_Addr_Overlap(uint32_t addr_begin,
                           uint32_t addr_end,
                           uint32_t f_addr_begin,
                           uint32_t f_addr_end) {
  uint8_t addr_begin_in_forbidden_range =
      (addr_begin >= f_addr_begin) && (addr_begin < f_addr_end);
  uint8_t addr_end_in_forbidden_range =
      (addr_end > f_addr_begin) && (addr_end <= f_addr_end);
  uint8_t forbidden_range_inside_addr_range =
      (addr_begin <= f_addr_begin) && (addr_end >= f_addr_end);

  return (addr_begin_in_forbidden_range || addr_end_in_forbidden_range ||
          forbidden_range_inside_addr_range);
}

HAL_StatusTypeDef LRA_Read_From_DualBuf(LRA_DualBuf_t* const dBuf,
                                        uint8_t* const pdata) {
  // Check if dualBuf and pdata are valid
  if (!dBuf || !pdata) {
    return HAL_ERROR;
  }

  // Buffer index to read from
  uint8_t buf_to_read;

  // Check if the non-current buffer is full
  if (dBuf->buf_full[1 - dBuf->cur_w_buf]) {
    buf_to_read = 1 - dBuf->cur_w_buf;
  } else {
    // No buffer is full, so we read from the current buffer and switch the
    // current buffer
    buf_to_read = dBuf->cur_w_buf;
    dBuf->cur_w_buf = 1 - dBuf->cur_w_buf;
  }

  // Ensure there's data in the buffer to read
  if (dBuf->buf_index[buf_to_read] == 0) {
    return HAL_ERROR;
  }

  // Copy the data from the buffer to pdata
  memcpy(pdata, dBuf->dbuf[buf_to_read],
         dBuf->buf_index[buf_to_read] * sizeof(uint8_t));

  // Indicate that the buffer is now empty
  dBuf->buf_full[buf_to_read] = 0;
  dBuf->buf_index[buf_to_read] = 0;

  return HAL_OK;
}

HAL_StatusTypeDef LRA_Write_To_DualBuf(LRA_DualBuf_t* const dBuf,
                                       const uint8_t* const pdata,
                                       const uint16_t len) {
  // Check if dualBuf and pdata are valid
  if (!dBuf || !pdata) {
    return HAL_ERROR;  // Invalid pointers
  }

  // Check if there's enough space in the current buffer
  if (len > dBuf->buf_size - dBuf->buf_index[dBuf->cur_w_buf] - 1) {
    // Not enough space in the buffer, mark it as full
    dBuf->buf_full[dBuf->cur_w_buf] = 1;

    // Switch to the other buffer
    dBuf->cur_w_buf = 1 - dBuf->cur_w_buf;
    dBuf->buf_index[dBuf->cur_w_buf] = 0;  // Reset the buffer index

    // Check again if there's enough space in the new current buffer
    if (len > dBuf->buf_size - dBuf->buf_index[dBuf->cur_w_buf] - 1) {
      return HAL_ERROR;  // Not enough space in the buffer
    }
  }

  // Copy the data into the buffer
  memcpy(&dBuf->dbuf[dBuf->cur_w_buf][dBuf->buf_index[dBuf->cur_w_buf]], pdata,
         len);

  // Update the buffer index
  dBuf->buf_index[dBuf->cur_w_buf] += len;

  // If the buffer is full now, mark it as full and switch to the other buffer
  if (dBuf->buf_index[dBuf->cur_w_buf] == dBuf->buf_size - 1) {
    dBuf->buf_full[dBuf->cur_w_buf] = 1;
    dBuf->cur_w_buf = 1 - dBuf->cur_w_buf;
    dBuf->buf_index[dBuf->cur_w_buf] = 0;  // Reset the buffer index
  }

  return HAL_OK;
}