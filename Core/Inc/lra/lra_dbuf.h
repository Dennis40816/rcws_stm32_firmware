/**
 * @file lra_dbuf.h
 * @author Dennis Liu
 * @brief Header of double buffering
 * @version 0.1
 * @date 2023-07-04
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INC_LRA_LRA_DBUF_H_
#define INC_LRA_LRA_DBUF_H_

/* includes */

#include "lra/lra_user_config.h"

#include "devices/adxl355.h"

/* structs */

typedef struct {
  ADXL355_DataSet_t data[2][LRA_ACC_BUFFER_SIZE];
  uint32_t current_buffer;
  const uint32_t size;
  uint32_t count[2];
} LRA_Acc_Dbuf_t;

/* extern variables */

extern LRA_Acc_Dbuf_t lra_acc_dbuf;

/* public functions */

void LRA_Maintain_Dbuf_States(LRA_Acc_Dbuf_t* dbuf, uint32_t n);
void LRA_Switch_Dbuf(LRA_Acc_Dbuf_t* dbuf);

#endif
