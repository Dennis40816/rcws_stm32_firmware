#include "lra/lra_dbuf.h"

/* extern variables definitions */

LRA_Acc_Dbuf_t lra_acc_dbuf = {.count = {0, 0},
                               .current_buffer = 0,
                               .size = LRA_ACC_BUFFER_SIZE};

/* public functions */

/**
 * @brief Control when to switch to other buffer
 *
 * @warning This is no len control of this function, you need to control the
 * number of step in every io dbuf function.
 * @param dbuf
 * @param n
 */
void LRA_Maintain_Dbuf_States(LRA_Acc_Dbuf_t* dbuf, uint32_t n) {
  dbuf->count[dbuf->current_buffer] += n;

  /* count number protect */
  if (dbuf->count[dbuf->current_buffer] > dbuf->size)
    dbuf->count[dbuf->current_buffer] = dbuf->size;

  /* switch check */
  if (dbuf->count[dbuf->current_buffer] == dbuf->size) {
    LRA_Switch_Dbuf(dbuf);
  }
}

void LRA_Switch_Dbuf(LRA_Acc_Dbuf_t* dbuf) {
  dbuf->current_buffer = 1 - dbuf->current_buffer;
  dbuf->count[dbuf->current_buffer] = 0;
}
