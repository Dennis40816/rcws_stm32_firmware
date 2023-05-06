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