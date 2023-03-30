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