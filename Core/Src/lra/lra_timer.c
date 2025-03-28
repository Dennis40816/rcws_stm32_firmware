/*
 * lra_timer.c
 *
 *  Created on: Mar 16, 2023
 *      Author: Dennis Liu
 */

/* includes */

#include "lra/lra_timer.h"

#include "main.h"

/* extern variables definitions */

extern TIM_HandleTypeDef htim7;

/* static variables */

static volatile unsigned long long lra_timer_ms_counter = 0;
static const float inv_1000 = 1.0f / 1000.0f;

/* public functions */

float LRA_Get_Time() {
  static volatile float last_time = 0.0f;
  uint32_t micros = __HAL_TIM_GET_COUNTER(&htim7);
  float time = (float)lra_timer_ms_counter + (float)micros * inv_1000;

  /* XXX: fix unknown reason delay */
  if (time - last_time < 0.0f)
  {
	  time = time + 0.001;
  }

  last_time = time;
  return time * inv_1000;
}

unsigned long long LRA_Get_Time_Ms() {
  return lra_timer_ms_counter;
}

/**
 * @brief Increase lra_timer_ms_counter. Usually called by timer interrupt.
 *
 */
void LRA_Ms_Increment() {
  lra_timer_ms_counter++;
}
