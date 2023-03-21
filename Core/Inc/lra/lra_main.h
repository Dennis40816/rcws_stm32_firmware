/*
 * lra_main.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_MAIN_H_
#define INC_LRA_LRA_MAIN_H_

/* predefined macros */

// comment next line if you don't need to debug
#define LRA_DEBUG

// comment next line to suspend system info send to Rasp
#define LRA_SYSTEM_INFO

/* enums */

// UEV for update event
typedef enum {
  LRA_PWM_FREQ_UEV_BIT = 0,
  LRA_PWM_DUTY_USV_BIT = 1,
} LRA_PWM_UPDATE_FLAG;

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

// for main.c (not lra_main.c)
#include "lra/lra_LED.h"

/* macros */

#define LRA_DEFAULT_PWM_FREQ (128 * 175)

// half of 1000 (1000‰)
#define LRA_DEFAULT_PWM_DUTY (500)

#define LRA_USB_BUFFER_SIZE (1024)
#define LRA_ACC_BUFFER_SIZE (1024)

/* structs */

/**
 * @brief A struct for double buffer
 *
 * @param cur_buf current activate buffer index, 0 or 1.
 * @param buf_full show buffer can't do next operation, 1 for full.
 * @param dbuf the buffer porinter array to store two buffers address
 * @param buf_size the maximum size of the buffer
 * @param buf_index where the buffer start to write, also equal to current
 * buffer len before writing
 *
 */
typedef struct {
  uint8_t cur_buf;
  uint8_t buf_full[2];
  uint8_t* dbuf[2];
  uint16_t buf_size[2];
  uint16_t buf_index[2];
} LRA_DualBuf_t;

/* extern variables -- buffers */

extern uint8_t LRA_USB_Rx_buf[];

extern LRA_DualBuf_t LRA_USB_Tx_dbuf;
extern LRA_DualBuf_t LRA_ACC_dbuf;

/* public functions */

void LRA_Main_EnterPoint(void);
HAL_StatusTypeDef LRA_Main_System_Init(void);

#endif /* INC_LRA_LRA_MAIN_H_ */