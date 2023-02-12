/*
 * adxl355.h
 *
 *  Created on: Feb 12, 2023
 *      Author: Dennis
 */

#ifndef INC_DEVICES_ADXL355_H_
#define INC_DEVICES_ADXL355_H_

/* macros */

/* includes */

#include "stm32f4xx_hal.h"

/* enums */

typedef enum {
  ADXL355_DEVID_AD = 0x00,
  ADXL355_DEVID_MST = 0x01,
  ADXL355_PARTID = 0x02,
  ADXL355_REVID = 0x03,
  ADXL355_Status = 0x04,
  ADXL355_FIFO_ENTRIES = 0x05,

  /* temp */
  ADXL355_TEMP2  = 0x06,
  ADXL355_TEMP1 = 0x07,

  /* newest data */
  ADXL355_XDATA3 = 0x08,
  ADXL355_XDATA2 = 0x09,
  ADXL355_XDATA1 = 0x0A,
  ADXL355_YDATA3 = 0x0B,
  ADXL355_YDATA2 = 0x0C,
  ADXL355_YDATA1 = 0x0D,
  ADXL355_ZDATA3 = 0x0E,
  ADXL355_ZDATA2 = 0x0F,
  ADXL355_ZDATA1 = 0x10,

  /* FIFO */
  ADXL355_FIFO_DATA = 0x11,
  ADXL355_FIFO_SAMPLES = 0x29,

  _ADXL355_WRITABLE_START = 0x1E,

  /* OFFSET */
  ADXL355_OFFSET_X_H = 0x1E,
  ADXL355_OFFSET_X_L = 0x1F,
  ADXL355_OFFSET_Y_H = 0x20,
  ADXL355_OFFSET_Y_L = 0x21,
  ADXL355_OFFSET_Z_H = 0x22,
  ADXL355_OFFSET_Z_L = 0x23,
  
  /* ACT related */
  ADXL355_ACT_EN = 0x24,
  ADXL355_ACT_THRESH_H = 0x25,
  ADXL355_ACT_THRESH_L = 0x26,
  ADXL355_ACT_COUNT = 0x27,

  /* Filter */
  ADXL355_Filter = 0x28,

  /* Interrupt */
  ADXL355_INT_MAP = 0x2A,

  /* External CLK */
  ADXL355_Sync = 0x2B,

  /* Range */
  ADXL355_Range = 0x2C,

  /* Power */
  ADXL355_POWER_CTL = 0x2D,

  /* Self test */
  ADXL355_SELF_TEST = 0x2E,

  /* Reset */
  ADXL355_Reset = 0x2F,
}ADXL355_Regs_t;

/* public constants */

const uint8_t adxl355_1st_regnum = 17; // before ADXL355_FIFO_DATA
const uint8_t adxl355_2st_regnum = 18; // after ADXL355_FIFO_DATA

/* structs */

typedef struct {
  uint16_t nss_pin;
  uint32_t timeout_ms;       // should > 0
  GPIO_TypeDef* nss_port;    // set to NULL if you are not using software NSS.
  SPI_HandleTypeDef* hspi;
}ADXL355_t;

/* public functions */

#endif /* INC_DEVICES_ADXL355_H_ */
