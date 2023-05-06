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
  ADXL355_TEMP2 = 0x06,
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
  ADXL355_CONTINUOUS_READ_FORBIDDEN_START = ADXL355_FIFO_DATA,
  ADXL355_FIFO_SAMPLES = 0x29,

  _ADXL355_WRITABLE_START = 0x1E,
  ADXL355_CONTINUOUS_READ_FORBIDDEN_END = _ADXL355_WRITABLE_START - 1,

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
} ADXL355_Regs_t;

typedef enum {
  ADXL355_ACC_2G = 0x01,
  ADXL355_ACC_4G = 0x02,
  ADXL355_ACC_8G = 0x03,
} ADXL355_Ranges_t;

/* public constants */

extern const uint8_t adxl355_1st_regnum;  // before ADXL355_FIFO_DATA
extern const uint8_t adxl355_2nd_regnum;  // after ADXL355_FIFO_DATA

/* structs */

typedef struct {
  uint16_t temp_intercept_lsb;  // default value is 1852;
  uint16_t nss_pin;  // set to 0 if you want to use uncompenstaed parser in
                     // ADXL355_Read_Temp()
  ADXL355_Ranges_t range;  // should be 0x01 ~ 0x03
  uint32_t timeout_ms;     // should > 0
  GPIO_TypeDef* nss_port;  // set to NULL if you are not using software NSS.
  SPI_HandleTypeDef* hspi;
  float temp_slope;  // set to 0 if you want to use uncompenstaed parser in
                     // ADXL355_Read_Temp(), should < 0
  float temp_intercept_Celsius;
} ADXL355_t;

typedef struct {
  float data[3];  // x, y, z
  float t;
} ADXL355_DataSet_t;

/* public functions */

HAL_StatusTypeDef ADXL355_GetRange(ADXL355_t* const pAdxl);
HAL_StatusTypeDef ADXL355_SetRange(ADXL355_t* const pAdxl,
                                   ADXL355_Ranges_t new_range);
HAL_StatusTypeDef ADXL355_Calibrate(ADXL355_t* const pAdxl);
HAL_StatusTypeDef ADXL355_Init(ADXL355_t* const pAdxl);
float ADXL355_GetCacheRange(const ADXL355_t* const pAdxl);
HAL_StatusTypeDef ADXL355_ParseOffset(ADXL355_t* const pAdxl,
                                      const uint8_t* const pOffset,
                                      ADXL355_DataSet_t* const pResult);
HAL_StatusTypeDef ADXL355_ParseDataSet(ADXL355_t* const pAdxl,
                                       uint8_t* pData,
                                       ADXL355_DataSet_t* pResult);
int8_t ADXL355_ID_Verify(ADXL355_t* const pAdxl);
HAL_StatusTypeDef ADXL355_SelfTest(ADXL355_t* const pAdxl, uint8_t st);
HAL_StatusTypeDef ADXL355_Start_Measure(ADXL355_t* const pAdxl);
HAL_StatusTypeDef ADXL355_Stop_Measure(ADXL355_t* const pAdxl);
HAL_StatusTypeDef ADXL355_Write_Offset(ADXL355_t* const pAdxl,
                                       uint8_t* const pOffset);
HAL_StatusTypeDef ADXL355_Read_Offset(ADXL355_t* const pAdxl,
                                      ADXL355_DataSet_t* const pResult);
HAL_StatusTypeDef ADXL355_Read_Temp(ADXL355_t* const pAdxl, float* pResult);
HAL_StatusTypeDef ADXL355_Read_FIFO(ADXL355_t* const pAdxl,
                                    uint8_t* const rx_buf,
                                    uint8_t num);
HAL_StatusTypeDef ADXL355_Read_NewestData(ADXL355_t* const pAdxl,
                                          uint8_t* const rx_buf);
HAL_StatusTypeDef ADXL355_Read_All(ADXL355_t* const pAdxl,
                                   uint8_t* const rx_buf);
HAL_StatusTypeDef ADXL355_Reset_Device(ADXL355_t* const pAdxl);
/* IO functions */

HAL_StatusTypeDef ADXL355_WriteReg(ADXL355_t* const pAdxl,
                                   const ADXL355_Regs_t iaddr,
                                   uint8_t val);
HAL_StatusTypeDef ADXL355_ReadReg(ADXL355_t* const pAdxl,
                                  const ADXL355_Regs_t iaddr,
                                  uint8_t* pResult);
HAL_StatusTypeDef ADXL355_LazyWrite(ADXL355_t* const pAdxl,
                                    const ADXL355_Regs_t iaddr,
                                    uint8_t* const tx_buf,
                                    const int16_t data_size);
HAL_StatusTypeDef ADXL355_LazyRead(ADXL355_t* const pAdxl,
                                   const ADXL355_Regs_t iaddr,
                                   uint8_t* const rx_buf,
                                   const int16_t data_size);
HAL_StatusTypeDef ADXL355_RW(ADXL355_t* const pAdxl,
                             uint8_t* const tx_buf,
                             uint8_t* const rx_buf,
                             const int16_t total_size);

#endif /* INC_DEVICES_ADXL355_H_ */
